#!/usr/bin/env python3
"""Run a ground-truth-independent motion profile for cuVSLAM simulation."""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class EstimatorProfile(Node):
    def __init__(
        self,
        speed: float,
        turn_rate: float,
        leg_duration: float,
        turn_duration: float,
        stop_distance: float,
        mode: str,
        cycles: int,
    ) -> None:
        super().__init__("cuvslam_sim_estimator_profile")
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(
            VisualSlamStatus,
            "/visual_slam/status",
            self._on_status,
            10,
        )
        self.create_subscription(
            LaserScan,
            "/scan",
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.speed = speed
        self.turn_rate = turn_rate
        self.leg_duration = leg_duration
        self.turn_duration = turn_duration
        self.stop_distance = stop_distance
        self.mode = mode
        self.cycles = cycles
        self.vo_state = None
        self.minimum_range = None
        self.last_scan_at = None
        self.abort_reason = None

    def _on_status(self, message: VisualSlamStatus) -> None:
        self.vo_state = message.vo_state
        if message.vo_state != 1:
            self.abort_reason = f"cuVSLAM tracking lost: vo_state={message.vo_state}"

    def _on_scan(self, message: LaserScan) -> None:
        valid_ranges = [
            value
            for value in message.ranges
            if math.isfinite(value)
            and message.range_min <= value <= message.range_max
        ]
        self.last_scan_at = time.monotonic()
        self.minimum_range = min(valid_ranges) if valid_ranges else None
        if (
            self.minimum_range is not None
            and self.minimum_range < self.stop_distance
        ):
            self.abort_reason = (
                f"obstacle clearance {self.minimum_range:.3f}m is below "
                f"{self.stop_distance:.3f}m"
            )

    def publish_velocity(self, linear: float, angular: float) -> None:
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        self.publisher.publish(message)

    def stop(self, duration: float = 1.0) -> None:
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_velocity(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_until_ready(self, timeout: float = 10.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            self.publish_velocity(0.0, 0.0)
            if (
                self.vo_state == 1
                and self.minimum_range is not None
                and self.minimum_range >= self.stop_distance
            ):
                self.abort_reason = None
                return
        raise RuntimeError("cuVSLAM status or LaserScan did not become ready")

    def check_safety(self) -> None:
        if self.abort_reason:
            raise RuntimeError(self.abort_reason)
        if (
            self.last_scan_at is None
            or time.monotonic() - self.last_scan_at > 1.0
        ):
            raise RuntimeError("LaserScan is stale for more than 1.0s")

    def run_phase(
        self,
        label: str,
        duration: float,
        linear: float,
        angular: float,
    ) -> None:
        print(
            f"{label}: duration={duration:.2f}s, "
            f"linear={linear:.3f}, angular={angular:.3f}",
            flush=True,
        )
        started = time.monotonic()
        next_report = started
        while rclpy.ok() and time.monotonic() - started < duration:
            rclpy.spin_once(self, timeout_sec=0.01)
            self.check_safety()
            self.publish_velocity(linear, angular)
            now = time.monotonic()
            if now >= next_report:
                print(
                    f"  elapsed={now - started:.1f}s, "
                    f"min_scan={self.minimum_range:.3f}m, "
                    f"vo_state={self.vo_state}",
                    flush=True,
                )
                next_report = now + 2.0
            time.sleep(0.04)
        self.stop(0.5)

    def run(self) -> None:
        self.wait_until_ready()
        print(
            f"ready: min_scan={self.minimum_range:.3f}m, "
            f"vo_state={self.vo_state}; ground truth is not subscribed",
            flush=True,
        )
        if self.mode == "straight":
            self.run_phase(
                "diagnostic straight",
                self.leg_duration,
                self.speed,
                0.0,
            )
        elif self.mode == "turn":
            self.run_phase(
                "diagnostic turn",
                self.turn_duration,
                0.0,
                self.turn_rate,
            )
        else:
            for index in range(self.cycles):
                self.run_phase(
                    f"leg {index + 1}/{self.cycles}",
                    self.leg_duration,
                    self.speed,
                    0.0,
                )
                self.run_phase(
                    f"turn {index + 1}/{self.cycles}",
                    self.turn_duration,
                    0.0,
                    self.turn_rate,
                )
        print(f"complete: vo_state={self.vo_state}", flush=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--speed", type=float, default=0.18)
    parser.add_argument("--turn-rate", type=float, default=-0.25)
    parser.add_argument("--leg-duration", type=float, default=6.0)
    parser.add_argument("--turn-duration", type=float, default=6.283)
    parser.add_argument("--stop-distance", type=float, default=0.35)
    parser.add_argument(
        "--mode",
        choices=("profile", "straight", "turn"),
        default="profile",
    )
    parser.add_argument("--cycles", type=int, default=4)
    args = parser.parse_args()
    if args.speed <= 0.0:
        parser.error("speed must be positive")
    if args.turn_rate == 0.0:
        parser.error("turn-rate must be non-zero")
    if args.leg_duration <= 0.0 or args.turn_duration <= 0.0:
        parser.error("phase durations must be positive")
    if args.stop_distance <= 0.0:
        parser.error("stop-distance must be positive")
    if args.cycles <= 0:
        parser.error("cycles must be positive")
    return args


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = EstimatorProfile(
        args.speed,
        args.turn_rate,
        args.leg_duration,
        args.turn_duration,
        args.stop_distance,
        args.mode,
        args.cycles,
    )
    try:
        node.run()
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
