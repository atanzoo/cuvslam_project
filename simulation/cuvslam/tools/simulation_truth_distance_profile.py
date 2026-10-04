#!/usr/bin/env python3
"""Drive a simulation straight to a truth-measured diagnostic distance."""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class TruthDistanceProfile(Node):
    def __init__(
        self,
        target_distance: float,
        speed: float,
        stop_distance: float,
    ) -> None:
        super().__init__("cuvslam_truth_distance_profile")
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(
            Odometry,
            "/ground_truth/odom",
            self._on_truth,
            10,
        )
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
        self.target_distance = target_distance
        self.speed = speed
        self.stop_distance = stop_distance
        self.start_position = None
        self.current_position = None
        self.vo_state = None
        self.minimum_range = None
        self.last_scan_at = None

    def _on_truth(self, message: Odometry) -> None:
        position = message.pose.pose.position
        self.current_position = (position.x, position.y, position.z)
        if self.start_position is None:
            self.start_position = self.current_position

    def _on_status(self, message: VisualSlamStatus) -> None:
        self.vo_state = message.vo_state

    def _on_scan(self, message: LaserScan) -> None:
        ranges = [
            value
            for value in message.ranges
            if math.isfinite(value)
            and message.range_min <= value <= message.range_max
        ]
        self.minimum_range = min(ranges) if ranges else None
        self.last_scan_at = time.monotonic()

    def publish_velocity(self, speed: float) -> None:
        message = Twist()
        message.linear.x = speed
        self.publisher.publish(message)

    def stop(self, duration: float = 1.0) -> None:
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_velocity(0.0)
            rclpy.spin_once(self, timeout_sec=0.05)

    def distance(self) -> float:
        if self.start_position is None or self.current_position is None:
            return 0.0
        return math.dist(self.start_position, self.current_position)

    def wait_until_ready(self, timeout: float = 15.0) -> None:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            self.publish_velocity(0.0)
            if (
                self.start_position is not None
                and self.vo_state == 1
                and self.minimum_range is not None
                and self.minimum_range >= self.stop_distance
            ):
                return
        raise RuntimeError("truth, cuVSLAM, or LaserScan did not become ready")

    def run(self, timeout: float) -> None:
        self.wait_until_ready()
        started = time.monotonic()
        next_report = started
        while rclpy.ok() and self.distance() < self.target_distance:
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.vo_state != 1:
                raise RuntimeError(f"cuVSLAM tracking lost: vo_state={self.vo_state}")
            if self.last_scan_at is None or time.monotonic() - self.last_scan_at > 1.0:
                raise RuntimeError("LaserScan is stale for more than 1.0s")
            if (
                self.minimum_range is not None
                and self.minimum_range < self.stop_distance
            ):
                raise RuntimeError(
                    f"obstacle clearance {self.minimum_range:.3f}m is below "
                    f"{self.stop_distance:.3f}m"
                )
            if time.monotonic() - started > timeout:
                raise RuntimeError(
                    f"timeout at truth distance {self.distance():.4f}m"
                )
            self.publish_velocity(self.speed)
            now = time.monotonic()
            if now >= next_report:
                print(
                    f"truth_distance={self.distance():.4f}m, "
                    f"min_scan={self.minimum_range:.3f}m, "
                    f"vo_state={self.vo_state}",
                    flush=True,
                )
                next_report = now + 2.0
            time.sleep(0.02)
        self.stop()
        print(
            f"complete: truth_distance={self.distance():.4f}m, "
            f"vo_state={self.vo_state}",
            flush=True,
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-distance", type=float, default=1.0)
    parser.add_argument("--speed", type=float, default=0.12)
    parser.add_argument("--stop-distance", type=float, default=0.35)
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()
    if min(args.target_distance, args.speed, args.stop_distance, args.timeout) <= 0:
        parser.error("all numeric arguments must be positive")

    rclpy.init()
    node = TruthDistanceProfile(
        args.target_distance,
        args.speed,
        args.stop_distance,
    )
    try:
        node.run(args.timeout)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
