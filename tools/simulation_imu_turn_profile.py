#!/usr/bin/env python3
"""Drive an IMU-gated in-place turn without wheel-odometry input."""

from __future__ import annotations

import argparse
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan


class ImuTurnProfile(Node):
    def __init__(
        self,
        target_angle: float,
        turn_rate: float,
        stop_distance: float,
    ) -> None:
        super().__init__("cuvslam_imu_turn_profile")
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(
            Imu,
            "/d435i/imu",
            self._on_imu,
            qos_profile_sensor_data,
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
        self.target_angle = target_angle
        self.turn_rate = turn_rate
        self.stop_distance = stop_distance
        self.integrated_angle = 0.0
        self.last_imu_stamp_ns = None
        self.last_yaw_rate = None
        self.imu_active = False
        self.last_imu_at = None
        self.vo_state = None
        self.minimum_range = None
        self.last_scan_at = None

    def _on_imu(self, message: Imu) -> None:
        self.last_imu_at = time.monotonic()
        if not self.imu_active:
            return
        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        yaw_rate = float(message.angular_velocity.z)
        if (
            self.last_imu_stamp_ns is not None
            and self.last_yaw_rate is not None
            and stamp_ns > self.last_imu_stamp_ns
        ):
            dt = (stamp_ns - self.last_imu_stamp_ns) / 1_000_000_000.0
            if dt <= 0.05:
                self.integrated_angle += (
                    self.last_yaw_rate + yaw_rate
                ) * 0.5 * dt
        self.last_imu_stamp_ns = stamp_ns
        self.last_yaw_rate = yaw_rate

    def _on_status(self, message: VisualSlamStatus) -> None:
        self.vo_state = message.vo_state

    def _on_scan(self, message: LaserScan) -> None:
        ranges = [
            value for value in message.ranges
            if math.isfinite(value)
            and message.range_min <= value <= message.range_max
        ]
        self.minimum_range = min(ranges) if ranges else None
        self.last_scan_at = time.monotonic()

    def publish_velocity(self, angular: float) -> None:
        message = Twist()
        message.angular.z = angular
        self.publisher.publish(message)

    def stop(self, duration: float = 1.0) -> None:
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_velocity(0.0)
            time.sleep(0.02)

    def check_health(self) -> None:
        now = time.monotonic()
        if self.vo_state != 1:
            raise RuntimeError(f"cuVSLAM tracking lost: vo_state={self.vo_state}")
        if self.last_imu_at is None or now - self.last_imu_at > 0.2:
            raise RuntimeError("D435i IMU is stale for more than 0.2s")
        if self.last_scan_at is None or now - self.last_scan_at > 1.0:
            raise RuntimeError("LaserScan is stale for more than 1.0s")
        if self.minimum_range is not None and self.minimum_range < self.stop_distance:
            raise RuntimeError(
                f"obstacle clearance {self.minimum_range:.3f}m is below "
                f"{self.stop_distance:.3f}m"
            )

    def wait_until_ready(self, timeout: float = 20.0) -> None:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_velocity(0.0)
            if (
                self.vo_state == 1
                and self.last_imu_at is not None
                and self.minimum_range is not None
                and self.minimum_range >= self.stop_distance
            ):
                return
            time.sleep(0.02)
        raise RuntimeError("cuVSLAM, D435i IMU, or LaserScan did not become ready")

    def settle(self, duration: float) -> None:
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            self.check_health()
            self.publish_velocity(0.0)
            time.sleep(0.02)

    def run(self, pre_turn_settle: float, timeout: float) -> None:
        self.wait_until_ready()
        self.settle(pre_turn_settle)
        self.integrated_angle = 0.0
        self.last_imu_stamp_ns = None
        self.last_yaw_rate = None
        self.imu_active = True
        started = time.monotonic()
        next_report = started
        commanded_rate = math.copysign(
            abs(self.turn_rate),
            self.target_angle,
        )
        while rclpy.ok() and abs(self.integrated_angle) < abs(self.target_angle):
            self.check_health()
            if time.monotonic() - started > timeout:
                raise RuntimeError(
                    "turn timed out at "
                    f"{math.degrees(self.integrated_angle):.2f}deg"
                )
            self.publish_velocity(commanded_rate)
            now = time.monotonic()
            if now >= next_report:
                print(
                    f"imu_angle={math.degrees(self.integrated_angle):.2f}deg, "
                    f"min_scan={self.minimum_range:.3f}m, "
                    f"vo_state={self.vo_state}",
                    flush=True,
                )
                next_report = now + 2.0
            time.sleep(0.02)
        self.imu_active = False
        self.stop()
        print(
            f"complete: imu_turn={math.degrees(self.integrated_angle):.2f}deg, "
            f"command_rate={math.degrees(commanded_rate):.2f}deg/s, "
            f"duration={time.monotonic() - started:.3f}s, "
            f"vo_state={self.vo_state}",
            flush=True,
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target-angle-deg", type=float, default=90.0)
    parser.add_argument("--turn-rate-deg-s", type=float, default=20.0)
    parser.add_argument("--stop-distance", type=float, default=0.35)
    parser.add_argument("--pre-turn-settle", type=float, default=8.0)
    parser.add_argument("--timeout", type=float, default=40.0)
    args = parser.parse_args()
    if min(
        args.target_angle_deg,
        args.turn_rate_deg_s,
        args.stop_distance,
        args.pre_turn_settle,
        args.timeout,
    ) <= 0.0:
        parser.error("all numeric arguments must be positive")

    rclpy.init()
    node = ImuTurnProfile(
        math.radians(args.target_angle_deg),
        math.radians(args.turn_rate_deg_s),
        args.stop_distance,
    )
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.run(args.pre_turn_settle, args.timeout)
    finally:
        node.stop()
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
