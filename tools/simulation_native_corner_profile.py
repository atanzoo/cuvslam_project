#!/usr/bin/env python3
"""Drive a corner route using native pose distance and D435i gyro rotation."""

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
from geometry_msgs.msg import PoseStamped


class NativeCornerProfile(Node):
    def __init__(
        self,
        first_leg: float,
        second_leg: float,
        speed: float,
        turn_rate: float,
        target_angle: float,
        stop_distance: float,
    ) -> None:
        super().__init__("cuvslam_native_corner_profile")
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(
            PoseStamped, "/simulation/native_pose", self._on_native_pose, 10
        )
        self.create_subscription(
            Imu, "/d435i/imu", self._on_imu, qos_profile_sensor_data
        )
        self.create_subscription(
            VisualSlamStatus, "/visual_slam/status", self._on_status, 10
        )
        self.create_subscription(
            LaserScan, "/scan", self._on_scan, qos_profile_sensor_data
        )
        self.first_leg = first_leg
        self.second_leg = second_leg
        self.speed = speed
        self.turn_rate = turn_rate
        self.target_angle = target_angle
        self.stop_distance = stop_distance
        self.position: tuple[float, float, float] | None = None
        self.last_native_at: float | None = None
        self.last_imu_at: float | None = None
        self.last_scan_at: float | None = None
        self.minimum_range: float | None = None
        self.vo_state: int | None = None
        self.imu_active = False
        self.integrated_angle = 0.0
        self.last_imu_stamp_ns: int | None = None
        self.last_yaw_rate: float | None = None

    def _on_native_pose(self, message: PoseStamped) -> None:
        value = message.pose.position
        self.position = (float(value.x), float(value.y), float(value.z))
        self.last_native_at = time.monotonic()

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
        values = [
            value
            for value in message.ranges
            if math.isfinite(value)
            and message.range_min <= value <= message.range_max
        ]
        self.minimum_range = min(values) if values else None
        self.last_scan_at = time.monotonic()

    def publish_velocity(self, linear: float = 0.0, angular: float = 0.0) -> None:
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        self.publisher.publish(message)

    def stop(self, duration: float = 0.8) -> None:
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_velocity()
            time.sleep(0.02)

    def check_health(self) -> None:
        now = time.monotonic()
        if self.vo_state != 1:
            raise RuntimeError(f"cuVSLAM tracking lost: vo_state={self.vo_state}")
        if self.last_native_at is None or now - self.last_native_at > 0.25:
            raise RuntimeError("Gazebo native pose is stale for more than 0.25s")
        if self.last_imu_at is None or now - self.last_imu_at > 0.2:
            raise RuntimeError("D435i IMU is stale for more than 0.2s")
        if self.last_scan_at is None or now - self.last_scan_at > 1.0:
            raise RuntimeError("LaserScan is stale for more than 1.0s")
        if self.minimum_range is not None and self.minimum_range < self.stop_distance:
            raise RuntimeError(
                f"obstacle clearance {self.minimum_range:.3f}m is below "
                f"{self.stop_distance:.3f}m"
            )

    def wait_until_ready(self, timeout: float = 25.0) -> None:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_velocity()
            if (
                self.position is not None
                and self.vo_state == 1
                and self.last_imu_at is not None
                and self.minimum_range is not None
            ):
                self.check_health()
                return
            time.sleep(0.02)
        raise RuntimeError("native pose, cuVSLAM, IMU, or scan did not become ready")

    def settle(self, duration: float) -> None:
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            self.check_health()
            self.publish_velocity()
            time.sleep(0.02)

    def run_linear(self, phase: str, target: float, timeout: float) -> float:
        assert self.position is not None
        origin = self.position
        started = time.monotonic()
        next_report = started
        distance = 0.0
        while rclpy.ok() and distance < target:
            self.check_health()
            assert self.position is not None
            distance = math.dist(origin, self.position)
            if time.monotonic() - started > timeout:
                raise RuntimeError(f"{phase} timed out at {distance:.3f}m")
            self.publish_velocity(linear=self.speed)
            now = time.monotonic()
            if now >= next_report:
                print(
                    f"phase={phase}, native_distance={distance:.3f}m, "
                    f"min_scan={self.minimum_range:.3f}m, vo_state={self.vo_state}",
                    flush=True,
                )
                next_report = now + 2.0
            time.sleep(0.02)
        self.stop()
        return distance

    def run_turn(self, timeout: float) -> float:
        self.integrated_angle = 0.0
        self.last_imu_stamp_ns = None
        self.last_yaw_rate = None
        self.imu_active = True
        started = time.monotonic()
        next_report = started
        command = math.copysign(abs(self.turn_rate), self.target_angle)
        while rclpy.ok() and abs(self.integrated_angle) < abs(self.target_angle):
            self.check_health()
            if time.monotonic() - started > timeout:
                raise RuntimeError(
                    f"turn timed out at {math.degrees(self.integrated_angle):.2f}deg"
                )
            self.publish_velocity(angular=command)
            now = time.monotonic()
            if now >= next_report:
                print(
                    f"phase=turn, imu_angle={math.degrees(self.integrated_angle):.2f}deg, "
                    f"min_scan={self.minimum_range:.3f}m, vo_state={self.vo_state}",
                    flush=True,
                )
                next_report = now + 2.0
            time.sleep(0.02)
        self.imu_active = False
        self.stop()
        return self.integrated_angle

    def run(self, pre_motion_settle: float, phase_settle: float, timeout: float) -> None:
        self.wait_until_ready()
        self.settle(pre_motion_settle)
        first = self.run_linear("first_leg", self.first_leg, timeout)
        self.settle(phase_settle)
        turn = self.run_turn(timeout)
        self.settle(phase_settle)
        second = self.run_linear("second_leg", self.second_leg, timeout)
        print(
            f"complete: first_leg={first:.4f}m, "
            f"imu_turn={math.degrees(turn):.2f}deg, "
            f"second_leg={second:.4f}m, vo_state={self.vo_state}",
            flush=True,
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--first-leg-distance", type=float, default=1.0)
    parser.add_argument("--second-leg-distance", type=float, default=0.5)
    parser.add_argument("--speed", type=float, default=0.12)
    parser.add_argument("--turn-rate-deg-s", type=float, default=20.0)
    parser.add_argument("--target-angle-deg", type=float, default=90.0)
    parser.add_argument("--stop-distance", type=float, default=0.30)
    parser.add_argument("--pre-motion-settle", type=float, default=8.0)
    parser.add_argument("--phase-settle", type=float, default=2.0)
    parser.add_argument("--timeout", type=float, default=120.0)
    args = parser.parse_args()
    if min(
        args.first_leg_distance,
        args.second_leg_distance,
        args.speed,
        args.turn_rate_deg_s,
        args.target_angle_deg,
        args.stop_distance,
        args.pre_motion_settle,
        args.phase_settle,
        args.timeout,
    ) <= 0.0:
        parser.error("all numeric arguments must be positive")

    rclpy.init()
    node = NativeCornerProfile(
        args.first_leg_distance,
        args.second_leg_distance,
        args.speed,
        math.radians(args.turn_rate_deg_s),
        math.radians(args.target_angle_deg),
        args.stop_distance,
    )
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        node.run(args.pre_motion_settle, args.phase_settle, args.timeout)
    finally:
        node.stop()
        executor.shutdown()
        thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
