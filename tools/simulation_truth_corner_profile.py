#!/usr/bin/env python3
"""Drive a truth-gated straight, 90-degree turn, and short straight route."""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

from simulation_truth_distance_profile import TruthDistanceProfile


def quaternion_yaw(orientation) -> float:
    return math.atan2(
        2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        ),
        1.0
        - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        ),
    )


def wrapped_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


class TruthCornerProfile(TruthDistanceProfile):
    def __init__(
        self,
        first_leg_distance: float,
        second_leg_distance: float,
        speed: float,
        turn_rate: float,
        target_angle: float,
        stop_distance: float,
    ) -> None:
        super().__init__(first_leg_distance, speed, stop_distance)
        self.second_leg_distance = second_leg_distance
        self.turn_rate = turn_rate
        self.target_angle = target_angle
        self.current_yaw = None
        self.imu_turn_angle = 0.0
        self.imu_turn_active = False
        self.last_imu_stamp_ns = None
        self.last_imu_yaw_rate = None
        self.create_subscription(Imu, "/d435i/imu", self._on_imu, 1)

    def _on_truth(self, message) -> None:
        super()._on_truth(message)
        self.current_yaw = quaternion_yaw(message.pose.pose.orientation)

    def _on_imu(self, message: Imu) -> None:
        if not self.imu_turn_active:
            return
        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        yaw_rate = float(message.angular_velocity.z)
        if (
            self.last_imu_stamp_ns is not None
            and self.last_imu_yaw_rate is not None
            and stamp_ns > self.last_imu_stamp_ns
        ):
            dt = (stamp_ns - self.last_imu_stamp_ns) / 1_000_000_000.0
            if dt <= 0.05:
                self.imu_turn_angle += (
                    self.last_imu_yaw_rate + yaw_rate
                ) * 0.5 * dt
        self.last_imu_stamp_ns = stamp_ns
        self.last_imu_yaw_rate = yaw_rate

    def check_health(self) -> None:
        if self.vo_state != 1:
            raise RuntimeError(f"cuVSLAM tracking lost: vo_state={self.vo_state}")
        if self.last_scan_at is None or time.monotonic() - self.last_scan_at > 1.0:
            raise RuntimeError("LaserScan is stale for more than 1.0s")
        if self.minimum_range is not None and self.minimum_range < self.stop_distance:
            raise RuntimeError(
                f"obstacle clearance {self.minimum_range:.3f}m is below "
                f"{self.stop_distance:.3f}m"
            )

    def run_until(self, phase: str, complete, linear: float, angular: float) -> None:
        started = time.monotonic()
        next_report = started
        while rclpy.ok() and not complete():
            for _ in range(16):
                rclpy.spin_once(self, timeout_sec=0.0)
            self.check_health()
            if time.monotonic() - started > 35.0:
                raise RuntimeError(f"{phase} timed out")
            message = Twist()
            message.linear.x = linear
            message.angular.z = angular
            self.publisher.publish(message)
            now = time.monotonic()
            if now >= next_report:
                print(
                    f"phase={phase}, min_scan={self.minimum_range:.3f}m, "
                    f"vo_state={self.vo_state}",
                    flush=True,
                )
                next_report = now + 2.0
            time.sleep(0.02)
        self.stop()

    def run_corner(self) -> None:
        self.wait_until_ready()
        self.run_until(
            "first_leg",
            lambda: self.distance() >= self.target_distance,
            self.speed,
            0.0,
        )
        first_leg_end = self.current_position
        turn_start_yaw = self.current_yaw
        print(
            f"first_leg_complete: truth_distance={self.distance():.4f}m",
            flush=True,
        )

        self.imu_turn_angle = 0.0
        self.last_imu_stamp_ns = None
        self.last_imu_yaw_rate = None
        self.imu_turn_active = True

        def turned_enough() -> bool:
            return abs(self.imu_turn_angle) >= abs(self.target_angle)

        self.run_until(
            "turn",
            turned_enough,
            0.0,
            math.copysign(abs(self.turn_rate), self.target_angle),
        )
        self.imu_turn_active = False
        achieved_angle = self.imu_turn_angle
        wheel_odom_angle = wrapped_angle(self.current_yaw - turn_start_yaw)
        print(
            f"turn_complete: imu_angle={math.degrees(achieved_angle):.2f}deg, "
            f"wheel_odom_angle={math.degrees(wheel_odom_angle):.2f}deg",
            flush=True,
        )

        def second_leg_distance() -> float:
            if first_leg_end is None or self.current_position is None:
                return 0.0
            return math.dist(first_leg_end, self.current_position)

        self.run_until(
            "second_leg",
            lambda: second_leg_distance() >= self.second_leg_distance,
            self.speed,
            0.0,
        )
        print(
            f"complete: first_leg={self.distance():.4f}m_from_start, "
            f"second_leg={second_leg_distance():.4f}m, "
            f"imu_turn={math.degrees(achieved_angle):.2f}deg, "
            f"wheel_odom_turn={math.degrees(wheel_odom_angle):.2f}deg, "
            f"vo_state={self.vo_state}",
            flush=True,
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--first-leg-distance", type=float, default=1.0)
    parser.add_argument("--second-leg-distance", type=float, default=0.5)
    parser.add_argument("--speed", type=float, default=0.12)
    parser.add_argument("--turn-rate", type=float, default=0.25)
    parser.add_argument("--target-angle-deg", type=float, default=90.0)
    parser.add_argument("--stop-distance", type=float, default=0.35)
    args = parser.parse_args()
    if min(
        args.first_leg_distance,
        args.second_leg_distance,
        args.speed,
        args.turn_rate,
        args.target_angle_deg,
        args.stop_distance,
    ) <= 0.0:
        parser.error("all numeric arguments must be positive")

    rclpy.init()
    node = TruthCornerProfile(
        args.first_leg_distance,
        args.second_leg_distance,
        args.speed,
        args.turn_rate,
        math.radians(args.target_angle_deg),
        args.stop_distance,
    )
    try:
        node.run_corner()
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
