#!/usr/bin/env python3
"""Measure one manual real-D435i motion segment without ros2 CLI tools."""

from __future__ import annotations

import argparse
import json
import math
import time
from collections import Counter

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message


def stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def position(message) -> list[float]:
    pose = message.pose.pose
    return [float(pose.position.x), float(pose.position.y), float(pose.position.z)]


def distance(first: list[float], current: list[float]) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, current)))


def interval_stats(stamps: list[int]) -> dict[str, float | int]:
    intervals = [(b - a) / 1e6 for a, b in zip(stamps, stamps[1:]) if b > a]
    if not intervals:
        return {"samples": len(stamps), "median_ms": 0.0, "p95_ms": 0.0, "max_ms": 0.0, "over_100ms": 0}
    ordered = sorted(intervals)
    index = min(len(ordered) - 1, math.ceil(len(ordered) * 0.95) - 1)
    return {
        "samples": len(stamps),
        "median_ms": ordered[len(ordered) // 2],
        "p95_ms": ordered[index],
        "max_ms": max(ordered),
        "over_100ms": sum(value > 100.0 for value in intervals),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=25.0)
    parser.add_argument("--countdown", type=int, default=3)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("real_d435i_manual_motion_measurement")
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
    odom_positions: list[list[float]] = []
    odom_stamps: list[int] = []
    states: Counter = Counter()

    def odom_callback(message) -> None:
        odom_positions.append(position(message))
        odom_stamps.append(stamp_ns(message))

    def status_callback(message) -> None:
        states[str(getattr(message, "vo_state", "unknown"))] += 1

    node.create_subscription(Odometry, "/visual_slam/tracking/odometry", odom_callback, qos)
    node.create_subscription(
        get_message("isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus"),
        "/visual_slam/status",
        status_callback,
        qos,
    )

    print("MOTION_TEST_READY", flush=True)
    for remaining in range(max(0, args.countdown), 0, -1):
        print(f"MOTION_TEST_START_IN {remaining}", flush=True)
        time.sleep(1.0)
    print("MOTION_TEST_START", flush=True)
    start = time.monotonic()
    while time.monotonic() - start < args.duration:
        rclpy.spin_once(node, timeout_sec=0.05)

    output: dict[str, object] = {
        "duration_s": time.monotonic() - start,
        "odom_interval": interval_stats(odom_stamps),
        "status_counts": dict(states),
        "odom_samples": len(odom_positions),
    }
    if odom_positions:
        origin = odom_positions[0]
        displacements = [distance(origin, sample) for sample in odom_positions]
        output["origin"] = origin
        output["final"] = odom_positions[-1]
        output["final_displacement_m"] = displacements[-1]
        output["max_displacement_m"] = max(displacements)
        output["final_horizontal_m"] = math.hypot(
            odom_positions[-1][0] - origin[0], odom_positions[-1][1] - origin[1]
        )
    else:
        output["result"] = "NO_ODOMETRY"
    print("MOTION_TEST_RESULT " + json.dumps(output, sort_keys=True), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
