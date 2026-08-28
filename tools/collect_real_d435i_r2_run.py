#!/usr/bin/env python3
"""Collect real-D435i R2 odometry, tracking, and timing evidence."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from collections import Counter

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from rosidl_runtime_py.utilities import get_message


def stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def percentile(values: list[float], percent: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    position = (len(ordered) - 1) * percent / 100.0
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def timing(stamps: list[int]) -> dict[str, float | int]:
    ordered = sorted(stamps)
    intervals = [(b - a) / 1e6 for a, b in zip(ordered, ordered[1:])]
    return {
        "messages": len(ordered),
        "rate_hz": ((len(ordered) - 1) / ((ordered[-1] - ordered[0]) / 1e9)
                    if len(ordered) > 1 and ordered[-1] > ordered[0] else 0.0),
        "period_median_ms": percentile(intervals, 50.0),
        "period_p95_ms": percentile(intervals, 95.0),
        "period_max_ms": max(intervals, default=float("nan")),
        "non_monotonic": sum(value <= 0.0 for value in intervals),
    }


def primitive_fields(message) -> dict[str, object]:
    result = {}
    for name, type_name in message.get_fields_and_field_types().items():
        if type_name in {
            "boolean", "byte", "char", "float32", "float64", "int8", "uint8",
            "int16", "uint16", "int32", "uint32", "int64", "uint64", "string",
        }:
            value = getattr(message, name)
            if isinstance(value, (str, int, float, bool)):
                result[name] = value
    return result


def point(message) -> list[float]:
    pose = message.pose.pose if hasattr(message, "pose") and hasattr(message.pose, "pose") else message.pose
    return [float(pose.position.x), float(pose.position.y), float(pose.position.z)]


def distance(first: list[float], current: list[float]) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, current)))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--output", required=True)
    parser.add_argument("--route", default="r2_manual_short_straight")
    parser.add_argument("--mode", default="stereo_only")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("real_d435i_r2_run_collector")
    sensor_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
    reliable_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
    counts = Counter()
    imu_stamps: list[int] = []
    odom_stamps: list[int] = []
    vo_stamps: list[int] = []
    odom_positions: list[list[float]] = []
    vo_positions: list[list[float]] = []
    path_lengths: list[float] = []
    status_samples: list[dict[str, object]] = []

    def count(topic: str):
        def callback(_message):
            counts[topic] += 1
        return callback

    def imu_callback(message):
        counts["/camera/imu"] += 1
        imu_stamps.append(stamp_ns(message))

    def odom_callback(message):
        counts["/visual_slam/tracking/odometry"] += 1
        odom_stamps.append(stamp_ns(message))
        odom_positions.append(point(message))

    def vo_callback(message):
        counts["/visual_slam/tracking/vo_pose"] += 1
        vo_stamps.append(stamp_ns(message))
        vo_positions.append(point(message))

    def path_callback(message):
        counts["/visual_slam/tracking/slam_path"] += 1
        if message.poses:
            total = 0.0
            previous = point(message.poses[0])
            for pose in message.poses[1:]:
                current = point(pose)
                total += distance(previous, current)
                previous = current
            path_lengths.append(total)

    def status_callback(message):
        counts["/visual_slam/status"] += 1
        status_samples.append(primitive_fields(message))

    # The rosbag recorder owns the full image streams.  Do not subscribe to
    # 848x480 Image payloads here: a single-threaded Python callback would
    # starve the lightweight IMU/odometry callbacks and make this collector
    # under-report their rates.
    node.create_subscription(Imu, "/camera/imu", imu_callback, reliable_qos)
    node.create_subscription(Odometry, "/visual_slam/tracking/odometry", odom_callback, reliable_qos)
    node.create_subscription(PoseStamped, "/visual_slam/tracking/vo_pose", vo_callback, reliable_qos)
    node.create_subscription(Path, "/visual_slam/tracking/slam_path", path_callback, reliable_qos)
    node.create_subscription(get_message("isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus"), "/visual_slam/status", status_callback, reliable_qos)

    start = time.monotonic()
    while time.monotonic() - start < args.duration:
        rclpy.spin_once(node, timeout_sec=0.2)

    output = {
        "route": args.route,
        "mode": args.mode,
        "duration_s": time.monotonic() - start,
        "counts": dict(counts),
        "imu_timing": timing(imu_stamps),
        "odometry_timing": timing(odom_stamps),
        "vo_pose_timing": timing(vo_stamps),
        "status_samples": status_samples,
        "odometry_samples": len(odom_positions),
        "vo_pose_samples": len(vo_positions),
        "slam_path_samples": len(path_lengths),
        "slam_path_length_latest_m": path_lengths[-1] if path_lengths else None,
    }
    if odom_positions:
        output["odometry_geometry"] = {
            "start": odom_positions[0],
            "end": odom_positions[-1],
            "end_displacement_m": distance(odom_positions[0], odom_positions[-1]),
            "max_displacement_m": max(distance(odom_positions[0], p) for p in odom_positions),
        }
    else:
        output["odometry_geometry"] = {"status": "NO_ODOMETRY_SAMPLES"}
    if status_samples:
        vo_states = [sample.get("vo_state") for sample in status_samples if "vo_state" in sample]
        output["tracking_gate"] = {
            "status_samples": len(status_samples),
            "vo_state_values": sorted({value for value in vo_states if value is not None}),
            "vo_state_1_fraction": (sum(value == 1 for value in vo_states) / len(vo_states) if vo_states else None),
        }
    else:
        output["tracking_gate"] = {"status": "NO_STATUS_SAMPLES"}

    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(output, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(output, indent=2, sort_keys=True))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
