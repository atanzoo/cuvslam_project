#!/usr/bin/env python3
"""Collect a bounded stationary real-D435i cuVSLAM fusion smoke test.

Run this inside the same ROS 2 container as the camera and Visual SLAM node.
That avoids relying on cross-container DDS data delivery for the evidence.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from collections import Counter

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Image, Imu


def stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def percentile(values: list[float], percent: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    index = (len(ordered) - 1) * percent / 100.0
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    fraction = index - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def timing(stamps: list[int]) -> dict[str, float | int]:
    intervals = [
        (second - first) / 1e6 for first, second in zip(stamps, stamps[1:])
    ]
    return {
        "messages": len(stamps),
        "rate_hz": (
            (len(stamps) - 1) / ((stamps[-1] - stamps[0]) / 1e9)
            if len(stamps) > 1 and stamps[-1] > stamps[0]
            else 0.0
        ),
        "period_median_ms": percentile(intervals, 50.0),
        "period_p95_ms": percentile(intervals, 95.0),
        "period_max_ms": max(intervals, default=float("nan")),
        "non_monotonic": sum(value <= 0.0 for value in intervals),
    }


def quaternion_angle(first: tuple[float, float, float, float],
                    current: tuple[float, float, float, float]) -> float:
    dot = abs(sum(a * b for a, b in zip(first, current)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def primitive_status_fields(message) -> dict[str, object]:
    result = {}
    for name, type_name in message.get_fields_and_field_types().items():
        if type_name in {
            "boolean",
            "byte",
            "char",
            "float32",
            "float64",
            "int8",
            "uint8",
            "int16",
            "uint16",
            "int32",
            "uint32",
            "int64",
            "uint64",
            "string",
        }:
            value = getattr(message, name)
            if isinstance(value, (str, int, float, bool)):
                result[name] = value
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=45.0)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("real_d435i_r1p5_smoke_collector")
    sensor_qos = QoSProfile(
        depth=20,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    reliable_qos = QoSProfile(
        depth=20,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    stamps = Counter()
    pose_samples = []
    status_samples = []

    def count(topic):
        def callback(message):
            stamps[topic] += 1

        return callback

    def pose_callback(message):
        stamps["/visual_slam/tracking/vo_pose"] += 1
        pose = message.pose
        pose_samples.append(
            {
                "stamp_ns": stamp_ns(message),
                "position": [float(pose.position.x), float(pose.position.y), float(pose.position.z)],
                "quaternion_xyzw": [
                    float(pose.orientation.x),
                    float(pose.orientation.y),
                    float(pose.orientation.z),
                    float(pose.orientation.w),
                ],
            }
        )

    def status_callback(message):
        stamps["/visual_slam/status"] += 1
        status_samples.append(primitive_status_fields(message))

    imu_stamps = []

    def imu_callback(message):
        stamps["/camera/imu"] += 1
        imu_stamps.append(stamp_ns(message))

    node.create_subscription(Image, "/camera/infra1/image_rect_raw", count("/camera/infra1/image_rect_raw"), sensor_qos)
    node.create_subscription(Image, "/camera/infra2/image_rect_raw", count("/camera/infra2/image_rect_raw"), sensor_qos)
    node.create_subscription(Imu, "/camera/imu", imu_callback, reliable_qos)
    node.create_subscription(PoseStamped, "/visual_slam/tracking/vo_pose", pose_callback, reliable_qos)
    node.create_subscription(
        get_message("isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus"),
        "/visual_slam/status",
        status_callback,
        reliable_qos,
    )

    start = time.monotonic()
    while time.monotonic() - start < args.duration:
        rclpy.spin_once(node, timeout_sec=0.2)

    output = {
        "duration_s": time.monotonic() - start,
        "counts": dict(stamps),
        "imu_timing": timing(sorted(imu_stamps)),
        "status_samples": status_samples,
        "pose_samples": pose_samples,
    }
    if pose_samples:
        first = pose_samples[0]
        first_position = first["position"]
        first_quaternion = tuple(first["quaternion_xyzw"])
        displacements = [
            math.sqrt(
                sum((sample["position"][axis] - first_position[axis]) ** 2 for axis in range(3))
            )
            for sample in pose_samples
        ]
        angles = [
            quaternion_angle(first_quaternion, tuple(sample["quaternion_xyzw"]))
            for sample in pose_samples
        ]
        output["pose_stability"] = {
            "first": first,
            "last": pose_samples[-1],
            "max_position_displacement_m": max(displacements),
            "p95_position_displacement_m": percentile(displacements, 95.0),
            "max_orientation_change_rad": max(angles),
            "p95_orientation_change_rad": percentile(angles, 95.0),
        }
    else:
        output["pose_stability"] = {"status": "NO_POSE_SAMPLES"}

    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(output, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(output, indent=2, sort_keys=True))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
