#!/usr/bin/env python3
"""Summarize a cuVSLAM odometry/status ROS 2 bag."""

from __future__ import annotations

import argparse
import math
from collections import Counter

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


DEFAULT_ODOMETRY_TOPIC = "/visual_slam/tracking/odometry"
DEFAULT_STATUS_TOPIC = "/visual_slam/status"


def quaternion_rotation_degrees(first, last) -> float:
    dot = abs(sum(a * b for a, b in zip(first, last)))
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--odometry-topic", default=DEFAULT_ODOMETRY_TOPIC)
    parser.add_argument("--status-topic", default=DEFAULT_STATUS_TOPIC)
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    if args.odometry_topic not in topic_types:
        raise RuntimeError(f"bag is missing {args.odometry_topic}")

    odometry_type = get_message(topic_types[args.odometry_topic])
    status_type = (
        get_message(topic_types[args.status_topic])
        if args.status_topic in topic_types
        else None
    )
    positions = []
    orientations = []
    timestamps_ns = []
    frames = Counter()
    status_counts = Counter()

    while reader.has_next():
        topic, data, bag_timestamp_ns = reader.read_next()
        if topic == args.odometry_topic:
            message = deserialize_message(data, odometry_type)
            position = message.pose.pose.position
            orientation = message.pose.pose.orientation
            positions.append((position.x, position.y, position.z))
            orientations.append(
                (orientation.x, orientation.y, orientation.z, orientation.w)
            )
            stamp = message.header.stamp
            timestamps_ns.append(
                stamp.sec * 1_000_000_000 + stamp.nanosec or bag_timestamp_ns
            )
            frames[(message.header.frame_id, message.child_frame_id)] += 1
        elif topic == args.status_topic and status_type is not None:
            message = deserialize_message(data, status_type)
            status_counts[message.vo_state] += 1

    if len(positions) < 2:
        raise RuntimeError("bag has fewer than two odometry samples")

    start = positions[0]
    end = positions[-1]
    displacement = math.dist(start, end)
    path_length = sum(
        math.dist(first, second)
        for first, second in zip(positions, positions[1:])
    )
    duration_s = (timestamps_ns[-1] - timestamps_ns[0]) / 1_000_000_000
    rotation_degrees = quaternion_rotation_degrees(
        orientations[0],
        orientations[-1],
    )

    print(f"bag: {args.bag}")
    print(f"samples: {len(positions)}")
    print(f"duration: {duration_s:.3f} s")
    print(f"frames: {dict(frames)}")
    print(f"start: ({start[0]:+.4f}, {start[1]:+.4f}, {start[2]:+.4f}) m")
    print(f"end: ({end[0]:+.4f}, {end[1]:+.4f}, {end[2]:+.4f}) m")
    print(f"displacement_3d: {displacement:.4f} m")
    print(f"path_length_3d: {path_length:.4f} m")
    print(f"rotation: {rotation_degrees:.2f} deg")
    if status_type is not None:
        print(f"vo_state_counts: {dict(sorted(status_counts.items()))}")


if __name__ == "__main__":
    main()
