#!/usr/bin/env python3
"""Report sparse cuVSLAM landmark-cloud evidence from a ROS 2 bag."""

from __future__ import annotations

import argparse
import math

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py.point_cloud2 import read_points


LANDMARK_TOPIC = "/visual_slam/vis/landmarks_cloud"


def point_xyz(point) -> tuple[float, float, float]:
    try:
        return float(point["x"]), float(point["y"]), float(point["z"])
    except (IndexError, KeyError, TypeError):
        return float(point[0]), float(point[1]), float(point[2])


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    if LANDMARK_TOPIC not in topic_types:
        raise SystemExit(f"{LANDMARK_TOPIC}: not recorded")

    message_type = get_message(topic_types[LANDMARK_TOPIC])
    message_count = 0
    nonempty_count = 0
    max_advertised_points = 0
    final_points = []
    final_frame = ""
    final_fields = []
    snapshot_summaries = []

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != LANDMARK_TOPIC:
            continue
        message = deserialize_message(data, message_type)
        message_count += 1
        advertised_points = int(message.width) * int(message.height)
        max_advertised_points = max(
            max_advertised_points,
            advertised_points,
        )
        if advertised_points == 0:
            continue

        points = []
        for point in read_points(
            message,
            field_names=("x", "y", "z"),
            skip_nans=True,
        ):
            xyz = point_xyz(point)
            if all(math.isfinite(value) for value in xyz):
                points.append(xyz)
        if points:
            nonempty_count += 1
            final_points = points
            final_frame = message.header.frame_id
            final_fields = [
                f"{field.name}:datatype={field.datatype},count={field.count}"
                for field in message.fields
            ]
            stamp = message.header.stamp
            snapshot_summaries.append(
                (
                    stamp.sec + stamp.nanosec / 1_000_000_000.0,
                    len(points),
                    tuple(
                        (min(values), max(values))
                        for values in zip(*points)
                    ),
                )
            )

    print(f"topic: {LANDMARK_TOPIC}")
    print(f"messages: {message_count}")
    print(f"nonempty_messages: {nonempty_count}")
    print(f"max_advertised_points: {max_advertised_points}")
    print(f"final_frame: {final_frame or '-'}")
    print(f"final_fields: {', '.join(final_fields) or '-'}")
    print(f"final_finite_points: {len(final_points)}")
    for index, (stamp, count, bounds) in enumerate(snapshot_summaries, start=1):
        print(
            f"snapshot_{index}: stamp={stamp:.6f}s, points={count}, "
            f"x=[{bounds[0][0]:.4f}, {bounds[0][1]:.4f}], "
            f"y=[{bounds[1][0]:.4f}, {bounds[1][1]:.4f}], "
            f"z=[{bounds[2][0]:.4f}, {bounds[2][1]:.4f}]"
        )
    if final_points:
        for axis, values in zip("xyz", zip(*final_points)):
            print(
                f"final_{axis}_bounds: "
                f"[{min(values):.4f}, {max(values):.4f}]"
            )


if __name__ == "__main__":
    main()
