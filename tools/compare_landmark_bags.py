#!/usr/bin/env python3
"""Compare final sparse landmark clouds from two ROS 2 bags."""

from __future__ import annotations

import argparse
import math


LANDMARK_TOPIC = "/visual_slam/vis/landmarks_cloud"


def percentile(values: list[float], percentage: float) -> float:
    ordered = sorted(values)
    position = (len(ordered) - 1) * percentage / 100.0
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def point_xyz(point) -> tuple[float, float, float]:
    try:
        return float(point["x"]), float(point["y"]), float(point["z"])
    except (IndexError, KeyError, TypeError):
        return float(point[0]), float(point[1]), float(point[2])


def read_clouds(path: str):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs_py.point_cloud2 import read_points

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    message_type = get_message(topic_types[LANDMARK_TOPIC])
    clouds = []
    fields = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != LANDMARK_TOPIC:
            continue
        message = deserialize_message(data, message_type)
        points = [
            point_xyz(point)
            for point in read_points(
                message,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
        ]
        if points:
            clouds.append(points)
            fields = [field.name for field in message.fields]
    if not clouds:
        raise RuntimeError(f"{path}: no non-empty landmark clouds")
    return clouds, fields


def nearest_distances(source, target) -> list[float]:
    return [
        min(math.dist(point, candidate) for candidate in target)
        for point in source
    ]


def summarize(label: str, distances: list[float]) -> None:
    print(
        f"{label}: median={percentile(distances, 50):.4f} m, "
        f"p90={percentile(distances, 90):.4f} m, "
        f"p95={percentile(distances, 95):.4f} m, "
        f"max={max(distances):.4f} m"
    )
    for threshold in (0.001, 0.01, 0.05, 0.25):
        count = sum(distance <= threshold for distance in distances)
        print(
            f"  within_{threshold:.3f}m: {count}/{len(distances)} "
            f"({100.0 * count / len(distances):.1f}%)"
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("first_bag")
    parser.add_argument("second_bag")
    args = parser.parse_args()

    first_clouds, first_fields = read_clouds(args.first_bag)
    second_clouds, second_fields = read_clouds(args.second_bag)
    first = first_clouds[-1]
    second = second_clouds[-1]
    print(f"first_snapshots: {len(first_clouds)}")
    print(f"second_snapshots: {len(second_clouds)}")
    print(f"first_final_points: {len(first)}")
    print(f"second_final_points: {len(second)}")
    print(f"first_fields: {', '.join(first_fields)}")
    print(f"second_fields: {', '.join(second_fields)}")
    summarize("first_to_second", nearest_distances(first, second))
    summarize("second_to_first", nearest_distances(second, first))


if __name__ == "__main__":
    main()
