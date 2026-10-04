#!/usr/bin/env python3
"""Measure closed-loop truth, VO, SLAM-path, and PoseGraph behavior."""

from __future__ import annotations

import argparse
from dataclasses import dataclass

from evaluate_native_topic_corner_bag import pose_stamped_transform
from frame_contract_math import relative, rotation_angle_degrees, translation_norm


TRUTH_TOPIC = "/simulation/native_pose"
VO_TOPIC = "/visual_slam/tracking/vo_pose"
SLAM_PATH_TOPIC = "/visual_slam/tracking/slam_path"
NODES_TOPIC = "/visual_slam/vis/pose_graph_nodes"
EDGES_TOPIC = "/visual_slam/vis/pose_graph_edges"


@dataclass(frozen=True)
class Sample:
    timestamp_ns: int
    pose: object


def yaw_degrees(transform) -> float:
    x, y, z, w = transform.rotation
    import math

    return math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def wrapped(value: float) -> float:
    import math

    return math.degrees(math.atan2(math.sin(math.radians(value)), math.cos(math.radians(value))))


def report_closure(name: str, samples: list[Sample], truth_start: Sample, truth_end: Sample) -> None:
    start = min(samples, key=lambda item: abs(item.timestamp_ns - truth_start.timestamp_ns))
    end = min(samples, key=lambda item: abs(item.timestamp_ns - truth_end.timestamp_ns))
    truth_motion = relative(truth_start.pose, truth_end.pose)
    estimate_motion = relative(start.pose, end.pose)
    print(
        f"{name}: samples={len(samples)}, "
        f"truth_translation={translation_norm(truth_motion):.4f}m, "
        f"estimate_translation={translation_norm(estimate_motion):.4f}m, "
        f"translation_error={translation_norm(relative(truth_motion, estimate_motion)):.4f}m, "
        f"truth_yaw={wrapped(yaw_degrees(truth_motion)):+.2f}deg, "
        f"estimate_yaw={wrapped(yaw_degrees(estimate_motion)):+.2f}deg, "
        f"yaw_error={wrapped(yaw_degrees(estimate_motion) - yaw_degrees(truth_motion)):+.2f}deg, "
        f"se3_rotation_error={rotation_angle_degrees(relative(truth_motion, estimate_motion)):.2f}deg"
    )


def main() -> None:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    required = (TRUTH_TOPIC, VO_TOPIC, SLAM_PATH_TOPIC, NODES_TOPIC, EDGES_TOPIC)
    missing = [topic for topic in required if topic not in topic_types]
    if missing:
        raise RuntimeError(f"bag is missing: {', '.join(missing)}")
    message_types = {topic: get_message(topic_types[topic]) for topic in required}

    truth: list[Sample] = []
    vo: list[Sample] = []
    slam: list[Sample] = []
    node_counts: list[int] = []
    edge_counts: list[int] = []
    while reader.has_next():
        topic, data, bag_timestamp_ns = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(data, message_types[topic])
        if topic == TRUTH_TOPIC:
            truth.append(Sample(bag_timestamp_ns, pose_stamped_transform(message)))
        elif topic == VO_TOPIC:
            vo.append(Sample(bag_timestamp_ns, pose_stamped_transform(message)))
        elif topic == SLAM_PATH_TOPIC and message.poses:
            slam.append(Sample(bag_timestamp_ns, pose_stamped_transform(message.poses[-1])))
        elif topic == NODES_TOPIC:
            node_counts.append(len(message.poses))
        elif topic == EDGES_TOPIC:
            edge_counts.append(len(message.points))

    for samples in (truth, vo, slam):
        samples.sort(key=lambda item: item.timestamp_ns)
    if min(len(truth), len(vo), len(slam)) < 3:
        raise RuntimeError("truth, VO, or SLAM path is incomplete")

    truth_start, truth_end = truth[0], truth[-1]
    print(
        f"truth_samples={len(truth)} vo_samples={len(vo)} "
        f"slam_path_samples={len(slam)}"
    )
    print(
        f"pose_graph_nodes={node_counts[0] if node_counts else 0}->"
        f"{node_counts[-1] if node_counts else 0} "
        f"pose_graph_edges={edge_counts[0] if edge_counts else 0}->"
        f"{edge_counts[-1] if edge_counts else 0}"
    )
    report_closure("truth", truth, truth_start, truth_end)
    report_closure("vo", vo, truth_start, truth_end)
    report_closure("slam_path", slam, truth_start, truth_end)

    comparisons = 0
    changed = 0
    for vo_sample in vo:
        nearest = min(slam, key=lambda item: abs(item.timestamp_ns - vo_sample.timestamp_ns))
        if abs(nearest.timestamp_ns - vo_sample.timestamp_ns) > 100_000_000:
            continue
        comparisons += 1
        if translation_norm(relative(vo_sample.pose, nearest.pose)) > 1e-4 or rotation_angle_degrees(relative(vo_sample.pose, nearest.pose)) > 0.01:
            changed += 1
    print(f"vo_vs_slam_path_comparisons={comparisons} changed_samples={changed}")


if __name__ == "__main__":
    main()
