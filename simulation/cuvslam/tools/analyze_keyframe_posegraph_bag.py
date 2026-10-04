#!/usr/bin/env python3
"""Audit PoseGraph/keyframe proxy updates against the native corner phases."""

from __future__ import annotations

import argparse
from dataclasses import dataclass

from evaluate_corner_bag import phase_boundary_indices, wrapped_degrees, yaw_degrees
from evaluate_native_topic_corner_bag import pose_stamped_transform


TRUTH_TOPIC = "/simulation/native_pose"
NODES_TOPIC = "/visual_slam/vis/pose_graph_nodes"
EDGES_TOPIC = "/visual_slam/vis/pose_graph_edges"


@dataclass(frozen=True)
class TruthSample:
    timestamp_ns: int
    yaw_deg: float


@dataclass(frozen=True)
class GraphUpdate:
    timestamp_ns: int
    item_count: int


def phase_label(timestamp_ns: int, bounds: tuple[int, int, int, int]) -> str:
    start, turn_start, turn_end, end = bounds
    if timestamp_ns < turn_start:
        return "first_leg"
    if timestamp_ns <= turn_end:
        return "turn"
    if timestamp_ns <= end:
        return "second_leg"
    return "outside"


def summarize(name: str, updates: list[GraphUpdate], bounds: tuple[int, int, int, int]) -> None:
    print(f"{name}_messages: {len(updates)}")
    phases = {phase: [] for phase in ("first_leg", "turn", "second_leg")}
    for update in updates:
        phase = phase_label(update.timestamp_ns, bounds)
        if phase in phases:
            phases[phase].append(update)
    for phase, phase_updates in phases.items():
        counts = [item.item_count for item in phase_updates]
        increases = sum(
            current > previous
            for previous, current in zip(counts, counts[1:])
        )
        added = sum(
            max(0, current - previous)
            for previous, current in zip(counts, counts[1:])
        )
        print(
            f"{name}_{phase}: messages={len(phase_updates)}, "
            f"first_count={counts[0] if counts else 0}, "
            f"last_count={counts[-1] if counts else 0}, "
            f"max_count={max(counts) if counts else 0}, "
            f"positive_increases={increases}, added_items={added}"
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
    required = (TRUTH_TOPIC, NODES_TOPIC, EDGES_TOPIC)
    missing = [topic for topic in required if topic not in topic_types]
    if missing:
        raise RuntimeError(f"bag is missing: {', '.join(missing)}")
    message_types = {topic: get_message(topic_types[topic]) for topic in required}

    truth: list[TruthSample] = []
    nodes: list[GraphUpdate] = []
    edges: list[GraphUpdate] = []
    while reader.has_next():
        topic, data, bag_timestamp_ns = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(data, message_types[topic])
        if topic == TRUTH_TOPIC:
            truth.append(
                TruthSample(
                    bag_timestamp_ns,
                    yaw_degrees(pose_stamped_transform(message)),
                )
            )
        elif topic == NODES_TOPIC:
            nodes.append(GraphUpdate(bag_timestamp_ns, len(message.poses)))
        elif topic == EDGES_TOPIC:
            edges.append(GraphUpdate(bag_timestamp_ns, len(message.points)))

    truth.sort(key=lambda item: item.timestamp_ns)
    nodes.sort(key=lambda item: item.timestamp_ns)
    edges.sort(key=lambda item: item.timestamp_ns)
    if len(truth) < 3:
        raise RuntimeError("native pose is incomplete")

    relative_yaws = [
        wrapped_degrees(item.yaw_deg - truth[0].yaw_deg) for item in truth
    ]
    turn_start_index, turn_end_index = phase_boundary_indices(relative_yaws)
    bounds = (
        truth[0].timestamp_ns,
        truth[turn_start_index].timestamp_ns,
        truth[turn_end_index].timestamp_ns,
        truth[-1].timestamp_ns,
    )
    print(
        "phase_bounds_ns: "
        f"first={bounds[0]} turn_start={bounds[1]} "
        f"turn_end={bounds[2]} end={bounds[3]}"
    )
    summarize("pose_graph_nodes", nodes, bounds)
    summarize("pose_graph_edges", edges, bounds)


if __name__ == "__main__":
    main()
