#!/usr/bin/env python3
"""Align truth motion with ROS stereo images and a cuVSLAM debug dump."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from collections import Counter
from pathlib import Path
from statistics import median

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TRUTH_TOPIC = "/ground_truth/odom"
LEFT_TOPIC = "/d435i/infra1/image_rect_raw"
RIGHT_TOPIC = "/d435i/infra2/image_rect_raw"
STATUS_TOPIC = "/visual_slam/status"


def stamp_ns(message, fallback_ns: int) -> int:
    stamp = message.header.stamp
    value = stamp.sec * 1_000_000_000 + stamp.nanosec
    return value or fallback_ns


def longest_run(values: list[str]) -> int:
    if not values:
        return 0
    longest = current = 1
    for previous, value in zip(values, values[1:]):
        if value == previous:
            current += 1
            longest = max(longest, current)
        else:
            current = 1
    return longest


def summarize_images(
    label: str,
    samples: list[tuple[int, str]],
    start_ns: int,
    end_ns: int,
) -> None:
    selected = [
        (timestamp_ns, digest)
        for timestamp_ns, digest in samples
        if start_ns <= timestamp_ns <= end_ns
    ]
    hashes = [digest for _, digest in selected]
    deltas_ms = [
        (second[0] - first[0]) / 1_000_000
        for first, second in zip(selected, selected[1:])
    ]
    print(f"{label}_frames_in_motion: {len(selected)}")
    print(f"{label}_unique_hashes_in_motion: {len(set(hashes))}")
    print(f"{label}_longest_identical_run_in_motion: {longest_run(hashes)}")
    if deltas_ms:
        print(f"{label}_median_timestamp_delta_ms: {median(deltas_ms):.3f}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("debug_dump")
    parser.add_argument("--motion-threshold-m", type=float, default=0.001)
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    required = (TRUTH_TOPIC, LEFT_TOPIC, RIGHT_TOPIC, STATUS_TOPIC)
    missing = [topic for topic in required if topic not in topic_types]
    if missing:
        raise RuntimeError(f"missing topics: {', '.join(missing)}")
    types = {topic: get_message(topic_types[topic]) for topic in required}

    truth: list[tuple[int, tuple[float, float, float]]] = []
    images = {LEFT_TOPIC: [], RIGHT_TOPIC: []}
    status = Counter()
    while reader.has_next():
        topic, data, fallback_ns = reader.read_next()
        if topic == TRUTH_TOPIC:
            message = deserialize_message(data, types[topic])
            position = message.pose.pose.position
            truth.append(
                (
                    stamp_ns(message, fallback_ns),
                    (position.x, position.y, position.z),
                )
            )
        elif topic in images:
            message = deserialize_message(data, types[topic])
            images[topic].append(
                (
                    stamp_ns(message, fallback_ns),
                    hashlib.sha256(bytes(message.data)).hexdigest(),
                )
            )
        elif topic == STATUS_TOPIC:
            message = deserialize_message(data, types[topic])
            status[message.vo_state] += 1

    truth.sort()
    origin = truth[0][1]
    moved = [
        sample
        for sample in truth
        if math.dist(origin, sample[1]) > args.motion_threshold_m
    ]
    if not moved:
        raise RuntimeError("truth does not exceed the motion threshold")
    start_ns = moved[0][0]
    final_position = moved[-1][1]
    settling = [
        sample
        for sample in truth
        if math.dist(sample[1], final_position) <= args.motion_threshold_m
    ]
    end_candidates = [sample for sample in settling if sample[0] >= start_ns]
    end_ns = end_candidates[0][0] if end_candidates else moved[-1][0]

    print(f"truth_motion_start_ns: {start_ns}")
    print(f"truth_motion_end_ns: {end_ns}")
    print(f"truth_motion_duration_s: {(end_ns - start_ns) / 1e9:.3f}")
    print(f"truth_displacement_m: {math.dist(origin, final_position):.4f}")
    print(f"vo_state_counts: {dict(sorted(status.items()))}")

    summarize_images("ros_left", images[LEFT_TOPIC], start_ns, end_ns)
    summarize_images("ros_right", images[RIGHT_TOPIC], start_ns, end_ns)

    left_stamps = {timestamp for timestamp, _ in images[LEFT_TOPIC]}
    right_stamps = {timestamp for timestamp, _ in images[RIGHT_TOPIC]}
    motion_left = {
        value for value in left_stamps if start_ns <= value <= end_ns
    }
    motion_right = {
        value for value in right_stamps if start_ns <= value <= end_ns
    }
    print(
        "ros_exact_stereo_timestamp_matches_in_motion: "
        f"{len(motion_left & motion_right)}"
    )

    debug_dir = Path(args.debug_dump)
    metadata_path = debug_dir / "frame_metadata.jsonl"
    debug_left: list[tuple[int, str]] = []
    debug_right: list[tuple[int, str]] = []
    for line in metadata_path.read_text().splitlines():
        if not line.strip():
            continue
        item = json.loads(line)
        left_path = debug_dir / item["left_img_filename"]
        right_path = debug_dir / item["right_img_filename"]
        debug_left.append(
            (
                int(item["left_timestamp"]),
                hashlib.sha256(left_path.read_bytes()).hexdigest(),
            )
        )
        debug_right.append(
            (
                int(item["right_timestamp"]),
                hashlib.sha256(right_path.read_bytes()).hexdigest(),
            )
        )

    summarize_images("debug_left", debug_left, start_ns, end_ns)
    summarize_images("debug_right", debug_right, start_ns, end_ns)


if __name__ == "__main__":
    main()
