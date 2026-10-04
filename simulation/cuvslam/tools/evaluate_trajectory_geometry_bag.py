#!/usr/bin/env python3
"""Compute fixed-scale aligned ATE and one-second RPE for a simulation bag."""

from __future__ import annotations

import argparse
import bisect
import math
from dataclasses import dataclass

from frame_contract_math import (
    Transform,
    compose,
    inverse,
    relative,
    rotation_angle_degrees,
    translation_norm,
)


TRUTH_TOPIC = "/ground_truth/odom"
ESTIMATE_TOPIC = "/visual_slam/tracking/odometry"


@dataclass(frozen=True)
class Sample:
    timestamp_ns: int
    pose: Transform


def percentile(values: list[float], percentage: float) -> float:
    ordered = sorted(values)
    position = (len(ordered) - 1) * percentage / 100.0
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def nearest(samples: list[Sample], stamps: list[int], target_ns: int) -> Sample:
    index = bisect.bisect_left(stamps, target_ns)
    candidates = []
    if index < len(samples):
        candidates.append(samples[index])
    if index:
        candidates.append(samples[index - 1])
    return min(candidates, key=lambda sample: abs(sample.timestamp_ns - target_ns))


def timestamp_ns(message, bag_timestamp_ns: int) -> int:
    stamp = message.header.stamp
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value if value else bag_timestamp_ns


def pose_transform(message) -> Transform:
    pose = message.pose.pose
    return Transform(
        translation=(pose.position.x, pose.position.y, pose.position.z),
        rotation=(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ),
    )


def summarize(label: str, values: list[float], unit: str) -> None:
    rmse = math.sqrt(sum(value * value for value in values) / len(values))
    print(
        f"{label}: rmse={rmse:.4f}{unit}, "
        f"median={percentile(values, 50):.4f}{unit}, "
        f"p95={percentile(values, 95):.4f}{unit}, "
        f"max={max(values):.4f}{unit}"
    )


def main() -> None:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--rpe-seconds", type=float, default=1.0)
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    message_types = {
        topic: get_message(topic_types[topic])
        for topic in (TRUTH_TOPIC, ESTIMATE_TOPIC)
    }
    values = {TRUTH_TOPIC: [], ESTIMATE_TOPIC: []}
    while reader.has_next():
        topic, data, bag_timestamp_ns = reader.read_next()
        if topic not in values:
            continue
        message = deserialize_message(data, message_types[topic])
        values[topic].append(
            Sample(
                timestamp_ns(message, bag_timestamp_ns),
                pose_transform(message),
            )
        )
    truth = sorted(values[TRUTH_TOPIC], key=lambda sample: sample.timestamp_ns)
    estimate = sorted(values[ESTIMATE_TOPIC], key=lambda sample: sample.timestamp_ns)
    if len(truth) < 2 or len(estimate) < 2:
        raise RuntimeError("truth or estimate has fewer than two samples")

    truth_stamps = [sample.timestamp_ns for sample in truth]
    common_start = max(truth[0].timestamp_ns, estimate[0].timestamp_ns)
    common_end = min(truth[-1].timestamp_ns, estimate[-1].timestamp_ns)
    selected_estimate = [
        sample
        for sample in estimate
        if common_start <= sample.timestamp_ns <= common_end
    ]
    start_estimate = selected_estimate[0]
    start_truth = nearest(
        truth,
        truth_stamps,
        start_estimate.timestamp_ns,
    )
    world_odom = compose(start_truth.pose, inverse(start_estimate.pose))

    translation_errors = []
    rotation_errors = []
    for estimate_sample in selected_estimate:
        truth_sample = nearest(
            truth,
            truth_stamps,
            estimate_sample.timestamp_ns,
        )
        aligned_estimate = compose(world_odom, estimate_sample.pose)
        error = relative(truth_sample.pose, aligned_estimate)
        translation_errors.append(translation_norm(error))
        rotation_errors.append(rotation_angle_degrees(error))

    horizon_ns = int(args.rpe_seconds * 1_000_000_000)
    rpe_translation = []
    rpe_rotation = []
    estimate_stamps = [sample.timestamp_ns for sample in selected_estimate]
    for index, first_estimate in enumerate(selected_estimate):
        target = first_estimate.timestamp_ns + horizon_ns
        second_index = bisect.bisect_left(estimate_stamps, target, lo=index + 1)
        if second_index >= len(selected_estimate):
            break
        second_estimate = selected_estimate[second_index]
        first_truth = nearest(truth, truth_stamps, first_estimate.timestamp_ns)
        second_truth = nearest(truth, truth_stamps, second_estimate.timestamp_ns)
        truth_motion = relative(first_truth.pose, second_truth.pose)
        estimate_motion = relative(first_estimate.pose, second_estimate.pose)
        residual = compose(inverse(truth_motion), estimate_motion)
        rpe_translation.append(translation_norm(residual))
        rpe_rotation.append(rotation_angle_degrees(residual))

    print("alignment: first synchronized pose, rigid SE(3), fixed scale")
    print(f"trajectory_samples: {len(selected_estimate)}")
    print(f"duration: {(common_end - common_start) / 1e9:.3f} s")
    summarize("ATE_translation", translation_errors, " m")
    summarize("ATE_rotation", rotation_errors, " deg")
    print(f"RPE_horizon: {args.rpe_seconds:.3f} s")
    summarize("RPE_translation", rpe_translation, " m")
    summarize("RPE_rotation", rpe_rotation, " deg")


if __name__ == "__main__":
    main()
