#!/usr/bin/env python3
"""Evaluate truth and cuVSLAM motion over a straight-turn-straight route."""

from __future__ import annotations

import argparse
import bisect
from collections import Counter
from dataclasses import dataclass
import math

from frame_contract_math import (
    Transform,
    compose,
    inverse,
    relative,
    rotation_angle_degrees,
    translation_norm,
)
from gz_pose_log import read_pose_log


TRUTH_TOPIC = "/ground_truth/odom"
ESTIMATE_TOPIC = "/visual_slam/tracking/odometry"
STATUS_TOPIC = "/visual_slam/status"


@dataclass(frozen=True)
class Sample:
    timestamp_ns: int
    pose: Transform


def wrapped_degrees(value: float) -> float:
    return math.degrees(
        math.atan2(
            math.sin(math.radians(value)),
            math.cos(math.radians(value)),
        )
    )


def yaw_degrees(transform: Transform) -> float:
    x, y, z, w = transform.rotation
    return math.degrees(
        math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )
    )


def phase_boundary_indices(
    relative_yaws: list[float],
    turn_start_threshold: float = 1.0,
    turn_end_threshold: float = 89.0,
) -> tuple[int, int]:
    turn_start = next(
        index
        for index, yaw in enumerate(relative_yaws)
        if abs(yaw) >= turn_start_threshold
    )
    turn_end = next(
        index
        for index in range(turn_start, len(relative_yaws))
        if abs(relative_yaws[index]) >= turn_end_threshold
    )
    return max(0, turn_start - 1), turn_end


def timestamp_ns(message, bag_timestamp_ns: int) -> int:
    stamp = message.header.stamp
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value if value else bag_timestamp_ns


def pose_transform(message) -> Transform:
    pose = message.pose.pose
    return Transform(
        (pose.position.x, pose.position.y, pose.position.z),
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ),
    )


def nearest(samples: list[Sample], stamps: list[int], target: int) -> Sample:
    index = bisect.bisect_left(stamps, target)
    candidates = []
    if index < len(samples):
        candidates.append(samples[index])
    if index:
        candidates.append(samples[index - 1])
    return min(candidates, key=lambda sample: abs(sample.timestamp_ns - target))


def print_phase(
    name: str,
    truth_start: Sample,
    truth_end: Sample,
    estimate_start: Sample,
    estimate_end: Sample,
) -> None:
    truth_motion = relative(truth_start.pose, truth_end.pose)
    estimate_motion = relative(estimate_start.pose, estimate_end.pose)
    residual = compose(inverse(truth_motion), estimate_motion)
    print(
        f"{name}: "
        f"truth_translation={translation_norm(truth_motion):.4f}m, "
        f"truth_rotation={rotation_angle_degrees(truth_motion):.2f}deg, "
        f"estimate_translation={translation_norm(estimate_motion):.4f}m, "
        f"estimate_rotation={rotation_angle_degrees(estimate_motion):.2f}deg, "
        f"translation_error={translation_norm(residual):.4f}m, "
        f"rotation_error={rotation_angle_degrees(residual):.2f}deg"
    )


def main() -> None:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--truth-gz-log")
    parser.add_argument("--truth-gz-entity", default="slam_bot")
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    required = (
        (ESTIMATE_TOPIC, STATUS_TOPIC)
        if args.truth_gz_log else
        (TRUTH_TOPIC, ESTIMATE_TOPIC, STATUS_TOPIC)
    )
    missing = [topic for topic in required if topic not in topic_types]
    if missing:
        raise RuntimeError(f"bag is missing: {', '.join(missing)}")
    message_types = {
        topic: get_message(topic_types[topic])
        for topic in required
    }
    samples = {
        TRUTH_TOPIC: (
            [
                Sample(
                    value.timestamp_ns,
                    Transform(value.translation, value.rotation),
                )
                for value in read_pose_log(
                    args.truth_gz_log,
                    args.truth_gz_entity,
                )
            ]
            if args.truth_gz_log else []
        ),
        ESTIMATE_TOPIC: [],
    }
    statuses = Counter()
    while reader.has_next():
        topic, data, bag_timestamp_ns = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(data, message_types[topic])
        if topic == STATUS_TOPIC:
            statuses[message.vo_state] += 1
        else:
            samples[topic].append(
                Sample(
                    timestamp_ns(message, bag_timestamp_ns),
                    pose_transform(message),
                )
            )

    truth = sorted(samples[TRUTH_TOPIC], key=lambda sample: sample.timestamp_ns)
    estimate = sorted(
        samples[ESTIMATE_TOPIC],
        key=lambda sample: sample.timestamp_ns,
    )
    if len(truth) < 3 or len(estimate) < 3:
        raise RuntimeError("insufficient truth or estimate samples")
    common_start_ns = max(truth[0].timestamp_ns, estimate[0].timestamp_ns)
    common_end_ns = min(truth[-1].timestamp_ns, estimate[-1].timestamp_ns)
    truth = [
        sample for sample in truth
        if common_start_ns <= sample.timestamp_ns <= common_end_ns
    ]
    estimate = [
        sample for sample in estimate
        if common_start_ns <= sample.timestamp_ns <= common_end_ns
    ]
    if len(truth) < 3 or len(estimate) < 3:
        raise RuntimeError("truth and estimate have insufficient overlap")

    start_yaw = yaw_degrees(truth[0].pose)
    relative_yaws = [
        wrapped_degrees(yaw_degrees(sample.pose) - start_yaw)
        for sample in truth
    ]
    try:
        turn_start_index, turn_end_index = phase_boundary_indices(relative_yaws)
    except StopIteration:
        peak_yaw = max(abs(value) for value in relative_yaws)
        turn_start_index, turn_end_index = phase_boundary_indices(
            relative_yaws,
            turn_end_threshold=peak_yaw - 0.2,
        )
    boundaries = (
        truth[0],
        truth[turn_start_index],
        truth[turn_end_index],
        truth[-1],
    )
    estimate_stamps = [sample.timestamp_ns for sample in estimate]
    estimate_boundaries = tuple(
        nearest(estimate, estimate_stamps, sample.timestamp_ns)
        for sample in boundaries
    )

    print_phase(
        "first_leg",
        boundaries[0],
        boundaries[1],
        estimate_boundaries[0],
        estimate_boundaries[1],
    )
    print_phase(
        "turn",
        boundaries[1],
        boundaries[2],
        estimate_boundaries[1],
        estimate_boundaries[2],
    )
    print_phase(
        "second_leg",
        boundaries[2],
        boundaries[3],
        estimate_boundaries[2],
        estimate_boundaries[3],
    )
    print_phase(
        "total",
        boundaries[0],
        boundaries[3],
        estimate_boundaries[0],
        estimate_boundaries[3],
    )
    print(
        "boundary_truth_yaw: "
        f"start={relative_yaws[0]:.2f}deg, "
        f"turn_start={relative_yaws[turn_start_index]:.2f}deg, "
        f"turn_end={relative_yaws[turn_end_index]:.2f}deg, "
        f"final={relative_yaws[-1]:.2f}deg"
    )
    print(
        "maximum_estimate_sync_skew: "
        f"{max(abs(truth_sample.timestamp_ns - estimate_sample.timestamp_ns) for truth_sample, estimate_sample in zip(boundaries, estimate_boundaries)) / 1e6:.3f} ms"
    )
    print(
        "vo_state_counts: "
        + ", ".join(
            f"{state}={count}"
            for state, count in sorted(statuses.items())
        )
    )


if __name__ == "__main__":
    main()
