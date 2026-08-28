#!/usr/bin/env python3
"""Evaluate an in-place turn using native pose recorded in the bag."""

from __future__ import annotations

import argparse
import bisect
from collections import Counter
from dataclasses import dataclass
import math

from evaluate_corner_bag import phase_boundary_indices, wrapped_degrees, yaw_degrees
from frame_contract_math import (
    Transform,
    compose,
    inverse,
    relative,
    rotation_angle_degrees,
    translation_norm,
)


TRUTH_TOPIC = "/simulation/native_pose"
ODOM_TOPIC = "/visual_slam/tracking/odometry"
VO_TOPIC = "/visual_slam/tracking/vo_pose"
STATUS_TOPIC = "/visual_slam/status"


@dataclass(frozen=True)
class Sample:
    timestamp_ns: int
    pose: Transform


def pose_from_native(message) -> Transform:
    pose = message.pose
    return Transform(
        (pose.position.x, pose.position.y, pose.position.z),
        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
    )


def pose_from_odom(message) -> Transform:
    pose = message.pose.pose
    return Transform(
        (pose.position.x, pose.position.y, pose.position.z),
        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
    )


def nearest(samples: list[Sample], stamps: list[int], target: int) -> Sample:
    index = bisect.bisect_left(stamps, target)
    candidates = []
    if index < len(samples):
        candidates.append(samples[index])
    if index:
        candidates.append(samples[index - 1])
    return min(candidates, key=lambda sample: abs(sample.timestamp_ns - target))


def rpy_degrees(transform: Transform) -> tuple[float, float, float]:
    x, y, z, w = transform.rotation
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


def main() -> None:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--max-yaw-error-deg", type=float, default=5.0)
    parser.add_argument("--max-false-translation-m", type=float, default=0.05)
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    required = (TRUTH_TOPIC, ODOM_TOPIC, VO_TOPIC, STATUS_TOPIC)
    missing = [topic for topic in required if topic not in topic_types]
    if missing:
        raise RuntimeError(f"bag is missing: {', '.join(missing)}")
    message_types = {topic: get_message(topic_types[topic]) for topic in required}
    truth: list[Sample] = []
    estimates: dict[str, list[Sample]] = {ODOM_TOPIC: [], VO_TOPIC: []}
    statuses = Counter()

    while reader.has_next():
        topic, data, bag_timestamp_ns = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(data, message_types[topic])
        if topic == TRUTH_TOPIC:
            truth.append(Sample(bag_timestamp_ns, pose_from_native(message)))
        elif topic == STATUS_TOPIC:
            statuses[message.vo_state] += 1
        else:
            pose = (
                pose_from_native(message)
                if topic == VO_TOPIC
                else pose_from_odom(message)
            )
            estimates[topic].append(Sample(bag_timestamp_ns, pose))

    truth.sort(key=lambda sample: sample.timestamp_ns)
    for samples in estimates.values():
        samples.sort(key=lambda sample: sample.timestamp_ns)
    common_start = max(truth[0].timestamp_ns, *(samples[0].timestamp_ns for samples in estimates.values()))
    common_end = min(truth[-1].timestamp_ns, *(samples[-1].timestamp_ns for samples in estimates.values()))
    truth = [sample for sample in truth if common_start <= sample.timestamp_ns <= common_end]
    for topic, samples in estimates.items():
        estimates[topic] = [sample for sample in samples if common_start <= sample.timestamp_ns <= common_end]
    if len(truth) < 3 or any(len(samples) < 3 for samples in estimates.values()):
        raise RuntimeError("native pose and cuVSLAM pose have insufficient overlap")

    truth_stamps = [sample.timestamp_ns for sample in truth]
    relative_yaws = [
        wrapped_degrees(yaw_degrees(sample.pose) - yaw_degrees(truth[0].pose))
        for sample in truth
    ]
    try:
        turn_start, turn_end = phase_boundary_indices(relative_yaws)
    except StopIteration:
        peak = max(abs(value) for value in relative_yaws)
        if peak < 1.0:
            raise RuntimeError(f"native pose never turned (peak={peak:.3f}deg)")
        turn_start, turn_end = phase_boundary_indices(
            relative_yaws,
            turn_end_threshold=max(1.0, peak - 0.2),
        )
    truth_start = truth[turn_start]
    truth_end = truth[turn_end]
    truth_motion = relative(truth_start.pose, truth_end.pose)
    truth_yaw = wrapped_degrees(yaw_degrees(truth_end.pose) - yaw_degrees(truth_start.pose))

    print(f"truth_samples: {len(truth)}")
    print(f"truth_turn_yaw: {truth_yaw:+.2f}deg")
    print(f"truth_turn_translation: {translation_norm(truth_motion):.4f}m")
    print(f"vo_state_counts: " + ", ".join(f"{key}={value}" for key, value in sorted(statuses.items())))

    for topic, samples in estimates.items():
        stamps = [sample.timestamp_ns for sample in samples]
        estimate_start = nearest(samples, stamps, truth_start.timestamp_ns)
        estimate_end = nearest(samples, stamps, truth_end.timestamp_ns)
        motion = relative(estimate_start.pose, estimate_end.pose)
        residual = compose(inverse(truth_motion), motion)
        estimate_yaw = wrapped_degrees(
            yaw_degrees(estimate_end.pose) - yaw_degrees(estimate_start.pose)
        )
        yaw_error = wrapped_degrees(estimate_yaw - truth_yaw)
        rpy = [rpy_degrees(sample.pose) for sample in samples]
        max_skew = max(
            abs(truth_start.timestamp_ns - estimate_start.timestamp_ns),
            abs(truth_end.timestamp_ns - estimate_end.timestamp_ns),
        ) / 1_000_000.0
        passed = (
            abs(yaw_error) <= args.max_yaw_error_deg
            and translation_norm(motion) <= args.max_false_translation_m
            and all(state == 1 for state in statuses for _ in range(1))
        )
        print(f"{topic}: samples={len(samples)}")
        print(
            f"  estimate_turn_yaw={estimate_yaw:+.2f}deg, "
            f"yaw_error={yaw_error:+.2f}deg, "
            f"false_translation={translation_norm(motion):.4f}m, "
            f"se3_translation_residual={translation_norm(residual):.4f}m, "
            f"se3_rotation_residual={rotation_angle_degrees(residual):.2f}deg"
        )
        print(
            f"  max_abs_roll={max(abs(value[0]) for value in rpy):.3f}deg, "
            f"max_abs_pitch={max(abs(value[1]) for value in rpy):.3f}deg, "
            f"sync_skew={max_skew:.3f}ms, gate={'PASS' if passed else 'FAIL'}"
        )


if __name__ == "__main__":
    main()
