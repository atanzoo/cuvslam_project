#!/usr/bin/env python3
"""Evaluate a straight-turn-straight route against Gazebo native pose."""

from __future__ import annotations

import argparse
from collections import Counter
import math

from evaluate_corner_bag import Sample, nearest, wrapped_degrees, yaw_degrees
from evaluate_imu_camera_contract_bag import measured_turn_boundary_indices
from frame_contract_math import (
    Transform,
    compose,
    inverse,
    relative,
    rotation_angle_degrees,
    translation_norm,
)
from gz_pose_log import read_pose_log


ESTIMATE_TOPIC = "/visual_slam/tracking/odometry"
STATUS_TOPIC = "/visual_slam/status"


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


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return (
        ordered[lower] * (upper - position)
        + ordered[upper] * (position - lower)
    )


def phase_metrics(
    truth_start: Sample,
    truth_end: Sample,
    estimate_start: Sample,
    estimate_end: Sample,
) -> tuple[Transform, Transform, Transform]:
    truth_motion = relative(truth_start.pose, truth_end.pose)
    estimate_motion = relative(estimate_start.pose, estimate_end.pose)
    residual = compose(inverse(truth_motion), estimate_motion)
    return truth_motion, estimate_motion, residual


def print_phase(
    name: str,
    truth_start: Sample,
    truth_end: Sample,
    estimate_start: Sample,
    estimate_end: Sample,
) -> tuple[Transform, Transform, Transform]:
    truth_motion, estimate_motion, residual = phase_metrics(
        truth_start, truth_end, estimate_start, estimate_end
    )
    truth_yaw = wrapped_degrees(
        yaw_degrees(truth_end.pose) - yaw_degrees(truth_start.pose)
    )
    estimate_yaw = wrapped_degrees(
        yaw_degrees(estimate_end.pose) - yaw_degrees(estimate_start.pose)
    )
    print(
        f"{name}: truth_translation={translation_norm(truth_motion):.4f}m, "
        f"estimate_translation={translation_norm(estimate_motion):.4f}m, "
        f"translation_residual={translation_norm(residual):.4f}m, "
        f"truth_yaw={truth_yaw:+.2f}deg, estimate_yaw={estimate_yaw:+.2f}deg, "
        f"yaw_error={wrapped_degrees(estimate_yaw - truth_yaw):+.2f}deg"
    )
    return truth_motion, estimate_motion, residual


def main() -> None:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("native_pose_log")
    parser.add_argument("--native-entity", default="slam_bot")
    args = parser.parse_args()

    truth = [
        Sample(sample.timestamp_ns, Transform(sample.translation, sample.rotation))
        for sample in read_pose_log(args.native_pose_log, args.native_entity)
    ]
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    required = (ESTIMATE_TOPIC, STATUS_TOPIC)
    missing = [topic for topic in required if topic not in topic_types]
    if missing:
        raise RuntimeError(f"bag is missing: {', '.join(missing)}")
    message_types = {
        topic: get_message(topic_types[topic]) for topic in required
    }
    estimate: list[Sample] = []
    statuses = Counter()
    while reader.has_next():
        topic, data, bag_stamp = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(data, message_types[topic])
        if topic == STATUS_TOPIC:
            statuses[message.vo_state] += 1
        else:
            estimate.append(
                Sample(timestamp_ns(message, bag_stamp), pose_transform(message))
            )

    truth.sort(key=lambda sample: sample.timestamp_ns)
    estimate.sort(key=lambda sample: sample.timestamp_ns)
    common_start = max(truth[0].timestamp_ns, estimate[0].timestamp_ns)
    common_end = min(truth[-1].timestamp_ns, estimate[-1].timestamp_ns)
    truth = [
        sample for sample in truth
        if common_start <= sample.timestamp_ns <= common_end
    ]
    estimate = [
        sample for sample in estimate
        if common_start <= sample.timestamp_ns <= common_end
    ]
    if len(truth) < 3 or len(estimate) < 3:
        raise RuntimeError("native truth and cuVSLAM have insufficient overlap")

    relative_yaws = [
        wrapped_degrees(yaw_degrees(sample.pose) - yaw_degrees(truth[0].pose))
        for sample in truth
    ]
    turn_start_index, turn_end_index = measured_turn_boundary_indices(
        relative_yaws
    )
    truth_boundaries = (
        truth[0],
        truth[turn_start_index],
        truth[turn_end_index],
        truth[-1],
    )
    estimate_stamps = [sample.timestamp_ns for sample in estimate]
    estimate_boundaries = tuple(
        nearest(estimate, estimate_stamps, sample.timestamp_ns)
        for sample in truth_boundaries
    )

    first = print_phase(
        "first_leg",
        truth_boundaries[0],
        truth_boundaries[1],
        estimate_boundaries[0],
        estimate_boundaries[1],
    )
    turn = print_phase(
        "turn",
        truth_boundaries[1],
        truth_boundaries[2],
        estimate_boundaries[1],
        estimate_boundaries[2],
    )
    second = print_phase(
        "second_leg",
        truth_boundaries[2],
        truth_boundaries[3],
        estimate_boundaries[2],
        estimate_boundaries[3],
    )
    route = print_phase(
        "complete_route",
        truth_boundaries[0],
        truth_boundaries[3],
        estimate_boundaries[0],
        estimate_boundaries[3],
    )

    truth_stamps = [sample.timestamp_ns for sample in truth]
    truth_origin = truth[0].pose
    estimate_origin = estimate[0].pose
    trajectory_translation_errors = []
    trajectory_rotation_errors = []
    maximum_skew_ms = 0.0
    for estimated in estimate:
        native = nearest(truth, truth_stamps, estimated.timestamp_ns)
        maximum_skew_ms = max(
            maximum_skew_ms,
            abs(native.timestamp_ns - estimated.timestamp_ns) / 1_000_000.0,
        )
        native_relative = relative(truth_origin, native.pose)
        estimate_relative = relative(estimate_origin, estimated.pose)
        residual = compose(inverse(native_relative), estimate_relative)
        trajectory_translation_errors.append(translation_norm(residual))
        trajectory_rotation_errors.append(rotation_angle_degrees(residual))

    turn_truth, turn_estimate, _ = turn
    turn_yaw_error = wrapped_degrees(
        yaw_degrees(turn_estimate) - yaw_degrees(turn_truth)
    )
    lost_count = sum(
        count for state, count in statuses.items() if state != 1
    )
    passed = (
        translation_norm(first[2]) <= 0.10
        and abs(turn_yaw_error) <= 5.0
        and translation_norm(turn_estimate) <= 0.05
        and translation_norm(second[2]) <= 0.10
        and translation_norm(route[2]) <= 0.15
        and lost_count == 0
    )
    print(
        f"trajectory_translation_error: median={percentile(trajectory_translation_errors, 0.5):.4f}m, "
        f"p95={percentile(trajectory_translation_errors, 0.95):.4f}m, "
        f"max={max(trajectory_translation_errors):.4f}m"
    )
    print(
        f"trajectory_rotation_error: median={percentile(trajectory_rotation_errors, 0.5):.2f}deg, "
        f"p95={percentile(trajectory_rotation_errors, 0.95):.2f}deg, "
        f"max={max(trajectory_rotation_errors):.2f}deg"
    )
    print(f"maximum_sync_skew: {maximum_skew_ms:.3f} ms")
    print(
        "vo_state_counts: "
        + ", ".join(
            f"{state}={count}" for state, count in sorted(statuses.items())
        )
    )
    print(
        "route_gate: "
        + ("PASS" if passed else "FAIL")
        + " (leg residuals<=0.10m, turn yaw<=5deg, "
        "turn false translation<=0.05m, final residual<=0.15m, tracking_lost=0)"
    )


if __name__ == "__main__":
    main()
