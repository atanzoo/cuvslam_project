#!/usr/bin/env python3
"""Evaluate an in-place cuVSLAM turn against Gazebo native model pose."""

from __future__ import annotations

import argparse
from collections import Counter

from evaluate_corner_bag import (
    Sample,
    nearest,
    wrapped_degrees,
    yaw_degrees,
)
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


def main() -> None:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("native_pose_log")
    parser.add_argument("--native-entity", default="slam_bot")
    parser.add_argument("--max-angle-error-deg", type=float, default=5.0)
    parser.add_argument("--max-false-translation-m", type=float, default=0.05)
    args = parser.parse_args()

    truth = [
        Sample(
            value.timestamp_ns,
            Transform(value.translation, value.rotation),
        )
        for value in read_pose_log(
            args.native_pose_log,
            args.native_entity,
        )
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
        topic: get_message(topic_types[topic])
        for topic in required
    }
    estimate = []
    statuses = Counter()
    while reader.has_next():
        topic, data, bag_timestamp_ns = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(data, message_types[topic])
        if topic == STATUS_TOPIC:
            statuses[message.vo_state] += 1
        else:
            estimate.append(
                Sample(
                    timestamp_ns(message, bag_timestamp_ns),
                    pose_transform(message),
                )
            )

    truth.sort(key=lambda sample: sample.timestamp_ns)
    estimate.sort(key=lambda sample: sample.timestamp_ns)
    if len(truth) < 3 or len(estimate) < 3:
        raise RuntimeError("insufficient native truth or cuVSLAM samples")
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
        raise RuntimeError("native truth and cuVSLAM have insufficient overlap")

    start_yaw = yaw_degrees(truth[0].pose)
    relative_yaws = [
        wrapped_degrees(yaw_degrees(sample.pose) - start_yaw)
        for sample in truth
    ]
    turn_start_index, turn_end_index = measured_turn_boundary_indices(
        relative_yaws,
    )
    truth_start = truth[turn_start_index]
    truth_end = truth[turn_end_index]
    estimate_stamps = [sample.timestamp_ns for sample in estimate]
    estimate_start = nearest(
        estimate,
        estimate_stamps,
        truth_start.timestamp_ns,
    )
    estimate_end = nearest(
        estimate,
        estimate_stamps,
        truth_end.timestamp_ns,
    )
    static_truth_start = truth[0]
    static_estimate_start = nearest(
        estimate,
        estimate_stamps,
        static_truth_start.timestamp_ns,
    )
    static_truth_motion = relative(static_truth_start.pose, truth_start.pose)
    static_estimate_motion = relative(
        static_estimate_start.pose,
        estimate_start.pose,
    )

    truth_motion = relative(truth_start.pose, truth_end.pose)
    estimate_motion = relative(estimate_start.pose, estimate_end.pose)
    residual = compose(inverse(truth_motion), estimate_motion)
    truth_yaw = wrapped_degrees(
        yaw_degrees(truth_end.pose) - yaw_degrees(truth_start.pose)
    )
    estimate_yaw = wrapped_degrees(
        yaw_degrees(estimate_end.pose) - yaw_degrees(estimate_start.pose)
    )
    yaw_error = wrapped_degrees(estimate_yaw - truth_yaw)
    false_translation = translation_norm(estimate_motion)
    lost_count = sum(
        count for state, count in statuses.items()
        if state != 1
    )
    passed = (
        abs(yaw_error) <= args.max_angle_error_deg
        and false_translation <= args.max_false_translation_m
        and lost_count == 0
    )

    print(f"truth_turn_yaw: {truth_yaw:.2f} deg")
    print(
        "pre_turn_static: "
        f"truth_translation={translation_norm(static_truth_motion):.4f}m, "
        f"truth_rotation={rotation_angle_degrees(static_truth_motion):.2f}deg, "
        f"estimate_translation={translation_norm(static_estimate_motion):.4f}m, "
        f"estimate_rotation={rotation_angle_degrees(static_estimate_motion):.2f}deg"
    )
    print(f"truth_turn_rotation_3d: {rotation_angle_degrees(truth_motion):.2f} deg")
    print(f"truth_turn_translation: {translation_norm(truth_motion):.4f} m")
    print(
        "truth_translation_xyz: "
        f"({truth_motion.translation[0]:+.4f}, "
        f"{truth_motion.translation[1]:+.4f}, "
        f"{truth_motion.translation[2]:+.4f}) m"
    )
    print(f"estimate_turn_yaw: {estimate_yaw:.2f} deg")
    print(
        "estimate_turn_rotation_3d: "
        f"{rotation_angle_degrees(estimate_motion):.2f} deg"
    )
    print(f"estimate_false_translation: {false_translation:.4f} m")
    print(
        "estimate_translation_xyz: "
        f"({estimate_motion.translation[0]:+.4f}, "
        f"{estimate_motion.translation[1]:+.4f}, "
        f"{estimate_motion.translation[2]:+.4f}) m"
    )
    print(f"yaw_error: {yaw_error:+.2f} deg")
    print(f"se3_translation_residual: {translation_norm(residual):.4f} m")
    print(f"se3_rotation_residual: {rotation_angle_degrees(residual):.2f} deg")
    print(
        "maximum_sync_skew: "
        f"{max(abs(truth_start.timestamp_ns - estimate_start.timestamp_ns), abs(truth_end.timestamp_ns - estimate_end.timestamp_ns)) / 1e6:.3f} ms"
    )
    print(
        "vo_state_counts: "
        + ", ".join(
            f"{state}={count}"
            for state, count in sorted(statuses.items())
        )
    )
    print(
        "pose_gate: "
        f"{'PASS' if passed else 'FAIL'} "
        f"(abs_yaw_error<={args.max_angle_error_deg:.1f}deg, "
        f"false_translation<={args.max_false_translation_m:.3f}m, "
        "tracking_lost=0)"
    )


if __name__ == "__main__":
    main()
