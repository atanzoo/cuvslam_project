#!/usr/bin/env python3
"""Compare simulation base truth with cuVSLAM in a common frame."""

from __future__ import annotations

import argparse
from dataclasses import dataclass

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from frame_contract_math import (
    Transform,
    base_to_left_optical,
    compose,
    inverse,
    relative,
    rotation_angle_degrees,
    translation_norm,
)
from gz_pose_log import read_pose_log


DEFAULT_TRUTH_TOPIC = "/ground_truth/odom"
DEFAULT_ESTIMATE_TOPIC = "/visual_slam/tracking/odometry"


@dataclass(frozen=True)
class Sample:
    timestamp_ns: int
    frame_id: str
    child_frame_id: str
    pose: Transform


def timestamp_ns(message, bag_timestamp_ns: int) -> int:
    stamp = message.header.stamp
    value = stamp.sec * 1_000_000_000 + stamp.nanosec
    return value if value else bag_timestamp_ns


def sample_from_odometry(message, bag_timestamp_ns: int) -> Sample:
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    return Sample(
        timestamp_ns=timestamp_ns(message, bag_timestamp_ns),
        frame_id=message.header.frame_id,
        child_frame_id=message.child_frame_id,
        pose=Transform(
            translation=(position.x, position.y, position.z),
            rotation=(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ),
        ),
    )


def nearest(samples: list[Sample], target_ns: int) -> Sample:
    return min(samples, key=lambda sample: abs(sample.timestamp_ns - target_ns))


def format_motion(label: str, motion: Transform) -> str:
    x, y, z = motion.translation
    return (
        f"{label}: translation=({x:+.4f}, {y:+.4f}, {z:+.4f}) m, "
        f"norm={translation_norm(motion):.4f} m, "
        f"rotation={rotation_angle_degrees(motion):.2f} deg"
    )


def read_samples(
    bag: str,
    truth_topic: str,
    estimate_topic: str,
) -> tuple[list[Sample], list[Sample]]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    missing = [
        topic
        for topic in (truth_topic, estimate_topic)
        if topic not in topic_types
    ]
    if missing:
        raise RuntimeError(f"bag is missing required topics: {', '.join(missing)}")

    message_types = {
        topic: get_message(topic_types[topic])
        for topic in (truth_topic, estimate_topic)
    }
    samples = {truth_topic: [], estimate_topic: []}
    while reader.has_next():
        topic, data, bag_timestamp_ns = reader.read_next()
        if topic not in samples:
            continue
        message = deserialize_message(data, message_types[topic])
        samples[topic].append(sample_from_odometry(message, bag_timestamp_ns))

    for topic, values in samples.items():
        if len(values) < 2:
            raise RuntimeError(f"{topic} has fewer than two pose samples")
        values.sort(key=lambda sample: sample.timestamp_ns)
    return samples[truth_topic], samples[estimate_topic]


def read_topic_samples(bag: str, topic: str) -> list[Sample]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    if topic not in topic_types:
        raise RuntimeError(f"bag is missing required topic: {topic}")
    message_type = get_message(topic_types[topic])
    samples: list[Sample] = []
    while reader.has_next():
        current_topic, data, bag_timestamp_ns = reader.read_next()
        if current_topic == topic:
            message = deserialize_message(data, message_type)
            samples.append(sample_from_odometry(message, bag_timestamp_ns))
    if len(samples) < 2:
        raise RuntimeError(f"{topic} has fewer than two pose samples")
    samples.sort(key=lambda sample: sample.timestamp_ns)
    return samples


def read_gz_truth_samples(path: str, entity: str) -> list[Sample]:
    return [
        Sample(
            timestamp_ns=value.timestamp_ns,
            frame_id="simulation_world",
            child_frame_id=entity,
            pose=Transform(value.translation, value.rotation),
        )
        for value in read_pose_log(path, entity)
    ]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--truth-topic", default=DEFAULT_TRUTH_TOPIC)
    parser.add_argument("--estimate-topic", default=DEFAULT_ESTIMATE_TOPIC)
    parser.add_argument(
        "--truth-gz-log",
        help="Gazebo dynamic_pose/info text log; replaces --truth-topic",
    )
    parser.add_argument("--truth-gz-entity", default="slam_bot")
    parser.add_argument("--camera-x", type=float, default=0.19)
    parser.add_argument("--camera-y", type=float, default=0.025)
    parser.add_argument("--camera-z", type=float, default=0.20)
    parser.add_argument(
        "--estimate-reference",
        choices=("camera", "base"),
        default="camera",
    )
    args = parser.parse_args()

    if args.truth_gz_log:
        truth = read_gz_truth_samples(
            args.truth_gz_log,
            args.truth_gz_entity,
        )
        estimate = read_topic_samples(args.bag, args.estimate_topic)
    else:
        truth, estimate = read_samples(
            args.bag,
            args.truth_topic,
            args.estimate_topic,
        )
    common_start_ns = max(truth[0].timestamp_ns, estimate[0].timestamp_ns)
    common_end_ns = min(truth[-1].timestamp_ns, estimate[-1].timestamp_ns)
    if common_end_ns <= common_start_ns:
        raise RuntimeError("truth and estimate topics have no overlapping time")

    truth_start = nearest(truth, common_start_ns)
    truth_end = nearest(truth, common_end_ns)
    estimate_start = nearest(estimate, common_start_ns)
    estimate_end = nearest(estimate, common_end_ns)
    base_left_optical = base_to_left_optical(
        args.camera_x,
        args.camera_y,
        args.camera_z,
    )

    truth_base_motion = relative(truth_start.pose, truth_end.pose)
    truth_camera_start = compose(truth_start.pose, base_left_optical)
    truth_camera_end = compose(truth_end.pose, base_left_optical)
    truth_camera_motion = relative(truth_camera_start, truth_camera_end)
    if args.estimate_reference == "camera":
        estimate_camera_start = estimate_start.pose
        estimate_camera_end = estimate_end.pose
        estimate_base_start = compose(
            estimate_camera_start,
            inverse(base_left_optical),
        )
        estimate_base_end = compose(
            estimate_camera_end,
            inverse(base_left_optical),
        )
    else:
        estimate_base_start = estimate_start.pose
        estimate_base_end = estimate_end.pose
        estimate_camera_start = compose(
            estimate_base_start,
            base_left_optical,
        )
        estimate_camera_end = compose(
            estimate_base_end,
            base_left_optical,
        )
    estimate_camera_motion = relative(
        estimate_camera_start,
        estimate_camera_end,
    )
    estimate_base_motion = relative(estimate_base_start, estimate_base_end)

    camera_residual = compose(
        inverse(truth_camera_motion),
        estimate_camera_motion,
    )
    base_residual = compose(inverse(truth_base_motion), estimate_base_motion)
    duration_s = (common_end_ns - common_start_ns) / 1_000_000_000
    start_skew_ms = (
        estimate_start.timestamp_ns - truth_start.timestamp_ns
    ) / 1_000_000
    end_skew_ms = (
        estimate_end.timestamp_ns - truth_end.timestamp_ns
    ) / 1_000_000

    print(f"bag: {args.bag}")
    print(f"overlap: {duration_s:.3f} s")
    print(
        "truth frames: "
        f"{truth_start.frame_id} -> {truth_start.child_frame_id}"
    )
    print(
        "estimate frames: "
        f"{estimate_start.frame_id} -> {estimate_start.child_frame_id}"
    )
    print(f"estimate reference: {args.estimate_reference}")
    print(
        f"endpoint skew: start={start_skew_ms:+.3f} ms, "
        f"end={end_skew_ms:+.3f} ms"
    )
    print(
        "base -> left optical extrinsic: "
        f"translation=({args.camera_x:.3f}, {args.camera_y:.3f}, "
        f"{args.camera_z:.3f}) m"
    )
    print(format_motion("ground-truth base motion", truth_base_motion))
    print(format_motion("predicted left-optical motion", truth_camera_motion))
    print(format_motion("cuVSLAM left-optical motion", estimate_camera_motion))
    print(format_motion("cuVSLAM inferred base motion", estimate_base_motion))
    print(format_motion("left-optical residual", camera_residual))
    print(format_motion("base residual", base_residual))
    print(
        "note: vector residual components assume the estimate obeys the "
        "declared optical-frame axes; compare invariant translation norms and "
        "rotation magnitudes first when auditing an axis-contract failure"
    )


if __name__ == "__main__":
    main()
