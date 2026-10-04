#!/usr/bin/env python3
"""Compare visual-odometry and SLAM-path yaw across a corner bag."""

from __future__ import annotations

import argparse
import bisect
from dataclasses import dataclass

from evaluate_corner_bag import phase_boundary_indices, wrapped_degrees, yaw_degrees
from evaluate_native_topic_corner_bag import odometry_transform, pose_stamped_transform
from frame_contract_math import Transform, relative, rotation_angle_degrees, translation_norm


TRUTH_TOPIC = "/simulation/native_pose"
VO_TOPIC = "/visual_slam/tracking/vo_pose"
SLAM_PATH_TOPIC = "/visual_slam/tracking/slam_path"


@dataclass(frozen=True)
class Sample:
    timestamp_ns: int
    pose: Transform


def nearest(samples: list[Sample], target: int) -> Sample:
    stamps = [item.timestamp_ns for item in samples]
    index = bisect.bisect_left(stamps, target)
    candidates = []
    if index < len(samples):
        candidates.append(samples[index])
    if index:
        candidates.append(samples[index - 1])
    return min(candidates, key=lambda item: abs(item.timestamp_ns - target))


def print_phase(name: str, truth_start: Sample, truth_end: Sample, estimate: list[Sample]) -> None:
    start = nearest(estimate, truth_start.timestamp_ns)
    end = nearest(estimate, truth_end.timestamp_ns)
    truth_motion = relative(truth_start.pose, truth_end.pose)
    estimate_motion = relative(start.pose, end.pose)
    print(
        f"{name}: samples={len(estimate)}, "
        f"truth_yaw={wrapped_degrees(yaw_degrees(truth_motion)):+.2f}deg, "
        f"estimate_yaw={wrapped_degrees(yaw_degrees(estimate_motion)):+.2f}deg, "
        f"yaw_error={wrapped_degrees(yaw_degrees(estimate_motion) - yaw_degrees(truth_motion)):+.2f}deg, "
        f"translation_residual={translation_norm(relative(truth_motion, estimate_motion)):.4f}m, "
        f"rotation_residual={rotation_angle_degrees(relative(truth_motion, estimate_motion)):.2f}deg"
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
    required = (TRUTH_TOPIC, VO_TOPIC, SLAM_PATH_TOPIC)
    missing = [topic for topic in required if topic not in topic_types]
    if missing:
        raise RuntimeError(f"bag is missing: {', '.join(missing)}")
    message_types = {topic: get_message(topic_types[topic]) for topic in required}

    truth: list[Sample] = []
    vo: list[Sample] = []
    slam: list[Sample] = []
    path_lengths: list[int] = []
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
            path_lengths.append(len(message.poses))

    for samples in (truth, vo, slam):
        samples.sort(key=lambda item: item.timestamp_ns)
    if len(truth) < 3 or len(vo) < 3 or len(slam) < 3:
        raise RuntimeError("truth, VO, or SLAM path is incomplete")

    relative_yaws = [
        wrapped_degrees(yaw_degrees(item.pose) - yaw_degrees(truth[0].pose))
        for item in truth
    ]
    turn_start, turn_end = phase_boundary_indices(relative_yaws)
    bounds = (truth[0], truth[turn_start], truth[turn_end], truth[-1])
    print(f"vo_samples={len(vo)} slam_path_samples={len(slam)}")
    print(f"slam_path_length={path_lengths[0]}->{path_lengths[-1]}")
    for name, start, end in (
        ("first_leg", bounds[0], bounds[1]),
        ("turn", bounds[1], bounds[2]),
        ("second_leg", bounds[2], bounds[3]),
        ("complete_route", bounds[0], bounds[3]),
    ):
        print_phase(f"vo_{name}", start, end, vo)
        print_phase(f"slam_{name}", start, end, slam)


if __name__ == "__main__":
    main()
