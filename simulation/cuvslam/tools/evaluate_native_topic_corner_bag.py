#!/usr/bin/env python3
"""Evaluate a GUI native-pose corner bag without a separate truth log."""

from __future__ import annotations

import argparse
import bisect
from collections import Counter
from dataclasses import dataclass
import math

from evaluate_corner_bag import phase_boundary_indices, wrapped_degrees, yaw_degrees
from evaluate_landmark_geometry_bag import (
    load_boxes,
    percentile,
    point_to_box_surface,
    point_xyz,
    transform_point,
)
from frame_contract_math import (
    Transform,
    compose,
    inverse,
    relative,
    rotation_angle_degrees,
    translation_norm,
)


TRUTH_TOPIC = "/simulation/native_pose"
ESTIMATE_TOPIC = "/visual_slam/tracking/odometry"
STATUS_TOPIC = "/visual_slam/status"
LANDMARK_TOPIC = "/visual_slam/vis/landmarks_cloud"
TF_TOPIC = "/tf"
OBSERVATION_TOPIC = "/visual_slam/vis/observations_cloud"


@dataclass(frozen=True)
class Sample:
    timestamp_ns: int
    pose: Transform


@dataclass(frozen=True)
class TimedTransform:
    timestamp_ns: int
    transform: Transform


def stamp(message, bag_timestamp_ns: int) -> int:
    # The GUI bag contains two valid but different clock domains: the native
    # Gazebo relay keeps the simulated epoch while cuVSLAM odometry restamps
    # from its process start.  rosbag2's recorder timestamp is the common
    # capture clock and is therefore the only safe cross-topic key here.
    return bag_timestamp_ns


def pose_stamped_transform(message) -> Transform:
    pose = message.pose
    return Transform(
        (pose.position.x, pose.position.y, pose.position.z),
        (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
    )


def odometry_transform(message) -> Transform:
    return Transform(
        (message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z),
        (
            message.pose.pose.orientation.x,
            message.pose.pose.orientation.y,
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w,
        ),
    )


def tf_transform(message) -> Transform:
    return Transform(
        (message.translation.x, message.translation.y, message.translation.z),
        (message.rotation.x, message.rotation.y, message.rotation.z, message.rotation.w),
    )


def nearest(samples: list[Sample], stamps: list[int], target: int) -> Sample:
    index = bisect.bisect_left(stamps, target)
    candidates = []
    if index < len(samples):
        candidates.append(samples[index])
    if index:
        candidates.append(samples[index - 1])
    return min(candidates, key=lambda sample: abs(sample.timestamp_ns - target))


def nearest_tf(samples: list[TimedTransform], target: int) -> TimedTransform:
    stamps = [sample.timestamp_ns for sample in samples]
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
    pitch_arg = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, pitch_arg)))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


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
    truth_yaw = wrapped_degrees(
        yaw_degrees(truth_end.pose) - yaw_degrees(truth_start.pose)
    )
    estimate_yaw = wrapped_degrees(
        yaw_degrees(estimate_end.pose) - yaw_degrees(estimate_start.pose)
    )
    print(
        f"{name}: truth_translation={translation_norm(truth_motion):.4f}m, "
        f"estimate_translation={translation_norm(estimate_motion):.4f}m, "
        f"translation_error={translation_norm(residual):.4f}m, "
        f"truth_yaw={truth_yaw:+.2f}deg, estimate_yaw={estimate_yaw:+.2f}deg, "
        f"yaw_error={wrapped_degrees(estimate_yaw - truth_yaw):+.2f}deg, "
        f"se3_rotation_error={rotation_angle_degrees(residual):.2f}deg"
    )


def main() -> None:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs_py.point_cloud2 import read_points

    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("sdf")
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    required = (
        TRUTH_TOPIC,
        ESTIMATE_TOPIC,
        STATUS_TOPIC,
        TF_TOPIC,
    )
    missing = [topic for topic in required if topic not in topic_types]
    if missing:
        raise RuntimeError(f"bag is missing: {', '.join(missing)}")
    message_types = {topic: get_message(topic_types[topic]) for topic in required}
    if LANDMARK_TOPIC in topic_types:
        message_types[LANDMARK_TOPIC] = get_message(topic_types[LANDMARK_TOPIC])

    truth: list[Sample] = []
    estimate: list[Sample] = []
    statuses = Counter()
    map_odom: list[TimedTransform] = []
    final_landmarks: list[tuple[float, float, float]] = []
    final_weights: list[float] = []
    landmark_messages = 0
    observation_messages = 0
    observation_points = 0

    while reader.has_next():
        topic, data, bag_timestamp_ns = reader.read_next()
        if topic not in message_types:
            # Count observations without deserializing their type contract.
            if topic == OBSERVATION_TOPIC:
                observation_messages += 1
            continue
        message = deserialize_message(data, message_types[topic])
        if topic == TRUTH_TOPIC:
            truth.append(Sample(stamp(message, bag_timestamp_ns), pose_stamped_transform(message)))
        elif topic == ESTIMATE_TOPIC:
            estimate.append(Sample(stamp(message, bag_timestamp_ns), odometry_transform(message)))
        elif topic == STATUS_TOPIC:
            statuses[message.vo_state] += 1
        elif topic == TF_TOPIC:
            for item in message.transforms:
                if item.header.frame_id == "map" and item.child_frame_id == "odom":
                    map_odom.append(
                        TimedTransform(stamp(item, bag_timestamp_ns), tf_transform(item.transform))
                    )
        elif topic == LANDMARK_TOPIC:
            fields = {field.name for field in message.fields}
            field_names = ("x", "y", "z", "weight") if "weight" in fields else ("x", "y", "z")
            raw = list(read_points(message, field_names=field_names, skip_nans=True))
            if raw:
                final_landmarks = [point_xyz(point) for point in raw]
                final_weights = [float(point[3]) for point in raw] if "weight" in fields else []
                landmark_messages += 1

    for samples in (truth, estimate, map_odom):
        samples.sort(key=lambda sample: sample.timestamp_ns)
    if len(truth) < 3 or len(estimate) < 3 or not map_odom:
        raise RuntimeError("native pose, cuVSLAM, or TF are incomplete")

    print(
        "raw_time_ranges_ns: "
        f"truth=[{truth[0].timestamp_ns},{truth[-1].timestamp_ns}], "
        f"estimate=[{estimate[0].timestamp_ns},{estimate[-1].timestamp_ns}], "
        f"map_odom=[{map_odom[0].timestamp_ns},{map_odom[-1].timestamp_ns}]"
    )

    common_start = max(truth[0].timestamp_ns, estimate[0].timestamp_ns)
    common_end = min(truth[-1].timestamp_ns, estimate[-1].timestamp_ns)
    truth = [sample for sample in truth if common_start <= sample.timestamp_ns <= common_end]
    estimate = [sample for sample in estimate if common_start <= sample.timestamp_ns <= common_end]
    truth_stamps = [sample.timestamp_ns for sample in truth]
    estimate_stamps = [sample.timestamp_ns for sample in estimate]
    relative_yaws = [
        wrapped_degrees(yaw_degrees(sample.pose) - yaw_degrees(truth[0].pose))
        for sample in truth
    ]
    try:
        turn_start, turn_end = phase_boundary_indices(relative_yaws)
    except StopIteration:
        peak_yaw = max(abs(value) for value in relative_yaws)
        if peak_yaw < 1.0:
            raise RuntimeError(
                f"native pose never reached a measurable turn (peak={peak_yaw:.3f}deg)"
            )
        turn_start, turn_end = phase_boundary_indices(
            relative_yaws,
            turn_end_threshold=max(1.0, peak_yaw - 0.2),
        )
    truth_bounds = (truth[0], truth[turn_start], truth[turn_end], truth[-1])
    estimate_bounds = tuple(nearest(estimate, estimate_stamps, item.timestamp_ns) for item in truth_bounds)

    print(f"truth_samples: {len(truth)}")
    print(f"estimate_samples: {len(estimate)}")
    print(f"landmark_messages: {landmark_messages}")
    print(f"landmarks: {len(final_landmarks)}")
    print(f"observation_messages_recorded: {observation_messages}")
    for name, start, end, est_start, est_end in (
        ("first_leg", truth_bounds[0], truth_bounds[1], estimate_bounds[0], estimate_bounds[1]),
        ("turn", truth_bounds[1], truth_bounds[2], estimate_bounds[1], estimate_bounds[2]),
        ("second_leg", truth_bounds[2], truth_bounds[3], estimate_bounds[2], estimate_bounds[3]),
        ("complete_route", truth_bounds[0], truth_bounds[3], estimate_bounds[0], estimate_bounds[3]),
    ):
        print_phase(name, start, end, est_start, est_end)

    trajectory_translation = []
    trajectory_rotation = []
    max_skew_ms = 0.0
    for sample in estimate:
        native = nearest(truth, truth_stamps, sample.timestamp_ns)
        max_skew_ms = max(max_skew_ms, abs(native.timestamp_ns - sample.timestamp_ns) / 1_000_000.0)
        truth_relative = relative(truth[0].pose, native.pose)
        estimate_relative = relative(estimate[0].pose, sample.pose)
        residual = compose(inverse(truth_relative), estimate_relative)
        trajectory_translation.append(translation_norm(residual))
        trajectory_rotation.append(rotation_angle_degrees(residual))

    estimate_rpy = [rpy_degrees(sample.pose) for sample in estimate]
    truth_rpy = [rpy_degrees(sample.pose) for sample in truth]
    print(
        "trajectory_translation_error: "
        f"median={percentile(trajectory_translation, 50):.4f}m, "
        f"p95={percentile(trajectory_translation, 95):.4f}m, "
        f"max={max(trajectory_translation):.4f}m"
    )
    print(
        "trajectory_rotation_error: "
        f"median={percentile(trajectory_rotation, 50):.2f}deg, "
        f"p95={percentile(trajectory_rotation, 95):.2f}deg, "
        f"max={max(trajectory_rotation):.2f}deg"
    )
    print(f"maximum_sync_skew: {max_skew_ms:.3f}ms")
    print(
        "native_abs_roll_pitch_max: "
        f"roll={max(abs(value[0]) for value in truth_rpy):.3f}deg, "
        f"pitch={max(abs(value[1]) for value in truth_rpy):.3f}deg"
    )
    print(
        "cuvslam_abs_roll_pitch_max: "
        f"roll={max(abs(value[0]) for value in estimate_rpy):.3f}deg, "
        f"pitch={max(abs(value[1]) for value in estimate_rpy):.3f}deg"
    )
    end_roll, end_pitch, end_yaw = estimate_rpy[-1]
    print(f"cuvslam_end_rpy: roll={end_roll:+.3f}deg, pitch={end_pitch:+.3f}deg, yaw={end_yaw:+.3f}deg")
    print("vo_state_counts: " + ", ".join(f"{key}={value}" for key, value in sorted(statuses.items())))

    if not final_landmarks:
        print("landmark_geometry: skipped (pose-only bag)")
        return

    start_ns = max(truth[0].timestamp_ns, estimate[0].timestamp_ns, map_odom[0].timestamp_ns)
    world_base = nearest(truth, truth_stamps, start_ns).pose
    odom_base = nearest(estimate, estimate_stamps, start_ns).pose
    map_odom_start = nearest_tf(map_odom, start_ns).transform
    world_map = compose(world_base, inverse(compose(map_odom_start, odom_base)))
    boxes = load_boxes(args.sdf)
    world_points = [transform_point(world_map, point) for point in final_landmarks]
    distances = [min(point_to_box_surface(point, box) for box in boxes) for point in world_points]
    within_025 = sum(value <= 0.25 for value in distances)
    print(
        "world_map_translation: "
        f"({world_map.translation[0]:+.4f}, {world_map.translation[1]:+.4f}, {world_map.translation[2]:+.4f})m"
    )
    print(
        "landmark_surface_distance: "
        f"median={percentile(distances, 50):.4f}m, p90={percentile(distances, 90):.4f}m, "
        f"p95={percentile(distances, 95):.4f}m, max={max(distances):.4f}m"
    )
    print(f"landmark_within_0.25m: {within_025}/{len(distances)} ({100.0 * within_025 / len(distances):.1f}%)")
    if final_weights:
        print(
            "landmark_weight: "
            f"median={percentile(final_weights, 50):.5f}, p90={percentile(final_weights, 90):.5f}, max={max(final_weights):.5f}"
        )


if __name__ == "__main__":
    main()
