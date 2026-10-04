#!/usr/bin/env python3
"""Audit camera/IMU timing and gyro consistency for a corner-route bag."""

from __future__ import annotations

import argparse
import bisect
from dataclasses import dataclass
import math

from evaluate_corner_bag import wrapped_degrees, yaw_degrees
from evaluate_landmark_geometry_bag import percentile
from frame_contract_math import Transform
from gz_pose_log import read_pose_log


IMAGE_TOPIC = "/d435i/infra1/image_rect_raw"
IMU_TOPIC = "/d435i/imu"
TRUTH_TOPIC = "/ground_truth/odom"


@dataclass(frozen=True)
class ImuSample:
    timestamp_ns: int
    frame_id: str
    angular_velocity: tuple[float, float, float]
    linear_acceleration: tuple[float, float, float]


@dataclass(frozen=True)
class TruthSample:
    timestamp_ns: int
    pose: Transform


def timestamp_ns(message, bag_timestamp_ns: int) -> int:
    stamp = message.header.stamp
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value if value else bag_timestamp_ns


def interval_counts(
    event_stamps: list[int],
    boundary_stamps: list[int],
) -> list[int]:
    return [
        bisect.bisect_left(event_stamps, end)
        - bisect.bisect_left(event_stamps, start)
        for start, end in zip(boundary_stamps, boundary_stamps[1:])
    ]


def integrate_angular_velocity(
    samples: list[ImuSample],
    start_ns: int,
    end_ns: int,
) -> tuple[float, float, float]:
    selected = [
        sample
        for sample in samples
        if start_ns <= sample.timestamp_ns <= end_ns
    ]
    integrated = [0.0, 0.0, 0.0]
    for first, second in zip(selected, selected[1:]):
        dt = (second.timestamp_ns - first.timestamp_ns) / 1_000_000_000.0
        for axis in range(3):
            integrated[axis] += (
                first.angular_velocity[axis]
                + second.angular_velocity[axis]
            ) * 0.5 * dt
    return tuple(math.degrees(value) for value in integrated)


def nearest_skews_ms(first: list[int], second: list[int]) -> list[float]:
    skews = []
    for target in first:
        index = bisect.bisect_left(second, target)
        candidates = []
        if index < len(second):
            candidates.append(second[index])
        if index:
            candidates.append(second[index - 1])
        skews.append(min(abs(value - target) for value in candidates) / 1e6)
    return skews


def measured_turn_boundary_indices(
    relative_yaws: list[float],
) -> tuple[int, int]:
    peak_yaw = max(abs(value) for value in relative_yaws)
    turn_start = next(
        index for index, yaw in enumerate(relative_yaws)
        if abs(yaw) >= 1.0
    )
    turn_end = next(
        index for index in range(turn_start, len(relative_yaws))
        if abs(relative_yaws[index]) >= peak_yaw - 0.2
    )
    return max(0, turn_start - 1), turn_end


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
    required = (IMAGE_TOPIC, IMU_TOPIC) if args.truth_gz_log else (
        IMAGE_TOPIC,
        IMU_TOPIC,
        TRUTH_TOPIC,
    )
    missing = [topic for topic in required if topic not in topic_types]
    if missing:
        raise RuntimeError(f"bag is missing: {', '.join(missing)}")
    message_types = {
        topic: get_message(topic_types[topic])
        for topic in required
    }

    images = []
    imu = []
    truth = (
        [
            TruthSample(
                sample.timestamp_ns,
                Transform(sample.translation, sample.rotation),
            )
            for sample in read_pose_log(
                args.truth_gz_log,
                args.truth_gz_entity,
            )
        ]
        if args.truth_gz_log else []
    )
    while reader.has_next():
        topic, data, bag_timestamp_ns = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(data, message_types[topic])
        stamp = timestamp_ns(message, bag_timestamp_ns)
        if topic == IMAGE_TOPIC:
            images.append(stamp)
        elif topic == IMU_TOPIC:
            imu.append(
                ImuSample(
                    stamp,
                    message.header.frame_id,
                    (
                        message.angular_velocity.x,
                        message.angular_velocity.y,
                        message.angular_velocity.z,
                    ),
                    (
                        message.linear_acceleration.x,
                        message.linear_acceleration.y,
                        message.linear_acceleration.z,
                    ),
                )
            )
        else:
            pose = message.pose.pose
            truth.append(
                TruthSample(
                    stamp,
                    Transform(
                        (pose.position.x, pose.position.y, pose.position.z),
                        (
                            pose.orientation.x,
                            pose.orientation.y,
                            pose.orientation.z,
                            pose.orientation.w,
                        ),
                    ),
                )
            )

    images.sort()
    imu.sort(key=lambda sample: sample.timestamp_ns)
    truth.sort(key=lambda sample: sample.timestamp_ns)
    if len(images) < 2 or len(imu) < 2 or len(truth) < 3:
        raise RuntimeError("insufficient image, IMU, or truth samples")
    imu_stamps = [sample.timestamp_ns for sample in imu]
    image_periods_ms = [
        (second - first) / 1e6
        for first, second in zip(images, images[1:])
    ]
    imu_periods_ms = [
        (second.timestamp_ns - first.timestamp_ns) / 1e6
        for first, second in zip(imu, imu[1:])
    ]
    imu_per_image = interval_counts(imu_stamps, images)
    image_imu_skews = nearest_skews_ms(images, imu_stamps)

    start_yaw = yaw_degrees(truth[0].pose)
    relative_yaws = [
        wrapped_degrees(yaw_degrees(sample.pose) - start_yaw)
        for sample in truth
    ]
    turn_start, turn_end = measured_turn_boundary_indices(relative_yaws)
    integrated_gyro = integrate_angular_velocity(
        imu,
        truth[turn_start].timestamp_ns,
        truth[turn_end].timestamp_ns,
    )
    active_turn_imu = [
        sample for sample in imu
        if abs(sample.angular_velocity[2]) >= math.radians(1.0)
    ]
    active_integrated_gyro = integrate_angular_velocity(
        imu,
        active_turn_imu[0].timestamp_ns,
        active_turn_imu[-1].timestamp_ns,
    )
    truth_yaw_rates = [
        abs(wrapped_degrees(second_yaw - first_yaw))
        / ((second.timestamp_ns - first.timestamp_ns) / 1_000_000_000.0)
        for first, second, first_yaw, second_yaw in zip(
            truth,
            truth[1:],
            relative_yaws,
            relative_yaws[1:],
        )
        if second.timestamp_ns > first.timestamp_ns
        and abs(wrapped_degrees(second_yaw - first_yaw)) >= 0.01
    ]
    acceleration_norms = [
        math.sqrt(sum(value * value for value in sample.linear_acceleration))
        for sample in imu
    ]
    frames = sorted({sample.frame_id for sample in imu})

    print(f"image_messages: {len(images)}")
    print(f"imu_messages: {len(imu)}")
    print(f"imu_frames: {frames}")
    print(
        "image_period: "
        f"median={percentile(image_periods_ms, 50):.3f}ms, "
        f"p95={percentile(image_periods_ms, 95):.3f}ms, "
        f"frequency={1000.0 / percentile(image_periods_ms, 50):.2f}Hz"
    )
    print(
        "imu_period: "
        f"median={percentile(imu_periods_ms, 50):.3f}ms, "
        f"p95={percentile(imu_periods_ms, 95):.3f}ms, "
        f"max={max(imu_periods_ms):.3f}ms, "
        f"frequency={1000.0 / percentile(imu_periods_ms, 50):.2f}Hz"
    )
    print(
        "imu_samples_per_image_interval: "
        f"median={percentile([float(value) for value in imu_per_image], 50):.1f}, "
        f"p05={percentile([float(value) for value in imu_per_image], 5):.1f}, "
        f"min={min(imu_per_image)}, max={max(imu_per_image)}"
    )
    print(
        "nearest_image_imu_skew: "
        f"median={percentile(image_imu_skews, 50):.3f}ms, "
        f"p95={percentile(image_imu_skews, 95):.3f}ms, "
        f"max={max(image_imu_skews):.3f}ms"
    )
    print(
        "turn_truth_yaw: "
        f"{relative_yaws[turn_end] - relative_yaws[turn_start]:.2f}deg"
    )
    print(
        "turn_truth_duration: "
        f"{(truth[turn_end].timestamp_ns - truth[turn_start].timestamp_ns) / 1e9:.3f}s"
    )
    print(
        "turn_integrated_gyro_xyz: "
        f"({integrated_gyro[0]:+.2f}, "
        f"{integrated_gyro[1]:+.2f}, "
        f"{integrated_gyro[2]:+.2f})deg"
    )
    print(
        "active_turn_gyro: "
        f"duration={(active_turn_imu[-1].timestamp_ns - active_turn_imu[0].timestamp_ns) / 1e9:.3f}s, "
        f"integrated_z={active_integrated_gyro[2]:+.2f}deg, "
        f"median_abs_z={percentile([abs(math.degrees(sample.angular_velocity[2])) for sample in active_turn_imu], 50):.2f}deg/s"
    )
    print(
        "truth_active_yaw_rate: "
        f"median={percentile(truth_yaw_rates, 50):.2f}deg/s, "
        f"p95={percentile(truth_yaw_rates, 95):.2f}deg/s"
    )
    print(
        "acceleration_norm: "
        f"median={percentile(acceleration_norms, 50):.4f}m/s^2, "
        f"p05={percentile(acceleration_norms, 5):.4f}m/s^2, "
        f"p95={percentile(acceleration_norms, 95):.4f}m/s^2"
    )


if __name__ == "__main__":
    main()
