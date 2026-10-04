#!/usr/bin/env python3
"""Compare current cuVSLAM observations with static SDF box surfaces."""

from __future__ import annotations

import argparse
import bisect
from dataclasses import dataclass
import math

from evaluate_landmark_geometry_bag import (
    load_boxes,
    percentile,
    point_to_box_surface,
    point_xyz,
    transform_point,
)
from frame_contract_math import Transform, base_to_left_optical, compose
from gz_pose_log import read_pose_log


OBSERVATION_TOPIC = "/visual_slam/vis/observations_cloud"
LEFT_INFO_TOPIC = "/d435i/infra1/camera_info"
CUVSLAM_RIGHT_INFO_TOPIC = "/cuvslam/input/infra2/camera_info"


@dataclass(frozen=True)
class PoseSample:
    timestamp_ns: int
    pose: Transform


@dataclass(frozen=True)
class CloudSample:
    timestamp_ns: int
    frame_id: str
    points: tuple[tuple[float, float, float], ...]
    fields: tuple[str, ...]


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


def nearest_pose(
    samples: list[PoseSample],
    stamps: list[int],
    target_ns: int,
) -> PoseSample:
    index = bisect.bisect_left(stamps, target_ns)
    candidates = []
    if index < len(samples):
        candidates.append(samples[index])
    if index:
        candidates.append(samples[index - 1])
    return min(
        candidates,
        key=lambda sample: abs(sample.timestamp_ns - target_ns),
    )


def base_to_cloud_frame(
    frame_id: str,
    interpretation: str = "declared",
    camera_x: float = 0.19,
) -> Transform:
    normalized = frame_id.lstrip("/")
    if interpretation == "physical":
        return Transform(translation=(camera_x, 0.025, 0.20))
    if interpretation == "optical":
        return base_to_left_optical(camera_x, 0.025, 0.20)
    if interpretation != "declared":
        raise ValueError(f"unsupported frame interpretation: {interpretation}")
    if normalized == "camera_infra1_frame":
        return Transform(translation=(camera_x, 0.025, 0.20))
    if normalized == "camera_infra1_optical_frame":
        return base_to_left_optical(camera_x, 0.025, 0.20)
    if normalized == "base_link":
        return Transform()
    raise ValueError(f"unsupported observation frame: {frame_id!r}")


def selected_indices(count: int, maximum: int) -> list[int]:
    if count <= maximum:
        return list(range(count))
    return sorted(
        {
            round(index * (count - 1) / (maximum - 1))
            for index in range(maximum)
        }
    )


def stereo_focal_baseline(left_info, right_info) -> tuple[float, float, float]:
    focal_x = float(left_info.k[0])
    right_focal_x = float(right_info.p[0])
    if focal_x <= 0.0 or right_focal_x <= 0.0:
        raise ValueError("camera focal length must be positive")
    baseline = -float(right_info.p[3]) / right_focal_x
    if baseline <= 0.0:
        raise ValueError("normalized right CameraInfo has no positive baseline")
    return focal_x, baseline, focal_x * baseline


def main() -> None:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs_py.point_cloud2 import read_points

    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("sdf")
    parser.add_argument("native_pose_log")
    parser.add_argument("--native-entity", default="slam_bot")
    parser.add_argument("--camera-x", type=float, default=0.19)
    parser.add_argument("--max-clouds", type=int, default=30)
    parser.add_argument("--max-truth-skew-ms", type=float, default=100.0)
    parser.add_argument(
        "--frame-interpretation",
        choices=("declared", "physical", "optical"),
        default="declared",
    )
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    required_topics = (
        OBSERVATION_TOPIC,
        LEFT_INFO_TOPIC,
        CUVSLAM_RIGHT_INFO_TOPIC,
    )
    missing = [topic for topic in required_topics if topic not in topic_types]
    if missing:
        raise RuntimeError(f"bag is missing: {', '.join(missing)}")
    message_types = {
        topic: get_message(topic_types[topic])
        for topic in required_topics
    }
    truth = [
        PoseSample(
            sample.timestamp_ns,
            Transform(sample.translation, sample.rotation),
        )
        for sample in read_pose_log(
            args.native_pose_log,
            args.native_entity,
        )
    ]
    clouds = []
    camera_info = {}
    while reader.has_next():
        topic, data, bag_timestamp_ns = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(data, message_types[topic])
        if topic == OBSERVATION_TOPIC:
            points = tuple(
                point_xyz(point)
                for point in read_points(
                    message,
                    field_names=("x", "y", "z"),
                    skip_nans=True,
                )
            )
            if points:
                clouds.append(
                    CloudSample(
                        timestamp_ns(message, bag_timestamp_ns),
                        message.header.frame_id,
                        points,
                        tuple(field.name for field in message.fields),
                    )
                )
        else:
            camera_info.setdefault(topic, message)

    truth.sort(key=lambda sample: sample.timestamp_ns)
    clouds.sort(key=lambda sample: sample.timestamp_ns)
    if not truth or not clouds:
        raise RuntimeError("truth or observation cloud is empty")
    if len(camera_info) != 2:
        raise RuntimeError("stereo CameraInfo is incomplete")
    focal_x, baseline, focal_baseline = stereo_focal_baseline(
        camera_info[LEFT_INFO_TOPIC],
        camera_info[CUVSLAM_RIGHT_INFO_TOPIC],
    )
    truth_stamps = [sample.timestamp_ns for sample in truth]
    frames = sorted({cloud.frame_id for cloud in clouds})
    if len(frames) != 1:
        raise RuntimeError(f"observation frames changed: {frames}")
    base_cloud = base_to_cloud_frame(
        frames[0],
        args.frame_interpretation,
        args.camera_x,
    )
    boxes = load_boxes(args.sdf)

    synchronized_clouds = []
    rejected_clouds = 0
    for cloud in clouds:
        truth_sample = nearest_pose(truth, truth_stamps, cloud.timestamp_ns)
        skew_ms = abs(
            truth_sample.timestamp_ns - cloud.timestamp_ns
        ) / 1_000_000.0
        if skew_ms <= args.max_truth_skew_ms:
            synchronized_clouds.append(cloud)
        else:
            rejected_clouds += 1
    if not synchronized_clouds:
        raise RuntimeError("no observation cloud passed truth synchronization")
    selected = [
        synchronized_clouds[index]
        for index in selected_indices(
            len(synchronized_clouds),
            args.max_clouds,
        )
    ]
    raw_points = [
        point
        for cloud in selected
        for point in cloud.points
    ]
    all_distances = []
    all_forward_depths = []
    cloud_medians = []
    cloud_p90s = []
    point_counts = []
    maximum_truth_skew_ms = 0.0
    for cloud in selected:
        truth_sample = nearest_pose(truth, truth_stamps, cloud.timestamp_ns)
        maximum_truth_skew_ms = max(
            maximum_truth_skew_ms,
            abs(truth_sample.timestamp_ns - cloud.timestamp_ns) / 1_000_000.0,
        )
        world_cloud = compose(truth_sample.pose, base_cloud)
        world_points = [
            transform_point(world_cloud, point)
            for point in cloud.points
        ]
        distances = [
            min(point_to_box_surface(point, box) for box in boxes)
            for point in world_points
        ]
        all_distances.extend(distances)
        if args.frame_interpretation in ("declared", "physical"):
            all_forward_depths.extend(point[0] for point in cloud.points)
        cloud_medians.append(percentile(distances, 50))
        cloud_p90s.append(percentile(distances, 90))
        point_counts.append(len(distances))

    within_010 = sum(distance <= 0.10 for distance in all_distances)
    within_025 = sum(distance <= 0.25 for distance in all_distances)
    outliers = sum(distance > 0.25 for distance in all_distances)
    print(f"observation_topic: {OBSERVATION_TOPIC}")
    print(f"observation_messages: {len(clouds)}")
    print(f"synchronized_messages: {len(synchronized_clouds)}")
    print(f"rejected_for_truth_skew: {rejected_clouds}")
    print(f"analyzed_messages: {len(selected)}")
    print(f"frame: {frames[0]}")
    print(f"frame_interpretation: {args.frame_interpretation}")
    print(f"stereo_focal_x: {focal_x:.6f} px")
    print(f"stereo_baseline: {baseline:.6f} m")
    print(f"fields: {', '.join(clouds[-1].fields)}")
    for axis, values in zip("xyz", zip(*raw_points)):
        print(f"raw_{axis}_bounds: [{min(values):.4f}, {max(values):.4f}]")
    print(
        "points_per_cloud: "
        f"median={percentile([float(value) for value in point_counts], 50):.1f}, "
        f"min={min(point_counts)}, max={max(point_counts)}"
    )
    print(f"analyzed_points: {len(all_distances)}")
    print(f"maximum_truth_time_skew: {maximum_truth_skew_ms:.3f} ms")
    print(
        "cloud_surface_median_distribution: "
        f"median={percentile(cloud_medians, 50):.4f} m, "
        f"p90={percentile(cloud_medians, 90):.4f} m, "
        f"max={max(cloud_medians):.4f} m"
    )
    print(
        "cloud_surface_p90_distribution: "
        f"median={percentile(cloud_p90s, 50):.4f} m, "
        f"p90={percentile(cloud_p90s, 90):.4f} m, "
        f"max={max(cloud_p90s):.4f} m"
    )
    print(f"surface_distance_median: {percentile(all_distances, 50):.4f} m")
    print(f"surface_distance_p90: {percentile(all_distances, 90):.4f} m")
    print(f"surface_distance_p95: {percentile(all_distances, 95):.4f} m")
    print(f"surface_distance_max: {max(all_distances):.4f} m")
    print(
        f"within_0.10m: {within_010}/{len(all_distances)} "
        f"({100.0 * within_010 / len(all_distances):.1f}%)"
    )
    print(
        f"within_0.25m: {within_025}/{len(all_distances)} "
        f"({100.0 * within_025 / len(all_distances):.1f}%)"
    )
    print(
        f"outlier_over_0.25m: {outliers}/{len(all_distances)} "
        f"({100.0 * outliers / len(all_distances):.1f}%)"
    )
    if len(all_forward_depths) == len(all_distances):
        depth_bins = (
            (0.0, 1.5),
            (1.5, 2.5),
            (2.5, 4.0),
            (4.0, float("inf")),
        )
        print("forward_depth_bins:")
        for lower, upper in depth_bins:
            selected_pairs = [
                (depth, distance)
                for depth, distance in zip(
                    all_forward_depths,
                    all_distances,
                )
                if lower <= depth < upper
            ]
            if not selected_pairs:
                continue
            depths = [depth for depth, _ in selected_pairs]
            distances = [distance for _, distance in selected_pairs]
            within = sum(distance <= 0.25 for distance in distances)
            median_depth = percentile(depths, 50)
            expected_disparity = focal_baseline / median_depth
            upper_label = "inf" if math.isinf(upper) else f"{upper:.1f}"
            print(
                f"  [{lower:.1f},{upper_label})m: "
                f"points={len(distances)}, "
                f"median_depth={median_depth:.3f}m, "
                f"expected_disparity={expected_disparity:.2f}px, "
                f"surface_median={percentile(distances, 50):.3f}m, "
                f"within_0.25m={100.0 * within / len(distances):.1f}%"
            )


if __name__ == "__main__":
    main()
