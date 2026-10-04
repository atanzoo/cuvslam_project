#!/usr/bin/env python3
"""Compare a final cuVSLAM landmark cloud with static SDF box surfaces."""

from __future__ import annotations

import argparse
import bisect
import math
from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

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
LANDMARK_TOPIC = "/visual_slam/vis/landmarks_cloud"
TF_TOPIC = "/tf"


@dataclass(frozen=True)
class TimedTransform:
    timestamp_ns: int
    transform: Transform


@dataclass(frozen=True)
class Box:
    name: str
    pose: Transform
    size: tuple[float, float, float]


def timestamp_ns(message, bag_timestamp_ns: int) -> int:
    stamp = message.header.stamp
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value if value else bag_timestamp_ns


def transform_from_pose(pose) -> Transform:
    return Transform(
        translation=(pose.position.x, pose.position.y, pose.position.z),
        rotation=(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ),
    )


def transform_from_tf(transform) -> Transform:
    return Transform(
        translation=(
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ),
        rotation=(
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ),
    )


def parse_pose(element: ET.Element | None) -> Transform:
    if element is None or not (element.text or "").strip():
        return Transform()
    values = [float(value) for value in element.text.split()]
    if len(values) != 6:
        raise ValueError(f"expected six pose values, got {values}")
    x, y, z, roll, pitch, yaw = values
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    quaternion = (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )
    return Transform((x, y, z), quaternion)


def load_boxes(path: str | Path) -> list[Box]:
    root = ET.parse(path).getroot()
    world = root.find("world")
    if world is None:
        raise ValueError("SDF does not contain a world")
    boxes = []
    for model in world.findall("model"):
        model_name = model.get("name", "unnamed")
        if model_name == "slam_bot":
            continue
        if (model.findtext("static") or "").strip().lower() != "true":
            continue
        model_pose = parse_pose(model.find("pose"))
        for link in model.findall("link"):
            link_pose = parse_pose(link.find("pose"))
            for kind in ("collision", "visual"):
                for geometry in link.findall(kind):
                    size_text = geometry.findtext("geometry/box/size")
                    if not size_text:
                        continue
                    size = tuple(float(value) for value in size_text.split())
                    if len(size) != 3:
                        raise ValueError(f"invalid box size {size_text!r}")
                    boxes.append(
                        Box(
                            name=f"{model_name}/{geometry.get('name', kind)}",
                            pose=compose(
                                compose(model_pose, link_pose),
                                parse_pose(geometry.find("pose")),
                            ),
                            size=size,  # type: ignore[arg-type]
                        )
                    )
    if not boxes:
        raise ValueError("no static SDF box surfaces found")
    return boxes


def nearest_timed(
    samples: list[TimedTransform],
    target_ns: int,
) -> TimedTransform:
    stamps = [sample.timestamp_ns for sample in samples]
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


def transform_point(transform: Transform, point) -> tuple[float, float, float]:
    point_transform = compose(
        transform,
        Transform(translation=(float(point[0]), float(point[1]), float(point[2]))),
    )
    return point_transform.translation


def point_to_box_surface(point, box: Box) -> float:
    local = transform_point(inverse(box.pose), point)
    half = tuple(value / 2.0 for value in box.size)
    outside = tuple(
        max(abs(coordinate) - extent, 0.0)
        for coordinate, extent in zip(local, half)
    )
    outside_distance = math.sqrt(sum(value * value for value in outside))
    if outside_distance:
        return outside_distance
    return min(
        extent - abs(coordinate)
        for coordinate, extent in zip(local, half)
    )


def percentile(values: list[float], percentage: float) -> float:
    ordered = sorted(values)
    position = (len(ordered) - 1) * percentage / 100.0
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def point_xyz(point) -> tuple[float, float, float]:
    try:
        return float(point["x"]), float(point["y"]), float(point["z"])
    except (IndexError, KeyError, TypeError):
        return float(point[0]), float(point[1]), float(point[2])


def point_weight(point) -> float:
    try:
        return float(point["weight"])
    except (IndexError, KeyError, TypeError):
        return float(point[3])


def pearson_correlation(first: list[float], second: list[float]) -> float:
    first_mean = sum(first) / len(first)
    second_mean = sum(second) / len(second)
    numerator = sum(
        (a - first_mean) * (b - second_mean)
        for a, b in zip(first, second)
    )
    first_energy = sum((value - first_mean) ** 2 for value in first)
    second_energy = sum((value - second_mean) ** 2 for value in second)
    denominator = math.sqrt(first_energy * second_energy)
    return numerator / denominator if denominator else float("nan")


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
    parser.add_argument("--max-snapshots", type=int, default=12)
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    required = (ESTIMATE_TOPIC, LANDMARK_TOPIC, TF_TOPIC)
    missing = [topic for topic in required if topic not in topic_types]
    if missing:
        raise RuntimeError(f"bag is missing: {', '.join(missing)}")
    message_types = {
        topic: get_message(topic_types[topic])
        for topic in required
    }

    truth = [
        TimedTransform(
            sample.timestamp_ns,
            Transform(sample.translation, sample.rotation),
        )
        for sample in read_pose_log(
            args.native_pose_log,
            args.native_entity,
        )
    ]
    estimate = []
    map_odom = []
    final_landmarks = []
    final_weights = []
    landmark_snapshots = []
    landmark_snapshot_timestamps = []
    landmark_frame = ""
    while reader.has_next():
        topic, data, bag_timestamp_ns = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(data, message_types[topic])
        if topic == ESTIMATE_TOPIC:
            estimate.append(
                TimedTransform(
                    timestamp_ns(message, bag_timestamp_ns),
                    transform_from_pose(message.pose.pose),
                )
            )
        elif topic == TF_TOPIC:
            for item in message.transforms:
                if item.header.frame_id == "map" and item.child_frame_id == "odom":
                    map_odom.append(
                        TimedTransform(
                            timestamp_ns(item, bag_timestamp_ns),
                            transform_from_tf(item.transform),
                        )
                    )
        else:
            has_weight = any(
                field.name == "weight" for field in message.fields
            )
            field_names = (
                ("x", "y", "z", "weight")
                if has_weight
                else ("x", "y", "z")
            )
            raw_points = list(
                read_points(
                    message,
                    field_names=field_names,
                    skip_nans=True,
                )
            )
            points = [point_xyz(point) for point in raw_points]
            if points:
                final_landmarks = points
                final_weights = (
                    [point_weight(point) for point in raw_points]
                    if has_weight
                    else []
                )
                landmark_snapshots.append(points)
                landmark_snapshot_timestamps.append(
                    timestamp_ns(message, bag_timestamp_ns)
                )
                landmark_frame = message.header.frame_id

    for samples in (truth, estimate, map_odom):
        samples.sort(key=lambda sample: sample.timestamp_ns)
    if not truth or not estimate or not map_odom or not final_landmarks:
        raise RuntimeError("required pose, TF, or landmark samples are empty")
    if landmark_frame != "map":
        raise RuntimeError(f"expected landmark frame 'map', got {landmark_frame!r}")
    if args.max_snapshots <= 0:
        raise RuntimeError("--max-snapshots must be positive")
    if len(landmark_snapshots) > args.max_snapshots:
        indices = sorted({
            round(
                index * (len(landmark_snapshots) - 1)
                / (args.max_snapshots - 1)
            )
            for index in range(args.max_snapshots)
        }) if args.max_snapshots > 1 else [len(landmark_snapshots) - 1]
        landmark_snapshots = [landmark_snapshots[index] for index in indices]
        landmark_snapshot_timestamps = [
            landmark_snapshot_timestamps[index] for index in indices
        ]

    start_ns = max(truth[0].timestamp_ns, estimate[0].timestamp_ns, map_odom[0].timestamp_ns)
    world_base = nearest_timed(truth, start_ns).transform
    odom_base = nearest_timed(estimate, start_ns).transform
    map_odom_start = nearest_timed(map_odom, start_ns).transform
    map_base = compose(map_odom_start, odom_base)
    world_map = compose(world_base, inverse(map_base))

    boxes = load_boxes(args.sdf)
    world_snapshots = [
        [transform_point(world_map, point) for point in snapshot]
        for snapshot in landmark_snapshots
    ]
    world_points = world_snapshots[-1]

    truth_origin = nearest_timed(truth, start_ns).transform
    for index, (snapshot, snapshot_timestamp) in enumerate(
        zip(world_snapshots, landmark_snapshot_timestamps),
        start=1,
    ):
        truth_pose = nearest_timed(truth, snapshot_timestamp).transform
        truth_motion = relative(truth_origin, truth_pose)
        snapshot_distances = [
            min(point_to_box_surface(point, box) for box in boxes)
            for point in snapshot
        ]
        within_025 = sum(
            distance <= 0.25 for distance in snapshot_distances
        )
        print(
            f"snapshot_{index}_geometry: points={len(snapshot)}, "
            f"median={percentile(snapshot_distances, 50):.4f}m, "
            f"p90={percentile(snapshot_distances, 90):.4f}m, "
            f"within_0.25m={within_025}/{len(snapshot)} "
            f"({100.0 * within_025 / len(snapshot):.1f}%), "
            f"truth_displacement={translation_norm(truth_motion):.3f}m, "
            f"truth_rotation={rotation_angle_degrees(truth_motion):.2f}deg"
        )

    distances = []
    nearest_names = []
    for point in world_points:
        candidates = [
            (point_to_box_surface(point, box), box.name)
            for box in boxes
        ]
        distance, name = min(candidates)
        distances.append(distance)
        nearest_names.append(name)

    print(f"landmark_frame: {landmark_frame}")
    print(f"landmarks: {len(world_points)}")
    print(f"sdf_box_surfaces: {len(boxes)}")
    print(
        "world_map_translation: "
        f"({world_map.translation[0]:+.4f}, "
        f"{world_map.translation[1]:+.4f}, "
        f"{world_map.translation[2]:+.4f}) m"
    )
    for axis, values in zip("xyz", zip(*world_points)):
        print(f"world_{axis}_bounds: [{min(values):.4f}, {max(values):.4f}]")
    print(f"surface_distance_median: {percentile(distances, 50):.4f} m")
    print(f"surface_distance_p90: {percentile(distances, 90):.4f} m")
    print(f"surface_distance_p95: {percentile(distances, 95):.4f} m")
    print(f"surface_distance_max: {max(distances):.4f} m")
    for threshold in (0.10, 0.25, 0.50):
        within = sum(distance <= threshold for distance in distances)
        print(
            f"within_{threshold:.2f}m: {within}/{len(distances)} "
            f"({100.0 * within / len(distances):.1f}%)"
        )
    outliers = sum(distance > 0.25 for distance in distances)
    print(
        f"outlier_over_0.25m: {outliers}/{len(distances)} "
        f"({100.0 * outliers / len(distances):.1f}%)"
    )
    if len(final_weights) == len(distances):
        print(
            "weight_distribution: "
            f"min={min(final_weights):.6f}, "
            f"median={percentile(final_weights, 50):.6f}, "
            f"p90={percentile(final_weights, 90):.6f}, "
            f"max={max(final_weights):.6f}"
        )
        print(
            "weight_distance_pearson: "
            f"{pearson_correlation(final_weights, distances):.6f}"
        )
        good_weights = [
            weight
            for weight, distance in zip(final_weights, distances)
            if distance <= 0.25
        ]
        bad_weights = [
            weight
            for weight, distance in zip(final_weights, distances)
            if distance > 0.25
        ]
        print(
            "weight_by_geometry: "
            f"within_0.25m_median={percentile(good_weights, 50):.6f}, "
            f"over_0.25m_median={percentile(bad_weights, 50):.6f}"
        )
        ranked = sorted(
            zip(final_weights, distances),
            key=lambda item: item[0],
            reverse=True,
        )
        for fraction in (0.25, 0.50, 0.75):
            retained = ranked[: max(1, round(len(ranked) * fraction))]
            within = sum(distance <= 0.25 for _, distance in retained)
            print(
                f"top_weight_{int(fraction * 100)}pct: "
                f"within_0.25m={within}/{len(retained)} "
                f"({100.0 * within / len(retained):.1f}%)"
            )
    counts = {}
    for name in nearest_names:
        counts[name] = counts.get(name, 0) + 1
    print("nearest_surfaces:")
    for name, count in sorted(counts.items(), key=lambda item: (-item[1], item[0]))[:10]:
        print(f"  {name}: {count}")


if __name__ == "__main__":
    main()
