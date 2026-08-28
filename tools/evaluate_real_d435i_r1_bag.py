#!/usr/bin/env python3
"""Evaluate a stationary real-D435i R1 rosbag contract."""

from __future__ import annotations

import argparse
import bisect
import json
import math
import statistics
from collections import Counter

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


LEFT_IMAGE = "/camera/infra1/image_rect_raw"
RIGHT_IMAGE = "/camera/infra2/image_rect_raw"
LEFT_INFO = "/camera/infra1/camera_info"
RIGHT_INFO = "/camera/infra2/camera_info"
IMU = "/camera/imu"
TF = "/tf"
TF_STATIC = "/tf_static"
REQUIRED = (LEFT_IMAGE, RIGHT_IMAGE, LEFT_INFO, RIGHT_INFO, IMU)


def stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def percentile(values: list[float], percent: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    index = (len(ordered) - 1) * percent / 100.0
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    if lower == upper:
        return ordered[lower]
    fraction = index - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def component_stats(values: list[tuple[float, float, float]]) -> dict[str, list[float]]:
    if not values:
        return {"mean": [float("nan")] * 3, "stddev": [float("nan")] * 3}
    return {
        "mean": [statistics.fmean(row[index] for row in values) for index in range(3)],
        "stddev": [
            statistics.pstdev(row[index] for row in values) for index in range(3)
        ],
    }


def stream_timing(stamps: list[int]) -> dict[str, float | int]:
    intervals_ms = [
        (second - first) / 1e6 for first, second in zip(stamps, stamps[1:])
    ]
    duration_s = (stamps[-1] - stamps[0]) / 1e9 if len(stamps) > 1 else 0.0
    rate_hz = (len(stamps) - 1) / duration_s if duration_s > 0.0 else 0.0
    return {
        "messages": len(stamps),
        "duration_s": duration_s,
        "rate_hz": rate_hz,
        "period_median_ms": percentile(intervals_ms, 50.0),
        "period_p95_ms": percentile(intervals_ms, 95.0),
        "period_max_ms": max(intervals_ms, default=float("nan")),
        "non_monotonic": sum(value <= 0.0 for value in intervals_ms),
    }


def nearest_skews_ms(first: list[int], second: list[int]) -> list[float]:
    result = []
    for value in first:
        index = bisect.bisect_left(second, value)
        candidates = []
        if index < len(second):
            candidates.append(abs(second[index] - value))
        if index > 0:
            candidates.append(abs(second[index - 1] - value))
        if candidates:
            result.append(min(candidates) / 1e6)
    return result


def camera_info_summary(message) -> dict[str, object]:
    baseline = -message.p[3] / message.p[0] if message.p[0] else float("nan")
    return {
        "frame_id": message.header.frame_id,
        "width": int(message.width),
        "height": int(message.height),
        "distortion_model": message.distortion_model,
        "d": list(message.d),
        "k": list(message.k),
        "r": list(message.r),
        "p": list(message.p),
        "projected_baseline_m": baseline,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--output")
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    missing = [topic for topic in REQUIRED if topic not in topic_types]
    if missing:
        raise RuntimeError(f"missing required topics: {missing}")
    selected = set(REQUIRED) | {TF, TF_STATIC}
    message_types = {
        topic: get_message(message_type)
        for topic, message_type in topic_types.items()
        if topic in selected
    }

    image_stamps = {LEFT_IMAGE: [], RIGHT_IMAGE: []}
    first_images = {}
    camera_info = {}
    imu_stamps = []
    imu_frames = Counter()
    gyro = []
    accel = []
    imu_covariances = {}
    tf_edges = Counter()
    tf_values = {}

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(data, message_types[topic])
        if topic in image_stamps:
            image_stamps[topic].append(stamp_ns(message))
            first_images.setdefault(topic, message)
        elif topic in (LEFT_INFO, RIGHT_INFO):
            camera_info.setdefault(topic, message)
        elif topic == IMU:
            imu_stamps.append(stamp_ns(message))
            imu_frames[message.header.frame_id] += 1
            gyro.append(
                (
                    float(message.angular_velocity.x),
                    float(message.angular_velocity.y),
                    float(message.angular_velocity.z),
                )
            )
            accel.append(
                (
                    float(message.linear_acceleration.x),
                    float(message.linear_acceleration.y),
                    float(message.linear_acceleration.z),
                )
            )
            if not imu_covariances:
                imu_covariances = {
                    "orientation": list(message.orientation_covariance),
                    "angular_velocity": list(message.angular_velocity_covariance),
                    "linear_acceleration": list(message.linear_acceleration_covariance),
                }
        elif topic in (TF, TF_STATIC):
            for transform in message.transforms:
                key = (topic, transform.header.frame_id, transform.child_frame_id)
                tf_edges[key] += 1
                tf_values.setdefault(
                    key,
                    {
                        "translation": [
                            float(transform.transform.translation.x),
                            float(transform.transform.translation.y),
                            float(transform.transform.translation.z),
                        ],
                        "rotation_xyzw": [
                            float(transform.transform.rotation.x),
                            float(transform.transform.rotation.y),
                            float(transform.transform.rotation.z),
                            float(transform.transform.rotation.w),
                        ],
                    },
                )

    for stamps in (*image_stamps.values(), imu_stamps):
        stamps.sort()
    skews = nearest_skews_ms(image_stamps[LEFT_IMAGE], image_stamps[RIGHT_IMAGE])
    imu_image_skews = nearest_skews_ms(image_stamps[LEFT_IMAGE], imu_stamps)
    gyro_norms = [math.sqrt(sum(value * value for value in row)) for row in gyro]
    accel_norms = [math.sqrt(sum(value * value for value in row)) for row in accel]
    gyro_stats = component_stats(gyro)
    accel_stats = component_stats(accel)

    result = {
        "left_timing": stream_timing(image_stamps[LEFT_IMAGE]),
        "right_timing": stream_timing(image_stamps[RIGHT_IMAGE]),
        "imu_timing": stream_timing(imu_stamps),
        "stereo_skew_ms": {
            "median": percentile(skews, 50.0),
            "p95": percentile(skews, 95.0),
            "max": max(skews, default=float("nan")),
            "exact_fraction": (
                len(set(image_stamps[LEFT_IMAGE]) & set(image_stamps[RIGHT_IMAGE]))
                / max(len(image_stamps[LEFT_IMAGE]), len(image_stamps[RIGHT_IMAGE]), 1)
            ),
        },
        "imu_image_skew_ms": {
            "median": percentile(imu_image_skews, 50.0),
            "p95": percentile(imu_image_skews, 95.0),
            "max": max(imu_image_skews, default=float("nan")),
        },
        "left_image": {
            "frame_id": first_images[LEFT_IMAGE].header.frame_id,
            "width": int(first_images[LEFT_IMAGE].width),
            "height": int(first_images[LEFT_IMAGE].height),
            "encoding": first_images[LEFT_IMAGE].encoding,
            "step": int(first_images[LEFT_IMAGE].step),
        },
        "right_image": {
            "frame_id": first_images[RIGHT_IMAGE].header.frame_id,
            "width": int(first_images[RIGHT_IMAGE].width),
            "height": int(first_images[RIGHT_IMAGE].height),
            "encoding": first_images[RIGHT_IMAGE].encoding,
            "step": int(first_images[RIGHT_IMAGE].step),
        },
        "left_camera_info": camera_info_summary(camera_info[LEFT_INFO]),
        "right_camera_info": camera_info_summary(camera_info[RIGHT_INFO]),
        "imu": {
            "frames": dict(imu_frames),
            "gyro_mean_rad_s": gyro_stats["mean"],
            "gyro_stddev_rad_s": gyro_stats["stddev"],
            "gyro_norm_median_rad_s": percentile(gyro_norms, 50.0),
            "gyro_norm_p95_rad_s": percentile(gyro_norms, 95.0),
            "accel_mean_m_s2": accel_stats["mean"],
            "accel_stddev_m_s2": accel_stats["stddev"],
            "accel_norm_median_m_s2": percentile(accel_norms, 50.0),
            "accel_norm_p05_m_s2": percentile(accel_norms, 5.0),
            "accel_norm_p95_m_s2": percentile(accel_norms, 95.0),
            "accel_norm_stddev_m_s2": statistics.pstdev(accel_norms),
            "covariances": imu_covariances,
        },
        "tf_edges": [
            {
                "topic": topic,
                "parent": parent,
                "child": child,
                "messages": count,
                **tf_values[(topic, parent, child)],
            }
            for (topic, parent, child), count in sorted(tf_edges.items())
        ],
    }

    failures = []
    for label in ("left_timing", "right_timing"):
        timing = result[label]
        if not 28.5 <= timing["rate_hz"] <= 31.5:
            failures.append(f"{label}_rate")
        if timing["non_monotonic"]:
            failures.append(f"{label}_timestamps")
        if timing["period_max_ms"] > 50.0:
            failures.append(f"{label}_gap")
    if result["stereo_skew_ms"]["p95"] > 1.0:
        failures.append("stereo_skew")
    if result["imu_image_skew_ms"]["p95"] > 5.0:
        failures.append("imu_image_skew")
    if not 0.04 <= result["right_camera_info"]["projected_baseline_m"] <= 0.06:
        failures.append("stereo_baseline")
    if not 190.0 <= result["imu_timing"]["rate_hz"] <= 210.0:
        failures.append("imu_rate")
    if result["imu_timing"]["non_monotonic"]:
        failures.append("imu_timestamps")
    if result["imu"]["gyro_norm_p95_rad_s"] > 0.05:
        failures.append("stationary_gyro")
    if not 8.8 <= result["imu"]["accel_norm_median_m_s2"] <= 10.8:
        failures.append("stationary_accel")
    result["status"] = "PASS" if not failures else "FAIL"
    result["failures"] = failures

    rendered = json.dumps(result, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as stream:
            stream.write(rendered + "\n")


if __name__ == "__main__":
    main()
