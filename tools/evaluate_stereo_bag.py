#!/usr/bin/env python3
"""Inspect stereo calibration, image timing, and image statistics in a bag."""

from __future__ import annotations

import argparse
import bisect
import math

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


LEFT_INFO = "/d435i/infra1/camera_info"
RIGHT_INFO = "/d435i/infra2/camera_info"
ADAPTED_RIGHT_INFO = "/cuvslam/input/infra2/camera_info"
LEFT_IMAGE = "/d435i/infra1/image_rect_raw"
RIGHT_IMAGE = "/d435i/infra2/image_rect_raw"
TOPICS = (LEFT_INFO, RIGHT_INFO, ADAPTED_RIGHT_INFO, LEFT_IMAGE, RIGHT_IMAGE)


def stamp_ns(message) -> int:
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def image_statistics(message) -> tuple[float, float]:
    values = message.data
    if not values:
        return 0.0, 0.0
    count = len(values)
    mean = sum(values) / count
    variance = sum((value - mean) ** 2 for value in values) / count
    return mean, math.sqrt(variance)


def nearest_differences(left: list[int], right: list[int]) -> list[int]:
    differences = []
    for timestamp in left:
        index = bisect.bisect_left(right, timestamp)
        candidates = []
        if index < len(right):
            candidates.append(abs(right[index] - timestamp))
        if index > 0:
            candidates.append(abs(right[index - 1] - timestamp))
        if candidates:
            differences.append(min(candidates))
    return differences


def best_image_shift(left_message, right_message) -> tuple[int, int, float]:
    left = np.frombuffer(left_message.data, dtype=np.uint8).reshape(
        left_message.height, left_message.width
    )
    right = np.frombuffer(right_message.data, dtype=np.uint8).reshape(
        right_message.height, right_message.width
    )
    left = left[20:-20, 30:-30].astype(np.int16)
    right = right[20:-20, 30:-30].astype(np.int16)
    best = (0, 0, float("inf"))
    for vertical in range(-3, 4):
        if vertical > 0:
            left_rows = left[vertical:, :]
            right_rows = right[:-vertical, :]
        elif vertical < 0:
            left_rows = left[:vertical, :]
            right_rows = right[-vertical:, :]
        else:
            left_rows = left
            right_rows = right
        for horizontal in range(-32, 33):
            if horizontal > 0:
                left_crop = left_rows[:, horizontal:]
                right_crop = right_rows[:, :-horizontal]
            elif horizontal < 0:
                left_crop = left_rows[:, :horizontal]
                right_crop = right_rows[:, -horizontal:]
            else:
                left_crop = left_rows
                right_crop = right_rows
            error = float(np.mean(np.abs(left_crop - right_crop)))
            if error < best[2]:
                best = (horizontal, vertical, error)
    return best


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    args = parser.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    message_types = {topic: get_message(topic_types[topic]) for topic in TOPICS}
    camera_info = {}
    image_stamps = {LEFT_IMAGE: [], RIGHT_IMAGE: []}
    first_images = {}

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in message_types:
            continue
        message = deserialize_message(data, message_types[topic])
        if topic in (LEFT_INFO, RIGHT_INFO, ADAPTED_RIGHT_INFO):
            camera_info.setdefault(topic, message)
        else:
            image_stamps[topic].append(stamp_ns(message))
            first_images.setdefault(topic, message)

    for topic in (LEFT_INFO, RIGHT_INFO, ADAPTED_RIGHT_INFO):
        if topic not in camera_info:
            continue
        message = camera_info[topic]
        projected_baseline = (
            -message.p[3] / message.p[0] if message.p[0] != 0.0 else float("nan")
        )
        print(
            f"{topic}: frame={message.header.frame_id}, "
            f"size={message.width}x{message.height}, "
            f"model={message.distortion_model}"
        )
        print(f"  D={list(message.d)}")
        print(f"  K={list(message.k)}")
        print(f"  P={list(message.p)}")
        print(f"  projected_baseline={projected_baseline:.6f}m")

    differences = nearest_differences(
        image_stamps[LEFT_IMAGE], image_stamps[RIGHT_IMAGE]
    )
    differences_ms = [value / 1_000_000.0 for value in differences]
    print(
        "stereo_timing: "
        f"left={len(image_stamps[LEFT_IMAGE])}, "
        f"right={len(image_stamps[RIGHT_IMAGE])}, "
        f"max_nearest_delta={max(differences_ms):.6f}ms, "
        f"mean_nearest_delta={sum(differences_ms) / len(differences_ms):.6f}ms, "
        "over_threshold="
        + ",".join(
            f"{threshold}ms:{sum(value > threshold for value in differences_ms)}"
            for threshold in (1, 5, 10, 20)
        )
    )

    for topic in (LEFT_IMAGE, RIGHT_IMAGE):
        message = first_images[topic]
        mean, standard_deviation = image_statistics(message)
        print(
            f"{topic}: frame={message.header.frame_id}, "
            f"encoding={message.encoding}, step={message.step}, "
            f"mean={mean:.3f}, stddev={standard_deviation:.3f}"
        )

    horizontal, vertical, error = best_image_shift(
        first_images[LEFT_IMAGE], first_images[RIGHT_IMAGE]
    )
    print(
        "stereo_global_shift: "
        f"horizontal={horizontal}px, vertical={vertical}px, "
        f"mean_absolute_error={error:.3f}; "
        "positive horizontal means x_left > x_right"
    )


if __name__ == "__main__":
    main()
