#!/usr/bin/env python3
"""Measure disparity polarity and epipolar residuals in live stereo images."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import json
import math
from pathlib import Path
import time
from typing import Sequence


@dataclass(frozen=True)
class PairMetrics:
    stamp_ns: int
    keypoints_left: int
    keypoints_right: int
    matches: int
    median_disparity_px: float
    disparity_p10_px: float
    disparity_p90_px: float
    median_abs_vertical_px: float
    vertical_p95_px: float
    positive_disparity_fraction: float


def percentile(values: Sequence[float], percentage: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    position = (len(ordered) - 1) * percentage / 100.0
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def summarize_matches(
    stamp_ns: int,
    keypoints_left: int,
    keypoints_right: int,
    disparities: Sequence[float],
    vertical_residuals: Sequence[float],
) -> PairMetrics:
    if len(disparities) != len(vertical_residuals):
        raise ValueError("Disparity and vertical residual counts must match")
    if not disparities:
        raise ValueError("At least one stereo match is required")
    absolute_vertical = [abs(value) for value in vertical_residuals]
    return PairMetrics(
        stamp_ns=stamp_ns,
        keypoints_left=keypoints_left,
        keypoints_right=keypoints_right,
        matches=len(disparities),
        median_disparity_px=percentile(disparities, 50.0),
        disparity_p10_px=percentile(disparities, 10.0),
        disparity_p90_px=percentile(disparities, 90.0),
        median_abs_vertical_px=percentile(absolute_vertical, 50.0),
        vertical_p95_px=percentile(absolute_vertical, 95.0),
        positive_disparity_fraction=(
            sum(value > 0.0 for value in disparities) / len(disparities)
        ),
    )


def mutual_ratio_matches(
    descriptors_left,
    descriptors_right,
    norm_type,
    cv2,
):
    matcher = cv2.BFMatcher(norm_type)

    def accepted(query, train):
        result = {}
        for candidates in matcher.knnMatch(query, train, k=2):
            if len(candidates) != 2:
                continue
            best, second = candidates
            if best.distance < 0.75 * second.distance:
                result[best.queryIdx] = best.trainIdx
        return result

    forward = accepted(descriptors_left, descriptors_right)
    reverse = accepted(descriptors_right, descriptors_left)
    return [
        (left_index, right_index)
        for left_index, right_index in forward.items()
        if reverse.get(right_index) == left_index
    ]


def analyze_pair(left, right, stamp_ns: int, detector_name: str, cv2):
    if detector_name == "orb":
        detector = cv2.ORB_create(
            nfeatures=1200,
            scaleFactor=1.2,
            nlevels=8,
            edgeThreshold=12,
            fastThreshold=7,
        )
        norm_type = cv2.NORM_HAMMING
    elif detector_name == "kaze":
        detector = cv2.KAZE_create()
        norm_type = cv2.NORM_L2
    else:
        raise ValueError(f"Unsupported detector: {detector_name}")
    keypoints_left, descriptors_left = detector.detectAndCompute(left, None)
    keypoints_right, descriptors_right = detector.detectAndCompute(right, None)
    if descriptors_left is None or descriptors_right is None:
        raise ValueError("ORB found no descriptors in one or both images")
    match_indices = mutual_ratio_matches(
        descriptors_left,
        descriptors_right,
        norm_type,
        cv2,
    )
    disparities = []
    vertical_residuals = []
    matches = []
    for left_index, right_index in match_indices:
        left_point = keypoints_left[left_index].pt
        right_point = keypoints_right[right_index].pt
        disparities.append(left_point[0] - right_point[0])
        vertical_residuals.append(left_point[1] - right_point[1])
        matches.append(
            cv2.DMatch(
                _queryIdx=left_index,
                _trainIdx=right_index,
                _distance=0.0,
            )
        )
    metrics = summarize_matches(
        stamp_ns,
        len(keypoints_left),
        len(keypoints_right),
        disparities,
        vertical_residuals,
    )
    return metrics, keypoints_left, keypoints_right, matches


def image_from_message(message, np):
    if message.encoding not in ("mono8", "8UC1"):
        raise ValueError(f"Unsupported encoding: {message.encoding}")
    rows = np.frombuffer(message.data, dtype=np.uint8).reshape(
        message.height,
        message.step,
    )
    return rows[:, : message.width].copy()


def stamp_ns(message) -> int:
    return (
        int(message.header.stamp.sec) * 1_000_000_000
        + int(message.header.stamp.nanosec)
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--left-topic",
        default="/d435i/infra1/image_rect_raw",
    )
    parser.add_argument(
        "--right-topic",
        default="/d435i/infra2/image_rect_raw",
    )
    parser.add_argument("--pairs", type=int, default=20)
    parser.add_argument(
        "--detector",
        choices=("orb", "kaze"),
        default="orb",
    )
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--minimum-matches", type=int, default=15)
    parser.add_argument("--maximum-vertical-p95", type=float, default=1.0)
    parser.add_argument(
        "--minimum-positive-disparity-fraction",
        type=float,
        default=0.90,
    )
    parser.add_argument("--output-dir")
    args = parser.parse_args()

    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    if args.pairs <= 0:
        raise ValueError("--pairs must be positive")

    rclpy.init()
    node = Node("stereo_geometry_validator")
    queues = {"left": {}, "right": {}}
    pending = deque()

    def receive(label):
        def callback(message):
            timestamp = stamp_ns(message)
            queues[label][timestamp] = image_from_message(message, np)
            other = "right" if label == "left" else "left"
            if timestamp in queues[other]:
                pending.append(
                    (
                        timestamp,
                        queues["left"].pop(timestamp),
                        queues["right"].pop(timestamp),
                    )
                )
            for queued_timestamp in list(queues[label])[:-10]:
                queues[label].pop(queued_timestamp, None)

        return callback

    subscriptions = [
        node.create_subscription(
            Image,
            args.left_topic,
            receive("left"),
            qos_profile_sensor_data,
        ),
        node.create_subscription(
            Image,
            args.right_topic,
            receive("right"),
            qos_profile_sensor_data,
        ),
    ]

    metrics = []
    annotated = None
    first_left = None
    first_right = None
    deadline = time.monotonic() + args.timeout
    analysis_failures = 0
    while len(metrics) < args.pairs and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        while pending and len(metrics) < args.pairs:
            timestamp, left, right = pending.popleft()
            try:
                result = analyze_pair(
                    left,
                    right,
                    timestamp,
                    args.detector,
                    cv2,
                )
            except ValueError as exc:
                analysis_failures += 1
                print(f"WARN stamp_ns={timestamp} reason={exc}")
                continue
            pair_metrics, left_points, right_points, matches = result
            metrics.append(pair_metrics)
            if annotated is None:
                first_left = left.copy()
                first_right = right.copy()
                annotated = cv2.drawMatches(
                    left,
                    left_points,
                    right,
                    right_points,
                    matches[:100],
                    None,
                    flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS,
                )
            print(json.dumps(pair_metrics.__dict__, sort_keys=True))

    del subscriptions
    node.destroy_node()
    rclpy.shutdown()

    if args.output_dir and annotated is not None:
        output_dir = Path(args.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(output_dir / "left.png"), first_left)
        cv2.imwrite(str(output_dir / "right.png"), first_right)
        cv2.imwrite(str(output_dir / "stereo_matches.png"), annotated)

    valid_pairs = [
        item
        for item in metrics
        if item.matches >= args.minimum_matches
    ]
    aggregate_vertical = (
        max(item.vertical_p95_px for item in metrics)
        if metrics
        else float("inf")
    )
    aggregate_positive = (
        min(item.positive_disparity_fraction for item in metrics)
        if metrics
        else 0.0
    )
    required_pairs = min(10, args.pairs)
    failure_reasons = []
    if len(valid_pairs) < required_pairs:
        failure_reasons.append("insufficient_matches")
    if aggregate_vertical > args.maximum_vertical_p95:
        failure_reasons.append("vertical_residual")
    if aggregate_positive < args.minimum_positive_disparity_fraction:
        failure_reasons.append("disparity_polarity")
    failures = len(failure_reasons)

    summary = {
        "status": "PASS" if failures == 0 else "FAIL",
        "detector": args.detector,
        "requested_pairs": args.pairs,
        "analyzed_pairs": len(metrics),
        "valid_pairs": len(valid_pairs),
        "analysis_failures": analysis_failures,
        "minimum_matches": min(
            (item.matches for item in metrics),
            default=0,
        ),
        "median_matches": percentile(
            [item.matches for item in metrics],
            50.0,
        ),
        "median_disparity_px": percentile(
            [item.median_disparity_px for item in metrics],
            50.0,
        ),
        "maximum_vertical_p95_px": aggregate_vertical,
        "minimum_positive_disparity_fraction": aggregate_positive,
        "failure_reasons": failure_reasons,
        "failures": failures,
    }
    print("SUMMARY " + json.dumps(summary, sort_keys=True))
    if failures:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
