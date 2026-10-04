#!/usr/bin/env python3
"""Measure simulated stereo cadence, pairing, image quality, and motion flow."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import time
from typing import Sequence


@dataclass(frozen=True)
class FrameSample:
    stamp_ns: int
    arrival_ns: int
    image: object
    linear_command: float
    angular_command: float


def percentile(values: Sequence[float], percentage: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    position = (len(ordered) - 1) * percentage / 100.0
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return float(ordered[lower])
    weight = position - lower
    return float(
        ordered[lower] * (1.0 - weight) + ordered[upper] * weight
    )


def intervals_ms(timestamps_ns: Sequence[int]) -> list[float]:
    return [
        (current - previous) / 1_000_000.0
        for previous, current in zip(timestamps_ns, timestamps_ns[1:])
    ]


def effective_rate_hz(timestamps_ns: Sequence[int]) -> float:
    if len(timestamps_ns) < 2:
        return 0.0
    duration_s = (timestamps_ns[-1] - timestamps_ns[0]) / 1_000_000_000.0
    return (len(timestamps_ns) - 1) / duration_s if duration_s > 0.0 else 0.0


def pairing_metrics(
    left_timestamps: Sequence[int],
    right_timestamps: Sequence[int],
) -> tuple[float, float, int]:
    left = set(left_timestamps)
    right = set(right_timestamps)
    denominator = max(len(left), len(right), 1)
    exact_fraction = len(left & right) / denominator
    if not left or not right:
        return exact_fraction, float("inf"), denominator
    right_sorted = sorted(right)
    left_sorted = sorted(left)
    unmatched_left = [
        timestamp
        for timestamp in left
        if timestamp not in right
        and right_sorted[0] <= timestamp <= right_sorted[-1]
    ]
    unmatched_right = [
        timestamp
        for timestamp in right
        if timestamp not in left
        and left_sorted[0] <= timestamp <= left_sorted[-1]
    ]
    max_skew_ns = 0
    for timestamp in unmatched_left:
        nearest = min(right_sorted, key=lambda value: abs(value - timestamp))
        max_skew_ns = max(max_skew_ns, abs(nearest - timestamp))
    for timestamp in unmatched_right:
        nearest = min(left_sorted, key=lambda value: abs(value - timestamp))
        max_skew_ns = max(max_skew_ns, abs(nearest - timestamp))
    return (
        exact_fraction,
        max_skew_ns / 1_000_000.0,
        len(unmatched_left) + len(unmatched_right),
    )


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


def flow_metrics(samples: Sequence[FrameSample], cv2, np) -> dict:
    all_flows = []
    moving_flows = []
    tracked_points = 0
    moving_pairs = 0
    for previous, current in zip(samples, samples[1:]):
        points = cv2.goodFeaturesToTrack(
            previous.image,
            maxCorners=300,
            qualityLevel=0.01,
            minDistance=7,
            blockSize=7,
        )
        if points is None:
            continue
        tracked, status, _ = cv2.calcOpticalFlowPyrLK(
            previous.image,
            current.image,
            points,
            None,
            winSize=(21, 21),
            maxLevel=3,
        )
        if tracked is None or status is None:
            continue
        valid = status.reshape(-1).astype(bool)
        if not np.any(valid):
            continue
        displacement = np.linalg.norm(
            tracked.reshape(-1, 2)[valid] - points.reshape(-1, 2)[valid],
            axis=1,
        )
        values = [float(value) for value in displacement]
        tracked_points += len(values)
        all_flows.extend(values)
        moving = (
            abs(previous.linear_command) > 0.01
            or abs(previous.angular_command) > 0.01
            or abs(current.linear_command) > 0.01
            or abs(current.angular_command) > 0.01
        )
        if moving:
            moving_pairs += 1
            moving_flows.extend(values)
    selected = moving_flows if moving_flows else all_flows
    return {
        "tracked_points": tracked_points,
        "moving_frame_pairs": moving_pairs,
        "median_px": percentile(selected, 50.0),
        "p95_px": percentile(selected, 95.0),
        "maximum_px": max(selected, default=float("nan")),
    }


def stream_metrics(samples: Sequence[FrameSample], np) -> dict:
    stamps = [sample.stamp_ns for sample in samples]
    arrivals = [sample.arrival_ns for sample in samples]
    stamp_intervals = intervals_ms(stamps)
    arrival_intervals = intervals_ms(arrivals)
    means = [float(np.mean(sample.image)) for sample in samples]
    deviations = [float(np.std(sample.image)) for sample in samples]
    dark = [
        float(np.mean(sample.image <= 5))
        for sample in samples
    ]
    bright = [
        float(np.mean(sample.image >= 250))
        for sample in samples
    ]
    temporal_differences = [
        float(
            np.mean(
                np.abs(
                    current.image.astype(np.int16)
                    - previous.image.astype(np.int16)
                )
            )
        )
        for previous, current in zip(samples, samples[1:])
    ]
    return {
        "frames": len(samples),
        "stamp_rate_hz": effective_rate_hz(stamps),
        "stamp_interval_mean_ms": (
            sum(stamp_intervals) / len(stamp_intervals)
            if stamp_intervals
            else float("nan")
        ),
        "stamp_interval_p95_ms": percentile(stamp_intervals, 95.0),
        "stamp_interval_max_ms": max(stamp_intervals, default=float("nan")),
        "arrival_interval_p95_ms": percentile(arrival_intervals, 95.0),
        "arrival_interval_max_ms": max(arrival_intervals, default=float("nan")),
        "intensity_mean_median": percentile(means, 50.0),
        "intensity_stddev_median": percentile(deviations, 50.0),
        "dark_fraction_median": percentile(dark, 50.0),
        "bright_fraction_median": percentile(bright, 50.0),
        "temporal_difference_p95": percentile(
            temporal_differences,
            95.0,
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--phase", choices=("static", "straight", "turn"), required=True)
    parser.add_argument("--duration", type=float, default=9.0)
    parser.add_argument("--output")
    parser.add_argument("--allow-no-cuvslam", action="store_true")
    parser.add_argument("--expected-rate", type=float, default=30.0)
    args = parser.parse_args()

    import cv2
    import numpy as np
    import rclpy
    from geometry_msgs.msg import Twist
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image

    rclpy.init()
    node = Node(f"sim_image_timing_{args.phase}")
    command = [0.0, 0.0]
    frames = {"left": [], "right": []}
    clocks = []
    status_track_times = []

    def receive_command(message: Twist) -> None:
        command[0] = float(message.linear.x)
        command[1] = float(message.angular.z)

    def receive_image(label):
        def callback(message):
            frames[label].append(
                FrameSample(
                    stamp_ns=stamp_ns(message),
                    arrival_ns=time.monotonic_ns(),
                    image=image_from_message(message, np),
                    linear_command=command[0],
                    angular_command=command[1],
                )
            )

        return callback

    def receive_clock(message: Clock) -> None:
        sim_ns = (
            int(message.clock.sec) * 1_000_000_000
            + int(message.clock.nanosec)
        )
        clocks.append((time.monotonic_ns(), sim_ns))

    def receive_status(message: VisualSlamStatus) -> None:
        status_track_times.append(
            (
                int(message.vo_state),
                float(message.track_execution_time),
                float(message.node_callback_execution_time),
            )
        )

    subscriptions = [
        node.create_subscription(
            Image,
            "/d435i/infra1/image_rect_raw",
            receive_image("left"),
            qos_profile_sensor_data,
        ),
        node.create_subscription(
            Image,
            "/d435i/infra2/image_rect_raw",
            receive_image("right"),
            qos_profile_sensor_data,
        ),
        node.create_subscription(
            Clock,
            "/clock",
            receive_clock,
            qos_profile_sensor_data,
        ),
        node.create_subscription(Twist, "/cmd_vel", receive_command, 10),
        node.create_subscription(
            VisualSlamStatus,
            "/visual_slam/status",
            receive_status,
            10,
        ),
    ]

    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    del subscriptions
    node.destroy_node()
    rclpy.shutdown()

    for values in frames.values():
        values.sort(key=lambda sample: sample.stamp_ns)
    left_metrics = stream_metrics(frames["left"], np)
    right_metrics = stream_metrics(frames["right"], np)
    (
        exact_pair_fraction,
        max_pair_skew_ms,
        interior_unmatched_frames,
    ) = pairing_metrics(
        [sample.stamp_ns for sample in frames["left"]],
        [sample.stamp_ns for sample in frames["right"]],
    )
    if len(clocks) >= 2:
        wall_duration = (clocks[-1][0] - clocks[0][0]) / 1_000_000_000.0
        sim_duration = (clocks[-1][1] - clocks[0][1]) / 1_000_000_000.0
        real_time_factor = sim_duration / wall_duration
    else:
        real_time_factor = float("nan")
    track_times = [value[1] for value in status_track_times]
    callback_times = [value[2] for value in status_track_times]
    vo_states = [value[0] for value in status_track_times]

    failures = []
    minimum_rate = args.expected_rate * 0.95
    maximum_rate = args.expected_rate * 1.05
    for label, metrics in (("left", left_metrics), ("right", right_metrics)):
        if not minimum_rate <= metrics["stamp_rate_hz"] <= maximum_rate:
            failures.append(f"{label}_rate")
        if metrics["stamp_interval_max_ms"] > 1.5 * 1000.0 / args.expected_rate:
            failures.append(f"{label}_stamp_gap")
        if not 10.0 <= metrics["intensity_mean_median"] <= 245.0:
            failures.append(f"{label}_exposure")
        if metrics["intensity_stddev_median"] < 10.0:
            failures.append(f"{label}_contrast")
    if exact_pair_fraction < 0.99:
        failures.append("pair_fraction")
    if max_pair_skew_ms > 1.0:
        failures.append("pair_skew")
    if not 0.95 <= real_time_factor <= 1.05:
        failures.append("real_time_factor")
    if not args.allow_no_cuvslam:
        if not vo_states or any(value != 1 for value in vo_states):
            failures.append("tracking_state")
        if max(track_times, default=float("inf")) >= 1.0 / args.expected_rate:
            failures.append("track_frame_budget")

    flow = flow_metrics(frames["left"], cv2, np)
    if args.phase == "static" and flow["p95_px"] > 0.25:
        failures.append("static_flow")

    result = {
        "status": "PASS" if not failures else "FAIL",
        "phase": args.phase,
        "duration_s": args.duration,
        "expected_rate_hz": args.expected_rate,
        "left": left_metrics,
        "right": right_metrics,
        "stereo": {
            "exact_pair_fraction": exact_pair_fraction,
            "max_pair_skew_ms": max_pair_skew_ms,
            "interior_unmatched_frames": interior_unmatched_frames,
        },
        "gazebo": {"real_time_factor": real_time_factor},
        "cuvslam": {
            "status_samples": len(status_track_times),
            "vo_states": sorted(set(vo_states)),
            "track_time_mean_ms": (
                1000.0 * sum(track_times) / len(track_times)
                if track_times
                else float("nan")
            ),
            "track_time_p95_ms": 1000.0 * percentile(track_times, 95.0),
            "track_time_max_ms": 1000.0 * max(
                track_times,
                default=float("nan"),
            ),
            "callback_time_max_ms": 1000.0 * max(
                callback_times,
                default=float("nan"),
            ),
        },
        "flow": flow,
        "failure_reasons": failures,
    }
    rendered = json.dumps(result, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        Path(args.output).write_text(rendered + "\n", encoding="utf-8")
    if failures:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
