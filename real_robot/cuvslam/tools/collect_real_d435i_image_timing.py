#!/usr/bin/env python3
"""Measure RealSense image arrival and header timestamp continuity."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def summary(values: list[float]) -> dict[str, float | int]:
    if not values:
        return {
            "samples": 0,
            "median_ms": math.nan,
            "p95_ms": math.nan,
            "max_ms": math.nan,
            "over_40ms": 0,
            "over_100ms": 0,
        }
    ordered = sorted(values)
    p95_index = min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)
    return {
        "samples": len(values) + 1,
        "median_ms": statistics.median(values),
        "p95_ms": ordered[p95_index],
        "max_ms": max(values),
        "over_40ms": sum(value > 40.0 for value in values),
        "over_100ms": sum(value > 100.0 for value in values),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("real_d435i_image_timing_collector")
    qos = QoSProfile(
        depth=20,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    arrival_stamps: list[float] = []
    header_stamps: list[int] = []

    def callback(message: Image) -> None:
        arrival_stamps.append(time.monotonic())
        header_stamps.append(
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )

    node.create_subscription(Image, "/camera/infra1/image_rect_raw", callback, qos)
    start = time.monotonic()
    while time.monotonic() - start < args.duration:
        rclpy.spin_once(node, timeout_sec=0.05)

    arrival_ms = [
        (b - a) * 1000.0 for a, b in zip(arrival_stamps, arrival_stamps[1:])
    ]
    header_ms = [
        (b - a) / 1_000_000.0 for a, b in zip(header_stamps, header_stamps[1:])
    ]
    output = {
        "duration_s": time.monotonic() - start,
        "message_count": len(header_stamps),
        "arrival": summary(arrival_ms),
        "header": summary(header_ms),
        "header_non_monotonic": sum(value <= 0.0 for value in header_ms),
    }
    print(json.dumps(output, indent=2, allow_nan=True), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
