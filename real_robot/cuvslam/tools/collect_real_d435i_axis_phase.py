#!/usr/bin/env python3
"""Collect one manually controlled IMU axis phase."""

from __future__ import annotations

import argparse
import json
import math
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


def stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", required=True)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--live-period", type=float, default=0.25)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("real_d435i_axis_phase_collector")
    qos = QoSProfile(
        depth=50,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    samples: list[tuple[int, tuple[float, float, float]]] = []

    def callback(message):
        samples.append(
            (
                stamp_ns(message),
                (
                    float(message.angular_velocity.x),
                    float(message.angular_velocity.y),
                    float(message.angular_velocity.z),
                ),
            )
        )

    node.create_subscription(Imu, "/camera/imu", callback, qos)
    print(
        f"PHASE {args.label}: begin now; duration={args.duration:.1f}s",
        flush=True,
    )
    start = time.monotonic()
    last_live = start
    live_peaks = [0.0, 0.0, 0.0]
    while time.monotonic() - start < args.duration:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.monotonic()
        if samples and now - last_live >= args.live_period:
            current = samples[-1][1]
            for axis in range(3):
                live_peaks[axis] = max(live_peaks[axis], abs(current[axis]))
            print(
                "LIVE "
                + json.dumps(
                    {
                        "elapsed_s": now - start,
                        "gyro_rad_s": current,
                        "peak_abs_gyro_rad_s": live_peaks,
                        "dominant_axis": max(
                            range(3), key=lambda axis: live_peaks[axis]
                        ),
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
            last_live = now

    integrals = [0.0, 0.0, 0.0]
    peaks = [0.0, 0.0, 0.0]
    for (first_stamp, first), (second_stamp, second) in zip(
        samples, samples[1:]
    ):
        dt = (second_stamp - first_stamp) / 1e9
        if 0.0 < dt < 0.1:
            for axis in range(3):
                integrals[axis] += 0.5 * (first[axis] + second[axis]) * dt
    for _, values in samples:
        for axis in range(3):
            peaks[axis] = max(peaks[axis], abs(values[axis]))

    result = {
        "label": args.label,
        "duration_s": time.monotonic() - start,
        "samples": len(samples),
        "integrated_gyro_rad": integrals,
        "peak_abs_gyro_rad_s": peaks,
        "dominant_axis": max(range(3), key=lambda axis: peaks[axis]) if samples else None,
    }
    print(json.dumps(result, sort_keys=True), flush=True)
    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
