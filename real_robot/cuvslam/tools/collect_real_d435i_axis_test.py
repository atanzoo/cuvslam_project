#!/usr/bin/env python3
"""Run a guided six-direction D435i optical-axis gyro test.

Run this inside the same ROS 2 container as the RealSense node.  The operator
follows the phase text printed by the script; the output records dominant gyro
axis and signed integrated angle for each phase.
"""

from __future__ import annotations

import argparse
import json
import math
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


PHASES = (
    ("settle", "把 D435i 放穩不動", None),
    ("roll_clockwise", "繞鏡頭光軸 X（畫面左右方向）順時針慢轉", 0),
    ("roll_counterclockwise", "繞鏡頭光軸 X 逆時針慢轉", 0),
    ("pitch_up", "繞光學 Y 軸向上抬起慢轉", 1),
    ("pitch_down", "繞光學 Y 軸向下壓回慢轉", 1),
    ("yaw_left", "繞光學 Z 軸向左轉慢轉", 2),
    ("yaw_right", "繞光學 Z 軸向右轉慢轉", 2),
)


def stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--phase-duration", type=float, default=5.0)
    parser.add_argument("--settle-duration", type=float, default=4.0)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("real_d435i_axis_test_collector")
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
    results = []
    for name, instruction, expected_axis in PHASES:
        duration = args.settle_duration if name == "settle" else args.phase_duration
        print(f"PHASE {name}: {instruction}; duration={duration:.1f}s", flush=True)
        phase_start = time.monotonic()
        begin_index = len(samples)
        while time.monotonic() - phase_start < duration:
            rclpy.spin_once(node, timeout_sec=0.05)
        phase_samples = samples[begin_index:]
        integrals = [0.0, 0.0, 0.0]
        peaks = [0.0, 0.0, 0.0]
        for (first_stamp, first), (second_stamp, second) in zip(
            phase_samples, phase_samples[1:]
        ):
            dt = (second_stamp - first_stamp) / 1e9
            if 0.0 < dt < 0.1:
                for axis in range(3):
                    integrals[axis] += 0.5 * (first[axis] + second[axis]) * dt
        for _, values in phase_samples:
            for axis in range(3):
                peaks[axis] = max(peaks[axis], abs(values[axis]))
        dominant_axis = max(range(3), key=lambda axis: peaks[axis]) if phase_samples else None
        result = {
            "phase": name,
            "expected_axis": expected_axis,
            "samples": len(phase_samples),
            "integrated_gyro_rad": integrals,
            "peak_abs_gyro_rad_s": peaks,
            "dominant_axis": dominant_axis,
        }
        print(json.dumps(result, sort_keys=True), flush=True)
        results.append(result)

    output = {"phase_duration_s": args.phase_duration, "phases": results}
    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(output, stream, indent=2, sort_keys=True)
        stream.write("\n")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
