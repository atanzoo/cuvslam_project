#!/usr/bin/env python3
"""Publish and verify an isolated static sensor TF tree against SDF targets."""

from __future__ import annotations

import argparse
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

from frame_contract_math import (
    Transform,
    compose,
    inverse,
    rotation_angle_degrees,
    translation_norm,
)
from static_tf_target_contract import (
    SENSOR_FRAMES,
    candidate_edges,
    load_sdf_truth,
)


def transform_message(
    node: Node,
    parent: str,
    child: str,
    transform: Transform,
) -> TransformStamped:
    message = TransformStamped()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = parent
    message.child_frame_id = child
    message.transform.translation.x = transform.translation[0]
    message.transform.translation.y = transform.translation[1]
    message.transform.translation.z = transform.translation[2]
    message.transform.rotation.x = transform.rotation[0]
    message.transform.rotation.y = transform.rotation[1]
    message.transform.rotation.z = transform.rotation[2]
    message.transform.rotation.w = transform.rotation[3]
    return message


def transform_from_message(message: TransformStamped) -> Transform:
    translation = message.transform.translation
    rotation = message.transform.rotation
    return Transform(
        translation=(translation.x, translation.y, translation.z),
        rotation=(rotation.x, rotation.y, rotation.z, rotation.w),
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sdf", required=True)
    parser.add_argument("--translation-tolerance", type=float, default=0.01)
    parser.add_argument("--rotation-tolerance-deg", type=float, default=0.5)
    parser.add_argument("--discovery-timeout", type=float, default=5.0)
    args = parser.parse_args()

    truth = load_sdf_truth(args.sdf)
    targets = sorted(
        frame for frame in truth if frame.startswith("validation_target_")
    )
    rclpy.init()
    node = Node("static_tf_target_validator")
    broadcaster = StaticTransformBroadcaster(node)
    buffer = Buffer(cache_time=Duration(seconds=10.0))
    listener = TransformListener(buffer, node)
    messages = [
        transform_message(
            node,
            edge.parent,
            edge.child,
            edge.transform,
        )
        for edge in candidate_edges(truth)
    ]
    broadcaster.sendTransform(messages)

    deadline = time.monotonic() + args.discovery_timeout
    expected_count = len(SENSOR_FRAMES) * len(targets)
    observed = {}
    while time.monotonic() < deadline and len(observed) < expected_count:
        rclpy.spin_once(node, timeout_sec=0.05)
        for sensor in SENSOR_FRAMES:
            for target in targets:
                key = (sensor, target)
                if key in observed:
                    continue
                try:
                    message = buffer.lookup_transform(sensor, target, Time())
                except Exception:
                    continue
                observed[key] = transform_from_message(message)

    failures = 0
    max_translation_error = 0.0
    max_rotation_error = 0.0
    for sensor in SENSOR_FRAMES:
        for target in targets:
            key = (sensor, target)
            if key not in observed:
                failures += 1
                print(f"FAIL frame={sensor} target={target} reason=missing")
                continue
            expected = compose(inverse(truth[sensor]), truth[target])
            residual = compose(inverse(expected), observed[key])
            translation_error = translation_norm(residual)
            rotation_error = rotation_angle_degrees(residual)
            max_translation_error = max(
                max_translation_error,
                translation_error,
            )
            max_rotation_error = max(max_rotation_error, rotation_error)
            passed = (
                translation_error <= args.translation_tolerance
                and rotation_error <= args.rotation_tolerance_deg
            )
            failures += 0 if passed else 1
            status = "PASS" if passed else "FAIL"
            print(
                f"{status} frame={sensor} target={target} "
                f"translation_error={translation_error:.9f}m "
                f"rotation_error={rotation_error:.9f}deg"
            )

    left_key = ("camera_infra1_optical_frame", targets[0])
    right_key = ("camera_infra2_optical_frame", targets[0])
    if left_key in observed and right_key in observed:
        baseline = compose(observed[left_key], inverse(observed[right_key]))
        baseline_error = abs(translation_norm(baseline) - 0.05)
    else:
        baseline_error = float("inf")
    if baseline_error > 0.001:
        failures += 1
    print(
        "SUMMARY "
        f"comparisons={expected_count} observed={len(observed)} "
        f"max_translation_error={max_translation_error:.9f}m "
        f"max_rotation_error={max_rotation_error:.9f}deg "
        f"baseline_error={baseline_error:.9f}m "
        f"failures={failures}"
    )

    del listener
    node.destroy_node()
    rclpy.shutdown()
    if failures:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
