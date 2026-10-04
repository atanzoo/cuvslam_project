#!/usr/bin/env python3
"""Validate odometry messages before they enter the fused-odom shadow filter.

The guard intentionally does not invent covariance values.  A message with a
zero, non-finite, or negative covariance in a field selected by the EKF is
rejected and reported on a diagnostic topic.  This keeps an invalid source
from looking artificially certain while its message contract is calibrated.
"""

from __future__ import annotations

import argparse
import copy
import math
import time
from types import SimpleNamespace


def _stamp_ns(message) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def validate_message(
    message,
    *,
    required_twist_indices: tuple[int, ...],
    expected_frame: str = "odom",
    expected_child_frame: str = "base_link",
    last_stamp_ns: int | None = None,
) -> tuple[bool, str]:
    """Return whether the selected odometry fields satisfy the input contract."""

    frame_id = str(message.header.frame_id or "")
    child_frame_id = str(message.child_frame_id or "")
    if frame_id != expected_frame:
        return False, f"frame_id={frame_id!r}, expected={expected_frame!r}"
    if child_frame_id != expected_child_frame:
        return False, (
            f"child_frame_id={child_frame_id!r}, "
            f"expected={expected_child_frame!r}"
        )

    stamp_ns = _stamp_ns(message)
    if stamp_ns <= 0:
        return False, "timestamp is zero"
    if last_stamp_ns is not None and stamp_ns <= last_stamp_ns:
        return False, f"timestamp is not increasing: {stamp_ns} <= {last_stamp_ns}"

    twist = message.twist.twist
    values = (twist.linear.x, twist.linear.y, twist.linear.z,
              twist.angular.x, twist.angular.y, twist.angular.z)
    for index in required_twist_indices:
        value = values[index]
        if not math.isfinite(float(value)):
            return False, f"twist[{index}] is not finite"

    covariance = list(message.twist.covariance)
    if len(covariance) < 36:
        return False, f"twist covariance has only {len(covariance)} entries"
    for index in required_twist_indices:
        value = covariance[index * 6 + index]
        if not math.isfinite(float(value)) or float(value) <= 0.0:
            return False, (
                f"twist covariance[{index * 6 + index}] is invalid: {value!r}"
            )

    return True, "accepted"


def _parse_indices(value: str) -> tuple[int, ...]:
    indices = tuple(int(item) for item in value.split(",") if item.strip())
    if not indices or any(index < 0 or index > 5 for index in indices):
        raise argparse.ArgumentTypeError("indices must be a non-empty list from 0 to 5")
    return indices


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True)
    parser.add_argument("--input", dest="input_topic", required=True)
    parser.add_argument("--output", dest="output_topic", required=True)
    parser.add_argument("--required-twist", type=_parse_indices, required=True)
    parser.add_argument("--expected-frame", default="odom")
    parser.add_argument("--expected-child-frame", default="base_link")
    return parser


def main() -> None:
    # Keep ROS imports inside main so the pure contract function remains
    # testable on the Mac without a ROS installation.
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile

    args = _build_parser().parse_args()
    rclpy.init()
    node = rclpy.create_node(f"odom_contract_guard_{args.source}")
    qos = QoSProfile(depth=20)
    publisher = node.create_publisher(Odometry, args.output_topic, qos)
    diagnostics = node.create_publisher(
        DiagnosticArray, "/fusion/odom_contract_diagnostics", qos
    )
    state = {"last_stamp_ns": None, "last_diag_monotonic": 0.0}

    def publish_diagnostic(level: int, message: str, accepted: bool) -> None:
        now = time.monotonic()
        if accepted and now - state["last_diag_monotonic"] < 1.0:
            return
        if not accepted and now - state["last_diag_monotonic"] < 0.25:
            return
        state["last_diag_monotonic"] = now
        status = DiagnosticStatus()
        status.level = level
        status.name = f"odom_contract_guard/{args.source}"
        status.hardware_id = args.source
        status.message = message
        status.values = [
            KeyValue(key="source", value=args.source),
            KeyValue(key="input_topic", value=args.input_topic),
            KeyValue(key="output_topic", value=args.output_topic),
            KeyValue(key="required_twist_indices", value=str(args.required_twist)),
            KeyValue(key="accepted", value=str(accepted).lower()),
        ]
        array = DiagnosticArray()
        array.header.stamp = node.get_clock().now().to_msg()
        array.status = [status]
        diagnostics.publish(array)

    def callback(message: Odometry) -> None:
        valid, reason = validate_message(
            message,
            required_twist_indices=args.required_twist,
            expected_frame=args.expected_frame,
            expected_child_frame=args.expected_child_frame,
            last_stamp_ns=state["last_stamp_ns"],
        )
        if not valid:
            node.get_logger().warning(f"{args.source} rejected: {reason}")
            publish_diagnostic(DiagnosticStatus.ERROR, reason, False)
            return
        state["last_stamp_ns"] = _stamp_ns(message)
        publisher.publish(copy.deepcopy(message))
        publish_diagnostic(DiagnosticStatus.OK, "accepted", True)

    node.create_subscription(Odometry, args.input_topic, callback, qos)
    node.get_logger().info(
        f"guarding {args.input_topic} -> {args.output_topic}; "
        f"required twist indices={args.required_twist}"
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
