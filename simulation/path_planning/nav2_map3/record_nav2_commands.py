#!/usr/bin/env python3
"""Read-only, bounded command evidence for the isolated Nav2 proxy gates."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
import time


def summarize(samples: list[dict], limits=(0.30, 0.02, 0.70)) -> dict:
    maxima = [0.0, 0.0, 0.0]
    nonfinite = 0
    overspeed = 0
    for sample in samples:
        values = tuple(float(value) for value in sample["velocity_body"])
        if len(values) != 3 or not all(math.isfinite(value) for value in values):
            nonfinite += 1
            continue
        maxima = [max(previous, abs(value)) for previous, value in zip(maxima, values)]
        overspeed += int(any(abs(value) > limit + 1e-9 for value, limit in zip(values, limits)))
    intervals = [
        float(second["receipt_elapsed_s"]) - float(first["receipt_elapsed_s"])
        for first, second in zip(samples, samples[1:])
    ]
    return {
        "sample_count": len(samples),
        "max_abs_velocity_body": maxima,
        "nonfinite_count": nonfinite,
        "overspeed_count": overspeed,
        "command_bounds_pass": bool(samples) and nonfinite == 0 and overspeed == 0,
        "receipt_interval_median_s": statistics.median(intervals) if intervals else None,
        "receipt_interval_max_s": max(intervals) if intervals else None,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scene-id", required=True)
    parser.add_argument("--duration-s", type=float, default=80.0)
    parser.add_argument("--stop-on-goal-terminal", action="store_true")
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f"preserving command evidence: {args.output}")
    if not args.scene_id or not math.isfinite(args.duration_s) or not 0 < args.duration_s <= 120:
        raise ValueError("scene identity and duration in (0, 120] seconds are required")

    import rclpy
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from geometry_msgs.msg import Twist
    from rclpy.node import Node

    rclpy.init()
    node = Node("map3_command_recorder")
    started = time.monotonic()
    samples: list[dict] = []
    goal_uuid = None
    terminal_status = None
    terminal_deadline = None
    completion = "duration_elapsed"

    def receive_command(message: Twist) -> None:
        values = [message.linear.x, message.linear.y, message.angular.z]
        samples.append({
            "receipt_elapsed_s": time.monotonic() - started,
            "ros_receipt_nanoseconds": node.get_clock().now().nanoseconds,
            "velocity_body": [value if math.isfinite(value) else repr(value) for value in values],
        })

    def receive_status(message: GoalStatusArray) -> None:
        nonlocal goal_uuid, terminal_status, terminal_deadline
        for item in message.status_list:
            identity = bytes(item.goal_info.goal_id.uuid).hex()
            if item.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING):
                if goal_uuid is None:
                    goal_uuid = identity
            elif identity == goal_uuid and item.status in (
                GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED,
            ):
                terminal_status = int(item.status)
                if terminal_deadline is None:
                    terminal_deadline = time.monotonic() + 0.30

    node.create_subscription(Twist, "/cmd_vel", receive_command, 10)
    if args.stop_on_goal_terminal:
        node.create_subscription(GoalStatusArray, "/follow_path/_action/status", receive_status, 10)
    print(json.dumps({"recorder": "ready", "scene_id": args.scene_id}), flush=True)
    try:
        while time.monotonic() - started < args.duration_s:
            rclpy.spin_once(node, timeout_sec=0.05)
            if terminal_deadline is not None and time.monotonic() >= terminal_deadline:
                completion = "observed_goal_terminal"
                break
    except KeyboardInterrupt:
        completion = "interrupted"
    finally:
        evidence = {
            "scene_id": args.scene_id,
            "topic": "/cmd_vel",
            "frame": "base_link_body_twist",
            "timestamps": "receipt times; Twist has no source header",
            "hard_limits_body": [0.30, 0.02, 0.70],
            "duration_requested_s": args.duration_s,
            "duration_observed_s": time.monotonic() - started,
            "completion_reason": completion,
            "observed_goal_uuid": goal_uuid,
            "nav2_terminal_status": terminal_status,
            **summarize(samples),
            "samples": samples,
        }
        node.destroy_node()
        rclpy.shutdown()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as stream:
            json.dump(evidence, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
        print(json.dumps({k: v for k, v in evidence.items() if k != "samples"}), flush=True)


if __name__ == "__main__":
    main()
