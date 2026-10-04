#!/usr/bin/env python3
"""Request and preserve a Nav2 ComputePathToPose result across ROS versions."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose


def _await(node: Node, future, timeout: float, label: str):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise TimeoutError(f"{label} did not finish within {timeout} s")
    return future.result()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--goal-x", type=float, required=True)
    parser.add_argument("--goal-y", type=float, required=True)
    parser.add_argument("--scene-id", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()
    if not all(math.isfinite(value) for value in (args.goal_x, args.goal_y, args.timeout)):
        raise ValueError("goal and timeout must be finite")
    if args.timeout <= 0.0:
        raise ValueError("timeout must be positive")
    output = args.output.resolve()
    if output.exists():
        raise FileExistsError(f"evidence already exists: {output}")

    rclpy.init()
    node = Node("map3_path_probe")
    client = None
    try:
        client = ActionClient(node, ComputePathToPose, "compute_path_to_pose")
        if not client.wait_for_server(timeout_sec=args.timeout):
            raise TimeoutError("real Nav2 compute_path_to_pose action is unavailable")
        goal = ComputePathToPose.Goal()
        goal_pose = getattr(goal, "goal", None)
        if goal_pose is None:
            goal_pose = getattr(goal, "pose", None)
        if goal_pose is None:
            raise RuntimeError("ComputePathToPose action has no supported goal pose field")
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = node.get_clock().now().to_msg()
        goal_pose.pose.position.x = args.goal_x
        goal_pose.pose.position.y = args.goal_y
        goal_pose.pose.orientation.w = 1.0
        goal.planner_id = "GridBased"
        goal_handle = _await(node, client.send_goal_async(goal), args.timeout, "goal acceptance")
        if not goal_handle.accepted:
            raise RuntimeError("Nav2 planner rejected the goal")
        wrapped = _await(node, goal_handle.get_result_async(), args.timeout, "path result")
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"Nav2 planner finished with status {wrapped.status}")
        path = wrapped.result.path
        if path.header.frame_id != "map" or len(path.poses) < 2:
            raise RuntimeError("Nav2 returned no usable map-frame path")
        points = [
            [float(item.pose.position.x), float(item.pose.position.y)]
            for item in path.poses
        ]
        if not all(math.isfinite(value) for point in points for value in point):
            raise RuntimeError("Nav2 returned a non-finite path")
        evidence = {
            "status": "real_nav2_planner_path_unverified_geometry",
            "scene_id": args.scene_id,
            "action": "compute_path_to_pose",
            "planner_id": goal.planner_id,
            "frame_id": path.header.frame_id,
            "goal": [args.goal_x, args.goal_y],
            "planning_time_sec": (
                float(wrapped.result.planning_time.sec)
                + float(wrapped.result.planning_time.nanosec) * 1e-9
            ),
            "points": points,
        }
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps({
            "output": str(output),
            "scene_id": args.scene_id,
            "point_count": len(points),
            "planning_time_sec": evidence["planning_time_sec"],
        }, sort_keys=True))
    finally:
        if client is not None:
            client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
