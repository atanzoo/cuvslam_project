#!/usr/bin/env python3
"""Run an isolated Foxy FollowPath action using a saved real Nav2 path."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node


def wait(node: Node, future, timeout_s: float, label: str):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    if not future.done():
        raise TimeoutError(f"{label} timed out after {timeout_s:.1f}s")
    result = future.result()
    if result is None:
        raise RuntimeError(f"{label} returned no result")
    return result


def load_path(path_file: Path, scene_id: str, node: Node) -> NavPath:
    evidence = json.loads(path_file.read_text(encoding="utf-8"))
    if evidence.get("scene_id") != scene_id:
        raise ValueError("path scene_id does not match the bridge scene")
    points = evidence.get("points")
    if not isinstance(points, list) or len(points) < 2:
        raise ValueError("path must contain at least two points")
    stamp = node.get_clock().now().to_msg()
    path = NavPath()
    path.header.frame_id = "map"
    path.header.stamp = stamp
    previous_yaw = 0.0
    for index, point in enumerate(points):
        if not isinstance(point, list) or len(point) != 2:
            raise ValueError(f"invalid point at index {index}")
        x, y = (float(value) for value in point)
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError(f"non-finite point at index {index}")
        if index + 1 < len(points):
            next_x, next_y = (float(value) for value in points[index + 1])
            dx, dy = next_x - x, next_y - y
            if math.hypot(dx, dy) > 1e-9:
                previous_yaw = math.atan2(dy, dx)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = stamp
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(previous_yaw * 0.5)
        pose.pose.orientation.w = math.cos(previous_yaw * 0.5)
        path.poses.append(pose)
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--path", type=Path, required=True)
    parser.add_argument("--scene-id", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--server-timeout", type=float, default=20.0)
    parser.add_argument("--result-timeout", type=float, default=80.0)
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise FileExistsError(f"preserving existing gate result: {output}")
    if min(args.server_timeout, args.result_timeout) <= 0.0:
        raise ValueError("timeouts must be positive")

    rclpy.init()
    node = Node("map3_follow_path_gate")
    client = ActionClient(node, FollowPath, "follow_path")
    goal_handle = None
    evidence = {
        "status": "not_started",
        "scene_id": args.scene_id,
        "action": "follow_path",
        "controller_id": "FollowPath",
        "result_status": None,
        "cancel_requested": False,
        "path_file": str(args.path),
    }
    try:
        path = load_path(args.path, args.scene_id, node)
        evidence["path_point_count"] = len(path.poses)
        evidence["start_xy"] = [path.poses[0].pose.position.x, path.poses[0].pose.position.y]
        evidence["goal_xy"] = [path.poses[-1].pose.position.x, path.poses[-1].pose.position.y]
        if not client.wait_for_server(timeout_sec=args.server_timeout):
            raise TimeoutError("Nav2 follow_path action server is unavailable")
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = "FollowPath"
        response = wait(node, client.send_goal_async(goal), args.server_timeout, "goal acceptance")
        if not response.accepted:
            evidence["status"] = "rejected"
            raise RuntimeError("Nav2 rejected the FollowPath goal")
        goal_handle = response
        evidence["status"] = "accepted"
        node.get_logger().info("FollowPath accepted; model command remains bridge-disarmed until MuJoCo pose loop arms it")
        wrapped = wait(node, goal_handle.get_result_async(), args.result_timeout, "FollowPath result")
        evidence["result_status"] = int(wrapped.status)
        evidence["status"] = (
            "succeeded" if wrapped.status == GoalStatus.STATUS_SUCCEEDED
            else "aborted" if wrapped.status == GoalStatus.STATUS_ABORTED
            else "canceled" if wrapped.status == GoalStatus.STATUS_CANCELED
            else "finished_with_non_success_status"
        )
    except KeyboardInterrupt:
        evidence["status"] = "interrupted"
        if goal_handle is not None:
            evidence["cancel_requested"] = True
            try:
                wait(node, goal_handle.cancel_goal_async(), 5.0, "goal cancellation")
            except Exception as error:  # preserve the cancellation failure in evidence
                evidence["cancel_error"] = str(error)
    except Exception as error:
        evidence["error"] = f"{type(error).__name__}: {error}"
        if evidence["status"] not in ("rejected",):
            evidence["status"] = "failed"
        if goal_handle is not None:
            evidence["cancel_requested"] = True
            try:
                wait(node, goal_handle.cancel_goal_async(), 5.0, "goal cancellation")
            except Exception as cancel_error:
                evidence["cancel_error"] = str(cancel_error)
        raise
    finally:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        node.get_logger().info("FOLLOW_PATH_GATE_RESULT=" + json.dumps(evidence, sort_keys=True))
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
