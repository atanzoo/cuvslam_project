#!/usr/bin/env python3
"""Loopback-only MuJoCo truth/command bridge for isolated Jetson Foxy Nav2.

Simulation only. It never publishes /cmd_vel and never connects to robot I/O.
Nav2 controller_server owns /cmd_vel; the guarded HTTP response is the only
command accepted by the Mac MuJoCo proxy. Tunnel HTTP over USB SSH.
"""

from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import threading

import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from map3_bridge_core import SimCommandGuard


class Map3BridgeNode(Node):
    def __init__(self, scene_id: str, bounds: tuple[float, float, float, float]) -> None:
        super().__init__("map3_sim_truth_bridge")
        self.guard = SimCommandGuard(scene_id=scene_id, map_bounds=bounds)
        self.goal_active = False
        self.last_goal_status_code: int | None = None
        self.odom_publisher = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.command_subscription = self.create_subscription(
            Twist, "/cmd_vel", self._on_nav2_command, 10,
        )
        self.goal_subscription = self.create_subscription(
            GoalStatusArray, "/follow_path/_action/status", self._on_goal_status, 10,
        )
        self.pose_timer = self.create_timer(0.05, self._publish_fresh_pose)
        self.get_logger().info(
            "SIMULATION ONLY: bridge owns odom->base_link and /odom; "
            "subscribes to isolated /cmd_vel; never publishes robot commands"
        )

    def _on_nav2_command(self, message: Twist) -> None:
        self.guard.accept_command(
            message.linear.x, message.linear.y, message.angular.z,
        )

    def _on_goal_status(self, message: GoalStatusArray) -> None:
        active_statuses = [
            int(item.status) for item in message.status_list
            if item.status in (
                GoalStatus.STATUS_ACCEPTED,
                GoalStatus.STATUS_EXECUTING,
                GoalStatus.STATUS_CANCELING,
            )
        ]
        self.goal_active = bool(active_statuses)
        if active_statuses:
            self.last_goal_status_code = active_statuses[-1]
        elif message.status_list:
            self.last_goal_status_code = int(message.status_list[-1].status)
        if not self.goal_active:
            self.guard.disarm("follow_path_not_active")

    def _publish_fresh_pose(self) -> None:
        # Also enforce watchdogs when no HTTP client is polling.
        self.guard.read_command()
        snapshot = self.guard.pose_snapshot()
        if snapshot is None:
            return
        _, (x, y, yaw, vx, vy, wz) = snapshot
        stamp = self.get_clock().now().to_msg()
        qz, qw = math.sin(yaw * 0.5), math.cos(yaw * 0.5)
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = "odom"
        odometry.child_frame_id = "base_link"
        odometry.pose.pose.position.x = x
        odometry.pose.pose.position.y = y
        odometry.pose.pose.orientation.z = qz
        odometry.pose.pose.orientation.w = qw
        odometry.twist.twist.linear.x = vx
        odometry.twist.twist.linear.y = vy
        odometry.twist.twist.angular.z = wz
        self.odom_publisher.publish(odometry)


def handler_for(node: Map3BridgeNode):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format_string, *args):
            # A pose stream at 10-20 Hz must not bury Nav2 diagnostics.
            pass

        def respond(self, code: int, payload: dict) -> None:
            body = (json.dumps(payload, sort_keys=True, allow_nan=False) + "\n").encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:
            if self.path not in ("/health", "/command"):
                self.respond(404, {"error": "unknown_endpoint"})
                return
            state = node.guard.read_command()
            state["goal_active"] = node.goal_active
            state["last_goal_status_code"] = node.last_goal_status_code
            state["fresh_pose"] = node.guard.pose_snapshot() is not None
            self.respond(200, state)

        def do_POST(self) -> None:
            if self.path not in ("/pose", "/arm", "/disarm"):
                self.respond(404, {"error": "unknown_endpoint"})
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if size <= 0 or size > 4096:
                    raise ValueError("request body size must be 1-4096 bytes")
                payload = json.loads(self.rfile.read(size).decode("utf-8"))
                if not isinstance(payload, dict):
                    raise ValueError("request body must be a JSON object")
                if self.path == "/pose":
                    node.guard.accept_pose(payload)
                elif self.path == "/arm":
                    if not node.goal_active:
                        raise RuntimeError("cannot arm without an active FollowPath goal")
                    node.guard.arm(payload.get("scene_id"))
                else:
                    if payload.get("scene_id") != node.guard.scene_id:
                        raise ValueError("disarm scene identity mismatch")
                    node.guard.disarm("explicit_disarm")
                self.respond(200, node.guard.read_command())
            except (ValueError, RuntimeError) as error:
                node.guard.disarm("invalid_bridge_request")
                self.respond(409, {"error": str(error), "velocity_body": [0.0, 0.0, 0.0]})

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene-id", required=True)
    parser.add_argument("--map-bounds", nargs=4, type=float, required=True,
                        metavar=("X_MIN", "X_MAX", "Y_MIN", "Y_MAX"))
    parser.add_argument("--port", type=int, default=8987)
    args = parser.parse_args()
    if not 1024 <= args.port <= 65535:
        raise ValueError("port must be 1024-65535")
    rclpy.init()
    node = Map3BridgeNode(args.scene_id, tuple(args.map_bounds))
    server = ThreadingHTTPServer(("127.0.0.1", args.port), handler_for(node))
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        node.get_logger().info(f"SIMULATION ONLY: HTTP bound to 127.0.0.1:{args.port}")
        rclpy.spin(node)
    finally:
        node.guard.disarm("bridge_shutdown")
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
