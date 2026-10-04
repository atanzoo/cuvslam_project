#!/usr/bin/env python3
"""Relay only the slam_bot pose from Gazebo's large Pose_V ROS conversion."""

from __future__ import annotations

import argparse
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class NativePoseRelay(Node):
    def __init__(self, expected_start: tuple[float, float, float]) -> None:
        super().__init__("simulation_native_pose_relay")
        self.expected_start = expected_start
        self.transform_index: int | None = None
        self.publisher = self.create_publisher(
            PoseStamped, "/simulation/native_pose", 10
        )
        self.create_subscription(
            TFMessage,
            "/simulation/native_pose_raw",
            self._on_raw_pose,
            10,
        )

    def _on_raw_pose(self, message: TFMessage) -> None:
        if self.transform_index is None:
            if not message.transforms:
                return
            candidates = []
            for index, item in enumerate(message.transforms):
                value = item.transform.translation
                position = (float(value.x), float(value.y), float(value.z))
                candidates.append((math.dist(position, self.expected_start), index))
            distance, index = min(candidates)
            if distance > 0.15:
                return
            self.transform_index = index
            self.get_logger().info(
                f"locked slam_bot native pose at Pose_V index {index}"
            )
        if self.transform_index >= len(message.transforms):
            self.transform_index = None
            return
        source = message.transforms[self.transform_index]
        output = PoseStamped()
        output.header = source.header
        output.header.frame_id = "gazebo_world"
        output.pose.position.x = source.transform.translation.x
        output.pose.position.y = source.transform.translation.y
        output.pose.position.z = source.transform.translation.z
        output.pose.orientation = source.transform.rotation
        self.publisher.publish(output)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--expected-start-x", type=float, default=-1.8)
    parser.add_argument("--expected-start-y", type=float, default=-1.8)
    parser.add_argument("--expected-start-z", type=float, default=0.06)
    args = parser.parse_args()
    rclpy.init()
    node = NativePoseRelay(
        (args.expected_start_x, args.expected_start_y, args.expected_start_z)
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
