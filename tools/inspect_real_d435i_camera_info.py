#!/usr/bin/env python3
"""Print the live RealSense stereo CameraInfo contract without ros2 CLI."""

from __future__ import annotations

import json
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo


def main() -> None:
    rclpy.init()
    node = rclpy.create_node("real_d435i_camera_info_inspector")
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
    received: dict[str, CameraInfo] = {}

    def callback(topic: str):
        def receive(message: CameraInfo) -> None:
            received[topic] = message

        return receive

    node.create_subscription(CameraInfo, "/camera/infra1/camera_info", callback("left"), qos)
    node.create_subscription(CameraInfo, "/camera/infra2/camera_info", callback("right"), qos)
    deadline = time.monotonic() + 10.0
    while len(received) < 2 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    output = {}
    for name, message in received.items():
        output[name] = {
            "width": message.width,
            "height": message.height,
            "distortion_model": message.distortion_model,
            "frame_id": message.header.frame_id,
            "k": list(message.k),
            "r": list(message.r),
            "p": list(message.p),
            "baseline_m": (-message.p[3] / message.p[0]) if name == "right" and message.p[0] else None,
        }
    print(json.dumps(output, indent=2), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
