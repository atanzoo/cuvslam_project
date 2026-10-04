#!/usr/bin/env python3
"""Read one /odom and odom->base_link TF sample from the isolated bridge."""

import json
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage


def main() -> None:
    rclpy.init()
    node = Node("map3_bridge_message_probe")
    samples = {"odom": None, "tf": None}

    def on_odom(message: Odometry) -> None:
        samples["odom"] = {
            "frame_id": message.header.frame_id,
            "child_frame_id": message.child_frame_id,
            "stamp_sec": message.header.stamp.sec,
            "xy": [message.pose.pose.position.x, message.pose.pose.position.y],
        }

    def on_tf(message: TFMessage) -> None:
        for transform in message.transforms:
            if transform.header.frame_id == "odom" and transform.child_frame_id == "base_link":
                samples["tf"] = {
                    "parent": transform.header.frame_id,
                    "child": transform.child_frame_id,
                    "stamp_sec": transform.header.stamp.sec,
                    "xy": [transform.transform.translation.x, transform.transform.translation.y],
                }

    node.create_subscription(Odometry, "/odom", on_odom, 10)
    node.create_subscription(TFMessage, "/tf", on_tf, 10)
    try:
        deadline = time.monotonic() + 7.0
        while time.monotonic() < deadline and (samples["odom"] is None or samples["tf"] is None):
            rclpy.spin_once(node, timeout_sec=0.1)
        if samples["odom"] is None or samples["tf"] is None:
            raise TimeoutError(f"missing odom/TF bridge samples: {samples}")
        if samples["odom"]["frame_id"] != "odom" or samples["odom"]["child_frame_id"] != "base_link":
            raise AssertionError("unexpected Odometry frame ownership")
        if samples["odom"]["xy"] != samples["tf"]["xy"]:
            raise AssertionError("Odometry and TF positions differ")
        print(json.dumps(samples, sort_keys=True))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
