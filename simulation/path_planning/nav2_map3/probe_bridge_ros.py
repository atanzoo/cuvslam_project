#!/usr/bin/env python3
"""No-daemon endpoint inventory for the isolated simulation ROS domain."""

import json
import time

import rclpy
from rclpy.node import Node


def main() -> None:
    rclpy.init()
    node = Node("map3_endpoint_probe")
    try:
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.count_publishers("/odom") > 0 and node.count_subscribers("/cmd_vel") > 0:
                break
        results = {
            topic: {
                "publishers": node.count_publishers(topic),
                "subscribers": node.count_subscribers(topic),
            }
            for topic in ("/odom", "/tf", "/tf_static", "/cmd_vel")
        }
        print(json.dumps(results, indent=2, sort_keys=True))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
