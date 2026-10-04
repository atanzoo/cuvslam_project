#!/usr/bin/env python3
"""Read-only diagnostic of the isolated Nav2 map server's OccupancyGrid."""

from collections import Counter
import json

import rclpy
from rclpy.node import Node
from nav_msgs.srv import GetMap


def main() -> None:
    rclpy.init()
    node = Node("map3_map_server_probe")
    try:
        client = node.create_client(GetMap, "/map_server/map")
        if not client.wait_for_service(timeout_sec=15.0):
            raise TimeoutError("Nav2 GetMap service is unavailable")
        future = client.call_async(GetMap.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)
        if not future.done() or future.result() is None:
            raise TimeoutError("Nav2 GetMap service did not respond")
        grid = future.result().map
        histogram = Counter(int(value) for value in grid.data)
        print(json.dumps({
            "status": "read_only_map_server_diagnostic",
            "frame_id": grid.header.frame_id,
            "width": grid.info.width,
            "height": grid.info.height,
            "resolution": grid.info.resolution,
            "origin_xy": [grid.info.origin.position.x, grid.info.origin.position.y],
            "occupancy_histogram": {str(key): value for key, value in sorted(histogram.items())},
        }, sort_keys=True))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
