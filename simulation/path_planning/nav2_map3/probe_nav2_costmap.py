#!/usr/bin/env python3
"""Save compact evidence and raw bytes from the isolated Nav2 global costmap."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav2_msgs.srv import GetCostmap


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--static-x", type=float, required=True)
    parser.add_argument("--static-y", type=float, required=True)
    parser.add_argument("--start-x", type=float, required=True)
    parser.add_argument("--start-y", type=float, required=True)
    parser.add_argument("--scene-id", required=True)
    parser.add_argument("--output-prefix", type=Path, required=True)
    parser.add_argument(
        "--service", choices=("/global_costmap/get_costmap", "/local_costmap/get_costmap"),
        default="/global_costmap/get_costmap",
    )
    args = parser.parse_args()
    if not all(math.isfinite(value) for value in (
        args.static_x, args.static_y, args.start_x, args.start_y,
    )):
        raise ValueError("sample coordinates must be finite")
    prefix = args.output_prefix.resolve()
    data_path = prefix.with_suffix(".bin")
    json_path = prefix.with_suffix(".json")
    if data_path.exists() or json_path.exists():
        raise FileExistsError("costmap evidence already exists")

    rclpy.init()
    node = Node("map3_costmap_probe")
    try:
        client = node.create_client(GetCostmap, args.service)
        if not client.wait_for_service(timeout_sec=15.0):
            raise TimeoutError("Nav2 global costmap service is unavailable")
        future = client.call_async(GetCostmap.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)
        if not future.done():
            raise TimeoutError("Nav2 global costmap did not respond")
        response = future.result()
        if response is None:
            raise RuntimeError("Nav2 global costmap returned no response")
        costmap = response.map
        meta = costmap.metadata
        width, height = int(meta.size_x), int(meta.size_y)
        resolution = float(meta.resolution)
        raw = bytes(costmap.data)
        if width <= 0 or height <= 0 or resolution <= 0 or len(raw) != width * height:
            raise RuntimeError("Nav2 returned an invalid global costmap")
        origin = (float(meta.origin.position.x), float(meta.origin.position.y))

        def sample(x: float, y: float) -> int:
            column = math.floor((x - origin[0]) / resolution)
            row = math.floor((y - origin[1]) / resolution)
            if not (0 <= column < width and 0 <= row < height):
                raise ValueError("sample is outside Nav2 global costmap")
            return int(raw[row * width + column])

        fixed_cost = sample(args.static_x, args.static_y)
        start_cost = sample(args.start_x, args.start_y)
        histogram = Counter(raw)
        print(json.dumps({
            "diagnostic": "global_costmap_raw_values_before_acceptance",
            "cost_histogram": {str(value): count for value, count in sorted(histogram.items())},
            "fixed_center_cost": fixed_cost,
            "start_cost": start_cost,
            "width": width,
            "height": height,
            "resolution": resolution,
            "origin_xy": list(origin),
        }, sort_keys=True), flush=True)
        if fixed_cost < 253 or start_cost >= 253 or histogram[255] == 0:
            raise AssertionError(
                f"costmap semantics fail: fixed={fixed_cost}, "
                f"start={start_cost}, unknown_cells={histogram[255]}"
            )
        evidence = {
            "status": (
                "real_nav2_global_costmap_sampled_not_closed_loop"
                if args.service.startswith("/global_")
                else "real_nav2_local_costmap_sampled_not_closed_loop"
            ),
            "scene_id": args.scene_id,
            "service": args.service,
            "width": width,
            "height": height,
            "resolution": resolution,
            "origin_xy": list(origin),
            "fixed_center_cost": fixed_cost,
            "start_cost": start_cost,
            "unknown_cells": histogram[255],
            "lethal_cells": histogram[254],
            "inscribed_cells": histogram[253],
            "costmap_sha256": hashlib.sha256(raw).hexdigest(),
            "costmap_bytes": len(raw),
        }
        prefix.parent.mkdir(parents=True, exist_ok=True)
        data_path.write_bytes(raw)
        json_path.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps(evidence, indent=2, sort_keys=True))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
