#!/usr/bin/env python3
"""Feed stationary simulation truth during isolated Nav2 launch checks only."""

import argparse
import json
import time
from urllib.request import Request, urlopen


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", default="http://127.0.0.1:8987")
    parser.add_argument("--scene-id", required=True)
    parser.add_argument("--sequence-start", type=int, required=True)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--yaw", type=float, default=0.0)
    args = parser.parse_args()
    if not 0 < args.duration <= 60 or args.sequence_start < 0:
        raise ValueError("bounded duration and nonnegative sequence required")
    start = time.monotonic()
    sequence = args.sequence_start
    while time.monotonic() - start < args.duration:
        body = json.dumps({
            "scene_id": args.scene_id, "sequence": sequence,
            "x": args.x, "y": args.y, "yaw": args.yaw,
            "vx": 0.0, "vy": 0.0, "wz": 0.0,
        }).encode("utf-8")
        request = Request(
            args.base + "/pose", data=body,
            headers={"Content-Type": "application/json"},
        )
        with urlopen(request, timeout=2.0) as response:
            if response.status != 200:
                raise RuntimeError(f"bridge rejected stationary pose: {response.status}")
        sequence += 1
        time.sleep(0.05)
    print(json.dumps({
        "scene_id": args.scene_id,
        "stationary_pose_messages": sequence - args.sequence_start,
        "elapsed_wall_s": time.monotonic() - start,
    }, sort_keys=True))


if __name__ == "__main__":
    main()
