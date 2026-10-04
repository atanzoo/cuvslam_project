#!/usr/bin/env python3
"""Non-motion HTTP contract smoke test for isolated Map-3 bridge on Jetson."""

from __future__ import annotations

import argparse
import json
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen


def request(base: str, endpoint: str, payload: dict | None = None) -> tuple[int, dict]:
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    req = Request(base + endpoint, data=data, headers={"Content-Type": "application/json"})
    try:
        with urlopen(req, timeout=2.0) as response:
            return response.status, json.load(response)
    except HTTPError as error:
        return error.code, json.load(error)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", default="http://127.0.0.1:8987")
    parser.add_argument("--scene-id", required=True)
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    args = parser.parse_args()
    status, initial = request(args.base, "/health")
    assert status == 200 and not initial["armed"] and initial["velocity_body"] == [0.0, 0.0, 0.0]
    for sequence in range(12):
        status, accepted = request(args.base, "/pose", {
            "scene_id": args.scene_id, "sequence": sequence,
            "x": args.x, "y": args.y, "yaw": 0.0,
            "vx": 0.0, "vy": 0.0, "wz": 0.0,
        })
        assert status == 200 and accepted["velocity_body"] == [0.0, 0.0, 0.0]
        time.sleep(0.05)
    status, live = request(args.base, "/health")
    assert status == 200 and live["fresh_pose"] and not live["armed"]
    status, rejected_arm = request(args.base, "/arm", {"scene_id": args.scene_id})
    assert status == 409 and "active FollowPath" in rejected_arm["error"]
    status, rejected_scene = request(args.base, "/pose", {
        "scene_id": "wrong_scene", "sequence": 12,
        "x": args.x, "y": args.y, "yaw": 0.0,
        "vx": 0.0, "vy": 0.0, "wz": 0.0,
    })
    assert status == 409 and rejected_scene["velocity_body"] == [0.0, 0.0, 0.0]
    time.sleep(0.36)
    status, stale = request(args.base, "/command")
    assert status == 200 and not stale["armed"] and not stale["fresh_pose"]
    assert stale["velocity_body"] == [0.0, 0.0, 0.0]
    print(json.dumps({
        "status": "isolated_bridge_static_zero_command_pass",
        "scene_id": args.scene_id,
        "pose_messages_accepted": 12,
        "arm_without_follow_path_rejected": True,
        "wrong_scene_rejected": True,
        "stale_pose_zero": True,
        "last_pose_sequence": stale["last_pose_sequence"],
    }, sort_keys=True))


if __name__ == "__main__":
    main()
