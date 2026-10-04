#!/usr/bin/env python3
"""Report the first and last Gazebo poses beyond an origin threshold."""

from __future__ import annotations

import argparse
import math

from gz_pose_log import read_pose_log


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("log")
    parser.add_argument("--entity", default="slam_bot")
    parser.add_argument("--threshold", type=float, default=0.001)
    args = parser.parse_args()

    samples = read_pose_log(args.log, args.entity)
    origin = samples[0].translation
    moved = [
        sample
        for sample in samples
        if math.dist(origin, sample.translation) > args.threshold
    ]
    if not moved:
        raise RuntimeError("no motion exceeds the requested threshold")

    print(f"origin: timestamp_ns={samples[0].timestamp_ns}, pose={origin}")
    print(
        f"motion_start: timestamp_ns={moved[0].timestamp_ns}, "
        f"pose={moved[0].translation}"
    )
    print(
        f"motion_end: timestamp_ns={moved[-1].timestamp_ns}, "
        f"pose={moved[-1].translation}"
    )
    print(f"displacement: {math.dist(origin, moved[-1].translation):.4f} m")


if __name__ == "__main__":
    main()
