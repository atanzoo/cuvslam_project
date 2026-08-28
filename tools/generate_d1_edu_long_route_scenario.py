#!/usr/bin/env python3
"""Write one replayable 30 m D1 Edu scenario without running the episode."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path

import numpy as np

from run_d1_edu_decision_layer_benchmark import sample_episode_config


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--route-length", type=float, default=30.0)
    parser.add_argument("--max-obstacle-speed", type=float, default=7.0)
    parser.add_argument(
        "--output",
        type=str,
        default="output/long_route/scenario_seed_0000.json",
    )
    args = parser.parse_args()
    if args.max_obstacle_speed != 7.0:
        raise ValueError("the current long-route profile uses the requested 7 m/s cap")
    config = sample_episode_config(
        "baseline",
        np.random.default_rng(args.seed),
        args.seed,
        route_length=args.route_length,
        scenario_mode="long_route",
    )
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "seed": args.seed,
        "profile": "long_route",
        "config": asdict(config),
    }
    with output_path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, allow_nan=True)
    print(f"scenario={output_path}")
    print(f"route_length={config.route_length}")
    print(f"obstacles={len(config.scenario_obstacles)}")
    print(f"lidar={config.lidar_model} rays={config.lidar_ray_count} range={config.lidar_max_range}m")


if __name__ == "__main__":
    main()
