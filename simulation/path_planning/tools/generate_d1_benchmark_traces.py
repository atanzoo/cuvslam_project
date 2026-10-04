#!/usr/bin/env python3
"""Replay every deterministic benchmark trial with trajectory traces."""

from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path

import numpy as np

from run_d1_edu_decision_layer_benchmark import sample_episode_config
from run_d1_edu_decision_layer_viewer import run_episode


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", type=int, default=20)
    parser.add_argument("--profile", choices=("baseline", "sensor_noise", "full_randomization"), default="baseline")
    parser.add_argument(
        "--scenario-mode",
        choices=("legacy", "multi_target", "irregular_multi_target", "people_multi_target", "people_random_multi_target", "people_balanced_multi_target", "long_route"),
        default="multi_target",
    )
    parser.add_argument("--duration", type=float, default=40.0)
    parser.add_argument("--route-length", type=float, default=3.20)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    master_rng = np.random.default_rng(args.seed)
    for trial in range(args.trials):
        trial_seed = int(master_rng.integers(0, 2**31 - 1))
        config = sample_episode_config(
            args.profile,
            master_rng,
            trial_seed,
            route_length=args.route_length,
            scenario_mode=args.scenario_mode,
        )
        config = replace(config, duration=args.duration)
        result = run_episode(config, trial_seed, record_trace=True)
        payload = {
            "trial": trial,
            "seed": trial_seed,
            "profile": args.profile,
            "config": asdict(config),
            "result": asdict(result),
        }
        path = args.output_dir / f"trial_{trial:04d}.json"
        path.write_text(json.dumps(payload, indent=2, allow_nan=True), encoding="utf-8")
        print(f"trial={trial} success={result.success} status={result.failure_reason}")


if __name__ == "__main__":
    main()
