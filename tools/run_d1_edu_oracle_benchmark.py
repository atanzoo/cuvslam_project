#!/usr/bin/env python3
"""Measure the high-level oracle/MPPI ceiling on fixed D1 scenarios."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np

from d1_edu_oracle_policy import GroundTruthOraclePolicy
from d1_edu_sb3_env import D1DecisionEnv
from run_d1_edu_decision_layer_viewer import run_episode


def _rate(rows: list[dict[str, Any]], key: str) -> float:
    return float(np.mean([bool(row[key]) for row in rows])) if rows else float("nan")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=100)
    parser.add_argument("--profile", choices=("baseline", "sensor_noise", "full_randomization"), default="full_randomization")
    parser.add_argument("--scenario-mode", choices=("legacy", "multi_target", "irregular_multi_target", "people_multi_target", "people_random_multi_target", "people_balanced_multi_target", "wait_yield_curriculum", "long_route"), default="people_random_multi_target")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--route-length", type=float, default=10.0)
    parser.add_argument("--duration", type=float, default=50.0)
    parser.add_argument("--mppi-samples", type=int, default=32)
    parser.add_argument("--mppi-horizon", type=int, default=30)
    parser.add_argument("--output", type=str, default="output/d1_oracle_mppi_fixed100_summary.json")
    args = parser.parse_args()
    if args.trials <= 0:
        raise ValueError("trials must be positive")

    policy = GroundTruthOraclePolicy()
    rows: list[dict[str, Any]] = []
    for trial in range(args.trials):
        trial_seed = args.seed + 10000 + trial
        env = D1DecisionEnv(
            profile=args.profile,
            scenario_mode=args.scenario_mode,
            seed=trial_seed,
            duration=args.duration,
            route_length=args.route_length,
            mppi_samples=args.mppi_samples,
            mppi_horizon=args.mppi_horizon,
            cruise_mode="goal_hysteresis",
        )
        reset_options = (
            {"curriculum_index": trial}
            if args.scenario_mode in {
                "wait_yield_curriculum", "people_balanced_multi_target"
            }
            else None
        )
        _, info = env.reset(seed=trial_seed, options=reset_options)
        config = info["config"]
        episode_seed = int(info["episode_seed"])
        result = run_episode(
            config,
            episode_seed,
            decision_policy=policy,
            record_trace=False,
        )
        rows.append({
            "trial": trial,
            "seed": episode_seed,
            "success": bool(result.success),
            "collision": bool(result.collision),
            "physical_contact": bool(result.physical_contact),
            "timeout": bool(result.timeout),
            "min_clearance": float(result.min_clearance),
            "path_length": float(result.path_length),
            "distance_to_goal": float(result.distance_to_goal),
            "failure_reason": result.failure_reason,
        })
        env.close()

    summary = {
        "trials": len(rows),
        "policy": "ground_truth_oracle",
        "profile": args.profile,
        "scenario_mode": args.scenario_mode,
        "seed": args.seed,
        "route_length": args.route_length,
        "duration": args.duration,
        "mppi_samples": args.mppi_samples,
        "mppi_horizon": args.mppi_horizon,
        "success_rate": _rate(rows, "success"),
        "collision_rate": _rate(rows, "collision"),
        "physical_contact_rate": _rate(rows, "physical_contact"),
        "timeout_rate": _rate(rows, "timeout"),
        "mean_min_clearance": float(np.mean([row["min_clearance"] for row in rows])),
        "mean_path_length": float(np.mean([row["path_length"] for row in rows])),
        "rows": rows,
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2, allow_nan=True), encoding="utf-8")
    print(f"trials={summary['trials']}")
    print(f"oracle_success_rate={summary['success_rate']}")
    print(f"oracle_collision_rate={summary['collision_rate']}")
    print(f"oracle_timeout_rate={summary['timeout_rate']}")
    print(f"summary={output}")


if __name__ == "__main__":
    main()
