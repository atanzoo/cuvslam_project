#!/usr/bin/env python3
"""Record compact top-view traces for an SB3 PPO evaluation."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from stable_baselines3 import PPO

from d1_edu_sb3_env import D1DecisionEnv


def _snapshot(env: D1DecisionEnv, elapsed: float, info: dict) -> dict:
    states = env._current_states()
    return {
        "t": round(float(elapsed), 3),
        "r": [round(float(env.data.qpos[0]), 3), round(float(env.data.qpos[1]), 3)],
        "o": [
            [round(float(state[0][0]), 3), round(float(state[0][1]), 3), int(bool(state[2]))]
            for state in states
        ],
        "m": info.get("mode", env.active_mode),
        "c": round(float(info.get("min_clearance", env.min_clearance)), 3),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--trials", type=int, default=20)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--scenario-mode",
        choices=("multi_target", "irregular_multi_target", "people_multi_target", "people_random_multi_target", "people_balanced_multi_target"),
        default="multi_target",
    )
    parser.add_argument("--duration", type=float, default=40.0)
    parser.add_argument("--route-length", type=float, default=3.20)
    parser.add_argument("--mppi-samples", type=int, default=96)
    parser.add_argument("--mppi-horizon", type=int, default=45)
    parser.add_argument("--collision-penalty", type=float, default=5.0)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    model = PPO.load(args.model, device="cpu")
    cases: list[dict] = []
    for trial in range(args.trials):
        trial_seed = args.seed + 10000 + trial
        env = D1DecisionEnv(
            profile="baseline",
            scenario_mode=args.scenario_mode,
            seed=trial_seed,
            duration=args.duration,
            route_length=args.route_length,
            mppi_samples=args.mppi_samples,
            mppi_horizon=args.mppi_horizon,
            collision_penalty=args.collision_penalty,
        )
        observation, reset_info = env.reset(seed=trial_seed)
        trace = [_snapshot(env, 0.0, {})]
        terminated = truncated = False
        info: dict = {}
        while not (terminated or truncated):
            action, _ = model.predict(observation, deterministic=True)
            observation, _, terminated, truncated, info = env.step(int(np.asarray(action).reshape(-1)[0]))
            trace.append(_snapshot(env, float(env.elapsed), info))

        if info.get("success", False):
            status = "success"
        elif info.get("collision", False):
            status = "collision"
        elif info.get("timeout", False):
            status = "timeout"
        else:
            status = "goal_failure"
        cases.append({
            "trial": trial,
            "seed": int(reset_info["episode_seed"]),
            "status": status,
            "clearance": round(float(info.get("min_clearance", env.min_clearance)), 3),
            "distance": round(float(info.get("distance_to_goal", 0.0)), 3),
            "path": round(float(info.get("path_length", env.path_length)), 3),
            "goal": [round(float(env.goal_world[0]), 3), round(float(env.goal_world[1]), 3)],
            "trace": trace,
        })
        env.close()

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(cases, ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
    print(f"cases={len(cases)}")
    print(f"output={output}")


if __name__ == "__main__":
    main()
