#!/usr/bin/env python3
"""Train and evaluate Stable-Baselines3 PPO for D1 high-level decisions."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.monitor import Monitor

from d1_edu_sb3_env import D1DecisionEnv
from run_d1_edu_decision_layer_viewer import run_episode


def _make_env(args: argparse.Namespace, seed: int) -> Monitor:
    return Monitor(D1DecisionEnv(
        profile=args.profile,
        scenario_mode=args.scenario_mode,
        seed=seed,
        duration=args.train_duration,
        route_length=args.route_length,
        mppi_samples=args.mppi_samples,
        mppi_horizon=args.mppi_horizon,
        collision_penalty=args.collision_penalty,
        timeout_penalty=args.timeout_penalty,
        cruise_mode=args.cruise_mode,
    ))


def evaluate_policy(
    model: PPO,
    args: argparse.Namespace,
) -> dict[str, Any]:
    class ModelDecisionPolicy:
        """Use the same decision-policy interface as the route replayer."""

        def reset_episode(self) -> None:
            return None

        def act(self, observation: np.ndarray) -> int:
            action, _ = model.predict(
                np.asarray(observation, dtype=np.float32),
                deterministic=True,
            )
            return int(np.asarray(action).reshape(-1)[0])

    decision_policy = ModelDecisionPolicy()
    rows: list[dict[str, Any]] = []
    for trial in range(args.eval_trials):
        env = D1DecisionEnv(
            profile=args.profile,
            scenario_mode=args.scenario_mode,
            seed=args.seed + 10000 + trial,
            duration=args.eval_duration,
            route_length=args.route_length,
            mppi_samples=args.mppi_samples,
            mppi_horizon=args.mppi_horizon,
            collision_penalty=args.collision_penalty,
            timeout_penalty=args.timeout_penalty,
            cruise_mode=args.cruise_mode,
        )
        reset_options = (
            {"curriculum_index": trial}
            if args.scenario_mode in {
                "wait_yield_curriculum", "people_balanced_multi_target"
            }
            else None
        )
        observation, info = env.reset(
            seed=args.seed + 10000 + trial,
            options=reset_options,
        )
        config = info["config"]
        episode_seed = int(info["episode_seed"])
        baseline = run_episode(config, episode_seed)
        ppo = run_episode(
            config,
            episode_seed,
            decision_policy=decision_policy,
            record_trace=True,
        )
        rows.append({
            "trial": trial,
            "seed": episode_seed,
            "curriculum_case": config.curriculum_case,
            "baseline_success": bool(baseline.success),
            "baseline_collision": bool(baseline.collision),
            "baseline_geometric_collision": bool(baseline.geometric_collision),
            "baseline_proxy_collision": bool(baseline.proxy_collision),
            "baseline_proxy_only": bool(baseline.proxy_only),
            "baseline_physical_contact": bool(baseline.physical_contact),
            "baseline_near_miss": bool(baseline.near_miss),
            "baseline_timeout": bool(baseline.timeout),
            "ppo_success": bool(ppo.success),
            "ppo_collision": bool(ppo.collision),
            "ppo_geometric_collision": bool(ppo.geometric_collision),
            "ppo_proxy_collision": bool(ppo.proxy_collision),
            "ppo_proxy_only": bool(ppo.proxy_only),
            "ppo_physical_contact": bool(ppo.physical_contact),
            "ppo_near_miss": bool(ppo.near_miss),
            "ppo_timeout": bool(ppo.timeout),
            "ppo_min_clearance": float(ppo.min_clearance),
            "ppo_path_length": float(ppo.path_length),
            "ppo_decision_metrics": ppo.decision_metrics,
        })
        env.close()

    def rate(key: str) -> float:
        return float(np.mean([row[key] for row in rows])) if rows else float("nan")

    action_counts: dict[str, int] = {}
    for row in rows:
        for name, count in row.get("ppo_decision_metrics", {}).get("action_counts", {}).items():
            action_counts[name] = action_counts.get(name, 0) + int(count)
    decision_updates = sum(
        int(row.get("ppo_decision_metrics", {}).get("decision_updates", 0))
        for row in rows
    )
    waitable_states = sum(
        int(row.get("ppo_decision_metrics", {}).get("waitable_states", 0))
        for row in rows
    )
    valid_wait_count = sum(
        int(row.get("ppo_decision_metrics", {}).get("valid_wait_count", 0))
        for row in rows
    )
    invalid_wait_count = sum(
        int(row.get("ppo_decision_metrics", {}).get("invalid_wait_count", 0))
        for row in rows
    )
    waitable_avoid_count = sum(
        int(row.get("ppo_decision_metrics", {}).get("waitable_avoid_count", 0))
        for row in rows
    )
    wait_segments = sum(
        int(row.get("ppo_decision_metrics", {}).get("wait_segments", 0))
        for row in rows
    )
    wait_followed_progress_count = sum(
        int(row.get("ppo_decision_metrics", {}).get("wait_followed_progress_count", 0))
        for row in rows
    )
    wait_returned_to_cruise_count = sum(
        int(row.get("ppo_decision_metrics", {}).get("wait_returned_to_cruise_count", 0))
        for row in rows
    )
    near_goal_timeout_count = sum(
        bool(row.get("ppo_decision_metrics", {}).get("near_goal_timeout", False))
        for row in rows
    )
    real_deadlock_count = sum(
        bool(row.get("ppo_decision_metrics", {}).get("real_deadlock", False))
        for row in rows
    )

    curriculum_case_metrics: dict[str, dict[str, Any]] = {}
    if args.scenario_mode == "wait_yield_curriculum":
        for row in rows:
            case = str(row.get("curriculum_case") or "unknown")
            metrics = row.get("ppo_decision_metrics", {})
            case_summary = curriculum_case_metrics.setdefault(case, {
                "trials": 0,
                "successes": 0,
                "collisions": 0,
                "timeouts": 0,
                "waitable_states": 0,
                "valid_wait_count": 0,
                "invalid_wait_count": 0,
                "waitable_avoid_count": 0,
                "wait_segments": 0,
                "wait_followed_progress_count": 0,
                "wait_returned_to_cruise_count": 0,
            })
            case_summary["trials"] += 1
            case_summary["successes"] += int(bool(row["ppo_success"]))
            case_summary["collisions"] += int(bool(row["ppo_collision"]))
            case_summary["timeouts"] += int(bool(row["ppo_timeout"]))
            for key in (
                "waitable_states", "valid_wait_count", "invalid_wait_count",
                "waitable_avoid_count", "wait_segments",
                "wait_followed_progress_count", "wait_returned_to_cruise_count",
            ):
                case_summary[key] += int(metrics.get(key, 0))
        for case_summary in curriculum_case_metrics.values():
            trials = max(int(case_summary["trials"]), 1)
            waitable = int(case_summary["waitable_states"])
            valid_wait = int(case_summary["valid_wait_count"])
            case_summary["success_rate"] = case_summary["successes"] / trials
            case_summary["collision_rate"] = case_summary["collisions"] / trials
            case_summary["timeout_rate"] = case_summary["timeouts"] / trials
            case_summary["wait_argmax_rate"] = (
                valid_wait / waitable if waitable else float("nan")
            )
            case_summary["invalid_wait_rate"] = (
                case_summary["invalid_wait_count"]
                / max(valid_wait + case_summary["invalid_wait_count"], 1)
            )
            case_summary["waitable_avoid_rate"] = (
                case_summary["waitable_avoid_count"] / waitable
                if waitable else float("nan")
            )
            segments = int(case_summary["wait_segments"])
            case_summary["wait_progress_rate"] = (
                case_summary["wait_followed_progress_count"] / segments
                if segments else float("nan")
            )
            case_summary["wait_return_rate"] = (
                case_summary["wait_returned_to_cruise_count"] / segments
                if segments else float("nan")
            )

    def safe_rate(numerator: int, denominator: int) -> float:
        return float(numerator / denominator) if denominator else float("nan")

    return {
        "trials": len(rows),
        "profile": args.profile,
        "scenario_mode": args.scenario_mode,
        "cruise_mode": args.cruise_mode,
        "train_duration": args.train_duration,
        "eval_duration": args.eval_duration,
        "baseline_success_rate": rate("baseline_success"),
        "baseline_collision_rate": rate("baseline_collision"),
        "baseline_geometric_collision_rate": rate("baseline_geometric_collision"),
        "baseline_proxy_collision_rate": rate("baseline_proxy_collision"),
        "baseline_proxy_only_rate": rate("baseline_proxy_only"),
        "baseline_physical_contact_rate": rate("baseline_physical_contact"),
        "baseline_near_miss_rate": rate("baseline_near_miss"),
        "baseline_timeout_rate": rate("baseline_timeout"),
        "ppo_success_rate": rate("ppo_success"),
        "ppo_collision_rate": rate("ppo_collision"),
        "ppo_geometric_collision_rate": rate("ppo_geometric_collision"),
        "ppo_proxy_collision_rate": rate("ppo_proxy_collision"),
        "ppo_proxy_only_rate": rate("ppo_proxy_only"),
        "ppo_physical_contact_rate": rate("ppo_physical_contact"),
        "ppo_near_miss_rate": rate("ppo_near_miss"),
        "ppo_timeout_rate": rate("ppo_timeout"),
        "mean_ppo_min_clearance": float(np.mean([row["ppo_min_clearance"] for row in rows])),
        "mean_ppo_path_length": float(np.mean([row["ppo_path_length"] for row in rows])),
        "ppo_action_counts": action_counts,
        "ppo_action_proportions": {
            name: safe_rate(count, decision_updates)
            for name, count in action_counts.items()
        },
        "ppo_decision_updates": decision_updates,
        "ppo_waitable_states": waitable_states,
        "ppo_valid_wait_count": valid_wait_count,
        "ppo_invalid_wait_count": invalid_wait_count,
        "ppo_wait_argmax_rate": safe_rate(valid_wait_count, waitable_states),
        "ppo_invalid_wait_rate": safe_rate(
            invalid_wait_count, invalid_wait_count + valid_wait_count
        ),
        "ppo_waitable_avoid_rate": safe_rate(waitable_avoid_count, waitable_states),
        "ppo_wait_segments": wait_segments,
        "ppo_wait_followed_progress_count": wait_followed_progress_count,
        "ppo_wait_returned_to_cruise_count": wait_returned_to_cruise_count,
        "ppo_wait_return_rate": safe_rate(wait_returned_to_cruise_count, wait_segments),
        "ppo_wait_progress_rate": safe_rate(wait_followed_progress_count, wait_segments),
        "ppo_near_goal_timeout_rate": safe_rate(near_goal_timeout_count, len(rows)),
        "ppo_real_deadlock_rate": safe_rate(real_deadlock_count, len(rows)),
        "curriculum_case_metrics": curriculum_case_metrics,
        "rows": rows,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timesteps", type=int, default=20_000)
    parser.add_argument("--eval-trials", type=int, default=20)
    parser.add_argument("--profile", choices=("baseline", "sensor_noise", "full_randomization"), default="baseline")
    parser.add_argument(
        "--scenario-mode",
        choices=("legacy", "multi_target", "irregular_multi_target", "people_multi_target", "people_random_multi_target", "people_balanced_multi_target", "wait_yield_curriculum", "long_route"),
        default="multi_target",
    )
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--route-length", type=float, default=3.20)
    parser.add_argument("--train-duration", type=float, default=40.0)
    parser.add_argument("--eval-duration", type=float, default=40.0)
    parser.add_argument("--mppi-samples", type=int, default=32)
    parser.add_argument("--mppi-horizon", type=int, default=30)
    parser.add_argument(
        "--cruise-mode",
        choices=("forward_only", "goal_hysteresis"),
        default="goal_hysteresis",
        help="CRUISE implementation used during SB3 PPO training and evaluation",
    )
    parser.add_argument(
        "--collision-penalty",
        type=float,
        default=20.0,
        help="absolute terminal collision penalty used by PPO reward shaping",
    )
    parser.add_argument(
        "--timeout-penalty",
        type=float,
        default=4.0,
        help="absolute terminal timeout penalty used by PPO reward shaping",
    )
    parser.add_argument("--n-steps", type=int, default=256)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--policy", type=str, default="output/d1_decision_sb3_ppo_v2")
    parser.add_argument(
        "--load-model",
        type=str,
        default=None,
        help="evaluate an existing SB3 .zip model without retraining",
    )
    parser.add_argument(
        "--resume-model",
        type=str,
        default=None,
        help="continue training an existing SB3 .zip model before saving --policy",
    )
    parser.add_argument("--summary", type=str, default="output/d1_decision_sb3_ppo_v2_summary.json")
    parser.add_argument("--tensorboard-log", type=str, default="output/sb3_tensorboard")
    parser.add_argument(
        "--checkpoint-freq",
        type=int,
        default=25_000,
        help="save a resumable PPO checkpoint every N environment steps; 0 disables",
    )
    parser.add_argument(
        "--checkpoint-dir",
        type=str,
        default="output/sb3_checkpoints",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="print SB3 rollout/update tables during training",
    )
    args = parser.parse_args()
    if args.load_model and args.resume_model:
        raise ValueError("--load-model and --resume-model are mutually exclusive")
    if args.eval_trials <= 0 or (args.load_model is None and args.timesteps <= 0):
        raise ValueError("eval-trials must be positive; timesteps must be positive when training")
    if args.checkpoint_freq < 0:
        raise ValueError("checkpoint-freq cannot be negative")

    checkpoint_callback = None
    if not args.load_model and args.checkpoint_freq > 0:
        checkpoint_path = Path(args.checkpoint_dir)
        checkpoint_path.mkdir(parents=True, exist_ok=True)
        checkpoint_callback = CheckpointCallback(
            save_freq=args.checkpoint_freq,
            save_path=str(checkpoint_path),
            name_prefix=Path(args.policy).name,
            save_replay_buffer=False,
            save_vecnormalize=False,
        )

    env = _make_env(args, args.seed)
    if args.load_model:
        model = PPO.load(args.load_model, env=env, device="cpu")
        policy_path = Path(args.load_model)
    elif args.resume_model:
        model = PPO.load(args.resume_model, env=env, device="cpu")
        model.learn(
            total_timesteps=args.timesteps,
            reset_num_timesteps=False,
            progress_bar=False,
            callback=checkpoint_callback,
        )
        policy_path = Path(args.policy)
        policy_path.parent.mkdir(parents=True, exist_ok=True)
        model.save(policy_path)
    else:
        model = PPO(
            "MlpPolicy",
            env,
            seed=args.seed,
            n_steps=args.n_steps,
            batch_size=args.batch_size,
            learning_rate=3e-4,
            gamma=0.99,
            gae_lambda=0.95,
            ent_coef=0.01,
            verbose=1 if args.verbose else 0,
            tensorboard_log=args.tensorboard_log,
        )
        model.learn(
            total_timesteps=args.timesteps,
            progress_bar=False,
            callback=checkpoint_callback,
        )
        policy_path = Path(args.policy)
        policy_path.parent.mkdir(parents=True, exist_ok=True)
        model.save(policy_path)
    env.close()

    summary = evaluate_policy(model, args)
    summary.update({
        "timesteps": int(model.num_timesteps),
        "policy": str(policy_path),
        "sb3_version": "2.9.0",
        "loaded_model": args.load_model,
        "collision_penalty": args.collision_penalty,
        "timeout_penalty": args.timeout_penalty,
        "cruise_mode": args.cruise_mode,
    })
    summary_path = Path(args.summary)
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(json.dumps(summary, indent=2, allow_nan=True), encoding="utf-8")
    print(f"baseline_success_rate={summary['baseline_success_rate']}")
    print(f"ppo_success_rate={summary['ppo_success_rate']}")
    print(f"baseline_collision_rate={summary['baseline_collision_rate']}")
    print(f"baseline_geometric_collision_rate={summary['baseline_geometric_collision_rate']}")
    print(f"baseline_proxy_collision_rate={summary['baseline_proxy_collision_rate']}")
    print(f"baseline_proxy_only_rate={summary['baseline_proxy_only_rate']}")
    print(f"baseline_physical_contact_rate={summary['baseline_physical_contact_rate']}")
    print(f"baseline_near_miss_rate={summary['baseline_near_miss_rate']}")
    print(f"ppo_collision_rate={summary['ppo_collision_rate']}")
    print(f"ppo_geometric_collision_rate={summary['ppo_geometric_collision_rate']}")
    print(f"ppo_proxy_collision_rate={summary['ppo_proxy_collision_rate']}")
    print(f"ppo_proxy_only_rate={summary['ppo_proxy_only_rate']}")
    print(f"ppo_physical_contact_rate={summary['ppo_physical_contact_rate']}")
    print(f"ppo_near_miss_rate={summary['ppo_near_miss_rate']}")
    print(f"baseline_timeout_rate={summary['baseline_timeout_rate']}")
    print(f"ppo_timeout_rate={summary['ppo_timeout_rate']}")
    print(f"ppo_wait_argmax_rate={summary['ppo_wait_argmax_rate']}")
    print(f"ppo_invalid_wait_rate={summary['ppo_invalid_wait_rate']}")
    print(f"ppo_waitable_avoid_rate={summary['ppo_waitable_avoid_rate']}")
    print(f"ppo_near_goal_timeout_rate={summary['ppo_near_goal_timeout_rate']}")
    print(f"ppo_real_deadlock_rate={summary['ppo_real_deadlock_rate']}")
    print(f"policy={policy_path}")
    print(f"summary={summary_path}")


if __name__ == "__main__":
    main()
