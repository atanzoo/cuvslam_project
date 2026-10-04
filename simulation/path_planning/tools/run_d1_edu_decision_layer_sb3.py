#!/usr/bin/env python3
"""Train and evaluate Stable-Baselines3 PPO for D1 high-level decisions."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CallbackList, CheckpointCallback
from stable_baselines3.common.monitor import Monitor

from d1_candidate_map import load_candidate_map
from d1_edu_sb3_env import ACTION_NAMES, D1DecisionEnv, OBSERVATION_DIM
from d1_robot_profile import available_robot_profiles, get_robot_profile
from run_d1_edu_decision_layer_viewer import run_episode


class EpisodeOutcomeTrend:
    """Aggregate completed training-episode outcomes into fixed step windows."""

    def __init__(self, window_steps: int) -> None:
        if window_steps <= 0:
            raise ValueError("window_steps must be positive")
        self.window_steps = int(window_steps)
        self._window_start = 0
        self._next_boundary = self.window_steps
        self._counts = {"episodes": 0, "successes": 0, "collisions": 0, "timeouts": 0}

    def record_episode(self, info: dict[str, Any]) -> None:
        self._counts["episodes"] += 1
        self._counts["successes"] += int(bool(info.get("success", False)))
        self._counts["collisions"] += int(bool(info.get("collision", False)))
        self._counts["timeouts"] += int(bool(info.get("timeout", False)))

    def _finish_window(self, end_step: int) -> dict[str, Any]:
        episodes = int(self._counts["episodes"])
        row: dict[str, Any] = {
            "start_step": self._window_start,
            "end_step": int(end_step),
            **self._counts,
        }
        for outcome, count_key in (
            ("success", "successes"),
            ("collision", "collisions"),
            ("timeout", "timeouts"),
        ):
            row[f"{outcome}_rate"] = (
                float(self._counts[count_key] / episodes) if episodes else None
            )
        self._window_start = int(end_step)
        self._counts = {"episodes": 0, "successes": 0, "collisions": 0, "timeouts": 0}
        return row

    def advance(self, completed_steps: int) -> list[dict[str, Any]]:
        rows = []
        while completed_steps >= self._next_boundary:
            rows.append(self._finish_window(self._next_boundary))
            self._next_boundary += self.window_steps
        return rows

    def flush_partial(self, completed_steps: int) -> dict[str, Any] | None:
        if not self._counts["episodes"]:
            return None
        return self._finish_window(completed_steps)


class EpisodeOutcomeTrendCallback(BaseCallback):
    """Log success, collision, and timeout rates per 5k training steps."""

    def __init__(self, window_steps: int = 5_000) -> None:
        super().__init__(verbose=0)
        self.trend = EpisodeOutcomeTrend(window_steps)
        self.windows: list[dict[str, Any]] = []

    def _on_step(self) -> bool:
        infos = self.locals.get("infos", [])
        dones = self.locals.get("dones", [])
        for done, info in zip(dones, infos):
            if done:
                self.trend.record_episode(info)
        for row in self.trend.advance(self.num_timesteps):
            self._record_window(row)
        return True

    def _record_window(self, row: dict[str, Any]) -> None:
        self.windows.append(row)
        self.logger.record("train_outcomes/completed_episodes", row["episodes"])
        for outcome in ("success", "collision", "timeout"):
            rate = row[f"{outcome}_rate"]
            if rate is not None:
                self.logger.record(f"train_outcomes/{outcome}_rate", rate)

    def _on_training_end(self) -> None:
        partial = self.trend.flush_partial(self.num_timesteps)
        if partial is not None:
            self._record_window(partial)


def _make_env(args: argparse.Namespace, seed: int) -> Monitor:
    return Monitor(D1DecisionEnv(
        profile=args.profile,
        robot_profile=args.robot_profile,
        kinematic_proxy=args.kinematic_proxy,
        scenario_mode=args.scenario_mode,
        seed=seed,
        duration=args.train_duration,
        route_length=args.route_length,
        mppi_samples=args.mppi_samples,
        mppi_horizon=args.mppi_horizon,
        collision_penalty=args.collision_penalty,
        timeout_penalty=args.timeout_penalty,
        cruise_mode=args.cruise_mode,
        candidate_map_yaml=getattr(args, "candidate_map_yaml", None),
        map_scene_scale=getattr(args, "map_scene_scale", 5.0),
    ))


def _candidate_map_identity(args: argparse.Namespace) -> dict[str, Any] | None:
    map_yaml = getattr(args, "candidate_map_yaml", None)
    if not map_yaml:
        return None
    candidate_map = load_candidate_map(
        map_yaml,
        scene_scale=float(getattr(args, "map_scene_scale", 5.0)),
        preserve_resolution=True,
    )
    manifest = candidate_map.manifest()
    return {
        "yaml_sha256": manifest["yaml_sha256"],
        "pgm_sha256": manifest["pgm_sha256"],
        "source_width": candidate_map.source_width,
        "source_height": candidate_map.source_height,
        "scene_scale": candidate_map.scene_scale,
        "scaled_resolution": candidate_map.scaled_resolution,
        "preserve_resolution": candidate_map.preserve_resolution,
    }


def _map_env_rollout(
    env: D1DecisionEnv,
    *,
    seed: int,
    trial: int,
    model: PPO | None,
) -> tuple[dict[str, Any], Any]:
    observation, reset_info = env.reset(
        seed=seed,
        options={"curriculum_index": trial},
    )
    action_counts: dict[str, int] = {}
    waitable_states = 0
    valid_wait_count = 0
    invalid_wait_count = 0
    waitable_avoid_count = 0
    last_info: dict[str, Any] = {}
    previous_mode = "CRUISE"
    wait_segments = 0
    wait_followed_progress_count = 0
    wait_returned_to_cruise_count = 0
    wait_progress_start = 0.0
    while True:
        if model is None:
            action = 0
        else:
            predicted, _ = model.predict(
                np.asarray(observation, dtype=np.float32),
                deterministic=True,
            )
            action = int(np.asarray(predicted).reshape(-1)[0])
        action_name = ACTION_NAMES[action]
        action_counts[action_name] = action_counts.get(action_name, 0) + 1
        observation, _, terminated, truncated, last_info = env.step(action)
        waitable_states += int(bool(last_info.get("wait_allowed", False)))
        valid_wait_count += int(
            action == 4
            and bool(last_info.get("wait_allowed", False))
            and not bool(last_info.get("action_overridden", False))
        )
        invalid_wait_count += int(
            action == 4 and bool(last_info.get("action_overridden", False))
        )
        waitable_avoid_count += int(
            bool(last_info.get("wait_allowed", False)) and action in (1, 2)
        )
        mode = str(last_info.get("mode", "CRUISE"))
        if mode == "WAIT_YIELD" and previous_mode != "WAIT_YIELD":
            wait_segments += 1
            wait_progress_start = float(last_info.get("path_progress", 0.0))
        if previous_mode == "WAIT_YIELD" and mode != "WAIT_YIELD":
            if float(last_info.get("path_progress", 0.0)) > wait_progress_start + 0.002:
                wait_followed_progress_count += 1
            if mode == "CRUISE":
                wait_returned_to_cruise_count += 1
        previous_mode = mode
        if terminated or truncated:
            break

    metrics = {
        "action_counts": action_counts,
        "decision_updates": sum(action_counts.values()),
        "waitable_states": waitable_states,
        "valid_wait_count": valid_wait_count,
        "invalid_wait_count": invalid_wait_count,
        "waitable_avoid_count": waitable_avoid_count,
        "wait_segments": wait_segments,
        "wait_followed_progress_count": wait_followed_progress_count,
        "wait_returned_to_cruise_count": wait_returned_to_cruise_count,
        "near_goal_timeout": bool(
            last_info.get("timeout", False)
            and float(last_info.get("distance_to_goal", float("inf")))
            <= max(0.30, env.config.goal_tolerance * 2.0)
        ),
        "real_deadlock": bool(last_info.get("deadlock", False)),
    }
    return {
        "success": bool(last_info.get("success", False)),
        "collision": bool(last_info.get("collision", False)),
        "geometric_collision": bool(last_info.get("geometric_collision", False)),
        "proxy_collision": bool(last_info.get("proxy_collision", False)),
        "proxy_only": bool(last_info.get("proxy_only", False)),
        "physical_contact": bool(last_info.get("physical_contact", False)),
        "near_miss": bool(last_info.get("near_miss", False)),
        "map_violation": bool(last_info.get("map_violation", False)),
        "timeout": bool(last_info.get("timeout", False)),
        "min_clearance": float(last_info.get("min_clearance", float("inf"))),
        "path_length": float(last_info.get("path_length", 0.0)),
        "decision_metrics": metrics,
    }, reset_info


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
    map_env = None
    if getattr(args, "candidate_map_yaml", None):
        map_env = D1DecisionEnv(
            profile=args.profile,
            robot_profile=args.robot_profile,
            kinematic_proxy=args.kinematic_proxy,
            scenario_mode=args.scenario_mode,
            seed=args.seed + 10000,
            duration=args.eval_duration,
            route_length=args.route_length,
            mppi_samples=args.mppi_samples,
            mppi_horizon=args.mppi_horizon,
            collision_penalty=args.collision_penalty,
            timeout_penalty=args.timeout_penalty,
            cruise_mode=args.cruise_mode,
            candidate_map_yaml=args.candidate_map_yaml,
            map_scene_scale=getattr(args, "map_scene_scale", 5.0),
        )
    try:
        for trial in range(args.eval_trials):
            trial_seed = args.seed + 10000 + trial
            if map_env is not None:
                baseline, _ = _map_env_rollout(
                    map_env, seed=trial_seed, trial=trial, model=None,
                )
                ppo, info = _map_env_rollout(
                    map_env, seed=trial_seed, trial=trial, model=model,
                )
                config = info["config"]
                episode_seed = int(info["episode_seed"])
            else:
                env = D1DecisionEnv(
                    profile=args.profile,
                    robot_profile=args.robot_profile,
                    kinematic_proxy=args.kinematic_proxy,
                    scenario_mode=args.scenario_mode,
                    seed=trial_seed,
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
                _, info = env.reset(seed=trial_seed, options=reset_options)
                config = info["config"]
                episode_seed = int(info["episode_seed"])
                baseline_result = run_episode(config, episode_seed)
                ppo_result = run_episode(
                    config,
                    episode_seed,
                    decision_policy=decision_policy,
                    record_trace=True,
                )
                baseline = {
                    "success": baseline_result.success,
                    "collision": baseline_result.collision,
                    "geometric_collision": baseline_result.geometric_collision,
                    "proxy_collision": baseline_result.proxy_collision,
                    "proxy_only": baseline_result.proxy_only,
                    "physical_contact": baseline_result.physical_contact,
                    "near_miss": baseline_result.near_miss,
                    "map_violation": False,
                    "timeout": baseline_result.timeout,
                }
                ppo = {
                    "success": ppo_result.success,
                    "collision": ppo_result.collision,
                    "geometric_collision": ppo_result.geometric_collision,
                    "proxy_collision": ppo_result.proxy_collision,
                    "proxy_only": ppo_result.proxy_only,
                    "physical_contact": ppo_result.physical_contact,
                    "near_miss": ppo_result.near_miss,
                    "map_violation": False,
                    "timeout": ppo_result.timeout,
                    "min_clearance": ppo_result.min_clearance,
                    "path_length": ppo_result.path_length,
                    "decision_metrics": ppo_result.decision_metrics,
                }
                env.close()
            rows.append({
                "trial": trial,
                "seed": episode_seed,
                "curriculum_case": config.curriculum_case,
                "baseline_success": bool(baseline["success"]),
                "baseline_collision": bool(baseline["collision"]),
                "baseline_geometric_collision": bool(baseline["geometric_collision"]),
                "baseline_proxy_collision": bool(baseline["proxy_collision"]),
                "baseline_proxy_only": bool(baseline["proxy_only"]),
                "baseline_physical_contact": bool(baseline["physical_contact"]),
                "baseline_near_miss": bool(baseline["near_miss"]),
                "baseline_timeout": bool(baseline["timeout"]),
                "baseline_map_violation": bool(baseline["map_violation"]),
                "ppo_success": bool(ppo["success"]),
                "ppo_collision": bool(ppo["collision"]),
                "ppo_geometric_collision": bool(ppo["geometric_collision"]),
                "ppo_proxy_collision": bool(ppo["proxy_collision"]),
                "ppo_proxy_only": bool(ppo["proxy_only"]),
                "ppo_physical_contact": bool(ppo["physical_contact"]),
                "ppo_near_miss": bool(ppo["near_miss"]),
                "ppo_timeout": bool(ppo["timeout"]),
                "ppo_map_violation": bool(ppo["map_violation"]),
                "ppo_min_clearance": float(ppo.get("min_clearance", float("inf"))),
                "ppo_path_length": float(ppo.get("path_length", 0.0)),
                "ppo_decision_metrics": ppo["decision_metrics"],
            })
    finally:
        if map_env is not None:
            map_env.close()

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
        "robot_profile": args.robot_profile,
        "kinematic_proxy": bool(args.kinematic_proxy),
        "scenario_mode": args.scenario_mode,
        "cruise_mode": args.cruise_mode,
        "candidate_map": _candidate_map_identity(args),
        "evaluation_backend": (
            "D1DecisionEnv candidate-map rollout"
            if getattr(args, "candidate_map_yaml", None)
            else "existing route replay"
        ),
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
        "baseline_map_violation_rate": rate("baseline_map_violation"),
        "ppo_success_rate": rate("ppo_success"),
        "ppo_collision_rate": rate("ppo_collision"),
        "ppo_geometric_collision_rate": rate("ppo_geometric_collision"),
        "ppo_proxy_collision_rate": rate("ppo_proxy_collision"),
        "ppo_proxy_only_rate": rate("ppo_proxy_only"),
        "ppo_physical_contact_rate": rate("ppo_physical_contact"),
        "ppo_near_miss_rate": rate("ppo_near_miss"),
        "ppo_timeout_rate": rate("ppo_timeout"),
        "ppo_map_violation_rate": rate("ppo_map_violation"),
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


def _metadata_path(model_path: str | Path) -> Path:
    """Return the sidecar path shared by ``foo`` and ``foo.zip``."""
    path = Path(model_path)
    if path.suffix == ".zip":
        path = path.with_suffix("")
    return Path(f"{path}.metadata.json")


def _validate_model_metadata(model_path: str | Path, args: argparse.Namespace) -> None:
    """Reject a checkpoint whose robot identity cannot be proven compatible."""
    metadata_path = _metadata_path(model_path)
    if not metadata_path.exists():
        if args.robot_profile == "d1_max_proxy":
            raise ValueError(
                f"D1 Max checkpoint metadata is missing: {metadata_path}. "
                "Do not reuse an unlabelled D1 Edu checkpoint; train or export "
                "a D1 Max kinematic-proxy checkpoint first."
            )
        return
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    expected = {
        "robot_profile": args.robot_profile,
        "kinematic_proxy": bool(args.kinematic_proxy),
        "candidate_map": _candidate_map_identity(args),
    }
    actual = {
        "robot_profile": metadata.get("robot_profile"),
        "kinematic_proxy": bool(metadata.get("kinematic_proxy", False)),
        "candidate_map": metadata.get("candidate_map"),
    }
    if actual != expected:
        raise ValueError(
            f"checkpoint identity mismatch for {model_path}: "
            f"expected {expected}, found {actual} in {metadata_path}"
        )


def _write_model_metadata(
    model_path: str | Path,
    args: argparse.Namespace,
    timesteps: int,
) -> Path:
    """Persist the simulation contract next to an SB3 checkpoint."""
    robot = get_robot_profile(args.robot_profile)
    metadata_path = _metadata_path(model_path)
    metadata_path.parent.mkdir(parents=True, exist_ok=True)
    metadata = {
        "schema_version": 1,
        "robot_profile": robot.name,
        "model_family": robot.model_family,
        "model_asset": robot.model_asset,
        "source_revision": robot.source_revision,
        "kinematic_proxy": bool(args.kinematic_proxy),
        "gait_or_torque_control": False,
        "command_backend": "SimulatedD1SDK -> kinematic base advance",
        "observation_dim": OBSERVATION_DIM,
        "action_names": list(ACTION_NAMES),
        "scenario_profile": args.profile,
        "scenario_mode": args.scenario_mode,
        "cruise_mode": args.cruise_mode,
        "candidate_map": _candidate_map_identity(args),
        "timesteps": int(timesteps),
    }
    metadata_path.write_text(
        json.dumps(metadata, indent=2, allow_nan=True),
        encoding="utf-8",
    )
    return metadata_path


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
    parser.add_argument(
        "--robot-profile",
        choices=available_robot_profiles(),
        default="d1_edu",
        help="robot target; d1_max_proxy requires --kinematic-proxy",
    )
    parser.add_argument(
        "--kinematic-proxy",
        action="store_true",
        help="use imported D1 Max geometry with a simulation-only pose-hold base",
    )
    parser.add_argument("--route-length", type=float, default=3.20)
    parser.add_argument(
        "--candidate-map-yaml",
        type=str,
        default=None,
        help="read-only candidate map YAML; requires the D1 Max kinematic proxy",
    )
    parser.add_argument(
        "--map-scene-scale",
        type=float,
        default=5.0,
        help="map geometry multiplier; robot and pedestrian dimensions remain fixed",
    )
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
    parser.add_argument(
        "--policy",
        type=str,
        default=None,
        help="output checkpoint; defaults to a profile-specific path",
    )
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
    parser.add_argument(
        "--summary",
        type=str,
        default=None,
        help="summary JSON; defaults to a profile-specific path",
    )
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
    if args.robot_profile == "d1_max_proxy" and not args.kinematic_proxy:
        raise ValueError(
            "D1 Max gait is not enabled; pass --kinematic-proxy explicitly "
            "for this simulation-only training stage"
        )
    if args.candidate_map_yaml and (
        args.robot_profile != "d1_max_proxy" or not args.kinematic_proxy
    ):
        raise ValueError(
            "candidate-map episodes require --robot-profile d1_max_proxy "
            "with --kinematic-proxy"
        )
    candidate_map_identity = _candidate_map_identity(args)
    profile_suffix = (
        "d1_max_proxy_kinematic" if args.robot_profile == "d1_max_proxy"
        else "d1_edu"
    )
    if args.policy is None:
        args.policy = f"output/d1_decision_sb3_ppo_{profile_suffix}"
    if args.summary is None:
        args.summary = f"output/d1_decision_sb3_ppo_{profile_suffix}_summary.json"
    if args.load_model and args.resume_model:
        raise ValueError("--load-model and --resume-model are mutually exclusive")
    if args.load_model:
        _validate_model_metadata(args.load_model, args)
    if args.resume_model:
        _validate_model_metadata(args.resume_model, args)
    if args.eval_trials <= 0 or (args.load_model is None and args.timesteps <= 0):
        raise ValueError("eval-trials must be positive; timesteps must be positive when training")
    if args.checkpoint_freq < 0:
        raise ValueError("checkpoint-freq cannot be negative")

    callbacks = []
    if not args.load_model and args.checkpoint_freq > 0:
        checkpoint_path = Path(args.checkpoint_dir)
        checkpoint_path.mkdir(parents=True, exist_ok=True)
        callbacks.append(CheckpointCallback(
            save_freq=args.checkpoint_freq,
            save_path=str(checkpoint_path),
            name_prefix=Path(args.policy).name,
            save_replay_buffer=False,
            save_vecnormalize=False,
        ))
    outcome_callback = None
    if not args.load_model:
        outcome_callback = EpisodeOutcomeTrendCallback(window_steps=5_000)
        callbacks.append(outcome_callback)
    training_callback = CallbackList(callbacks) if callbacks else None

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
            callback=training_callback,
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
            callback=training_callback,
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
        "robot_profile": args.robot_profile,
        "kinematic_proxy": bool(args.kinematic_proxy),
        "candidate_map": candidate_map_identity,
        "training_environment": (
            "D1 Max kinematic proxy on scaled candidate map"
            if candidate_map_identity else "existing D1 decision environment"
        ),
        "training_outcome_window_steps": 5_000,
        "training_outcome_trend": (
            outcome_callback.windows if outcome_callback is not None else []
        ),
    })
    metadata_path = _write_model_metadata(
        policy_path,
        args,
        int(model.num_timesteps),
    )
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
    print(f"baseline_map_violation_rate={summary['baseline_map_violation_rate']}")
    print(f"ppo_timeout_rate={summary['ppo_timeout_rate']}")
    print(f"ppo_map_violation_rate={summary['ppo_map_violation_rate']}")
    print(f"ppo_wait_argmax_rate={summary['ppo_wait_argmax_rate']}")
    print(f"ppo_invalid_wait_rate={summary['ppo_invalid_wait_rate']}")
    print(f"ppo_waitable_avoid_rate={summary['ppo_waitable_avoid_rate']}")
    print(f"ppo_near_goal_timeout_rate={summary['ppo_near_goal_timeout_rate']}")
    print(f"ppo_real_deadlock_rate={summary['ppo_real_deadlock_rate']}")
    print(f"policy={policy_path}")
    print(f"policy_metadata={metadata_path}")
    print(f"summary={summary_path}")


if __name__ == "__main__":
    main()
