#!/usr/bin/env python3
"""Monte Carlo benchmark for the D1 Edu dynamic-obstacle decision layer."""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, replace
import json
from pathlib import Path
from typing import Any

import numpy as np

from d1_edu_scenario import (
    generate_long_route_scenario,
    generate_multi_target_scenario,
    generate_people_random_multi_target_scenario,
    generate_people_balanced_multi_target_scenario,
    generate_wait_yield_curriculum_scenario,
    wait_yield_curriculum_case,
)
from run_d1_edu_decision_layer_viewer import (
    EpisodeConfig,
    EpisodeResult,
    run_episode,
)


BENCHMARK_DURATION = 40.0


CSV_FIELDS = [
    "trial",
    "seed",
    "profile",
    "scenario_mode",
    "curriculum_case",
    "curriculum_index",
    "route_length",
    "max_obstacle_speed",
    "lidar_model",
    "trajectory_prediction_horizon",
    "trajectory_sample_dt",
    "stagnation_window",
    "minimum_progress",
    "recovery_duration",
    "risk_confirmation_updates",
    "rejoin_gain",
    "rejoin_deadband",
    "rejoin_max_lateral_speed",
    "rejoin_delay",
    "rejoin_ramp_time",
    "global_path_lookahead",
    "global_path_lateral_gain",
    "global_path_lateral_deadband",
    "global_path_max_lateral_speed",
    "cruise_mode",
    "collision_tolerance",
    "box_x",
    "box_y",
    "box_vx",
    "box_vy",
    "box_size_x",
    "box_size_y",
    "duration",
    "lidar_noise_std",
    "lidar_dropout_rate",
    "command_delay",
    "success",
    "collision",
    "geometric_collision",
    "proxy_collision",
    "proxy_only",
    "physical_contact",
    "near_miss",
    "reached_goal",
    "timeout",
    "min_clearance",
    "min_proxy_clearance",
    "elapsed_time",
    "path_length",
    "distance_to_goal",
    "decision_triggered",
    "avoid_duration",
    "tp",
    "tn",
    "fp",
    "fn",
    "failure_reason",
    "collision_obstacle_index",
    "contact_geom_name",
    "contact_distance",
    "contact_position",
    "tracking_failure",
    "recovery_triggered",
    "recovery_active_duration",
    "decision_updates",
    "action_counts",
    "action_proportions",
    "waitable_states",
    "wait_requested_count",
    "valid_wait_count",
    "invalid_wait_count",
    "wait_argmax_rate",
    "invalid_wait_rate",
    "waitable_avoid_count",
    "waitable_avoid_rate",
    "mean_consecutive_wait_duration",
    "wait_followed_progress_count",
    "wait_returned_to_cruise_count",
    "near_goal_timeout",
    "real_deadlock",
]


def _sample_box_velocity(rng: np.random.Generator) -> tuple[float, float]:
    """Sample velocities while retaining both crossing and non-crossing cases."""
    vx = float(rng.uniform(-0.10, 0.10))
    vy = float(rng.uniform(-0.55, 0.55))
    if abs(vy) < 0.08:
        vy = 0.08 if rng.random() < 0.5 else -0.08
    return vx, vy


def sample_episode_config(
    profile: str,
    rng: np.random.Generator,
    trial_seed: int,
    route_length: float = 3.20,
    scenario_mode: str = "legacy",
    curriculum_index: int | None = None,
) -> EpisodeConfig:
    """Create a profile-specific randomized episode configuration."""
    if profile not in {"baseline", "sensor_noise", "full_randomization"}:
        raise ValueError(f"unknown profile: {profile}")

    box_vx, box_vy = _sample_box_velocity(rng)
    kwargs: dict[str, Any] = {
        "scenario_mode": scenario_mode,
        "route_length": route_length,
        "box_x": float(rng.uniform(0.90, 2.40)),
        "box_y": float(rng.uniform(-1.45, 1.45)),
        "box_vx": box_vx,
        "box_vy": box_vy,
        "goal_x": 2.40,
        "duration": BENCHMARK_DURATION,
        # Fewer samples make hundreds of trials practical while retaining the
        # same MPPI interface and cost function used by the viewer.
        "mppi_samples": 96,
        "mppi_horizon": 45,
        "mppi_seed": int(trial_seed ^ 0x5EED1234),
    }

    if profile == "baseline":
        # Baseline isolates scene/trajectory variation: no sensor or command
        # interface corruption, with the nominal D1-side limits.
        kwargs.update({
            "lidar_noise_std": 0.0,
            "lidar_dropout_rate": 0.0,
            "command_delay": 0.0,
        })
    elif profile == "sensor_noise":
        kwargs.update({
            "lidar_noise_std": float(rng.uniform(0.005, 0.025)),
            "lidar_dropout_rate": float(rng.uniform(0.02, 0.15)),
            "command_delay": 0.0,
        })
    else:
        kwargs.update({
            "box_size_x": float(rng.uniform(0.18, 0.34)),
            "box_size_y": float(rng.uniform(0.18, 0.34)),
            "robot_cruise_speed": float(rng.uniform(0.18, 0.28)),
            "lidar_noise_std": float(rng.uniform(0.005, 0.040)),
            "lidar_dropout_rate": float(rng.uniform(0.02, 0.20)),
            "command_delay": float(rng.uniform(0.0, 0.22)),
            "max_vx": float(rng.uniform(0.28, 0.42)),
            "max_vy": float(rng.uniform(0.16, 0.30)),
            "max_yaw_rate": float(rng.uniform(0.45, 0.75)),
            "max_ax": float(rng.uniform(0.40, 0.80)),
            "max_ay": float(rng.uniform(0.35, 0.70)),
            "max_yaw_acceleration": float(rng.uniform(1.00, 2.00)),
        })

    if scenario_mode == "wait_yield_curriculum":
        index = 0 if curriculum_index is None else int(curriculum_index)
        scenario = generate_wait_yield_curriculum_scenario(
            trial_seed,
            route_length=route_length,
            curriculum_index=index,
            max_obstacle_speed=1.5,
        )
        kwargs.update({
            "goal_x": route_length,
            "duration": BENCHMARK_DURATION,
            "curriculum_index": index,
            "curriculum_case": wait_yield_curriculum_case(index),
            "scenario_obstacles": tuple(
                obstacle.to_dict() for obstacle in scenario.obstacles
            ),
            "lidar_model": scenario.lidar_model,
            "lidar_ray_count": 121,
            "lidar_fov": np.pi,
            "lidar_max_range": scenario.lidar_max_range,
            "lidar_scan_rate_hz": scenario.lidar_scan_rate_hz,
            "max_obstacle_speed": scenario.max_obstacle_speed,
            "mppi_samples": 32,
            "mppi_horizon": 30,
            "clearance_uncertainty": 0.04,
            "obstacle_position_uncertainty": 0.03,
            "obstacle_velocity_uncertainty": 0.10,
            "robot_cruise_speed": 0.34,
            "max_vx": 0.44,
            "wait_max_clear_time": 3.00,
            "wait_max_duration": 2.50,
            "wait_max_clear_fraction": 0.95,
            "wait_cooldown": 0.60,
            # Curriculum credit should favour a complete WAIT -> clear ->
            # CRUISE sequence, not repeated WAIT requests while stationary.
            "wait_step_penalty": 0.050,
            "wait_no_progress_penalty": 0.080,
            "wait_invalid_penalty": 0.20,
            "wait_entry_bonus": 0.30,
            "wait_clear_bonus": 0.75,
            "wait_wrong_action_penalty": 0.35,
            "wait_wrong_cruise_penalty": 0.60,
        })
    elif scenario_mode in {"people_random_multi_target", "people_balanced_multi_target"}:
        scenario = (
            generate_people_balanced_multi_target_scenario(
                trial_seed,
                route_length=route_length,
                max_obstacle_speed=1.5,
                curriculum_index=0 if curriculum_index is None else curriculum_index,
            )
            if scenario_mode == "people_balanced_multi_target"
            else generate_people_random_multi_target_scenario(
                trial_seed,
                route_length=route_length,
                max_obstacle_speed=1.5,
            )
        )
        kwargs.update({
            "goal_x": route_length,
            "duration": BENCHMARK_DURATION,
            "scenario_obstacles": tuple(
                obstacle.to_dict() for obstacle in scenario.obstacles
            ),
            "lidar_model": scenario.lidar_model,
            "lidar_ray_count": 121,
            "lidar_fov": np.pi,
            "lidar_max_range": scenario.lidar_max_range,
            "lidar_scan_rate_hz": scenario.lidar_scan_rate_hz,
            "max_obstacle_speed": scenario.max_obstacle_speed,
            "mppi_samples": 96,
            "mppi_horizon": 45,
            "clearance_uncertainty": 0.06,
            "obstacle_position_uncertainty": 0.05,
            "obstacle_velocity_uncertainty": 0.16,
        })
    elif scenario_mode in {"multi_target", "irregular_multi_target", "people_multi_target"}:
        people_mode = scenario_mode == "people_multi_target"
        scenario = generate_multi_target_scenario(
            trial_seed,
            route_length=route_length,
            max_obstacle_speed=1.5,
            obstacle_count=3,
            irregular=scenario_mode in {"irregular_multi_target", "people_multi_target"},
            include_person=people_mode,
        )
        kwargs.update({
            "goal_x": route_length if route_length >= 5.0 else min(route_length, 2.40),
            # The multi-target route needs enough time to rejoin after a
            # safe detour.  Keep ``--duration`` available for controlled
            # comparisons with the historical 14-second benchmark.
            "duration": BENCHMARK_DURATION,
            "scenario_obstacles": tuple(
                obstacle.to_dict() for obstacle in scenario.obstacles
            ),
            "lidar_model": scenario.lidar_model,
            "lidar_ray_count": 121,
            "lidar_fov": np.pi,
            "lidar_max_range": scenario.lidar_max_range,
            "lidar_scan_rate_hz": scenario.lidar_scan_rate_hz,
            "max_obstacle_speed": scenario.max_obstacle_speed,
            "mppi_samples": 96,
            "mppi_horizon": 45,
        })
        if scenario_mode in {"irregular_multi_target", "people_multi_target"}:
            kwargs.update({
                "clearance_uncertainty": 0.06,
                "obstacle_position_uncertainty": 0.05,
                "obstacle_velocity_uncertainty": 0.16,
            })
    elif scenario_mode == "long_route":
        scenario = generate_long_route_scenario(
            trial_seed,
            route_length=route_length,
            max_obstacle_speed=7.0,
            obstacle_count=6,
        )
        kwargs.update({
            "goal_x": route_length,
            "duration": BENCHMARK_DURATION,
            "robot_cruise_speed": 0.30,
            "max_vx": 0.50,
            "max_vy": 0.35,
            "max_obstacle_speed": scenario.max_obstacle_speed,
            "scenario_obstacles": tuple(
                obstacle.to_dict() for obstacle in scenario.obstacles
            ),
            # RPLIDAR A2M8 nominal profile: 360 degrees, 0.45 degrees,
            # 12 m range, 10 Hz.  The sensor model remains configurable.
            "lidar_model": scenario.lidar_model,
            "lidar_ray_count": 800,
            "lidar_fov": 2.0 * np.pi,
            "lidar_max_range": scenario.lidar_max_range,
            "lidar_scan_rate_hz": scenario.lidar_scan_rate_hz,
            "mppi_samples": 64,
            "mppi_horizon": 45,
        })
    elif scenario_mode != "legacy":
        raise ValueError(f"unknown scenario_mode: {scenario_mode}")

    return EpisodeConfig(**kwargs)


def _result_row(
    trial: int,
    profile: str,
    config: EpisodeConfig,
    result: EpisodeResult,
) -> dict[str, Any]:
    metrics = result.decision_metrics or {}
    row: dict[str, Any] = {
        "trial": trial,
        "seed": result.seed,
        "profile": profile,
        "scenario_mode": config.scenario_mode,
        "route_length": config.route_length,
        "max_obstacle_speed": config.max_obstacle_speed,
        "lidar_model": config.lidar_model,
        **asdict(config),
        "success": result.success,
        "collision": result.collision,
        "geometric_collision": result.geometric_collision,
        "proxy_collision": result.proxy_collision,
        "proxy_only": result.proxy_only,
        "physical_contact": result.physical_contact,
        "near_miss": result.near_miss,
        "reached_goal": result.reached_goal,
        "timeout": result.timeout,
        "min_clearance": result.min_clearance,
        "min_proxy_clearance": result.min_proxy_clearance,
        "elapsed_time": result.elapsed_time,
        "path_length": result.path_length,
        "distance_to_goal": result.distance_to_goal,
        "decision_triggered": result.decision_triggered,
        "avoid_duration": result.avoid_duration,
        "tp": result.true_positive,
        "tn": result.true_negative,
        "fp": result.false_positive,
        "fn": result.false_negative,
        "failure_reason": result.failure_reason,
        "collision_obstacle_index": result.collision_obstacle_index,
        "contact_geom_name": result.contact_geom_name,
        "contact_distance": result.contact_distance,
        "contact_position": result.contact_position,
        "tracking_failure": result.tracking_failure,
        "recovery_triggered": result.recovery_triggered,
        "recovery_active_duration": result.recovery_active_duration,
        "decision_updates": metrics.get("decision_updates", 0),
        "action_counts": json.dumps(metrics.get("action_counts", {}), sort_keys=True),
        "action_proportions": json.dumps(
            metrics.get("action_proportions", {}), sort_keys=True, allow_nan=True
        ),
        "waitable_states": metrics.get("waitable_states", 0),
        "wait_requested_count": metrics.get("wait_requested_count", 0),
        "valid_wait_count": metrics.get("valid_wait_count", 0),
        "invalid_wait_count": metrics.get("invalid_wait_count", 0),
        "wait_argmax_rate": metrics.get("wait_argmax_rate", float("nan")),
        "invalid_wait_rate": metrics.get("invalid_wait_rate", 0.0),
        "waitable_avoid_count": metrics.get("waitable_avoid_count", 0),
        "waitable_avoid_rate": metrics.get("waitable_avoid_rate", float("nan")),
        "mean_consecutive_wait_duration": metrics.get(
            "mean_consecutive_wait_duration", 0.0
        ),
        "wait_followed_progress_count": metrics.get(
            "wait_followed_progress_count", 0
        ),
        "wait_returned_to_cruise_count": metrics.get(
            "wait_returned_to_cruise_count", 0
        ),
        "near_goal_timeout": metrics.get("near_goal_timeout", False),
        "real_deadlock": metrics.get("real_deadlock", False),
    }
    return {field: row[field] for field in CSV_FIELDS}


def _safe_divide(numerator: float, denominator: float) -> float:
    return float(numerator / denominator) if denominator else float("nan")


def summarize_results(results: list[EpisodeResult]) -> dict[str, Any]:
    total = len(results)
    collision_count = sum(result.collision for result in results)
    geometric_collision_count = sum(result.geometric_collision for result in results)
    proxy_collision_count = sum(result.proxy_collision for result in results)
    proxy_only_count = sum(result.proxy_only for result in results)
    physical_contact_count = sum(result.physical_contact for result in results)
    near_miss_count = sum(result.near_miss for result in results)
    timeout_count = sum(result.timeout for result in results)
    clearance_count = sum(result.failure_reason == "clearance" for result in results)
    goal_count = sum(result.failure_reason == "goal" for result in results)
    tracking_count = sum(result.failure_reason == "tracking" for result in results)
    decision_count = sum(result.decision_triggered for result in results)
    recovery_count = sum(result.recovery_triggered for result in results)
    recovery_durations = np.asarray(
        [result.recovery_active_duration for result in results], dtype=float
    )
    tp = sum(result.true_positive for result in results)
    tn = sum(result.true_negative for result in results)
    fp = sum(result.false_positive for result in results)
    fn = sum(result.false_negative for result in results)
    min_clearances = np.asarray([result.min_clearance for result in results], dtype=float)
    min_proxy_clearances = np.asarray(
        [result.min_proxy_clearance for result in results],
        dtype=float,
    )
    elapsed = np.asarray([result.elapsed_time for result in results], dtype=float)
    path_lengths = np.asarray([result.path_length for result in results], dtype=float)
    metrics = [result.decision_metrics or {} for result in results]
    action_counts: dict[str, int] = {}
    for metric in metrics:
        for name, count in metric.get("action_counts", {}).items():
            action_counts[name] = action_counts.get(name, 0) + int(count)
    decision_updates = sum(int(metric.get("decision_updates", 0)) for metric in metrics)
    waitable_states = sum(int(metric.get("waitable_states", 0)) for metric in metrics)
    valid_wait_count = sum(int(metric.get("valid_wait_count", 0)) for metric in metrics)
    invalid_wait_count = sum(int(metric.get("invalid_wait_count", 0)) for metric in metrics)
    waitable_avoid_count = sum(int(metric.get("waitable_avoid_count", 0)) for metric in metrics)
    near_goal_timeout_count = sum(bool(metric.get("near_goal_timeout", False)) for metric in metrics)
    real_deadlock_count = sum(bool(metric.get("real_deadlock", False)) for metric in metrics)

    return {
        "trials": total,
        "successful_trials": sum(result.success for result in results),
        "success_rate": _safe_divide(sum(result.success for result in results), total),
        "collision_rate": _safe_divide(collision_count, total),
        "geometric_collision_rate": _safe_divide(geometric_collision_count, total),
        "proxy_collision_rate": _safe_divide(proxy_collision_count, total),
        "proxy_only_rate": _safe_divide(proxy_only_count, total),
        "physical_contact_rate": _safe_divide(physical_contact_count, total),
        "near_miss_rate": _safe_divide(near_miss_count, total),
        "timeout_rate": _safe_divide(timeout_count, total),
        "clearance_failure_rate": _safe_divide(clearance_count, total),
        "goal_failure_rate": _safe_divide(goal_count, total),
        "tracking_failure_rate": _safe_divide(tracking_count, total),
        "decision_trigger_rate": _safe_divide(decision_count, total),
        "recovery_trigger_rate": _safe_divide(recovery_count, total),
        "mean_recovery_active_duration": (
            float(np.mean(recovery_durations)) if total else float("nan")
        ),
        "true_positive": tp,
        "true_negative": tn,
        "false_positive": fp,
        "false_negative": fn,
        "collision_recall": _safe_divide(tp, tp + fn),
        "false_negative_rate": _safe_divide(fn, tp + fn),
        "false_alarm_rate": _safe_divide(fp, fp + tn),
        "mean_min_clearance": float(np.mean(min_clearances)) if total else float("nan"),
        "mean_min_proxy_clearance": (
            float(np.mean(min_proxy_clearances)) if total else float("nan")
        ),
        "median_min_clearance": float(np.median(min_clearances)) if total else float("nan"),
        "p05_min_clearance": float(np.percentile(min_clearances, 5)) if total else float("nan"),
        "mean_elapsed_time": float(np.mean(elapsed)) if total else float("nan"),
        "mean_path_length": float(np.mean(path_lengths)) if total else float("nan"),
        "action_counts": action_counts,
        "action_proportions": {
            name: _safe_divide(count, decision_updates)
            for name, count in action_counts.items()
        },
        "decision_updates": decision_updates,
        "waitable_states": waitable_states,
        "valid_wait_count": valid_wait_count,
        "invalid_wait_count": invalid_wait_count,
        "wait_argmax_rate": _safe_divide(valid_wait_count, waitable_states),
        "invalid_wait_rate": _safe_divide(invalid_wait_count, invalid_wait_count + valid_wait_count),
        "waitable_avoid_count": waitable_avoid_count,
        "waitable_avoid_rate": _safe_divide(waitable_avoid_count, waitable_states),
        "near_goal_timeout_count": near_goal_timeout_count,
        "near_goal_timeout_rate": _safe_divide(near_goal_timeout_count, total),
        "real_deadlock_count": real_deadlock_count,
        "real_deadlock_rate": _safe_divide(real_deadlock_count, total),
    }


def _failure_sort_key(
    item: tuple[int, EpisodeConfig, EpisodeResult],
) -> tuple[int, float, int]:
    _, _, result = item
    # Collision is always ranked ahead of non-collision failures, then the
    # smallest clearance, then the largest actionable FN count.
    return (0 if result.collision else 1, result.min_clearance, -result.false_negative)


def run_benchmark(
    trials: int,
    profile: str,
    seed: int,
    output: str | Path,
    route_length: float = 3.20,
    scenario_mode: str = "legacy",
    save_failure_traces: bool = True,
    duration: float | None = None,
    cruise_mode: str = "goal_hysteresis",
) -> dict[str, Any]:
    if trials <= 0:
        raise ValueError("trials must be positive")
    if duration is not None and duration <= 0.0:
        raise ValueError("duration override must be positive")
    output_path = Path(output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    failure_dir = output_path.parent / "failures"
    failure_dir.mkdir(parents=True, exist_ok=True)

    master_rng = np.random.default_rng(seed)
    records: list[tuple[int, EpisodeConfig, EpisodeResult]] = []
    rows: list[dict[str, Any]] = []
    for trial in range(trials):
        trial_seed = int(master_rng.integers(0, 2**31 - 1))
        config = sample_episode_config(
            profile,
            master_rng,
            trial_seed,
            route_length=route_length,
            scenario_mode=scenario_mode,
            curriculum_index=trial,
        )
        if duration is not None:
            config = replace(config, duration=float(duration))
        config = replace(config, cruise_mode=cruise_mode)
        result = run_episode(config, seed=trial_seed)
        records.append((trial, config, result))
        rows.append(_result_row(trial, profile, config, result))

    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)

    failure_cases = sorted(
        [record for record in records if not record[2].success],
        key=_failure_sort_key,
    )[:20]
    failure_paths: list[str] = []
    for trial, config, result in failure_cases:
        failure_path = failure_dir / f"trial_{trial:04d}.json"
        # Keep the normal Monte Carlo loop light when trace saving is
        # disabled.  The default stores the primary 40-second trace for the
        # top failures so they can be plotted and replayed later.
        traced_result = (
            run_episode(
                config,
                seed=result.seed,
                record_trace=True,
            )
            if save_failure_traces
            else result
        )
        result_payload = asdict(traced_result)
        replay_payload = {
            "trial": trial,
            "seed": result.seed,
            "profile": profile,
            "config": asdict(config),
            "result": result_payload,
        }
        with failure_path.open("w", encoding="utf-8") as handle:
            json.dump(replay_payload, handle, indent=2, allow_nan=True)
        failure_paths.append(str(failure_path))

    summary = summarize_results([record[2] for record in records])
    summary.update({
        "profile": profile,
        "scenario_mode": scenario_mode,
        "route_length": route_length,
        "duration_override": duration,
        "seed": seed,
        "output_csv": str(output_path),
        "failure_cases": failure_paths,
    })
    summary_path = output_path.parent / "d1_decision_benchmark_summary.json"
    with summary_path.open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2, allow_nan=True)

    print(f"trials={summary['trials']}")
    for name in (
        "success_rate",
        "collision_rate",
        "geometric_collision_rate",
        "proxy_collision_rate",
        "proxy_only_rate",
        "physical_contact_rate",
        "near_miss_rate",
        "timeout_rate",
        "clearance_failure_rate",
        "goal_failure_rate",
        "decision_trigger_rate",
        "recovery_trigger_rate",
        "mean_recovery_active_duration",
        "collision_recall",
        "false_negative_rate",
        "false_alarm_rate",
        "mean_min_clearance",
        "mean_min_proxy_clearance",
        "median_min_clearance",
        "p05_min_clearance",
        "mean_elapsed_time",
        "mean_path_length",
        "wait_argmax_rate",
        "invalid_wait_rate",
        "waitable_avoid_rate",
        "near_goal_timeout_rate",
        "real_deadlock_rate",
    ):
        print(f"{name}={summary[name]}")
    print(f"csv={output_path}")
    print(f"summary_json={summary_path}")
    print(f"saved_failure_cases={len(failure_paths)}")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=500)
    parser.add_argument(
        "--profile",
        choices=("baseline", "sensor_noise", "full_randomization"),
        default="baseline",
    )
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--route-length", type=float, default=3.20)
    parser.add_argument(
        "--duration",
        type=float,
        default=BENCHMARK_DURATION,
        help="episode time limit in seconds (default: 40)",
    )
    parser.add_argument(
        "--scenario-mode",
        choices=("legacy", "multi_target", "irregular_multi_target", "people_multi_target", "people_random_multi_target", "people_balanced_multi_target", "wait_yield_curriculum", "long_route"),
        default="legacy",
    )
    parser.add_argument(
        "--cruise-mode",
        choices=("forward_only", "goal_hysteresis"),
        default="goal_hysteresis",
        help="CRUISE implementation used by the benchmark",
    )
    parser.add_argument(
        "--no-failure-traces",
        action="store_true",
        help="skip expensive trajectory replays when only aggregate metrics are needed",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="output/d1_decision_benchmark.csv",
    )
    args = parser.parse_args()
    run_benchmark(
        args.trials,
        args.profile,
        args.seed,
        args.output,
        route_length=args.route_length,
        scenario_mode=args.scenario_mode,
        save_failure_traces=not args.no_failure_traces,
        duration=args.duration,
        cruise_mode=args.cruise_mode,
    )


if __name__ == "__main__":
    main()
