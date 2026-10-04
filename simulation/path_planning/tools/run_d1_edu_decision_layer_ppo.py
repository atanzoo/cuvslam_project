#!/usr/bin/env python3
"""Train and evaluate a small NumPy PPO policy for high-level D1 decisions.

The policy chooses one of five high-level actions while the existing MPPI and
SimulatedD1SDK remain responsible for local motion generation and command
limits.  This file intentionally has no PyTorch/Gym dependency so it can run
in the current project virtual environment on the Mac.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, replace
import json
from pathlib import Path
from typing import Any

import numpy as np

from run_d1_edu_decision_layer_benchmark import (
    CSV_FIELDS,
    sample_episode_config,
    summarize_results,
)
from run_d1_edu_decision_layer_viewer import (
    EpisodeConfig,
    EpisodeResult,
    run_episode,
)


# 4 robot/goal values + 3 ranked target slots * 10 values + risk + elapsed
# + 7 global-path reference values.
TARGET_FEATURE_DIM = 10
MAX_TARGETS = 3
GLOBAL_PATH_FEATURE_DIM = 7
WAIT_YIELD_FEATURE_DIM = 9
OBSERVATION_DIM = (
    4 + MAX_TARGETS * TARGET_FEATURE_DIM + 2
    + GLOBAL_PATH_FEATURE_DIM + WAIT_YIELD_FEATURE_DIM
)
RISK_OBSERVATION_INDEX = 4 + MAX_TARGETS * TARGET_FEATURE_DIM
WAIT_YIELD_FEATURE_START = RISK_OBSERVATION_INDEX + 2 + GLOBAL_PATH_FEATURE_DIM
ACTION_COUNT = 5
ACTION_NAMES = ("CRUISE", "AVOID_LEFT", "AVOID_RIGHT", "SLOWDOWN", "WAIT_YIELD")


class NumpyCategoricalPPO:
    """A compact linear categorical PPO implementation for this task."""

    def __init__(
        self,
        observation_dim: int = OBSERVATION_DIM,
        action_count: int = ACTION_COUNT,
        seed: int = 0,
        learning_rate: float = 0.035,
        value_learning_rate: float = 0.05,
        clip_ratio: float = 0.20,
    ) -> None:
        self.observation_dim = observation_dim
        self.action_count = action_count
        self.learning_rate = learning_rate
        self.value_learning_rate = value_learning_rate
        self.clip_ratio = clip_ratio
        self.rng = np.random.default_rng(seed)
        self.policy_w = self.rng.normal(0.0, 0.03, (observation_dim, action_count))
        self.policy_b = np.zeros(action_count, dtype=float)
        # Start from the existing safe default: continue toward the goal
        # unless the observation indicates danger.  PPO can still move this
        # prior as training discovers useful avoidance decisions.
        self.policy_b[0] = 1.20
        self.value_w = np.zeros(observation_dim, dtype=float)
        self.value_b = 0.0
        self.training = True
        self._episode: list[tuple[np.ndarray, int, float, float]] = []
        self._rollout: list[tuple[np.ndarray, int, float, float, float, float]] = []

    @staticmethod
    def _softmax(logits: np.ndarray) -> np.ndarray:
        shifted = logits - np.max(logits)
        values = np.exp(shifted)
        return values / (np.sum(values) + 1e-12)

    def _distribution(self, observation: np.ndarray) -> tuple[np.ndarray, float]:
        obs = np.asarray(observation, dtype=float).reshape(self.observation_dim)
        probabilities = self._softmax(obs @ self.policy_w + self.policy_b)
        value = float(obs @ self.value_w + self.value_b)
        return probabilities, value

    def act(self, observation: np.ndarray) -> int:
        probabilities, value = self._distribution(observation)
        if self.training:
            action = int(self.rng.choice(self.action_count, p=probabilities))
        else:
            action = int(np.argmax(probabilities))
        log_probability = float(np.log(probabilities[action] + 1e-12))
        self._episode.append((
            np.asarray(observation, dtype=float).copy(),
            action,
            log_probability,
            value,
        ))
        return action

    def reset_episode(self) -> None:
        """Drop transient action history before a new rollout/replay."""
        self._episode.clear()

    def finish_episode(self, result: EpisodeResult) -> float:
        """Convert one episode result into a shaped PPO return sequence."""
        if not self._episode:
            return 0.0
        if result.success:
            terminal_reward = 2.0
        elif result.collision:
            terminal_reward = -2.0
        elif result.timeout:
            terminal_reward = -5.0
        elif result.failure_reason == "tracking":
            terminal_reward = -0.8
        else:
            terminal_reward = -0.7

        rewards = np.full(len(self._episode), -0.008, dtype=float)
        actions = np.asarray([item[1] for item in self._episode], dtype=int)
        observations = np.asarray([item[0] for item in self._episode], dtype=float)
        risk_observation = observations[:, RISK_OBSERVATION_INDEX] > 0.5
        safe_observation = ~risk_observation
        rewards += np.where(risk_observation & np.isin(actions, [1, 2]), 0.080, 0.0)
        rewards -= np.where(risk_observation & np.isin(actions, [0, 3]), 0.080, 0.0)
        rewards += np.where(safe_observation & (actions == 0), 0.025, 0.0)
        rewards -= np.where(safe_observation & np.isin(actions, [1, 2]), 0.060, 0.0)
        rewards -= np.where(actions == 3, 0.020, 0.0)
        path_blocked_now = observations[:, WAIT_YIELD_FEATURE_START] > 0.5
        path_blocked_future = observations[:, WAIT_YIELD_FEATURE_START + 1] > 0.5
        waitable = observations[:, WAIT_YIELD_FEATURE_START + 7] > 0.5
        stationary_risk = observations[:, WAIT_YIELD_FEATURE_START + 8] > 0.5
        # Do not provide a recurring positive reward for stopping.  That
        # reward was exploitable and produced the high WAIT/timeout rate.
        rewards += np.where(waitable & ~stationary_risk & (actions == 4), 0.08, 0.0)
        rewards -= np.where(actions == 4, 0.025, 0.0)
        rewards -= np.where(
            (~waitable | stationary_risk) & (actions == 4),
            0.12,
            0.0,
        )
        rewards -= np.where(
            waitable & ~stationary_risk & np.isin(actions, [0, 3]),
            0.08,
            0.0,
        )
        rewards -= np.where(
            waitable & ~stationary_risk & np.isin(actions, [1, 2]),
            0.10,
            0.0,
        )
        rewards -= np.where(
            path_blocked_now & safe_observation & np.isin(actions, [0, 3]),
            0.06,
            0.0,
        )
        rewards[-1] += terminal_reward

        gamma = 0.985
        returns = np.zeros(len(rewards), dtype=float)
        running = 0.0
        for index in range(len(rewards) - 1, -1, -1):
            running = rewards[index] + gamma * running
            returns[index] = running
        values = np.asarray([item[3] for item in self._episode], dtype=float)
        advantages = returns - values
        for item, ret, advantage in zip(self._episode, returns, advantages):
            self._rollout.append((item[0], item[1], item[2], item[3], ret, advantage))
        self._episode.clear()
        return float(np.sum(rewards))

    def update(self, epochs: int = 8) -> None:
        if not self._rollout:
            return
        observations = np.asarray([item[0] for item in self._rollout], dtype=float)
        actions = np.asarray([item[1] for item in self._rollout], dtype=int)
        old_logp = np.asarray([item[2] for item in self._rollout], dtype=float)
        returns = np.asarray([item[4] for item in self._rollout], dtype=float)
        advantages = np.asarray([item[5] for item in self._rollout], dtype=float)
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)

        count = len(observations)
        for _ in range(epochs):
            logits = observations @ self.policy_w + self.policy_b
            logits -= np.max(logits, axis=1, keepdims=True)
            probabilities = np.exp(logits)
            probabilities /= np.sum(probabilities, axis=1, keepdims=True) + 1e-12
            new_logp = np.log(probabilities[np.arange(count), actions] + 1e-12)
            ratios = np.exp(new_logp - old_logp)
            clipped = np.clip(ratios, 1.0 - self.clip_ratio, 1.0 + self.clip_ratio)
            active = ~(
                ((advantages >= 0.0) & (ratios > 1.0 + self.clip_ratio))
                | ((advantages < 0.0) & (ratios < 1.0 - self.clip_ratio))
            )
            coefficient = np.where(active, ratios * advantages, 0.0)
            one_hot = np.zeros_like(probabilities)
            one_hot[np.arange(count), actions] = 1.0
            grad_logits = coefficient[:, None] * (one_hot - probabilities)
            grad_w = observations.T @ grad_logits / count
            grad_b = grad_logits.mean(axis=0)
            self.policy_w += self.learning_rate * grad_w
            self.policy_b += self.learning_rate * grad_b

            values = observations @ self.value_w + self.value_b
            value_error = values - returns
            self.value_w -= self.value_learning_rate * (observations.T @ value_error / count)
            self.value_b -= self.value_learning_rate * float(value_error.mean())
        self._rollout.clear()

    def save(self, path: str | Path) -> None:
        np.savez(
            path,
            policy_w=self.policy_w,
            policy_b=self.policy_b,
            value_w=self.value_w,
            value_b=np.asarray([self.value_b]),
        )

    def load(self, path: str | Path) -> None:
        payload = np.load(path)
        self.policy_w = payload["policy_w"]
        self.policy_b = payload["policy_b"]
        self.value_w = payload["value_w"]
        self.value_b = float(payload["value_b"][0])


def _training_config(
    profile: str,
    rng: np.random.Generator,
    trial_seed: int,
    route_length: float = 3.20,
    scenario_mode: str = "legacy",
    cruise_mode: str = "goal_hysteresis",
) -> EpisodeConfig:
    config = sample_episode_config(
        profile,
        rng,
        trial_seed,
        route_length=route_length,
        scenario_mode=scenario_mode,
    )
    if scenario_mode == "long_route":
        return replace(
            config,
            cruise_mode=cruise_mode,
            mppi_samples=32,
            mppi_horizon=30,
        )
    return replace(
        config,
        cruise_mode=cruise_mode,
        duration=40.0,
        goal_x=2.0,
        mppi_samples=32,
        mppi_horizon=30,
    )


def train_policy(
    policy: NumpyCategoricalPPO,
    episodes: int,
    profile: str,
    seed: int,
    route_length: float = 3.20,
    scenario_mode: str = "legacy",
    cruise_mode: str = "goal_hysteresis",
) -> list[dict[str, Any]]:
    rng = np.random.default_rng(seed)
    records: list[dict[str, Any]] = []
    for episode in range(episodes):
        episode_seed = int(rng.integers(0, 2**31 - 1))
        config = _training_config(
            profile,
            rng,
            episode_seed,
            route_length=route_length,
            scenario_mode=scenario_mode,
            cruise_mode=cruise_mode,
        )
        result = run_episode(config, episode_seed, decision_policy=policy)
        episode_return = policy.finish_episode(result)
        records.append({
            "episode": episode + 1,
            "seed": episode_seed,
            "return": float(episode_return),
            "success": bool(result.success),
            "collision": bool(result.collision),
            "timeout": bool(result.timeout),
            "reached_goal": bool(result.reached_goal),
            "failure_reason": result.failure_reason,
            "min_clearance": float(result.min_clearance),
            "distance_to_goal": float(result.distance_to_goal),
            "decision_triggered": bool(result.decision_triggered),
            "avoid_duration": float(result.avoid_duration),
            "policy_update": False,
        })
        if (episode + 1) % 16 == 0:
            policy.update()
            records[-1]["policy_update"] = True
            print(
                f"train_episode={episode + 1} "
                f"mean_return={np.mean([item['return'] for item in records[-16:]]):+.3f} "
                f"success_rate={np.mean([item['success'] for item in records[-16:]]):.3f}"
            )
    policy.update()
    for index, record in enumerate(records):
        window = records[max(0, index - 15):index + 1]
        record["rolling_mean_return"] = float(np.mean([
            item["return"] for item in window
        ]))
        record["rolling_success_rate"] = float(np.mean([
            item["success"] for item in window
        ]))
        record["rolling_timeout_rate"] = float(np.mean([
            item["timeout"] for item in window
        ]))
    return records


def write_learning_curve_csv(
    records: list[dict[str, Any]],
    path: str | Path,
) -> None:
    """Persist one row per training episode for reproducible curve analysis."""
    if not records:
        raise ValueError("learning curve cannot be empty")
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(records[0].keys())
    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(records)


def write_learning_curve_svg(
    records: list[dict[str, Any]],
    path: str | Path,
) -> None:
    """Write a dependency-free SVG showing return and task-rate curves."""
    if not records:
        raise ValueError("learning curve cannot be empty")
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    width, height = 1100, 680
    left, right = 75, 30
    top, gap, panel_height = 55, 70, 235
    plot_width = width - left - right
    returns = np.asarray([record["return"] for record in records], dtype=float)
    rolling_returns = np.asarray([
        record["rolling_mean_return"] for record in records
    ], dtype=float)
    y_min = float(min(returns.min(), rolling_returns.min()))
    y_max = float(max(returns.max(), rolling_returns.max()))
    if np.isclose(y_min, y_max):
        y_min -= 1.0
        y_max += 1.0

    def points(values: np.ndarray, y0: float, panel_min: float, panel_max: float) -> str:
        x_values = np.linspace(left, left + plot_width, len(values))
        y_values = y0 + panel_height - (
            (values - panel_min) / max(panel_max - panel_min, 1e-9)
        ) * panel_height
        return " ".join(f"{x:.1f},{y:.1f}" for x, y in zip(x_values, y_values))

    success = np.asarray([
        record["rolling_success_rate"] for record in records
    ], dtype=float)
    timeout = np.asarray([
        record["rolling_timeout_rate"] for record in records
    ], dtype=float)
    second_y = top + panel_height + gap
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        '<style>text{font-family:Arial,sans-serif;fill:#1e293b} .axis{stroke:#94a3b8;stroke-width:1} .grid{stroke:#e2e8f0;stroke-width:1}</style>',
        '<text x="75" y="30" font-size="20" font-weight="bold">PPO learning curve</text>',
        f'<line class="axis" x1="{left}" y1="{top + panel_height}" x2="{left + plot_width}" y2="{top + panel_height}"/>',
        f'<line class="axis" x1="{left}" y1="{top}" x2="{left}" y2="{top + panel_height}"/>',
        f'<polyline fill="none" stroke="#94a3b8" stroke-width="1" points="{points(returns, top, y_min, y_max)}"/>',
        f'<polyline fill="none" stroke="#2563eb" stroke-width="3" points="{points(rolling_returns, top, y_min, y_max)}"/>',
        f'<text x="{left}" y="{top - 12}" font-size="14">return (blue: rolling 16-episode mean)</text>',
        f'<text x="{left}" y="{top + panel_height + 28}" font-size="12">episode 1</text>',
        f'<text x="{left + plot_width - 65}" y="{top + panel_height + 28}" font-size="12">episode {len(records)}</text>',
        f'<line class="axis" x1="{left}" y1="{second_y + panel_height}" x2="{left + plot_width}" y2="{second_y + panel_height}"/>',
        f'<line class="axis" x1="{left}" y1="{second_y}" x2="{left}" y2="{second_y + panel_height}"/>',
        f'<polyline fill="none" stroke="#16a34a" stroke-width="3" points="{points(success, second_y, 0.0, 1.0)}"/>',
        f'<polyline fill="none" stroke="#dc2626" stroke-width="3" points="{points(timeout, second_y, 0.0, 1.0)}"/>',
        f'<text x="{left}" y="{second_y - 12}" font-size="14">rolling success (green) / timeout (red)</text>',
        '<text x="900" y="645" font-size="12">window = 16 episodes</text>',
        '</svg>',
    ]
    output_path.write_text("\n".join(svg), encoding="utf-8")


def _row(
    trial: int,
    profile: str,
    controller: str,
    config: EpisodeConfig,
    result: EpisodeResult,
) -> dict[str, Any]:
    row = {
        "trial": trial,
        "seed": result.seed,
        "profile": profile,
        "controller": controller,
        **asdict(config),
        "success": result.success,
        "collision": result.collision,
        "reached_goal": result.reached_goal,
        "timeout": result.timeout,
        "min_clearance": result.min_clearance,
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
        "tracking_failure": result.tracking_failure,
    }
    return {field: row[field] for field in ["controller"] + CSV_FIELDS}


def evaluate(
    policy: NumpyCategoricalPPO,
    trials: int,
    profile: str,
    seed: int,
    route_length: float = 3.20,
    scenario_mode: str = "legacy",
    cruise_mode: str = "goal_hysteresis",
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    rng = np.random.default_rng(seed)
    baseline_results: list[EpisodeResult] = []
    ppo_results: list[EpisodeResult] = []
    rows: list[dict[str, Any]] = []
    policy.training = False
    for trial in range(trials):
        trial_seed = int(rng.integers(0, 2**31 - 1))
        config = sample_episode_config(
            profile,
            rng,
            trial_seed,
            route_length=route_length,
            scenario_mode=scenario_mode,
            cruise_mode=cruise_mode,
        )
        baseline = run_episode(config, trial_seed)
        ppo = run_episode(
            config,
            trial_seed,
            decision_policy=policy,
        )
        baseline_results.append(baseline)
        ppo_results.append(ppo)
        rows.extend([
            _row(trial, profile, "baseline", config, baseline),
            _row(trial, profile, "ppo", config, ppo),
        ])
    summary = {
        "trials": trials,
        "profile": profile,
        "seed": seed,
        "baseline": summarize_results(baseline_results),
        "ppo": summarize_results(ppo_results),
        "success_rate_delta": (
            summarize_results(ppo_results)["success_rate"]
            - summarize_results(baseline_results)["success_rate"]
        ),
        "timeout_rate_delta": (
            summarize_results(ppo_results)["timeout_rate"]
            - summarize_results(baseline_results)["timeout_rate"]
        ),
    }
    return summary, rows


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--train-episodes", type=int, default=500)
    parser.add_argument("--eval-trials", type=int, default=100)
    parser.add_argument(
        "--profile",
        choices=("baseline", "sensor_noise", "full_randomization"),
        default="baseline",
    )
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--route-length", type=float, default=3.20)
    parser.add_argument(
        "--scenario-mode",
        choices=("legacy", "multi_target", "irregular_multi_target", "people_multi_target", "people_random_multi_target", "long_route"),
        default="multi_target",
    )
    parser.add_argument(
        "--cruise-mode",
        choices=("forward_only", "goal_hysteresis"),
        default="goal_hysteresis",
        help="CRUISE implementation used during PPO training and evaluation",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="output/d1_decision_ppo_500.csv",
    )
    parser.add_argument(
        "--policy",
        type=str,
        default="output/d1_decision_ppo_500_policy.npz",
    )
    parser.add_argument(
        "--learning-curve",
        type=str,
        default="output/d1_decision_ppo_learning_curve_500.csv",
    )
    parser.add_argument(
        "--learning-curve-svg",
        type=str,
        default="output/d1_decision_ppo_learning_curve_500.svg",
    )
    args = parser.parse_args()
    if args.train_episodes <= 0 or args.eval_trials <= 0:
        raise ValueError("train and evaluation counts must be positive")

    policy = NumpyCategoricalPPO(seed=args.seed)
    learning_records = train_policy(
        policy,
        args.train_episodes,
        args.profile,
        args.seed + 1,
        route_length=args.route_length,
        scenario_mode=args.scenario_mode,
        cruise_mode=args.cruise_mode,
    )
    learning_curve_path = Path(args.learning_curve)
    write_learning_curve_csv(learning_records, learning_curve_path)
    learning_curve_svg_path = Path(args.learning_curve_svg)
    write_learning_curve_svg(learning_records, learning_curve_svg_path)
    policy_path = Path(args.policy)
    policy_path.parent.mkdir(parents=True, exist_ok=True)
    policy.save(policy_path)

    summary, rows = evaluate(
        policy,
        args.eval_trials,
        args.profile,
        args.seed + 2,
        route_length=args.route_length,
        scenario_mode=args.scenario_mode,
        cruise_mode=args.cruise_mode,
    )
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = ["controller"] + CSV_FIELDS
    with output_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    summary_path = output_path.parent / f"{output_path.stem}_summary.json"
    summary.update({
        "output_csv": str(output_path),
        "policy": str(policy_path),
        "train_episodes": args.train_episodes,
        "learning_curve_csv": str(learning_curve_path),
        "learning_curve_svg": str(learning_curve_svg_path),
        "training_final_16_episode_mean_return": learning_records[-1]["rolling_mean_return"],
        "training_final_16_episode_success_rate": learning_records[-1]["rolling_success_rate"],
        "training_final_16_episode_timeout_rate": learning_records[-1]["rolling_timeout_rate"],
        "scenario_mode": args.scenario_mode,
        "route_length": args.route_length,
        "cruise_mode": args.cruise_mode,
    })
    with summary_path.open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2, allow_nan=True)

    for controller in ("baseline", "ppo"):
        stats = summary[controller]
        print(
            f"{controller}: success_rate={stats['success_rate']} "
            f"collision_rate={stats['collision_rate']} "
            f"timeout_rate={stats['timeout_rate']} "
            f"false_alarm_rate={stats['false_alarm_rate']}"
        )
    print(f"success_rate_delta={summary['success_rate_delta']}")
    print(f"timeout_rate_delta={summary['timeout_rate_delta']}")
    print(f"csv={output_path}")
    print(f"summary_json={summary_path}")
    print(f"policy={policy_path}")
    print(f"learning_curve_csv={learning_curve_path}")
    print(f"learning_curve_svg={learning_curve_svg_path}")


if __name__ == "__main__":
    main()
