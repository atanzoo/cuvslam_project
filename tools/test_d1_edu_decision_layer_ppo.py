#!/usr/bin/env python3
"""Small dependency-free tests for the PPO decision controller."""

from __future__ import annotations

from pathlib import Path
import tempfile

import numpy as np

from run_d1_edu_decision_layer_ppo import (
    NumpyCategoricalPPO,
    OBSERVATION_DIM,
    train_policy,
    write_learning_curve_csv,
    write_learning_curve_svg,
)
from run_d1_edu_decision_layer_viewer import EpisodeConfig, EpisodeResult, run_episode


def test_actions_are_bounded_and_reproducible() -> None:
    first = NumpyCategoricalPPO(seed=4)
    second = NumpyCategoricalPPO(seed=4)
    first.training = False
    second.training = False
    observation = np.zeros(OBSERVATION_DIM, dtype=float)
    assert first.act(observation) == second.act(observation)
    assert first.act(observation) in range(5)


def test_ppo_update_and_save_load() -> None:
    policy = NumpyCategoricalPPO(seed=5)
    for _ in range(4):
        policy.act(np.zeros(OBSERVATION_DIM, dtype=float))
    result = EpisodeResult(
        success=True,
        collision=False,
        reached_goal=True,
        timeout=False,
        min_clearance=0.4,
        elapsed_time=1.0,
        path_length=0.5,
        decision_triggered=False,
        avoid_duration=0.0,
        true_positive=0,
        true_negative=4,
        false_positive=0,
        false_negative=0,
        seed=5,
        failure_reason="none",
    )
    policy.finish_episode(result)
    policy.update()
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "policy.npz"
        policy.save(path)
        restored = NumpyCategoricalPPO(seed=99)
        restored.load(path)
        np.testing.assert_allclose(restored.policy_w, policy.policy_w)
        np.testing.assert_allclose(restored.policy_b, policy.policy_b)


def test_policy_can_be_attached_to_episode() -> None:
    policy = NumpyCategoricalPPO(seed=7)
    policy.training = False
    config = EpisodeConfig(
        box_x=2.8,
        box_y=1.2,
        box_vy=0.2,
        duration=0.25,
        goal_x=0.8,
        mppi_samples=4,
        mppi_horizon=5,
        command_delay=0.0,
    )
    result = run_episode(config, seed=7, decision_policy=policy)
    assert result.failure_reason in {"none", "timeout", "goal", "tracking"}
    assert all(mode in {"CRUISE", "AVOID", "WAIT_YIELD"} for mode in result.mode_history)


def test_wait_yield_action_sends_zero_target_through_sdk() -> None:
    class WaitPolicy:
        def reset_episode(self) -> None:
            return None

        def act(self, observation: np.ndarray) -> int:
            return 4

    result = run_episode(
        EpisodeConfig(
            box_x=2.8,
            box_y=1.3,
            box_vy=0.2,
            duration=0.25,
            goal_x=0.8,
            mppi_samples=4,
            mppi_horizon=5,
            command_delay=0.0,
        ),
        seed=11,
        decision_policy=WaitPolicy(),
        record_trace=True,
    )
    assert "WAIT_YIELD" in result.mode_history
    assert result.trace
    assert np.allclose(result.trace[0]["command"], np.zeros(3))


def test_learning_curve_records_and_writes() -> None:
    policy = NumpyCategoricalPPO(seed=8)
    records = train_policy(
        policy,
        episodes=2,
        profile="baseline",
        seed=9,
        scenario_mode="legacy",
    )
    assert len(records) == 2
    assert "rolling_mean_return" in records[-1]
    with tempfile.TemporaryDirectory() as directory:
        csv_path = Path(directory) / "learning_curve.csv"
        svg_path = Path(directory) / "learning_curve.svg"
        write_learning_curve_csv(records, csv_path)
        write_learning_curve_svg(records, svg_path)
        assert csv_path.exists()
        assert svg_path.exists()
        assert "PPO learning curve" in svg_path.read_text(encoding="utf-8")


def main() -> None:
    tests = [
        test_actions_are_bounded_and_reproducible,
        test_ppo_update_and_save_load,
        test_policy_can_be_attached_to_episode,
        test_learning_curve_records_and_writes,
    ]
    for test in tests:
        test()
        print(f"PASS {test.__name__}")
    print("status=PASS: PPO decision-layer suite")


if __name__ == "__main__":
    main()
