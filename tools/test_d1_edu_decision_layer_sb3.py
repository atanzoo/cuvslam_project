#!/usr/bin/env python3
"""Smoke tests for the Stable-Baselines3 D1 decision environment."""

from __future__ import annotations

import numpy as np

from d1_edu_sb3_env import D1DecisionEnv, OBSERVATION_DIM


def main() -> None:
    env = D1DecisionEnv(
        profile="baseline",
        scenario_mode="multi_target",
        seed=17,
        duration=0.40,
        mppi_samples=4,
        mppi_horizon=5,
    )
    observation, info = env.reset(seed=17)
    assert observation.shape == (OBSERVATION_DIM,)
    assert env.observation_space.contains(observation)
    assert env.action_space.contains(0)
    assert env.action_space.n == 5
    next_observation, reward, terminated, truncated, step_info = env.step(0)
    assert next_observation.shape == (OBSERVATION_DIM,)
    assert np.isfinite(reward)
    assert isinstance(terminated, bool)
    assert isinstance(truncated, bool)
    assert "episode_seed" in step_info
    assert len(info["config"].scenario_obstacles) == 3
    env.close()
    print("PASS SB3 environment reset/step smoke test")
    print("status=PASS: Stable-Baselines3 D1 environment is callable")


if __name__ == "__main__":
    main()
