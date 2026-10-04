#!/usr/bin/env python3
"""Stable-Baselines3 policy adapter for the existing D1 episode runner."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from stable_baselines3 import PPO


class SB3DecisionPolicy:
    """Adapt an SB3 PPO model to ``run_episode(decision_policy=...)``.

    The adapter deliberately exposes the same tiny interface as the previous
    NumPy policy: ``reset_episode()`` and ``act(observation)``.  This keeps the
    existing viewer and benchmark loop unchanged while replacing only the
    policy implementation.
    """

    def __init__(self, model_path: str | Path, *, deterministic: bool = True) -> None:
        self.model_path = str(model_path)
        self.deterministic = bool(deterministic)
        self.model = PPO.load(self.model_path, device="cpu")

    def reset_episode(self) -> None:
        """Reset the policy-side episode state.

        MlpPolicy PPO has no recurrent hidden state, so there is nothing to
        clear.  The method exists for compatibility with the current runner.
        """

    def act(self, observation: np.ndarray) -> int:
        action, _ = self.model.predict(
            np.asarray(observation, dtype=np.float32),
            deterministic=self.deterministic,
        )
        return int(np.asarray(action).reshape(-1)[0])

