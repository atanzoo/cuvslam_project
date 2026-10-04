#!/usr/bin/env python3
"""Small multi-model motion predictor for dynamic obstacle tracks.

This is an IMM-style predictor rather than a full SLAM estimator: the
existing LiDAR association and filtered track state remain unchanged, while
CV, CA, and CTRV hypotheses are mixed into one mean trajectory plus a scalar
uncertainty tube for MPPI.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class IMMPrediction:
    trajectory: np.ndarray
    uncertainty_radius: np.ndarray
    model_weights: dict[str, float]
    turn_rate: float


def _ctrv_trajectory(
    position: np.ndarray,
    velocity: np.ndarray,
    turn_rate: float,
    times: np.ndarray,
) -> np.ndarray:
    speed = float(np.linalg.norm(velocity))
    if speed < 1e-6 or abs(turn_rate) < 1e-4:
        return position[None, :] + times[:, None] * velocity[None, :]
    heading = float(np.arctan2(velocity[1], velocity[0]))
    angles = heading + turn_rate * times
    return position[None, :] + (speed / turn_rate) * np.column_stack((
        np.sin(angles) - np.sin(heading),
        -np.cos(angles) + np.cos(heading),
    ))


class IMMObstaclePredictor:
    """Fuse constant-velocity, constant-acceleration and CTRV hypotheses."""

    model_names = ("CV", "CA", "CTRV")

    def predict(
        self,
        position: np.ndarray,
        velocity: np.ndarray,
        acceleration: np.ndarray,
        times: Iterable[float],
        *,
        history: Iterable[tuple[float, np.ndarray]] = (),
        position_uncertainty: float = 0.0,
        velocity_uncertainty: float = 0.0,
    ) -> IMMPrediction:
        p = np.asarray(position, dtype=float).reshape(2)
        v = np.asarray(velocity, dtype=float).reshape(2)
        a = np.asarray(acceleration, dtype=float).reshape(2)
        t = np.asarray(list(times), dtype=float).reshape(-1)
        if t.size == 0 or np.any(t < 0.0):
            raise ValueError("prediction times must be non-empty and non-negative")
        history_count = len(list(history))
        speed = float(np.linalg.norm(v))
        turn_rate = 0.0
        if speed > 0.08:
            turn_rate = float((v[0] * a[1] - v[1] * a[0]) / max(speed * speed, 1e-6))
        turn_rate = float(np.clip(turn_rate, -1.5, 1.5))

        cv = p[None, :] + t[:, None] * v[None, :]
        ca = cv + 0.5 * (t[:, None] ** 2) * a[None, :]
        ctrv = _ctrv_trajectory(p, v, turn_rate, t)

        accel_evidence = float(np.clip(np.linalg.norm(a) / 1.5, 0.0, 1.0))
        turn_evidence = float(np.clip(abs(turn_rate) / 0.8, 0.0, 1.0))
        if history_count < 3:
            weights = np.array([0.65, 0.25, 0.10], dtype=float)
        else:
            weights = np.array([
                0.48 - 0.20 * accel_evidence - 0.16 * turn_evidence,
                0.30 + 0.28 * accel_evidence,
                0.22 + 0.36 * turn_evidence,
            ], dtype=float)
            weights = np.clip(weights, 0.04, None)
            weights /= np.sum(weights)

        hypotheses = np.stack((cv, ca, ctrv), axis=0)
        mean = np.sum(weights[:, None, None] * hypotheses, axis=0)
        spread = np.sqrt(np.sum(
            weights[:, None]
            * np.sum((hypotheses - mean[None, :, :]) ** 2, axis=2),
            axis=0,
        ))
        base = (
            max(0.0, float(position_uncertainty))
            + max(0.0, float(velocity_uncertainty)) * t
            + 0.015 * t
            # This is model-mismatch uncertainty, not the full physical
            # displacement caused by the measured acceleration.  Using the
            # latter would inflate a 3 s tube to metres and make MPPI stop.
            + 0.08 * min(float(np.linalg.norm(a)), 1.0) * t * t
        )
        uncertainty = np.maximum(base + spread, 0.0)
        return IMMPrediction(
            trajectory=mean,
            uncertainty_radius=uncertainty,
            model_weights={
                name: float(value)
                for name, value in zip(self.model_names, weights)
            },
            turn_rate=turn_rate,
        )
