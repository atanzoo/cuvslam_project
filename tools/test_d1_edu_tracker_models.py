#!/usr/bin/env python3
"""Regression tests for CV/CA/CTRV mixture prediction and uncertainty tubes."""

from __future__ import annotations

import numpy as np

from d1_edu_tracker_models import IMMObstaclePredictor
from run_d1_edu_mppi_obstacle_viewer import ObstacleMPPI


def main() -> None:
    predictor = IMMObstaclePredictor()
    times = np.linspace(0.0, 3.0, 13)
    prediction = predictor.predict(
        np.array([0.0, 0.0]),
        np.array([0.5, 0.0]),
        np.array([0.0, 0.35]),
        times,
        history=[
            (0.0, np.array([0.0, 0.0])),
            (0.1, np.array([0.05, 0.0])),
            (0.2, np.array([0.10, 0.004])),
        ],
        position_uncertainty=0.02,
        velocity_uncertainty=0.05,
    )
    assert prediction.trajectory.shape == (len(times), 2)
    assert prediction.uncertainty_radius.shape == (len(times),)
    assert np.all(np.isfinite(prediction.trajectory))
    assert np.all(prediction.uncertainty_radius >= 0.0)
    assert prediction.uncertainty_radius[-1] >= prediction.uncertainty_radius[0]
    assert abs(sum(prediction.model_weights.values()) - 1.0) < 1e-6
    assert prediction.model_weights["CTRV"] > 0.0

    planner = ObstacleMPPI(samples=4, horizon=5, robot_radius=0.34)
    command = planner.update(
        np.zeros(3),
        np.empty((0, 2)),
        np.array([2.0, 0.0]),
        dynamic_trajectories_local=[
            (
                prediction.trajectory,
                np.array([0.5, 0.0]),
                0.30,
                prediction.uncertainty_radius,
            ),
        ],
    )
    assert command.shape == (3,)
    print(
        "PASS IMM tracker: "
        f"weights={prediction.model_weights} "
        f"turn_rate={prediction.turn_rate:.3f} "
        f"tube={prediction.uncertainty_radius[0]:.3f}->"
        f"{prediction.uncertainty_radius[-1]:.3f}m"
    )


if __name__ == "__main__":
    main()
