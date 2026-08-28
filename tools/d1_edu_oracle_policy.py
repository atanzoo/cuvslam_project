#!/usr/bin/env python3
"""Ground-truth high-level oracle for measuring the PPO decision ceiling.

The oracle is deliberately limited to the same five high-level actions as
PPO.  It does not command joints or bypass MPPI/SimulatedD1SDK.  Its only
privilege is access to analytic obstacle ground truth when selecting an
action, which makes it useful as an upper-bound diagnostic rather than a
deployable controller.
"""

from __future__ import annotations

from dataclasses import dataclass
from types import SimpleNamespace
from typing import Any

import numpy as np

from d1_edu_nav2_global_path import estimate_wait_yield
from run_d1_edu_decision_layer_viewer import rotation_world_from_local


@dataclass
class GroundTruthOraclePolicy:
    """Select CRUISE/AVOID/WAIT from exact simulated obstacle trajectories."""

    decision_name: str = "ground_truth_oracle"
    uses_ground_truth: bool = True

    def reset_episode(self) -> None:
        return None

    def act(self, observation: np.ndarray) -> int:
        raise RuntimeError("GroundTruthOraclePolicy requires act_with_context")

    def act_with_context(
        self,
        observation: np.ndarray,
        context: dict[str, Any],
    ) -> int:
        del observation
        config = context["config"]
        elapsed = float(context["elapsed"])
        robot_xy = np.asarray(context["robot_xy"], dtype=float)
        yaw = float(context["yaw"])
        trajectories = context["trajectories"]
        obstacle_specs = context["obstacle_specs"]
        current_states = context["current_states"]
        path_reference = context["path_reference"]
        robot_radius = float(context["robot_radius"])

        sample_dt = max(float(config.trajectory_sample_dt), 0.05)
        horizon = max(float(config.trajectory_prediction_horizon), sample_dt)
        times = np.arange(0.0, horizon + 0.5 * sample_dt, sample_dt)
        exact_predictions: list[dict[str, Any]] = []
        exact_radii: list[float] = []
        for trajectory, obstacle_spec, state in zip(
            trajectories, obstacle_specs, current_states
        ):
            if not state[2]:
                continue
            points: list[np.ndarray] = []
            velocities: list[np.ndarray] = []
            for offset in times:
                position, velocity, active = trajectory.state(elapsed + float(offset))
                if not active:
                    break
                points.append(np.asarray(position, dtype=float))
                velocities.append(np.asarray(velocity, dtype=float))
            if not points:
                continue
            exact_predictions.append({
                "track": SimpleNamespace(confidence=100),
                "trajectory_world": np.asarray(points, dtype=float),
                "prediction_times": times[:len(points)],
                "velocity_world": np.asarray(velocities[0], dtype=float),
            })
            exact_radii.append(float(np.hypot(
                obstacle_spec.half_length,
                obstacle_spec.half_width,
            )))

        obstacle_radius = max(
            (
                float(np.hypot(spec.half_length, spec.half_width))
                for spec in obstacle_specs
            ),
            default=0.25,
        )
        wait_estimate = estimate_wait_yield(
            context["global_path"],
            exact_predictions,
            robot_xy,
            robot_radius=robot_radius,
            obstacle_radius=obstacle_radius,
            lookahead_distance=max(
                float(config.global_path_lookahead) * 3.0,
                float(config.global_path_robot_radius) * 4.0,
            ),
            max_wait_seconds=float(config.wait_max_clear_time),
            min_obstacle_speed=float(config.wait_min_obstacle_speed),
        )
        rotation = rotation_world_from_local(yaw)

        def candidate_clearance(local_velocity: np.ndarray) -> float:
            world_velocity = rotation @ np.asarray(local_velocity, dtype=float)
            robot_points = robot_xy[None, :] + times[:, None] * world_velocity[None, :]
            clearance = float("inf")
            for prediction, obstacle_radius_i in zip(exact_predictions, exact_radii):
                obstacle_points = np.asarray(
                    prediction["trajectory_world"], dtype=float
                )
                count = min(len(robot_points), len(obstacle_points))
                if count == 0:
                    continue
                distances = np.linalg.norm(
                    robot_points[:count] - obstacle_points[:count], axis=1
                )
                clearance = min(
                    clearance,
                    float(np.min(distances)) - robot_radius - obstacle_radius_i,
                )
            return clearance

        target_local = np.asarray(path_reference.target_local, dtype=float)
        target_norm = float(np.linalg.norm(target_local))
        direction = (
            target_local / target_norm
            if target_norm > 1e-9
            else np.array([1.0, 0.0], dtype=float)
        )
        cruise_speed = min(
            float(config.robot_cruise_speed),
            float(path_reference.reference_speed),
        )
        cruise_velocity = direction * cruise_speed
        lateral_speed = min(float(config.max_vy), 0.25)
        avoid_forward = max(0.10, 0.55 * cruise_speed)
        candidates = {
            0: candidate_clearance(cruise_velocity),
            1: candidate_clearance(np.array([avoid_forward, lateral_speed])),
            2: candidate_clearance(np.array([avoid_forward, -lateral_speed])),
            4: candidate_clearance(np.zeros(2, dtype=float)),
        }

        # A short moving crossing is best handled by yielding when standing
        # still remains physically safe.  For every other case, continue on
        # the route when the exact cruise rollout is safe; otherwise select
        # the avoidance side with the larger predicted clearance.
        if (
            wait_estimate.waitable > 0.5
            and wait_estimate.stationary_collision_risk < 0.5
            and candidates[4] >= 0.0
        ):
            return 4
        safety_floor = max(0.0, float(config.required_clearance))
        if candidates[0] >= safety_floor:
            return 0
        return max((1, 2), key=lambda action: candidates[action])
