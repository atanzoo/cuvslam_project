#!/usr/bin/env python3
"""Explainable dynamic-obstacle risk geometry for offline evaluation."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class RiskAssessment:
    ttc_s: float
    predicted_min_distance_m: float
    swept_arc_intersects: bool
    warning: bool
    collision_radius_m: float
    method: str = "piecewise_swept_arc"

    def to_dict(self) -> dict[str, object]:
        return {
            "ttc_s": self.ttc_s,
            "predicted_min_distance_m": self.predicted_min_distance_m,
            "swept_arc_intersects": self.swept_arc_intersects,
            "warning": self.warning,
            "collision_radius_m": self.collision_radius_m,
            "method": self.method,
        }


def ctrv_prediction(
    position: Iterable[float],
    velocity: Iterable[float],
    turn_rate_radps: float,
    times: Iterable[float],
) -> np.ndarray:
    """Return a constant-turn-rate-and-velocity trajectory."""
    p = np.asarray(tuple(position), dtype=float).reshape(2)
    v = np.asarray(tuple(velocity), dtype=float).reshape(2)
    t = np.asarray(tuple(times), dtype=float).reshape(-1)
    if t.size == 0 or np.any(t < 0.0) or np.any(np.diff(t) < 0.0):
        raise ValueError("times must be non-empty, non-negative and sorted")
    speed = float(np.linalg.norm(v))
    turn_rate = float(turn_rate_radps)
    if speed < 1e-9 or abs(turn_rate) < 1e-9:
        return p[None, :] + t[:, None] * v[None, :]
    heading = math.atan2(float(v[1]), float(v[0]))
    angles = heading + turn_rate * t
    return p[None, :] + (speed / turn_rate) * np.column_stack((
        np.sin(angles) - math.sin(heading),
        -np.cos(angles) + math.cos(heading),
    ))


def _point_segment_distance(point: np.ndarray, first: np.ndarray, second: np.ndarray) -> float:
    direction = second - first
    denominator = float(direction @ direction)
    if denominator <= 1e-12:
        return float(np.linalg.norm(point - first))
    ratio = float(np.clip(((point - first) @ direction) / denominator, 0.0, 1.0))
    projection = first + ratio * direction
    return float(np.linalg.norm(point - projection))


def _orientation(first: np.ndarray, second: np.ndarray, third: np.ndarray) -> float:
    vector_a = second - first
    vector_b = third - first
    return float(vector_a[0] * vector_b[1] - vector_a[1] * vector_b[0])


def _on_segment(first: np.ndarray, second: np.ndarray, point: np.ndarray) -> bool:
    return (
        min(first[0], second[0]) - 1e-9 <= point[0] <= max(first[0], second[0]) + 1e-9
        and min(first[1], second[1]) - 1e-9 <= point[1] <= max(first[1], second[1]) + 1e-9
    )


def _segment_distance(
    first_a: np.ndarray,
    second_a: np.ndarray,
    first_b: np.ndarray,
    second_b: np.ndarray,
) -> float:
    orientations = (
        _orientation(first_a, second_a, first_b),
        _orientation(first_a, second_a, second_b),
        _orientation(first_b, second_b, first_a),
        _orientation(first_b, second_b, second_a),
    )
    if (
        orientations[0] * orientations[1] < 0.0
        and orientations[2] * orientations[3] < 0.0
    ):
        return 0.0
    if abs(orientations[0]) <= 1e-9 and _on_segment(first_a, second_a, first_b):
        return 0.0
    if abs(orientations[1]) <= 1e-9 and _on_segment(first_a, second_a, second_b):
        return 0.0
    if abs(orientations[2]) <= 1e-9 and _on_segment(first_b, second_b, first_a):
        return 0.0
    if abs(orientations[3]) <= 1e-9 and _on_segment(first_b, second_b, second_a):
        return 0.0
    return min(
        _point_segment_distance(first_a, first_b, second_b),
        _point_segment_distance(second_a, first_b, second_b),
        _point_segment_distance(first_b, first_a, second_a),
        _point_segment_distance(second_b, first_a, second_a),
    )


def assess_swept_arc_risk(
    robot_trajectory: Iterable[Iterable[float]],
    obstacle_trajectory: Iterable[Iterable[float]],
    times: Iterable[float],
    *,
    robot_radius_m: float,
    obstacle_radius_m: float,
    uncertainty_radius_m: Iterable[float] | None = None,
    warning_ttc_s: float = 3.0,
) -> RiskAssessment:
    """Assess conservative piecewise swept-arc risk over synchronized samples."""
    robot = np.asarray(tuple(tuple(point) for point in robot_trajectory), dtype=float)
    obstacle = np.asarray(tuple(tuple(point) for point in obstacle_trajectory), dtype=float)
    t = np.asarray(tuple(times), dtype=float)
    if robot.ndim != 2 or robot.shape[1] != 2 or obstacle.shape != robot.shape:
        raise ValueError("robot and obstacle trajectories must have the same Nx2 shape")
    if t.shape != (len(robot),) or len(t) == 0 or np.any(np.diff(t) < 0.0):
        raise ValueError("times must match trajectories and be sorted")
    if robot_radius_m < 0.0 or obstacle_radius_m < 0.0:
        raise ValueError("radii must be non-negative")
    if warning_ttc_s < 0.0:
        raise ValueError("warning TTC must be non-negative")
    if uncertainty_radius_m is None:
        uncertainty = np.zeros(len(t), dtype=float)
    else:
        uncertainty = np.asarray(tuple(uncertainty_radius_m), dtype=float)
        if uncertainty.shape != (len(t),) or np.any(uncertainty < 0.0):
            raise ValueError("uncertainty radius must match trajectory length and be non-negative")

    collision_radius = float(robot_radius_m + obstacle_radius_m)
    distances: list[float] = []
    collision_times: list[float] = []
    for index in range(len(t)):
        if index == 0:
            distance = float(np.linalg.norm(robot[index] - obstacle[index]))
        else:
            distance = _segment_distance(
                robot[index - 1], robot[index],
                obstacle[index - 1], obstacle[index],
            )
        distances.append(distance)
        threshold = collision_radius + float(uncertainty[index])
        if distance <= threshold:
            collision_times.append(float(t[index]))
    minimum_distance = float(min(distances))
    ttc = min(collision_times, default=float("inf"))
    return RiskAssessment(
        ttc_s=ttc,
        predicted_min_distance_m=minimum_distance,
        swept_arc_intersects=bool(collision_times),
        warning=bool(collision_times and ttc <= warning_ttc_s),
        collision_radius_m=collision_radius,
    )


def compare_candidate_trajectories(
    robot_candidates: dict[str, Iterable[Iterable[float]]],
    obstacle_trajectory: Iterable[Iterable[float]],
    times: Iterable[float],
    **risk_kwargs: object,
) -> dict[str, RiskAssessment]:
    """Evaluate PASS_LEFT/PASS_RIGHT (or other labelled) candidate paths."""
    return {
        label: assess_swept_arc_risk(path, obstacle_trajectory, times, **risk_kwargs)
        for label, path in robot_candidates.items()
    }
