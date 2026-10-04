#!/usr/bin/env python3
"""Deterministic dynamic-obstacle scenario manifests on a candidate map.

This module samples scenario anchors from the map's largest known-free
component.  It deliberately records that only anchors are map-validated until
the route/planner unit checks the full actor trajectories against occupancy.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np

from d1_candidate_map import CandidateMap


SCENARIO_FAMILIES = (
    "crossing",
    "oncoming",
    "same_direction",
    "accelerating",
    "turning",
    "sudden_corner_appearance",
    "walk_stop_walk",
    "multi_person_occlusion",
)


@dataclass(frozen=True)
class MapScenarioActor:
    actor_id: str
    kind: str
    motion_kind: str
    start: tuple[float, float]
    waypoints: tuple[tuple[float, float], ...]
    speed_mps: float
    acceleration_mps2: float
    appear_time_s: float
    radius_m: float
    occlusion_group: str | None = None
    stop_interval_s: tuple[float, float] | None = None
    turn_rate_radps: float = 0.0
    turn_radius_m: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "actor_id": self.actor_id,
            "kind": self.kind,
            "motion_kind": self.motion_kind,
            "start": list(self.start),
            "waypoints": [list(point) for point in self.waypoints],
            "speed_mps": self.speed_mps,
            "acceleration_mps2": self.acceleration_mps2,
            "appear_time_s": self.appear_time_s,
            "radius_m": self.radius_m,
            "occlusion_group": self.occlusion_group,
            "stop_interval_s": (
                list(self.stop_interval_s) if self.stop_interval_s is not None else None
            ),
            "turn_rate_radps": self.turn_rate_radps,
            "turn_radius_m": self.turn_radius_m,
        }
        return payload


@dataclass(frozen=True)
class MapScenario:
    scenario_family: str
    seed: int
    curriculum_index: int
    schedule_index: int
    start: tuple[float, float]
    goal: tuple[float, float]
    route_heading_rad: float
    actors: tuple[MapScenarioActor, ...]
    map_manifest: dict[str, Any]
    geometry_validation: str = "anchors_known_free_only"
    route_source: str = "candidate_map_component"

    def to_dict(self) -> dict[str, Any]:
        return {
            "scenario_family": self.scenario_family,
            "seed": self.seed,
            "curriculum_index": self.curriculum_index,
            "schedule_index": self.schedule_index,
            "start": list(self.start),
            "goal": list(self.goal),
            "route_heading_rad": self.route_heading_rad,
            "actors": [actor.to_dict() for actor in self.actors],
            "map": self.map_manifest,
            "geometry_validation": self.geometry_validation,
            "route_source": self.route_source,
        }


def _world_points(
    candidate_map: CandidateMap,
    cells: tuple[tuple[int, int], ...] | None = None,
) -> np.ndarray:
    selected_cells = cells if cells is not None else candidate_map.largest_free_component()
    selected_cells = sorted(selected_cells, key=lambda cell: (cell[1], cell[0]))
    return np.asarray(
        [candidate_map.cell_to_world(column, row) for column, row in selected_cells],
        dtype=float,
    )


def _route_axis(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    center = np.mean(points, axis=0)
    centered = points - center
    covariance = centered.T @ centered
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    axis = eigenvectors[:, int(np.argmax(eigenvalues))]
    if axis[0] < 0.0 or (abs(axis[0]) < 1e-12 and axis[1] < 0.0):
        axis = -axis
    return center, axis / max(float(np.linalg.norm(axis)), 1e-12)


def _point_at_fraction(points: np.ndarray, axis: np.ndarray, fraction: float) -> np.ndarray:
    center = np.mean(points, axis=0)
    projections = (points - center) @ axis
    ordered = np.argsort(projections, kind="stable")
    index = int(round(np.clip(fraction, 0.0, 1.0) * (len(ordered) - 1)))
    return points[ordered[index]].copy()


def _crossing_points(points: np.ndarray, center: np.ndarray, axis: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    normal = np.array([-axis[1], axis[0]])
    projections = (points - center) @ axis
    target_projection = float(np.median(projections))
    window = max(0.25, 0.08 * (float(np.max(projections)) - float(np.min(projections))))
    candidates = points[np.abs(projections - target_projection) <= window]
    if len(candidates) < 2:
        candidates = points
    lateral = (candidates - center) @ normal
    return candidates[int(np.argmin(lateral))].copy(), candidates[int(np.argmax(lateral))].copy()


def _actor(
    actor_id: str,
    motion_kind: str,
    start: np.ndarray,
    waypoints: tuple[np.ndarray, ...],
    *,
    speed: float = 0.45,
    acceleration: float = 0.0,
    appear_time: float = 0.0,
    radius: float = 0.25,
    occlusion_group: str | None = None,
    stop_interval: tuple[float, float] | None = None,
    turn_rate: float = 0.0,
    turn_radius: float = 0.0,
) -> MapScenarioActor:
    return MapScenarioActor(
        actor_id=actor_id,
        kind="person",
        motion_kind=motion_kind,
        start=(float(start[0]), float(start[1])),
        waypoints=tuple((float(point[0]), float(point[1])) for point in waypoints),
        speed_mps=float(speed),
        acceleration_mps2=float(acceleration),
        appear_time_s=float(appear_time),
        radius_m=float(radius),
        occlusion_group=occlusion_group,
        stop_interval_s=stop_interval,
        turn_rate_radps=float(turn_rate),
        turn_radius_m=float(turn_radius),
    )


def generate_map_scenario(
    candidate_map: CandidateMap,
    *,
    seed: int,
    curriculum_index: int,
    scenario_family: str | None = None,
    schedule_index: int | None = None,
    route_endpoints: tuple[tuple[float, float], tuple[float, float]] | None = None,
    anchor_cells: tuple[tuple[int, int], ...] | None = None,
) -> MapScenario:
    """Create a deterministic map-anchored scenario manifest.

    ``schedule_index`` is explicit in the manifest so a replay cannot silently
    change motion assignment when a curriculum generator is revised.
    """
    if scenario_family is None:
        scenario_family = SCENARIO_FAMILIES[int(curriculum_index) % len(SCENARIO_FAMILIES)]
    if scenario_family not in SCENARIO_FAMILIES:
        raise ValueError(f"unknown scenario family: {scenario_family}")
    if curriculum_index < 0:
        raise ValueError("curriculum_index must be non-negative")
    resolved_schedule = int(curriculum_index if schedule_index is None else schedule_index)
    if resolved_schedule < 0:
        raise ValueError("schedule_index must be non-negative")

    rng = np.random.default_rng(int(seed))
    points = _world_points(candidate_map, anchor_cells)
    if len(points) < 2:
        raise ValueError("candidate map needs at least two known-free cells")
    if route_endpoints is None:
        center, axis = _route_axis(points)
        start = _point_at_fraction(points, axis, 0.08)
        goal = _point_at_fraction(points, axis, 0.92)
        route_source = "candidate_map_component"
    else:
        start = np.asarray(route_endpoints[0], dtype=float).reshape(2)
        goal = np.asarray(route_endpoints[1], dtype=float).reshape(2)
        route_delta = goal - start
        if float(np.linalg.norm(route_delta)) <= 1e-9:
            raise ValueError("route endpoints must be distinct")
        axis = route_delta / float(np.linalg.norm(route_delta))
        center = (start + goal) / 2.0
        route_source = "planner_traversable_component"
    route_heading = float(math.atan2(axis[1], axis[0]))
    route_points = {
        fraction: _point_at_fraction(points, axis, fraction)
        for fraction in (0.30, 0.42, 0.50, 0.58, 0.70, 0.78)
    }
    actor_list: list[MapScenarioActor] = []

    if scenario_family == "crossing":
        before, after = _crossing_points(points, center, axis)
        actor_list.append(_actor("person_00", "crossing", before, (after,)))
    elif scenario_family == "oncoming":
        actor_list.append(_actor("person_00", "oncoming", route_points[0.78], (route_points[0.42],)))
    elif scenario_family == "same_direction":
        actor_list.append(_actor("person_00", "same_direction", route_points[0.30], (route_points[0.70],)))
    elif scenario_family == "accelerating":
        actor_list.append(_actor(
            "person_00", "accelerating", route_points[0.30], (route_points[0.78],),
            speed=0.20, acceleration=0.12,
        ))
    elif scenario_family == "turning":
        turn_rate = float(rng.choice((-0.45, 0.45)))
        actor_list.append(_actor(
            "person_00", "turning", route_points[0.42],
            (route_points[0.58], route_points[0.70]),
            speed=0.40, turn_rate=turn_rate, turn_radius=1.0,
        ))
    elif scenario_family == "sudden_corner_appearance":
        actor_list.append(_actor(
            "person_00", "sudden_corner_appearance", route_points[0.58],
            (route_points[0.70],), appear_time=4.0,
        ))
    elif scenario_family == "walk_stop_walk":
        actor_list.append(_actor(
            "person_00", "walk_stop_walk", route_points[0.30],
            (route_points[0.42], route_points[0.58], route_points[0.78]),
            stop_interval=(2.0, 4.0),
        ))
    elif scenario_family == "multi_person_occlusion":
        before, after = _crossing_points(points, center, axis)
        actor_list.extend((
            _actor("person_00", "crossing", before, (after,), occlusion_group="group_00"),
            _actor("person_01", "same_direction", route_points[0.30], (route_points[0.70],), occlusion_group="group_00"),
            _actor("person_02", "static_occluder", route_points[0.50], (), speed=0.0, occlusion_group="group_00"),
        ))

    return MapScenario(
        scenario_family=scenario_family,
        seed=int(seed),
        curriculum_index=int(curriculum_index),
        schedule_index=resolved_schedule,
        start=(float(start[0]), float(start[1])),
        goal=(float(goal[0]), float(goal[1])),
        route_heading_rad=route_heading,
        actors=tuple(actor_list),
        map_manifest=candidate_map.manifest(),
        geometry_validation=(
            "anchors_traversable_component"
            if anchor_cells is not None
            else "anchors_known_free_only"
        ),
        route_source=route_source,
    )
