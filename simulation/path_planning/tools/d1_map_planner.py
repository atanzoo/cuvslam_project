#!/usr/bin/env python3
"""Map-aware offline routing and conservative geometry validation."""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from typing import Any

import numpy as np
from scipy import ndimage

from d1_candidate_map import CandidateMap
from d1_map_scenario import MapScenario


@dataclass(frozen=True)
class MapRoute:
    requested_start: tuple[float, float]
    requested_goal: tuple[float, float]
    cells: tuple[tuple[int, int], ...]
    world_points: tuple[tuple[float, float], ...]
    length_m: float
    robot_radius_m: float
    obstacle_margin_m: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "requested_start": list(self.requested_start),
            "requested_goal": list(self.requested_goal),
            "cells": [list(cell) for cell in self.cells],
            "world_points": [list(point) for point in self.world_points],
            "length_m": self.length_m,
            "robot_radius_m": self.robot_radius_m,
            "obstacle_margin_m": self.obstacle_margin_m,
        }


class CandidateMapPlanner:
    """A deterministic grid planner over known-free candidate-map cells.

    The planner is deliberately independent from ROS/Nav2.  Unknown cells are
    always blocked and the robot radius is not scaled when ``scene_scale`` is
    greater than one.
    """

    def __init__(
        self,
        candidate_map: CandidateMap,
        *,
        robot_radius_m: float,
        obstacle_margin_m: float = 0.12,
    ) -> None:
        if robot_radius_m <= 0.0 or obstacle_margin_m < 0.0:
            raise ValueError("robot radius must be positive and margin non-negative")
        self.candidate_map = candidate_map
        self.robot_radius_m = float(robot_radius_m)
        self.obstacle_margin_m = float(obstacle_margin_m)
        self.inflation_radius_m = self.robot_radius_m + self.obstacle_margin_m
        self.inflation_cells = int(math.ceil(self.inflation_radius_m / candidate_map.scaled_resolution))
        self._traversable = self._build_traversable_mask()
        self._traversable_coordinates = np.argwhere(self._traversable)
        self._largest_component_coordinates: np.ndarray | None = None

    def _build_traversable_mask(self) -> np.ndarray:
        candidate = self.candidate_map
        raw = np.frombuffer(candidate.pixels, dtype=np.uint8).reshape(candidate.height, candidate.width)
        if candidate.metadata.negate:
            occupancy = raw.astype(np.float32) / 255.0
        else:
            occupancy = 1.0 - raw.astype(np.float32) / 255.0
        known_free = (raw != 205) & (occupancy <= candidate.metadata.free_thresh)
        padding = max(1, self.inflation_cells)
        padded = np.pad(known_free, padding, mode="constant", constant_values=False)
        distances = ndimage.distance_transform_edt(padded, sampling=candidate.scaled_resolution)
        height, width = known_free.shape
        interior = distances[padding:padding + height, padding:padding + width]
        return known_free & (interior > self.inflation_radius_m)

    def is_traversable_cell(self, column: int, row: int) -> bool:
        if not (0 <= column < self.candidate_map.width and 0 <= row < self.candidate_map.height):
            return False
        return bool(self._traversable[row, column])

    def is_traversable_world(self, x: float, y: float) -> bool:
        """Check a world-frame robot-center position against inflated free space."""
        try:
            cell = self.candidate_map.world_to_cell(float(x), float(y))
        except (IndexError, ValueError):
            return False
        return self.is_traversable_cell(*cell)

    def traversable_cell_count(self) -> int:
        return int(np.count_nonzero(self._traversable))

    def _largest_component_array(self) -> np.ndarray:
        if self._largest_component_coordinates is None:
            labels, count = ndimage.label(
                self._traversable,
                structure=np.array(((0, 1, 0), (1, 1, 1), (0, 1, 0)), dtype=np.uint8),
            )
            if count == 0:
                raise ValueError("candidate map has no traversable cells after inflation")
            sizes = np.bincount(labels.ravel())
            sizes[0] = 0
            largest_label = int(np.argmax(sizes))
            self._largest_component_coordinates = np.argwhere(labels == largest_label)
        return self._largest_component_coordinates

    def largest_traversable_component(self) -> tuple[tuple[int, int], ...]:
        coordinates = self._largest_component_array()
        return tuple((int(column), int(row)) for row, column in coordinates)

    def largest_traversable_component_sample(
        self, *, max_cells: int = 50_000,
    ) -> tuple[tuple[int, int], ...]:
        if max_cells <= 0:
            raise ValueError("max_cells must be positive")
        coordinates = self._largest_component_array()
        stride = max(1, int(math.ceil(len(coordinates) / max_cells)))
        return tuple(
            (int(column), int(row)) for row, column in coordinates[::stride]
        )

    def safe_route_endpoints(self) -> tuple[tuple[float, float], tuple[float, float]]:
        component = self._largest_component_array()
        stride = max(1, int(math.ceil(len(component) / 50_000)))
        component = component[::stride]
        points = np.asarray(
            [self.candidate_map.cell_to_world(int(column), int(row)) for row, column in component],
            dtype=float,
        )
        center = np.mean(points, axis=0)
        centered = points - center
        covariance = centered.T @ centered
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        axis = eigenvectors[:, int(np.argmax(eigenvalues))]
        if axis[0] < 0.0 or (abs(axis[0]) < 1e-12 and axis[1] < 0.0):
            axis = -axis
        projections = centered @ axis
        ordered = np.argsort(projections, kind="stable")
        start = points[ordered[int(round(0.08 * (len(ordered) - 1)))]]
        goal = points[ordered[int(round(0.92 * (len(ordered) - 1)))]]
        return (float(start[0]), float(start[1])), (float(goal[0]), float(goal[1]))

    def _nearest_traversable(self, cell: tuple[int, int]) -> tuple[int, int]:
        if self.is_traversable_cell(*cell):
            return cell
        if len(self._traversable_coordinates) == 0:
            raise ValueError("candidate map has no traversable cells")
        deltas = self._traversable_coordinates[:, ::-1] - np.asarray(cell, dtype=np.int32)
        nearest_index = int(np.argmin(np.einsum("ij,ij->i", deltas, deltas)))
        row, column = self._traversable_coordinates[nearest_index]
        return int(column), int(row)

    def plan(self, start: tuple[float, float], goal: tuple[float, float]) -> MapRoute:
        start_cell = self._nearest_traversable(self.candidate_map.world_to_cell(*start))
        goal_cell = self._nearest_traversable(self.candidate_map.world_to_cell(*goal))
        queue: list[tuple[float, int, tuple[int, int]]] = [(0.0, 0, start_cell)]
        came_from: dict[tuple[int, int], tuple[int, int] | None] = {start_cell: None}
        cost_so_far: dict[tuple[int, int], float] = {start_cell: 0.0}
        counter = 1
        while queue:
            _, _, current = heapq.heappop(queue)
            if current == goal_cell:
                break
            column, row = current
            for dx, dy, step_cost in (
                (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
                (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
            ):
                neighbor = (column + dx, row + dy)
                if not self.is_traversable_cell(*neighbor):
                    continue
                if dx and dy and not (
                    self.is_traversable_cell(column + dx, row)
                    and self.is_traversable_cell(column, row + dy)
                ):
                    continue
                new_cost = cost_so_far[current] + step_cost
                if new_cost >= cost_so_far.get(neighbor, float("inf")):
                    continue
                cost_so_far[neighbor] = new_cost
                heuristic = math.hypot(neighbor[0] - goal_cell[0], neighbor[1] - goal_cell[1])
                heapq.heappush(queue, (new_cost + heuristic, counter, neighbor))
                counter += 1
                came_from[neighbor] = current
        if goal_cell not in came_from:
            raise ValueError("no traversable route exists between requested endpoints")
        cells: list[tuple[int, int]] = []
        current: tuple[int, int] | None = goal_cell
        while current is not None:
            cells.append(current)
            current = came_from[current]
        cells.reverse()
        world_points = tuple(
            self.candidate_map.cell_to_world(column, row) for column, row in cells
        )
        length = sum(
            math.dist(first, second)
            for first, second in zip(world_points, world_points[1:])
        )
        return MapRoute(
            requested_start=(float(start[0]), float(start[1])),
            requested_goal=(float(goal[0]), float(goal[1])),
            cells=tuple(cells),
            world_points=world_points,
            length_m=float(length),
            robot_radius_m=self.robot_radius_m,
            obstacle_margin_m=self.obstacle_margin_m,
        )

    def validate_scenario(self, scenario: MapScenario) -> dict[str, Any]:
        reasons: list[str] = []
        try:
            route = self.plan(scenario.start, scenario.goal)
        except (IndexError, ValueError) as error:
            route = None
            reasons.append(f"route:{error}")

        actor_results: list[dict[str, Any]] = []
        for actor in scenario.actors:
            points = (actor.start, *actor.waypoints)
            anchor_valid = all(
                self.is_traversable_cell(*self.candidate_map.world_to_cell(*point))
                for point in points
            )
            segment_valid = anchor_valid
            planned_segments: list[dict[str, Any]] = []
            if anchor_valid:
                for first, second in zip(points, points[1:]):
                    try:
                        segment_route = self.plan(first, second)
                    except (IndexError, ValueError) as error:
                        segment_valid = False
                        reasons.append(f"actor:{actor.actor_id}:no_grid_path:{error}")
                        break
                    planned_segments.append(segment_route.to_dict())
            if not anchor_valid:
                reasons.append(f"actor:{actor.actor_id}:anchor_not_traversable")
            actor_results.append({
                "actor_id": actor.actor_id,
                "anchor_valid": anchor_valid,
                "waypoint_segment_valid": segment_valid,
                "motion_kind": actor.motion_kind,
                "planned_segments": planned_segments,
                "planned_path_length_m": sum(
                    float(segment["length_m"]) for segment in planned_segments
                ),
            })

        return {
            "valid": route is not None and not reasons,
            "route_valid": route is not None,
            "route": route.to_dict() if route is not None else None,
            "actors": actor_results,
            "reasons": reasons,
            "trajectory_validation": "grid_path_between_waypoints",
            "swept_arc_validation": "not_run",
            "dynamic_safety_ready": False,
        }
