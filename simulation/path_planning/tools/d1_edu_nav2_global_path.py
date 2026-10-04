#!/usr/bin/env python3
"""Nav2-compatible global-path adapter for the MuJoCo D1 prototype.

The public interface intentionally matches the information consumed by the
PPO and MPPI layers.  On the Mac-only simulator, ``plan()`` builds a small
2-D costmap and runs an obstacle-aware A* planner.  A future ROS 2 adapter can
replace the planner output with ``nav_msgs/Path`` while keeping
``reference()`` unchanged.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import heapq
import math
from typing import Any, Iterable

import numpy as np


def _wrap_angle(angle: float) -> float:
    return float((angle + math.pi) % (2.0 * math.pi) - math.pi)


def _world_to_local(delta_world: np.ndarray, yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    delta = np.asarray(delta_world, dtype=float)
    return np.array([c * delta[0] + s * delta[1], -s * delta[0] + c * delta[1]])


def _box_fields(obstacle: Any) -> tuple[float, float, float, float]:
    """Read an axis-aligned obstacle from a dataclass or mapping."""
    if isinstance(obstacle, dict):
        getter = obstacle.get
    else:
        getter = lambda name: getattr(obstacle, name)
    return (
        float(getter("x")),
        float(getter("y")),
        float(getter("half_length")),
        float(getter("half_width")),
    )


@dataclass(frozen=True)
class GlobalPathReference:
    """Local path information consumed by PPO, CRUISE and MPPI."""

    target_local: np.ndarray
    lateral_error: float
    heading_error: float
    remaining_distance: float
    progress: float
    nearest_index: int
    remaining_fraction: float
    reference_speed: float
    terminal_risk: float = 0.0


@dataclass(frozen=True)
class WaitYieldEstimate:
    """Planner-side estimate used by the learned WAIT_YIELD action.

    These values describe predicted occupancy of the current global-path
    corridor. They are observations for PPO, not collision ground truth;
    physical contact is still measured independently by MuJoCo.
    """

    path_blocked_now: float = 0.0
    path_blocked_future: float = 0.0
    time_to_path_block: float = 1.0
    time_to_path_clear: float = 1.0
    blocking_duration: float = 0.0
    wait_cost: float = 0.0
    detour_cost: float = 0.0
    # A wait is only valid when a moving blocker is predicted to leave the
    # corridor within a finite, short horizon.  These two fields keep that
    # distinction explicit instead of rewarding every blocked corridor.
    waitable: float = 0.0
    stationary_collision_risk: float = 0.0


class Nav2GlobalPath:
    """Replayable global planner with a local Nav2-style path interface.

    The costmap uses rectangular obstacle footprints inflated by the robot
    radius and a configurable margin.  A* searches 8-connected cells and
    applies a soft cost near inflated obstacles, so the selected route is not
    merely any collision-free grid path.  Dynamic obstacle motion remains a
    local MPPI responsibility; the global planner uses the current map
    snapshot, just as a standard static/global costmap would.
    """

    def __init__(
        self,
        *,
        lookahead: float = 0.80,
        spacing: float = 0.10,
        grid_resolution: float = 0.10,
        search_padding: float = 1.50,
        soft_cost_radius: float = 0.12,
        cost_weight: float = 1.0,
        reference_speed_max: float = 0.40,
        reference_speed_min: float = 0.04,
        terminal_slowdown_distance: float = 1.20,
    ) -> None:
        if lookahead <= 0.0 or spacing <= 0.0:
            raise ValueError("lookahead and spacing must be positive")
        if grid_resolution <= 0.0 or search_padding <= 0.0:
            raise ValueError("grid resolution and search padding must be positive")
        if soft_cost_radius <= 0.0 or cost_weight < 0.0:
            raise ValueError("soft cost radius must be positive and weight non-negative")
        if reference_speed_max <= 0.0 or reference_speed_min < 0.0:
            raise ValueError("reference speeds must be non-negative")
        if reference_speed_min > reference_speed_max:
            raise ValueError("reference_speed_min cannot exceed reference_speed_max")
        if terminal_slowdown_distance <= 0.0:
            raise ValueError("terminal slowdown distance must be positive")
        self.lookahead = float(lookahead)
        self.spacing = float(spacing)
        self.grid_resolution = float(grid_resolution)
        self.search_padding = float(search_padding)
        self.soft_cost_radius = float(soft_cost_radius)
        self.cost_weight = float(cost_weight)
        self.reference_speed_max = float(reference_speed_max)
        self.reference_speed_min = float(reference_speed_min)
        self.terminal_slowdown_distance = float(terminal_slowdown_distance)
        self.path = np.empty((0, 2), dtype=float)
        self.arc_lengths = np.empty(0, dtype=float)
        self.total_length = 0.0
        self.costmap = np.empty((0, 0), dtype=float)
        self.occupied = np.empty((0, 0), dtype=bool)
        self.costmap_origin = np.zeros(2, dtype=float)
        self.plan_status = "unplanned"
        self.plan_cost = float("inf")

    def plan(
        self,
        start_world: np.ndarray,
        goal_world: np.ndarray,
        *,
        obstacles: Iterable[Any] | None = None,
        robot_radius: float = 0.25,
        obstacle_margin: float = 0.12,
    ) -> np.ndarray:
        """Plan an obstacle-aware path from start to goal.

        ``obstacles`` is optional for backwards compatibility.  When omitted
        the planner returns the direct path, matching the old adapter API.
        """
        start = np.asarray(start_world, dtype=float).reshape(2)
        goal = np.asarray(goal_world, dtype=float).reshape(2)
        if robot_radius < 0.0 or obstacle_margin < 0.0:
            raise ValueError("robot radius and obstacle margin cannot be negative")
        obstacle_list = list(obstacles or [])
        if not obstacle_list:
            self._set_direct_path(start, goal)
            self.plan_status = "direct_no_obstacles"
            self.plan_cost = self.total_length
            return self.path.copy()

        boxes = [_box_fields(obstacle) for obstacle in obstacle_list]
        self._build_costmap(start, goal, boxes, robot_radius, obstacle_margin)
        start_index = self._nearest_free(self._world_to_index(start))
        goal_index = self._nearest_free(self._world_to_index(goal))
        index_path, cost = self._astar(start_index, goal_index)
        if not index_path:
            self._set_direct_path(start, goal)
            self.plan_status = "direct_fallback_no_path"
            self.plan_cost = self.total_length
            return self.path.copy()

        raw_path = np.asarray([self._index_to_world(index) for index in index_path])
        simplified = self._simplify_path(raw_path)
        # Preserve the exact requested start and goal.  This avoids changing
        # goal tolerance semantics when the nearest free cell is offset by a
        # fraction of a grid cell.
        if len(simplified) == 1:
            simplified = np.vstack((start, goal))
        else:
            simplified[0] = start
            simplified[-1] = goal
        self.set_path(simplified)
        self.plan_status = "astar_costmap"
        self.plan_cost = float(cost)
        return self.path.copy()

    def _set_direct_path(self, start: np.ndarray, goal: np.ndarray) -> None:
        distance = float(np.linalg.norm(goal - start))
        count = max(2, int(math.ceil(distance / self.spacing)) + 1)
        self.set_path(np.linspace(start, goal, count))
        self.costmap = np.zeros((0, 0), dtype=float)
        self.occupied = np.zeros((0, 0), dtype=bool)
        self.costmap_origin = np.zeros(2, dtype=float)

    def _build_costmap(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        boxes: list[tuple[float, float, float, float]],
        robot_radius: float,
        obstacle_margin: float,
    ) -> None:
        resolution = self.grid_resolution
        all_x = [start[0], goal[0], *[box[0] for box in boxes]]
        all_y = [start[1], goal[1], *[box[1] for box in boxes]]
        xmin = math.floor((min(all_x) - self.search_padding) / resolution) * resolution
        xmax = math.ceil((max(all_x) + self.search_padding) / resolution) * resolution
        ymin = math.floor((min(all_y) - self.search_padding) / resolution) * resolution
        ymax = math.ceil((max(all_y) + self.search_padding) / resolution) * resolution
        nx = int(round((xmax - xmin) / resolution)) + 1
        ny = int(round((ymax - ymin) / resolution)) + 1
        self.costmap_origin = np.array([xmin, ymin], dtype=float)
        xs = xmin + np.arange(nx, dtype=float) * resolution
        ys = ymin + np.arange(ny, dtype=float) * resolution
        xx, yy = np.meshgrid(xs, ys)
        costmap = np.zeros((ny, nx), dtype=float)
        occupied = np.zeros((ny, nx), dtype=bool)
        for box_x, box_y, half_length, half_width in boxes:
            dx = np.maximum(np.abs(xx - box_x) - half_length, 0.0)
            dy = np.maximum(np.abs(yy - box_y) - half_width, 0.0)
            clearance = np.hypot(dx, dy) - float(robot_radius)
            occupied |= clearance < float(obstacle_margin)
            near = clearance < float(obstacle_margin + self.soft_cost_radius)
            normalized = np.clip(
                (obstacle_margin + self.soft_cost_radius - clearance)
                / self.soft_cost_radius,
                0.0,
                1.0,
            )
            costmap = np.maximum(costmap, np.where(near, normalized, 0.0))
        self.costmap = costmap
        self.occupied = occupied

    def _world_to_index(self, point: np.ndarray) -> tuple[int, int]:
        relative = (np.asarray(point, dtype=float) - self.costmap_origin) / self.grid_resolution
        ix = int(np.clip(np.rint(relative[0]), 0, self.occupied.shape[1] - 1))
        iy = int(np.clip(np.rint(relative[1]), 0, self.occupied.shape[0] - 1))
        return ix, iy

    def _index_to_world(self, index: tuple[int, int]) -> np.ndarray:
        ix, iy = index
        return self.costmap_origin + self.grid_resolution * np.array([ix, iy], dtype=float)

    def _nearest_free(self, index: tuple[int, int]) -> tuple[int, int]:
        ix, iy = index
        if not self.occupied[iy, ix]:
            return index
        max_radius = max(self.occupied.shape)
        for radius in range(1, max_radius):
            for dx in range(-radius, radius + 1):
                for dy in (-radius, radius):
                    candidate = (ix + dx, iy + dy)
                    if self._is_free(candidate):
                        return candidate
            for dy in range(-radius + 1, radius):
                for dx in (-radius, radius):
                    candidate = (ix + dx, iy + dy)
                    if self._is_free(candidate):
                        return candidate
        return index

    def _is_free(self, index: tuple[int, int]) -> bool:
        ix, iy = index
        return (
            0 <= ix < self.occupied.shape[1]
            and 0 <= iy < self.occupied.shape[0]
            and not bool(self.occupied[iy, ix])
        )

    def _astar(
        self,
        start: tuple[int, int],
        goal: tuple[int, int],
    ) -> tuple[list[tuple[int, int]], float]:
        if not self._is_free(start) or not self._is_free(goal):
            return [], float("inf")
        neighbors = (
            (-1, -1, math.sqrt(2.0)), (-1, 0, 1.0), (-1, 1, math.sqrt(2.0)),
            (0, -1, 1.0), (0, 1, 1.0),
            (1, -1, math.sqrt(2.0)), (1, 0, 1.0), (1, 1, math.sqrt(2.0)),
        )

        def heuristic(index: tuple[int, int]) -> float:
            return math.hypot(index[0] - goal[0], index[1] - goal[1])

        open_set: list[tuple[float, float, tuple[int, int]]] = []
        heapq.heappush(open_set, (heuristic(start), 0.0, start))
        came_from: dict[tuple[int, int], tuple[int, int]] = {}
        g_score = {start: 0.0}
        while open_set:
            _, current_cost, current = heapq.heappop(open_set)
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path, current_cost
            if current_cost > g_score.get(current, float("inf")) + 1e-12:
                continue
            for dx, dy, step_cost in neighbors:
                candidate = (current[0] + dx, current[1] + dy)
                if not self._is_free(candidate):
                    continue
                ix, iy = candidate
                cell_cost = 1.0 + self.cost_weight * float(self.costmap[iy, ix])
                tentative = current_cost + step_cost * cell_cost
                if tentative + 1e-12 < g_score.get(candidate, float("inf")):
                    came_from[candidate] = current
                    g_score[candidate] = tentative
                    heapq.heappush(
                        open_set,
                        (tentative + heuristic(candidate), tentative, candidate),
                    )
        return [], float("inf")

    def _segment_is_free(self, start: np.ndarray, goal: np.ndarray) -> bool:
        distance = float(np.linalg.norm(goal - start))
        count = max(2, int(math.ceil(distance / self.grid_resolution * 2.0)))
        for fraction in np.linspace(0.0, 1.0, count):
            if not self._is_free(self._world_to_index(start + fraction * (goal - start))):
                return False
        return True

    def _simplify_path(self, path: np.ndarray) -> np.ndarray:
        if len(path) <= 2:
            return path
        simplified = [path[0]]
        anchor = 0
        candidate = 2
        while candidate < len(path):
            if not self._segment_is_free(path[anchor], path[candidate]):
                simplified.append(path[candidate - 1])
                anchor = candidate - 1
            candidate += 1
        simplified.append(path[-1])
        return np.asarray(simplified, dtype=float)

    def set_path(self, path_world: np.ndarray) -> None:
        """Load a world-frame polyline, including a future ROS Nav2 path."""
        path = np.asarray(path_world, dtype=float)
        if path.ndim != 2 or path.shape[1] != 2 or len(path) < 2:
            raise ValueError("global path must have shape (N, 2) with N >= 2")
        segments = np.diff(path, axis=0)
        lengths = np.linalg.norm(segments, axis=1)
        if np.any(lengths <= 1e-9):
            keep = np.concatenate(([True], lengths > 1e-9))
            path = path[keep]
            segments = np.diff(path, axis=0)
            lengths = np.linalg.norm(segments, axis=1)
        if len(path) < 2 or not np.all(lengths > 1e-9):
            raise ValueError("global path must contain two distinct points")
        self.path = path.copy()
        self.arc_lengths = np.concatenate(([0.0], np.cumsum(lengths)))
        self.total_length = float(self.arc_lengths[-1])

    def _require_path(self) -> None:
        if len(self.path) < 2:
            raise RuntimeError("global path has not been planned")

    def reference(self, robot_xy: np.ndarray, yaw: float) -> GlobalPathReference:
        """Return nearest-path and lookahead errors in the robot frame."""
        self._require_path()
        robot = np.asarray(robot_xy, dtype=float).reshape(2)
        segments = np.diff(self.path, axis=0)
        lengths = np.linalg.norm(segments, axis=1)
        tangents = segments / lengths[:, None]
        starts = self.path[:-1]
        relative = robot[None, :] - starts
        projection = np.sum(relative * tangents, axis=1)
        projection = np.clip(projection, 0.0, lengths)
        candidates = starts + tangents * projection[:, None]
        distances = np.linalg.norm(robot[None, :] - candidates, axis=1)
        segment_index = int(np.argmin(distances))
        nearest = candidates[segment_index]
        tangent = tangents[segment_index]
        arc_at_nearest = float(self.arc_lengths[segment_index] + projection[segment_index])
        cross_track = float(
            tangent[0] * (robot[1] - nearest[1])
            - tangent[1] * (robot[0] - nearest[0])
        )
        path_heading = math.atan2(float(tangent[1]), float(tangent[0]))
        lookahead_arc = min(self.total_length, arc_at_nearest + self.lookahead)
        target_segment = int(np.searchsorted(self.arc_lengths, lookahead_arc, side="right") - 1)
        target_segment = min(max(target_segment, 0), len(self.path) - 2)
        target_fraction = (
            lookahead_arc - self.arc_lengths[target_segment]
        ) / lengths[target_segment]
        target_world = self.path[target_segment] + target_fraction * segments[target_segment]
        target_local = _world_to_local(target_world - robot, yaw)
        remaining_distance = max(0.0, self.total_length - arc_at_nearest)
        return GlobalPathReference(
            target_local=target_local,
            lateral_error=cross_track,
            heading_error=_wrap_angle(path_heading - float(yaw)),
            remaining_distance=remaining_distance,
            progress=float(np.clip(arc_at_nearest / self.total_length, 0.0, 1.0)),
            nearest_index=segment_index,
            remaining_fraction=float(np.clip(
                (self.total_length - arc_at_nearest) / max(self.total_length, 1e-9),
                0.0,
                1.0,
            )),
            reference_speed=self._reference_speed(remaining_distance),
        )

    def _reference_speed(self, remaining_distance: float) -> float:
        """Return a smooth planner-side speed profile near the terminal path."""
        phase = np.clip(
            remaining_distance / self.terminal_slowdown_distance,
            0.0,
            1.0,
        )
        phase = phase * phase * (3.0 - 2.0 * phase)
        return float(
            self.reference_speed_min
            + (self.reference_speed_max - self.reference_speed_min) * phase
        )


def terminal_path_risk(
    planner: Nav2GlobalPath,
    predictions: Iterable[dict[str, Any]],
    *,
    robot_radius: float = 0.25,
    obstacle_radius: float = float(np.hypot(0.25, 0.25)),
    terminal_distance: float = 1.50,
    safety_margin: float = 0.12,
) -> float:
    """Estimate risk from tracked trajectories intersecting the final path.

    This is a reference feature, not a collision ground-truth signal.  It
    tells PPO/MPPI that an obstacle is predicted along the terminal corridor;
    physical contact remains evaluated independently by MuJoCo.
    """
    if terminal_distance <= 0.0 or len(planner.path) < 2:
        return 0.0
    start_arc = max(0.0, planner.total_length - terminal_distance)
    arcs = np.linspace(start_arc, planner.total_length, 16)
    segments = np.diff(planner.path, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    points: list[np.ndarray] = []
    for arc in arcs:
        segment = int(np.searchsorted(planner.arc_lengths, arc, side="right") - 1)
        segment = min(max(segment, 0), len(segments) - 1)
        fraction = (arc - planner.arc_lengths[segment]) / max(lengths[segment], 1e-9)
        points.append(planner.path[segment] + fraction * segments[segment])
    terminal_points = np.asarray(points, dtype=float)
    threshold = float(robot_radius + obstacle_radius + safety_margin)
    risk = 0.0
    for prediction in predictions:
        trajectory = prediction.get("trajectory_world")
        if trajectory is None:
            continue
        trajectory = np.asarray(trajectory, dtype=float).reshape(-1, 2)
        if trajectory.size == 0:
            continue
        closest = float(np.min(
            np.linalg.norm(
                trajectory[:, None, :] - terminal_points[None, :, :],
                axis=2,
            )
        ))
        risk = max(risk, float(np.clip((threshold - closest) / max(threshold, 1e-9), 0.0, 1.0)))
    return float(risk)


def estimate_wait_yield(
    planner: Nav2GlobalPath,
    predictions: Iterable[dict[str, Any]],
    robot_xy: np.ndarray,
    *,
    robot_radius: float = 0.25,
    obstacle_radius: float = float(np.hypot(0.25, 0.25)),
    safety_margin: float = 0.12,
    lookahead_distance: float = 3.0,
    max_wait_seconds: float = 2.0,
    min_obstacle_speed: float = 0.05,
) -> WaitYieldEstimate:
    """Estimate whether waiting can let a moving obstacle clear the path.

    The estimate uses tracked predicted trajectories and the current global
    path corridor. It deliberately does not use MuJoCo contact or ground
    truth obstacle state, so it remains a realistic observation for PPO.
    """
    if len(planner.path) < 2 or planner.total_length <= 1e-9:
        return WaitYieldEstimate()

    robot = np.asarray(robot_xy, dtype=float).reshape(2)
    reference = planner.reference(robot, 0.0)
    current_arc = planner.total_length * (1.0 - reference.remaining_fraction)
    end_arc = min(
        planner.total_length,
        current_arc + max(float(lookahead_distance), planner.lookahead),
    )
    path_arcs = np.linspace(current_arc, end_arc, 64)
    segments = np.diff(planner.path, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    path_points: list[np.ndarray] = []
    for arc in path_arcs:
        segment = int(np.searchsorted(planner.arc_lengths, arc, side="right") - 1)
        segment = min(max(segment, 0), len(segments) - 1)
        fraction = (arc - planner.arc_lengths[segment]) / max(lengths[segment], 1e-9)
        path_points.append(planner.path[segment] + fraction * segments[segment])
    path_points_array = np.asarray(path_points, dtype=float)
    corridor = float(robot_radius + obstacle_radius + safety_margin)

    earliest_block = float("inf")
    latest_block = 0.0
    longest_block = 0.0
    blocked_now = False
    future_block = False
    usable_prediction = False
    clear_times: list[float] = []
    moving_blocker = False
    stationary_collision_risk = False
    for prediction in predictions:
        track = prediction.get("track")
        if track is not None and int(getattr(track, "confidence", 0)) < 2:
            continue
        trajectory = np.asarray(prediction.get("trajectory_world", []), dtype=float)
        times = np.asarray(prediction.get("prediction_times", []), dtype=float)
        if trajectory.ndim != 2 or trajectory.shape[1] != 2 or len(trajectory) == 0:
            continue
        if len(times) != len(trajectory):
            times = np.linspace(0.0, 1.0, len(trajectory))
        usable_prediction = True
        track_velocity = np.asarray(
            prediction.get("velocity_world", prediction.get("velocity_local", [0.0, 0.0])),
            dtype=float,
        )
        moving = bool(np.linalg.norm(track_velocity) >= min_obstacle_speed)
        distances = np.min(
            np.linalg.norm(
                trajectory[:, None, :] - path_points_array[None, :, :],
                axis=2,
            ),
            axis=1,
        )
        blocked = distances <= corridor
        robot_distances = np.linalg.norm(trajectory - robot[None, :], axis=1)
        if np.any(robot_distances <= corridor):
            stationary_collision_risk = True
        if not np.any(blocked):
            continue
        blocked_times = times[blocked]
        start = float(np.min(blocked_times))
        end = float(np.max(blocked_times))
        earliest_block = min(earliest_block, start)
        latest_block = max(latest_block, end)
        longest_block = max(longest_block, end - start)
        blocked_now = blocked_now or bool(np.any(blocked_times <= 1e-9))
        future_block = future_block or bool(np.any(blocked_times > 1e-9))
        moving_blocker = moving_blocker or moving
        if blocked[0]:
            clear_indices = np.flatnonzero(~blocked)
            if clear_indices.size:
                clear_times.append(float(times[int(clear_indices[0])]))

    if not usable_prediction or not np.isfinite(earliest_block):
        return WaitYieldEstimate()

    horizon = max(float(latest_block), 1e-6)
    remaining = max(float(reference.remaining_distance), 1.0)
    wait_cost = np.clip(
        latest_block * max(float(reference.reference_speed), 0.04) / remaining,
        0.0,
        1.0,
    )
    planned_length = float(planner.plan_cost * planner.grid_resolution)
    extra_detour = max(0.0, planned_length - planner.total_length)
    detour_cost = np.clip(extra_detour / remaining, 0.0, 1.0)
    clear_time = min(clear_times) if clear_times else float("inf")
    waitable = bool(
        blocked_now
        and moving_blocker
        and np.isfinite(clear_time)
        and clear_time <= max(float(max_wait_seconds), 1e-6)
        and not stationary_collision_risk
    )
    return WaitYieldEstimate(
        path_blocked_now=float(blocked_now),
        path_blocked_future=float(future_block),
        time_to_path_block=float(np.clip(earliest_block / horizon, 0.0, 1.0)),
        time_to_path_clear=float(np.clip(
            (clear_time if np.isfinite(clear_time) else max_wait_seconds)
            / max(float(max_wait_seconds), 1e-6),
            0.0,
            1.0,
        )),
        blocking_duration=float(np.clip(longest_block / horizon, 0.0, 1.0)),
        wait_cost=float(wait_cost),
        detour_cost=float(detour_cost),
        waitable=float(waitable),
        stationary_collision_risk=float(stationary_collision_risk),
    )


def gate_wait_yield(
    planner: Nav2GlobalPath,
    estimate: WaitYieldEstimate,
    predictions: Iterable[dict[str, Any]],
    *,
    min_track_confidence: int = 4,
    min_lateral_speed: float = 0.12,
    max_longitudinal_speed: float = 0.35,
    max_clear_fraction: float = 0.85,
) -> WaitYieldEstimate:
    """Apply a crossing-specific gate to the soft WAIT estimate.

    The geometric estimator answers whether a predicted path corridor will
    clear.  It can still be true for a slowly turning, same-direction, or
    noisy nearly-static target.  WAIT is only exposed to the policy when a
    sufficiently mature track is moving laterally across the local global
    path, has no stationary collision risk, and clears within a short part of
    the estimator horizon.  AVOID remains the fallback for every other case.
    """
    if estimate.waitable <= 0.5 or estimate.stationary_collision_risk > 0.5:
        return replace(estimate, waitable=0.0)
    if estimate.time_to_path_clear > float(np.clip(max_clear_fraction, 0.0, 1.0)):
        return replace(estimate, waitable=0.0)

    crossing_target = False
    for prediction in predictions:
        track = prediction.get("track")
        if track is not None and int(getattr(track, "confidence", 0)) < min_track_confidence:
            continue
        position = np.asarray(
            prediction.get("position_world", prediction.get("trajectory_world", [[0.0, 0.0]])[0]),
            dtype=float,
        ).reshape(2)
        velocity = np.asarray(
            prediction.get("velocity_world", [0.0, 0.0]),
            dtype=float,
        ).reshape(2)
        if float(np.linalg.norm(velocity)) < min_lateral_speed:
            continue
        reference = planner.reference(position, 0.0)
        tangent = np.array([
            np.cos(reference.heading_error),
            np.sin(reference.heading_error),
        ])
        lateral = np.array([-tangent[1], tangent[0]])
        lateral_speed = abs(float(velocity @ lateral))
        longitudinal_speed = abs(float(velocity @ tangent))
        if (
            lateral_speed >= min_lateral_speed
            and longitudinal_speed <= max_longitudinal_speed
        ):
            crossing_target = True
            break
    return replace(estimate, waitable=float(crossing_target))


def path_lateral_command(
    reference: GlobalPathReference,
    *,
    gain: float,
    deadband: float,
    max_speed: float,
) -> float:
    """Convert signed cross-track error into a bounded local ``vy`` command."""
    if abs(reference.lateral_error) <= deadband:
        return 0.0
    return float(np.clip(
        -gain * reference.lateral_error,
        -max_speed,
        max_speed,
    ))


def path_heading_command(
    reference: GlobalPathReference,
    *,
    gain: float,
    max_rate: float,
    deadband: float = 0.02,
) -> float:
    """Turn toward the active path tangent with a bounded yaw-rate command."""
    if gain < 0.0 or max_rate <= 0.0 or deadband < 0.0:
        raise ValueError("heading gain/deadband must be non-negative and max rate positive")
    error = float(reference.heading_error)
    if abs(error) <= deadband:
        return 0.0
    return float(np.clip(gain * error, -max_rate, max_rate))
