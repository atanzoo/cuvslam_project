#!/usr/bin/env python3
"""Gymnasium environment for high-level D1 Edu PPO decisions.

The action space controls only the high-level decision.  MPPI still creates
the local velocity command and the simulated SDK still applies velocity,
acceleration, and delay limits.
"""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from typing import Any
import math

import gymnasium as gym
from gymnasium import spaces
import mujoco
import numpy as np

from d1_candidate_map import CandidateMap, load_candidate_map, occupied_rectangles
from d1_map_planner import CandidateMapPlanner, MapRoute
from run_d1_edu_decision_layer_benchmark import sample_episode_config
from run_d1_edu_decision_layer_viewer import (
    DecisionLayer,
    DynamicObstacleBody,
    EpisodeConfig,
    MultiTargetTracker,
    apply_lidar_model,
    collision_clearance,
    forward_only_cruise_command,
    goal_tracking_cruise_command,
    gradual_rejoin_lateral_command,
    initialize,
    local_goal,
    make_decision_observation,
    bounding_circle_clearance,
    mujoco_obstacle_contacts,
    rotation_world_from_local,
)
from d1_edu_nav2_global_path import (
    GlobalPathReference,
    Nav2GlobalPath,
    WaitYieldEstimate,
    estimate_wait_yield,
    gate_wait_yield,
    path_heading_command,
    path_lateral_command,
    terminal_path_risk,
)
from run_d1_edu_mppi_obstacle_viewer import (
    ObstacleMPPI,
    SimulatedLidar,
    advance,
    yaw_to_quat,
)
from run_d1_edu_obstacle_avoidance_viewer import Obstacle, build_scene
from d1_edu_scenario import ScenarioObstacle
from d1_edu_sim_sdk import SimulatedD1SDK
from d1_edu_footprint import calibrate_robot_footprint
from d1_robot_profile import get_robot_profile, require_runnable_profile


ACTION_NAMES = ("CRUISE", "AVOID_LEFT", "AVOID_RIGHT", "SLOWDOWN", "WAIT_YIELD")
MAP_PEDESTRIAN_MOTION_KINDS = (
    "crossing",
    "same_direction",
    "accelerating",
    "turning",
    "irregular",
)
# 4 robot/goal + 3 target slots * 10 + risk/elapsed + 7 global-reference
# values (lateral, heading, remaining fraction, lookahead lateral, progress,
# reference speed, terminal-path risk).
# + 9 planner wait/yield features: the seven original values plus an
# explicit waitable flag and stationary-robot collision risk.
OBSERVATION_DIM = 4 + 3 * 10 + 2 + 7 + 9


class D1DecisionEnv(gym.Env[np.ndarray, int]):
    """A step-based high-level decision environment for SB3 PPO."""

    metadata = {"render_modes": []}

    def __init__(
        self,
        *,
        profile: str = "baseline",
        robot_profile: str = "d1_edu",
        kinematic_proxy: bool = False,
        scenario_mode: str = "multi_target",
        seed: int = 0,
        duration: float = 24.0,
        route_length: float = 3.20,
        mppi_samples: int = 32,
        mppi_horizon: int = 30,
        collision_penalty: float = 20.0,
        timeout_penalty: float = 4.0,
        cruise_mode: str = "goal_hysteresis",
        candidate_map_yaml: str | Path | None = None,
        map_scene_scale: float = 5.0,
        map_route_center_buffer_m: float = 0.25,
    ) -> None:
        super().__init__()
        self.profile = profile
        self.robot_profile = get_robot_profile(robot_profile)
        self.kinematic_proxy = bool(kinematic_proxy)
        if self.kinematic_proxy and self.robot_profile.name != "d1_max_proxy":
            raise ValueError("kinematic_proxy is only valid with d1_max_proxy")
        if not self.kinematic_proxy:
            self.robot_profile = require_runnable_profile(robot_profile)
        self.scenario_mode = scenario_mode
        self.base_seed = int(seed)
        self.duration = float(duration)
        self.route_length = float(route_length)
        self.mppi_samples = int(mppi_samples)
        self.mppi_horizon = int(mppi_horizon)
        self.collision_penalty = float(collision_penalty)
        self.timeout_penalty = float(timeout_penalty)
        self.cruise_mode = str(cruise_mode)
        self.candidate_map: CandidateMap | None = None
        self.map_planner: CandidateMapPlanner | None = None
        self.map_route_planner: CandidateMapPlanner | None = None
        self.map_full_route: MapRoute | None = None
        self.map_manifest: dict[str, Any] | None = None
        self.map_wall_obstacles: list[Obstacle] = []
        self.map_wall_geom_ids: tuple[int, ...] = ()
        self.map_crossing_anchors: tuple[
            tuple[float, float, tuple[float, ...]], ...
        ] = ()
        self.map_scene_scale = float(map_scene_scale)
        self.map_route_center_buffer_m = float(map_route_center_buffer_m)
        if self.map_route_center_buffer_m < 0.0:
            raise ValueError("map route-center buffer cannot be negative")
        self.map3_dual_target_controller = bool(
            candidate_map_yaml is not None
            and math.isclose(self.map_scene_scale, 3.0, abs_tol=1e-9)
            and math.isclose(self.route_length, 10.0, abs_tol=1e-9)
        )
        self.active_map_path: np.ndarray | None = None
        if candidate_map_yaml is not None:
            if self.robot_profile.name != "d1_max_proxy" or not self.kinematic_proxy:
                raise ValueError(
                    "candidate-map episodes require --robot-profile d1_max_proxy "
                    "with --kinematic-proxy"
                )
            self.candidate_map = load_candidate_map(
                candidate_map_yaml,
                scene_scale=self.map_scene_scale,
                preserve_resolution=True,
            )
            self.map_planner = CandidateMapPlanner(
                self.candidate_map,
                robot_radius_m=0.49,
                obstacle_margin_m=0.12,
            )
            route_margin = 0.12 + (
                self.map_route_center_buffer_m
                if self.map3_dual_target_controller
                else 0.0
            )
            self.map_route_planner = (
                CandidateMapPlanner(
                    self.candidate_map,
                    robot_radius_m=0.49,
                    obstacle_margin_m=route_margin,
                )
                if route_margin > 0.12 + 1e-12
                else self.map_planner
            )
            map_start, map_goal = self.map_route_planner.safe_route_endpoints()
            self.map_full_route = self.map_route_planner.plan(map_start, map_goal)
            if self.route_length <= 0.0 or self.route_length >= self.map_full_route.length_m:
                raise ValueError(
                    "route_length must be positive and shorter than the candidate-map route"
                )
            self.map_manifest = self.candidate_map.manifest()
            self.map_manifest.update({
                "planner_robot_radius_m": self.map_planner.robot_radius_m,
                "planner_obstacle_margin_m": self.map_planner.obstacle_margin_m,
                "planner_inflation_cells": self.map_planner.inflation_cells,
                "route_selection_obstacle_margin_m": self.map_route_planner.obstacle_margin_m,
                "route_selection_inflation_cells": self.map_route_planner.inflation_cells,
                "route_center_extra_buffer_m": max(
                    0.0,
                    self.map_route_planner.obstacle_margin_m
                    - self.map_planner.obstacle_margin_m,
                ),
                "map_wall_rectangle_count": 0,
            })
            self.map_wall_obstacles = [Obstacle(
                x=wall.x,
                y=wall.y,
                half_length=wall.half_length,
                half_width=wall.half_width,
                height=1.25,
                kind="map_wall",
            ) for wall in occupied_rectangles(self.candidate_map)]
            self.map_manifest["map_wall_rectangle_count"] = len(self.map_wall_obstacles)
            self.map_crossing_anchors = self._find_map_crossing_anchors()
            self.map_manifest["crossing_anchor_count"] = len(self.map_crossing_anchors)
            self.map_manifest["pedestrian_motion_families"] = list(
                MAP_PEDESTRIAN_MOTION_KINDS
            )
            self.map_manifest["fixed_obstacles_per_episode"] = 1
        self.action_space = spaces.Discrete(5)
        self.observation_space = spaces.Box(
            low=-10.0,
            high=10.0,
            shape=(OBSERVATION_DIM,),
            dtype=np.float32,
        )
        self.master_rng = np.random.default_rng(self.base_seed)
        self.episode_count = 0
        self.episode_rng: np.random.Generator | None = None
        self.config: EpisodeConfig | None = None
        self.episode_seed = 0
        self.model: mujoco.MjModel | None = None
        self.data: mujoco.MjData | None = None
        self.gait: Any = None
        self.trajectories: list[ScenarioObstacle] = []
        self.obstacle_specs: list[Obstacle] = []
        self.obstacle_bodies: list[DynamicObstacleBody] = []
        self.lidar: SimulatedLidar | None = None
        self.tracker: MultiTargetTracker | None = None
        self.decision: DecisionLayer | None = None
        self.planner: ObstacleMPPI | None = None
        self.sdk: SimulatedD1SDK | None = None
        self.global_path: Nav2GlobalPath | None = None
        self.latest_path_reference: GlobalPathReference | None = None
        self.latest_wait_estimate = WaitYieldEstimate()
        self.goal_world = np.zeros(2, dtype=float)
        self.yaw = 0.0
        self.elapsed = 0.0
        self.sim_dt = 0.0
        self.active_mode = "CRUISE"
        self.wait_duration = 0.0
        self.wait_cooldown_until = 0.0
        self.wait_recovery_deadline: float | None = None
        self.last_action_overridden = False
        self.cruise_reverse_active = False
        self.rejoin_started_at: float | None = None
        self.progress_reference_time = 0.0
        self.progress_reference_distance = 0.0
        self.stagnation_windows = 0
        self.initial_goal_distance = 0.0
        self.deadlock_latched = False
        self.collision = False
        self.geometric_collision = False
        self.proxy_collision = False
        self.proxy_only = False
        self.physical_contact = False
        self.near_miss = False
        self.reached_goal = False
        self.path_length = 0.0
        self.min_clearance = float("inf")
        self.avoid_duration = 0.0
        self.last_observation = np.zeros(OBSERVATION_DIM, dtype=np.float32)
        self.latest_points = np.empty((0, 2), dtype=float)
        self.latest_predictions: list[dict[str, Any]] = []
        self.latest_risk = False
        self.latest_ttc = float("inf")
        self.latest_closest = float("inf")
        self.latest_confidence = 0
        self.latest_baseline_mode = "CRUISE"
        self.latest_obstacle_position_local: np.ndarray | None = None
        self.latest_obstacle_velocity_local: np.ndarray | None = None
        self.footprint: Any = None
        self.global_robot_radius = 0.25
        self.map_violation = False

    def _require_ready(self) -> None:
        if self.config is None or self.model is None or self.data is None:
            raise RuntimeError("environment has not been reset")

    def _current_states(self) -> list[tuple[np.ndarray, np.ndarray, bool]]:
        return [trajectory.state(self.elapsed) for trajectory in self.trajectories]

    def _update_bodies(self, elapsed: float) -> None:
        self._require_ready()
        for body in self.obstacle_bodies:
            body.update(self.data, elapsed)
        mujoco.mj_forward(self.model, self.data)

    @staticmethod
    def _polyline_point(
        points: np.ndarray,
        cumulative: np.ndarray,
        distance: float,
    ) -> np.ndarray:
        segment_lengths = np.diff(cumulative)
        index = min(
            max(int(np.searchsorted(cumulative, distance, side="right") - 1), 0),
            len(segment_lengths) - 1,
        )
        fraction = (distance - cumulative[index]) / segment_lengths[index]
        return points[index] + fraction * (points[index + 1] - points[index])

    def _find_map_crossing_anchors(
        self,
    ) -> tuple[tuple[float, float, tuple[float, ...]], ...]:
        if self.map_planner is None or self.map_full_route is None:
            return ()
        points = np.asarray(self.map_full_route.world_points, dtype=float)
        cumulative = np.concatenate((
            [0.0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1)),
        ))
        total = float(cumulative[-1])
        fractions = (0.25, 0.375, 0.50, 0.625, 0.75)
        anchor_spacing = max(0.25, self.candidate_map.scaled_resolution * 5.0)
        lateral_spacing = max(0.10, self.candidate_map.scaled_resolution)
        anchors: list[tuple[float, float, tuple[float, ...]]] = []
        for distance in np.arange(0.0, total + anchor_spacing * 0.5, anchor_spacing):
            distance = min(float(distance), total)
            valid_fractions = tuple(
                fraction for fraction in fractions
                if distance - fraction * self.route_length >= 0.0
                and distance + (1.0 - fraction) * self.route_length <= total
            )
            if not valid_fractions:
                continue
            before = self._polyline_point(
                points, cumulative, max(0.0, distance - 0.10),
            )
            after = self._polyline_point(
                points, cumulative, min(total, distance + 0.10),
            )
            tangent = after - before
            norm = float(np.linalg.norm(tangent))
            if norm <= 1e-9:
                continue
            normal = np.array([-tangent[1], tangent[0]], dtype=float) / norm
            center = self._polyline_point(points, cumulative, distance)
            available = 0.0
            for offset in np.arange(lateral_spacing, 2.51, lateral_spacing):
                sample_offsets = np.arange(
                    -offset, offset + lateral_spacing * 0.5, lateral_spacing,
                )
                if all(
                    self.map_route_planner.is_traversable_world(
                        *(center + normal * float(sample_offset))
                    )
                    for sample_offset in sample_offsets
                ):
                    available = float(offset)
                else:
                    break
            if available >= 0.40:
                anchors.append((distance, available, valid_fractions))
        if not anchors:
            raise ValueError("candidate map has no validated pedestrian crossing corridor")
        return tuple(anchors)

    def _route_frame_at(
        self,
        route_path: np.ndarray,
        route_cumulative: np.ndarray,
        distance: float,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        point = self._polyline_point(route_path, route_cumulative, distance)
        before = self._polyline_point(
            route_path, route_cumulative, max(0.0, distance - 0.10),
        )
        after = self._polyline_point(
            route_path,
            route_cumulative,
            min(float(route_cumulative[-1]), distance + 0.10),
        )
        tangent = after - before
        norm = float(np.linalg.norm(tangent))
        if norm <= 1e-9:
            raise ValueError("candidate-map route has no local tangent")
        tangent /= norm
        return point, tangent, np.array([-tangent[1], tangent[0]], dtype=float)

    def _is_known_free_world(self, x: float, y: float) -> bool:
        if self.candidate_map is None:
            return False
        try:
            column, row = self.candidate_map.world_to_cell(x, y)
        except (IndexError, ValueError):
            return False
        return self.candidate_map.classify_pixel(
            self.candidate_map.pixel(column, row)
        ) == "free"

    def _map_static_obstacle_candidates(
        self,
        route_path: np.ndarray,
        route_cumulative: np.ndarray,
    ) -> list[tuple[float, np.ndarray]]:
        """Find route-blocking object poses with at least one map-safe bypass."""
        if self.map_planner is None:
            return []
        candidates: list[tuple[float, np.ndarray]] = []
        bypass_offset = (
            self.map_planner.robot_radius_m
            + math.hypot(0.25, 0.25)
            + self.map_planner.obstacle_margin_m
        )
        for distance in np.arange(1.5, self.route_length - 1.25, 0.25):
            point, _, _ = self._route_frame_at(
                route_path, route_cumulative, float(distance),
            )
            box_samples = [
                point + np.array([dx, dy], dtype=float)
                for dx in np.arange(-0.25, 0.251, 0.05)
                for dy in np.arange(-0.25, 0.251, 0.05)
            ]
            if not all(
                self._is_known_free_world(float(sample[0]), float(sample[1]))
                for sample in box_samples
            ):
                continue
            for side in (-1.0, 1.0):
                bypass_is_free = True
                for along in np.arange(-1.25, 1.251, 0.10):
                    sample_distance = float(np.clip(
                        distance + along, 0.0, self.route_length,
                    ))
                    route_point, _, normal = self._route_frame_at(
                        route_path, route_cumulative, sample_distance,
                    )
                    bypass = route_point + side * bypass_offset * normal
                    if not self.map_planner.is_traversable_world(
                        float(bypass[0]), float(bypass[1]),
                    ):
                        bypass_is_free = False
                        break
                if bypass_is_free:
                    candidates.append((float(distance), point.copy()))
                    break
        return candidates

    def _map_pedestrian_track_valid(
        self,
        actor: ScenarioObstacle,
        *,
        static_center: np.ndarray | None = None,
    ) -> bool:
        if self.map_planner is None:
            return False
        actor_radius = math.hypot(actor.half_length, actor.half_width)
        static_radius = math.hypot(0.25, 0.25)
        for elapsed in np.arange(0.0, self.duration + 0.05, 0.10):
            position, _, active = actor.state(float(elapsed))
            if not active:
                continue
            if not self.map_planner.is_traversable_world(
                float(position[0]), float(position[1]),
            ):
                return False
            if static_center is not None and np.linalg.norm(position - static_center) <= (
                actor_radius + static_radius + 0.10
            ):
                return False
        return True

    @staticmethod
    def _map_person_candidate(
        motion_kind: str,
        *,
        center: np.ndarray,
        tangent: np.ndarray,
        normal: np.ndarray,
        mirror_sign: float,
        lateral_offset: float,
        rng: np.random.Generator,
    ) -> ScenarioObstacle:
        side_center = center + mirror_sign * lateral_offset * normal
        speed = float(rng.uniform(0.16, 0.24))
        values: dict[str, Any] = {
            "obstacle_id": f"map_person_{motion_kind}_00",
            "kind": "person",
            "motion_kind": motion_kind,
            "x": float(side_center[0]),
            "y": float(side_center[1]),
            "half_length": 0.25,
            "half_width": 0.25,
            "appear_time": 0.0,
            "height": 0.85,
            "max_speed": 0.50,
        }
        if motion_kind == "same_direction":
            values.update({
                "vx": float(min(speed, 0.05) * tangent[0]),
                "vy": float(min(speed, 0.05) * tangent[1]),
                "disappear_time": 12.0,
            })
        elif motion_kind == "accelerating":
            acceleration = 0.008
            values.update({
                "vx": float(0.01 * tangent[0]),
                "vy": float(0.01 * tangent[1]),
                "ax": float(acceleration * tangent[0]),
                "ay": float(acceleration * tangent[1]),
                "disappear_time": 10.0,
            })
        elif motion_kind == "turning":
            radius = 0.10
            heading = math.atan2(float(tangent[1]), float(tangent[0]))
            phase = heading + mirror_sign * float(rng.uniform(-math.pi, math.pi))
            turn_center = center + mirror_sign * min(lateral_offset, 0.05) * normal
            values.update({
                "x": float(turn_center[0] + radius * math.cos(phase)),
                "y": float(turn_center[1] + radius * math.sin(phase)),
                "turn_center_x": float(turn_center[0]),
                "turn_center_y": float(turn_center[1]),
                "turn_radius": radius,
                "turn_phase": phase,
                "turn_rate": mirror_sign * float(rng.uniform(0.30, 0.50)),
                "disappear_time": 10.0,
            })
        elif motion_kind == "irregular":
            phase = float(rng.uniform(-math.pi, math.pi))
            values.update({
                "vx": float(0.025 * tangent[0]),
                "vy": float(0.025 * tangent[1]),
                "irregular_lateral_amplitude": 0.020,
                "irregular_frequency": 0.85,
                "irregular_phase": phase if mirror_sign > 0 else -phase,
                "irregular_secondary_amplitude": 0.008,
                "irregular_secondary_frequency": 1.70,
                "irregular_secondary_phase": phase + math.pi,
                "disappear_time": 10.0,
            })
        else:
            raise ValueError(f"unsupported map pedestrian motion: {motion_kind}")
        return ScenarioObstacle(**values)

    def _make_map_episode_once(
        self,
        episode_seed: int,
        curriculum_index: int,
    ) -> tuple[np.ndarray, tuple[ScenarioObstacle, ScenarioObstacle], str]:
        if self.map_planner is None or self.map_full_route is None:
            raise RuntimeError("candidate-map environment is not configured")
        points = np.asarray(self.map_full_route.world_points, dtype=float)
        cumulative = np.concatenate((
            [0.0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1)),
        ))
        rng = np.random.default_rng(episode_seed)
        index = max(int(curriculum_index), 0)
        motion_kind = MAP_PEDESTRIAN_MOTION_KINDS[index % len(MAP_PEDESTRIAN_MOTION_KINDS)]
        mirror_sign = 1.0 if (index // len(MAP_PEDESTRIAN_MOTION_KINDS)) % 2 == 0 else -1.0
        crossing_options = [
            (anchor, fraction)
            for anchor in self.map_crossing_anchors
            for fraction in anchor[2]
            if 2.00 <= fraction * self.route_length <= 3.75
        ]
        if not crossing_options:
            crossing_options = [
                (anchor, fraction)
                for anchor in self.map_crossing_anchors
                for fraction in anchor[2]
            ]
        (anchor_distance, available, _), fraction = crossing_options[
            int(rng.integers(0, len(crossing_options)))
        ]
        start_distance = anchor_distance - self.route_length * fraction
        end_distance = start_distance + self.route_length
        start = self._polyline_point(points, cumulative, start_distance)
        goal = self._polyline_point(points, cumulative, end_distance)
        interior = points[(cumulative > start_distance + 1e-8) & (cumulative < end_distance - 1e-8)]
        route_path = np.vstack((start, interior, goal))
        route_cumulative = np.concatenate((
            [0.0], np.cumsum(np.linalg.norm(np.diff(route_path, axis=0), axis=1)),
        ))

        # Keep the moving target inside the robot's 5 m sensing horizon early
        # in the route; place the fixed object later so both matter in one
        # episode instead of the pedestrian disappearing before encounter.
        person_center_distance = min(2.60, self.route_length - 1.0)
        static_candidates = self._map_static_obstacle_candidates(
            route_path, route_cumulative,
        )
        if motion_kind == "crossing":
            crossing_center_distance = fraction * self.route_length
            center, _, normal = self._route_frame_at(
                route_path, route_cumulative, crossing_center_distance,
            )
            offset = min(1.50, available - 0.05)
            if offset <= 0.0:
                raise ValueError("candidate-map crossing has no usable lateral width")
            speed = 0.18
            mirrored_actors: dict[float, ScenarioObstacle] = {}
            for side_sign in (1.0, -1.0):
                actor_start = center - side_sign * offset * normal
                actor_goal = center + side_sign * offset * normal
                delta = actor_goal - actor_start
                distance = float(np.linalg.norm(delta))
                mirrored_actors[side_sign] = ScenarioObstacle(
                    obstacle_id="map_person_crossing_00",
                    kind="person",
                    motion_kind="crossing",
                    x=float(actor_start[0]),
                    y=float(actor_start[1]),
                    half_length=0.25,
                    half_width=0.25,
                    vx=float(delta[0] / distance * speed),
                    vy=float(delta[1] / distance * speed),
                    appear_time=0.0,
                    disappear_time=distance / speed + 0.10,
                    height=0.85,
                    max_speed=speed,
                )
            actor = mirrored_actors[mirror_sign]
            person_center_distance = crossing_center_distance
            if not all(self._map_pedestrian_track_valid(item) for item in mirrored_actors.values()):
                raise ValueError("generated pedestrian crossing leaves inflated known-free space")
            separated_candidates = [
                candidate for candidate in static_candidates
                if abs(candidate[0] - person_center_distance) >= 2.50
                and all(
                    self._map_pedestrian_track_valid(item, static_center=candidate[1])
                    for item in mirrored_actors.values()
                )
            ]
        else:
            separated_candidates = []
            station_candidates = sorted(
                np.arange(1.25, 4.01, 0.25),
                key=lambda distance: abs(float(distance) - person_center_distance),
            )
            for station in station_candidates:
                center, tangent, normal = self._route_frame_at(
                    route_path, route_cumulative, float(station),
                )
                for lateral_offset in (0.15, 0.05, 0.0):
                    candidate_seed = (
                        int(episode_seed) ^ int(round(float(station) * 1000.0))
                        ^ int(round(lateral_offset * 1000.0))
                    )
                    mirrored_actors = {
                        side_sign: self._map_person_candidate(
                            motion_kind,
                            center=center,
                            tangent=tangent,
                            normal=normal,
                            mirror_sign=side_sign,
                            lateral_offset=lateral_offset,
                            rng=np.random.default_rng(candidate_seed),
                        )
                        for side_sign in (1.0, -1.0)
                    }
                    if not all(
                        self._map_pedestrian_track_valid(item)
                        for item in mirrored_actors.values()
                    ):
                        continue
                    available_objects = [
                        candidate for candidate in static_candidates
                        if abs(candidate[0] - float(station)) >= 2.50
                        and all(
                            self._map_pedestrian_track_valid(
                                item, static_center=candidate[1],
                            )
                            for item in mirrored_actors.values()
                        )
                    ]
                    if available_objects:
                        actor = mirrored_actors[mirror_sign]
                        person_center_distance = float(station)
                        separated_candidates = available_objects
                        break
                if separated_candidates:
                    break
            if not separated_candidates:
                raise ValueError(
                    f"candidate map has no valid {motion_kind} track and separated "
                    "fixed-object bypass"
                )

        if not separated_candidates:
            raise ValueError(
                "candidate map has no safe fixed-object bypass separated from pedestrian "
                f"(pedestrian_s={person_center_distance:.2f}, "
                f"valid_object_s={[round(item[0], 2) for item in static_candidates]})"
            )
        static_distance, static_center = max(
            separated_candidates,
            key=lambda candidate: abs(candidate[0] - person_center_distance),
        )
        fixed_obstacle = ScenarioObstacle(
            obstacle_id="map_fixed_object_00",
            kind="object",
            motion_kind="static",
            x=float(static_center[0]),
            y=float(static_center[1]),
            half_length=0.25,
            half_width=0.25,
            height=0.30,
            max_speed=0.0,
        )
        if not self._map_pedestrian_track_valid(actor, static_center=static_center):
            raise ValueError("pedestrian trajectory leaves the safe dual-target envelope")
        if not all(
            self.map_route_planner.is_traversable_world(float(point[0]), float(point[1]))
            for point in route_path
        ):
            raise ValueError("generated 10 m route leaves inflated known-free space")
        if abs(static_distance - person_center_distance) < 2.50:
            raise AssertionError("fixed object and pedestrian were not longitudinally separated")
        return route_path, (fixed_obstacle, actor), motion_kind

    def _make_map_episode(
        self,
        episode_seed: int,
        curriculum_index: int,
    ) -> tuple[np.ndarray, tuple[ScenarioObstacle, ScenarioObstacle], str, int]:
        """Retry deterministic route seeds when this map segment cannot host both targets."""
        failures: list[str] = []
        modulus = 2**31 - 1
        for attempt in range(32):
            scene_seed = (int(episode_seed) + attempt * 104_729) % modulus
            try:
                route, obstacles, motion = self._make_map_episode_once(
                    scene_seed, curriculum_index,
                )
                return route, obstacles, motion, scene_seed
            except ValueError as error:
                failures.append(str(error))
        raise ValueError(
            "unable to generate a valid map dual-target episode after 32 deterministic "
            f"route candidates; last rejection: {failures[-1] if failures else 'none'}"
        )

    def _setup_episode(
        self,
        config: EpisodeConfig,
        episode_seed: int,
        map_path: np.ndarray | None = None,
    ) -> None:
        self.config = config
        self.episode_seed = int(episode_seed)
        self.episode_rng = np.random.default_rng(self.episode_seed)
        if config.scenario_obstacles:
            self.trajectories = [
                ScenarioObstacle.from_mapping(dict(mapping))
                for mapping in config.scenario_obstacles
            ]
        else:
            self.trajectories = [ScenarioObstacle(
                obstacle_id="legacy_box",
                kind="legacy_dynamic_box",
                x=config.box_x,
                y=config.box_y,
                half_length=config.box_size_x,
                half_width=config.box_size_y,
                vx=config.box_vx,
                vy=config.box_vy,
                max_speed=config.max_obstacle_speed,
            )]
        self.obstacle_specs = [Obstacle(
            x=trajectory.x,
            y=trajectory.y,
            half_length=trajectory.half_length,
            half_width=trajectory.half_width,
            height=trajectory.height,
            kind=trajectory.kind,
        ) for trajectory in self.trajectories]
        if self.robot_profile.model_asset is None:
            raise RuntimeError(
                f"robot profile {self.robot_profile.name!r} has no model asset"
            )
        model_path = Path(self.robot_profile.model_asset)
        if not model_path.is_absolute():
            model_path = Path(__file__).resolve().parents[3] / model_path
        if self.model is None or self.candidate_map is None:
            if self.candidate_map is not None:
                x_min, y_min, x_max, y_max = self.candidate_map.world_extent
                world_length = max(x_max - x_min, y_max - y_min)
            else:
                world_length = config.route_length
            self.model, _ = build_scene(
                with_lidar=True,
                dynamic_obstacles=self.obstacle_specs,
                static_obstacles=(self.map_wall_obstacles or None),
                lidar_ray_count=config.lidar_ray_count,
                lidar_fov=config.lidar_fov,
                lidar_max_range=config.lidar_max_range,
                world_length=world_length,
                model_path=model_path,
            )
        self.data, self.gait = initialize(
            self.model,
            kinematic_proxy=self.kinematic_proxy,
        )
        self.footprint = calibrate_robot_footprint(
            self.model,
            self.data,
            planner_min_radius=0.49 if self.kinematic_proxy else 0.32,
            planner_max_radius=0.49 if self.kinematic_proxy else 0.35,
        )
        self.global_robot_radius = float(self.footprint.planner_radius)
        if self.map_planner is not None and not math.isclose(
            self.global_robot_radius,
            self.map_planner.robot_radius_m,
            rel_tol=0.0,
            abs_tol=1e-6,
        ):
            raise RuntimeError(
                "candidate-map inflation radius does not match the D1 Max proxy footprint"
            )
        # Use the calibrated robot radius for every global-path and geometric
        # clearance query in this environment.  This prevents D1 Max training
        # from silently using the D1 Edu 0.25 m default.
        self.config = config = replace(
            config,
            robot_profile=self.robot_profile.name,
            kinematic_proxy=self.kinematic_proxy,
            global_path_robot_radius=self.global_robot_radius,
        )
        self.yaw = 0.0
        if map_path is not None:
            if len(map_path) < 2:
                raise ValueError("candidate-map episode path must contain at least two points")
            self.active_map_path = np.asarray(map_path, dtype=float).copy()
            self.data.qpos[:2] = self.active_map_path[0]
            initial_delta = self.active_map_path[1] - self.active_map_path[0]
            self.yaw = float(math.atan2(initial_delta[1], initial_delta[0]))
            self.data.qpos[3:7] = yaw_to_quat(self.yaw)
            mujoco.mj_forward(self.model, self.data)
        else:
            self.active_map_path = None
        self.obstacle_bodies = [
            DynamicObstacleBody(self.model, trajectory, index=index)
            for index, trajectory in enumerate(self.trajectories)
        ]
        dynamic_geom_ids = tuple(
            int(self.model.geom(
                "moving_obstacle_geom"
                if index == 0
                else f"moving_obstacle_{index:02d}_geom"
            ).id)
            for index in range(len(self.obstacle_specs))
        )
        self.map_wall_geom_ids = tuple(
            int(self.model.geom(f"candidate_map_wall_{index:04d}_geom").id)
            for index in range(len(self.map_wall_obstacles))
        ) if self.candidate_map is not None else ()
        self.obstacle_geom_ids = dynamic_geom_ids + self.map_wall_geom_ids
        self.lidar = SimulatedLidar(
            self.model,
            ray_count=config.lidar_ray_count,
            max_range=config.lidar_max_range,
            fov=config.lidar_fov,
        )
        self.tracker = MultiTargetTracker(
            obstacle_half_length=max(obstacle.half_length for obstacle in self.obstacle_specs),
            obstacle_half_width=max(obstacle.half_width for obstacle in self.obstacle_specs),
            robot_safety_radius=self.global_robot_radius,
            prediction_position_uncertainty=config.obstacle_position_uncertainty,
            prediction_velocity_uncertainty=config.obstacle_velocity_uncertainty,
        )
        decision_exit_distance = max(
            0.95,
            self.tracker.effective_safe_distance + 0.10,
        )
        self.decision = DecisionLayer(
            enter_safe_distance=self.tracker.effective_safe_distance,
            exit_safe_distance=decision_exit_distance,
            risk_updates_required=config.risk_confirmation_updates,
        )
        planner_seed = config.mppi_seed if config.mppi_seed is not None else episode_seed
        self.planner = ObstacleMPPI(
            horizon=config.mppi_horizon,
            samples=config.mppi_samples,
            dynamic_obstacle_radius=float(np.hypot(
                self.obstacle_specs[0].half_length,
                self.obstacle_specs[0].half_width,
            )),
            rng_seed=planner_seed,
            clearance_uncertainty=config.clearance_uncertainty,
            dynamic_position_uncertainty=config.obstacle_position_uncertainty,
            dynamic_velocity_uncertainty=config.obstacle_velocity_uncertainty,
            command_delay=config.command_delay,
            robot_radius=self.footprint.planner_radius,
        )
        self.sdk = SimulatedD1SDK(
            max_velocity=np.array([config.max_vx, config.max_vy, config.max_yaw_rate]),
            max_acceleration=np.array([config.max_ax, config.max_ay, config.max_yaw_acceleration]),
            command_delay=config.command_delay,
        )
        self.goal_world = np.array([config.goal_x, config.goal_y], dtype=float)
        self.global_path = Nav2GlobalPath(
            lookahead=config.global_path_lookahead,
            grid_resolution=config.global_path_grid_resolution,
            search_padding=config.global_path_search_padding,
            soft_cost_radius=config.global_path_soft_cost_radius,
            cost_weight=config.global_path_cost_weight,
            reference_speed_max=config.max_vx,
            reference_speed_min=min(config.cruise_min_approach_speed, config.max_vx),
        )
        if map_path is not None:
            self.global_path.set_path(map_path)
        else:
            self.global_path.plan(
                self.data.qpos[:2],
                self.goal_world,
                obstacles=self.obstacle_specs,
                robot_radius=config.global_path_robot_radius,
                obstacle_margin=config.global_path_obstacle_margin,
            )
        self.last_global_replan = 0.0
        self.sim_dt = self.model.opt.timestep * 4.0
        self.elapsed = 0.0
        self.active_mode = "CRUISE"
        self.wait_duration = 0.0
        self.wait_cooldown_until = 0.0
        self.wait_recovery_deadline = None
        self.cruise_reverse_active = False
        self.rejoin_started_at = None
        self.progress_reference_time = 0.0
        self.progress_reference_distance = float(np.linalg.norm(self.data.qpos[:2] - self.goal_world))
        self.stagnation_windows = 0
        self.initial_goal_distance = self.progress_reference_distance
        self.deadlock_latched = False
        self.collision = False
        self.map_violation = False
        self.proxy_collision = False
        self.physical_contact = False
        self.near_miss = False
        self.reached_goal = False
        self.path_length = 0.0
        # Put the MuJoCo obstacle geoms at t=0 before collecting both
        # geometric diagnostics and physical contact telemetry.
        self._update_bodies(0.0)
        initial_clearances = [
            collision_clearance(
                self.data.qpos[:2],
                state[0],
                obstacle,
                robot_radius=self.global_robot_radius,
            )
            for state, obstacle in zip(self._current_states(), self.obstacle_specs)
            if state[2]
        ]
        self.min_clearance = min(initial_clearances) if initial_clearances else float("inf")
        initial_proxy_clearances = [
            bounding_circle_clearance(
                self.data.qpos[:2],
                state[0],
                obstacle,
                robot_radius=self.global_robot_radius,
            )
            for state, obstacle in zip(self._current_states(), self.obstacle_specs)
            if state[2]
        ]
        self.min_proxy_clearance = (
            min(initial_proxy_clearances)
            if initial_proxy_clearances
            else float("inf")
        )
        self.geometric_collision = self.min_clearance < -config.collision_tolerance
        self.near_miss = (
            self.min_clearance < 0.0
            and self.min_clearance >= -config.collision_tolerance
        )
        self.proxy_collision = self.min_proxy_clearance < 0.0
        self.physical_contact, _, _ = mujoco_obstacle_contacts(
            self.model,
            self.data,
            self.obstacle_geom_ids,
        )
        self.collision = bool(self.physical_contact)
        self.near_miss = bool(self.near_miss and not self.physical_contact)
        self.proxy_only = bool(
            self.proxy_collision
            and not self.physical_contact
            and not self.geometric_collision
        )
        self.avoid_duration = 0.0

    def _observe(self) -> np.ndarray:
        self._require_ready()
        assert self.config is not None
        assert self.lidar is not None
        assert self.tracker is not None
        assert self.decision is not None
        assert self.sdk is not None
        assert self.episode_rng is not None
        assert self.global_path is not None
        raw_scan = self.lidar.scan(self.data)
        scan = apply_lidar_model(
            raw_scan,
            self.episode_rng,
            self.config.lidar_noise_std,
            self.config.lidar_dropout_rate,
            self.config.lidar_max_range,
        )
        self.latest_points = self.lidar.point_cloud(scan)
        ray_indices = self.lidar.hit_ray_indices(scan)
        robot_xy = self.data.qpos[:2].copy()
        current_states = self._current_states()
        if (
            self.map_planner is None
            and self.elapsed - self.last_global_replan
            >= self.config.global_path_replan_period - 1e-9
        ):
            dynamic_specs = [
                Obstacle(
                    x=float(state[0][0]),
                    y=float(state[0][1]),
                    half_length=obstacle.half_length,
                    half_width=obstacle.half_width,
                    height=obstacle.height,
                )
                for state, obstacle in zip(current_states, self.obstacle_specs)
                if state[2]
            ]
            self.global_path.plan(
                robot_xy,
                self.goal_world,
                obstacles=dynamic_specs,
                robot_radius=self.config.global_path_robot_radius,
                obstacle_margin=self.config.global_path_obstacle_margin,
            )
            self.last_global_replan = self.elapsed
        self.latest_path_reference = self.global_path.reference(robot_xy, self.yaw)
        robot_velocity_world = self.sdk.getWorldVelocity(self.yaw)[:2]
        self.tracker.update(self.latest_points, robot_xy, self.yaw, self.elapsed, ray_indices)
        self.latest_risk, self.latest_ttc, self.latest_closest = self.tracker.collision_prediction(
            robot_xy,
            robot_velocity_world,
            yaw=self.yaw,
            horizon=self.config.trajectory_prediction_horizon,
        )
        self.latest_predictions = self.tracker.predictions(
            robot_xy,
            robot_velocity_world,
            yaw=self.yaw,
            horizon=self.config.trajectory_prediction_horizon,
            sample_dt=self.config.trajectory_sample_dt,
        )
        self.latest_path_reference = replace(
            self.latest_path_reference,
            terminal_risk=terminal_path_risk(
                self.global_path,
                self.latest_predictions,
                robot_radius=self.config.global_path_robot_radius,
                obstacle_radius=float(np.hypot(
                    self.tracker.obstacle_half_length,
                    self.tracker.obstacle_half_width,
                )),
            ),
        )
        self.latest_wait_estimate = estimate_wait_yield(
            self.global_path,
            self.latest_predictions,
            robot_xy,
            robot_radius=self.config.global_path_robot_radius,
            obstacle_radius=float(np.hypot(
                self.tracker.obstacle_half_length,
                self.tracker.obstacle_half_width,
            )),
            lookahead_distance=max(
                self.config.global_path_lookahead * 3.0,
                self.config.global_path_robot_radius * 4.0,
            ),
            max_wait_seconds=self.config.wait_max_clear_time,
            min_obstacle_speed=self.config.wait_min_obstacle_speed,
        )
        self.latest_wait_estimate = gate_wait_yield(
            self.global_path,
            self.latest_wait_estimate,
            self.latest_predictions,
            min_track_confidence=self.config.wait_min_track_confidence,
            min_lateral_speed=self.config.wait_min_lateral_speed,
            max_longitudinal_speed=self.config.wait_max_longitudinal_speed,
            max_clear_fraction=self.config.wait_max_clear_fraction,
        )
        self.latest_confidence = self.tracker.confidence
        self.latest_obstacle_position_local = None
        self.latest_obstacle_velocity_local = None
        if self.tracker.observation is not None:
            self.latest_obstacle_position_local = (
                rotation_world_from_local(self.yaw).T
                @ (self.tracker.observation - robot_xy)
            )
            self.latest_obstacle_velocity_local = (
                rotation_world_from_local(self.yaw).T @ self.tracker.velocity
            )
        self.latest_baseline_mode = self.decision.update(
            risk=self.latest_risk,
            time_to_closest=self.latest_ttc,
            closest_distance=self.latest_closest,
            confidence=self.latest_confidence,
        )
        observation = make_decision_observation(
            local_goal(self.data, self.yaw, self.goal_world),
            self.sdk.getBodyVelocity()[:2],
            self.latest_obstacle_position_local,
            self.latest_obstacle_velocity_local,
            self.latest_confidence,
            self.latest_risk,
            self.latest_ttc,
            self.latest_closest,
            self.elapsed,
            self.config.duration,
            tracked_obstacles=self.latest_predictions,
            path_reference=self.latest_path_reference,
            wait_estimate=self.latest_wait_estimate,
        )
        self.last_observation = np.asarray(observation, dtype=np.float32)
        return self.last_observation

    def _wait_is_allowed(self) -> bool:
        """Return whether WAIT_YIELD is a valid action in this state."""
        assert self.config is not None
        estimate = self.latest_wait_estimate
        return bool(
            estimate.waitable > 0.5
            and estimate.stationary_collision_risk < 0.5
            and self.wait_duration < self.config.wait_max_duration
            and self.elapsed >= self.wait_cooldown_until
        )

    def _command_for_action(self, action: int) -> np.ndarray:
        self._require_ready()
        assert self.config is not None
        assert self.planner is not None
        assert self.latest_path_reference is not None
        if action not in range(5):
            raise ValueError(f"invalid PPO action: {action}")
        previous_mode = self.active_mode
        selected_action = int(action)
        self.last_action_overridden = False
        immediate_risk = bool(
            self.latest_risk
            and self.latest_ttc < self.config.minimum_reaction_time
        )
        if immediate_risk:
            obstacle_side = 1.0 if (
                self.latest_obstacle_position_local is None
                or self.latest_obstacle_position_local[1] <= 0.0
            ) else -1.0
            selected_action = 1 if obstacle_side > 0.0 else 2
        elif selected_action == 4 and not self._wait_is_allowed():
            # A persistent/static blocker or an obstacle that would hit a
            # stationary robot must never be handled by indefinite waiting.
            # Convert the learned request into a side-specific MPPI avoid.
            obstacle_side = 1.0 if (
                self.latest_obstacle_position_local is None
                or self.latest_obstacle_position_local[1] <= 0.0
            ) else -1.0
            selected_action = 1 if obstacle_side > 0.0 else 2
            self.last_action_overridden = True
        if immediate_risk:
            mode = "AVOID"
        elif selected_action in (1, 2):
            mode = "AVOID"
        elif selected_action == 4:
            mode = "WAIT_YIELD"
        elif self.latest_baseline_mode == "AVOID":
            # The baseline hysteresis is a safety floor only while a current
            # collision risk exists. When the prediction is safe, PPO may
            # choose WAIT_YIELD and let a crossing obstacle clear instead of
            # being forced into another avoidance command.
            mode = "AVOID"
        else:
            mode = "CRUISE"
        if previous_mode in ("AVOID", "WAIT_YIELD") and mode == "CRUISE":
            self.planner.avoid_side = None
            self.rejoin_started_at = self.elapsed
        if previous_mode == "WAIT_YIELD" and mode != "WAIT_YIELD":
            self.wait_cooldown_until = self.elapsed + self.config.wait_cooldown
        if mode == "WAIT_YIELD":
            self.planner.avoid_side = None
            self.cruise_reverse_active = False
            self.active_mode = mode
            return np.zeros(3, dtype=float)
        if mode == "AVOID":
            self.cruise_reverse_active = False
            if selected_action in (1, 2) and not self.latest_risk:
                self.planner.avoid_side = 1.0 if selected_action == 1 else -1.0
            command = self.planner.update(
                np.zeros(3),
                self.latest_points,
                self.latest_path_reference.target_local,
                terminal_goal=local_goal(self.data, self.yaw, self.goal_world),
                reference_speed=self.latest_path_reference.reference_speed,
                terminal_risk=self.latest_path_reference.terminal_risk,
                current_time=self.elapsed,
                dynamic_trajectories_local=[
                    (
                        np.asarray(prediction["trajectory_local"], dtype=float),
                        np.asarray(prediction["velocity_local"], dtype=float),
                            float(np.hypot(self.tracker.obstacle_half_length, self.tracker.obstacle_half_width)),
                            np.asarray(
                                prediction.get("trajectory_uncertainty_world", [0.0]),
                                dtype=float,
                            ),
                        )
                    for prediction in self.latest_predictions
                    if prediction["track"].confidence >= 2
                ],
                trajectory_sample_dt=self.config.trajectory_sample_dt,
            )
            path_vy = path_lateral_command(
                self.latest_path_reference,
                gain=self.config.global_path_lateral_gain,
                deadband=self.config.global_path_lateral_deadband,
                max_speed=min(
                    self.config.max_vy,
                    self.config.global_path_max_lateral_speed,
                ),
            )
            if not self.latest_risk and self.rejoin_started_at is not None:
                rejoin_vy, _ = gradual_rejoin_lateral_command(
                    local_goal(self.data, self.yaw, self.goal_world)[1],
                    self.elapsed - self.rejoin_started_at,
                    self.config,
                )
                command[1] = path_vy if abs(path_vy) > 0.0 else rejoin_vy
        else:
            cruise_speed = min(
                self.config.robot_cruise_speed,
                self.latest_path_reference.reference_speed,
            ) * (0.55 if selected_action == 3 else 1.0)
            goal_local_vector = local_goal(self.data, self.yaw, self.goal_world)
            rejoin_age = None if self.rejoin_started_at is None else self.elapsed - self.rejoin_started_at
            rejoin_vy, _ = gradual_rejoin_lateral_command(
                goal_local_vector[1], rejoin_age, self.config
            )
            path_vy = path_lateral_command(
                self.latest_path_reference,
                gain=self.config.global_path_lateral_gain,
                deadband=self.config.global_path_lateral_deadband,
                max_speed=min(
                    self.config.max_vy,
                    self.config.global_path_max_lateral_speed,
                ),
            )
            goal_lateral_vy = path_vy if abs(path_vy) > 0.0 else rejoin_vy
            command, self.cruise_reverse_active = goal_tracking_cruise_command(
                goal_local_vector,
                cruise_speed,
                goal_lateral_vy,
                self.config,
                reverse_active=self.cruise_reverse_active,
            ) if self.config.cruise_mode == "goal_hysteresis" else forward_only_cruise_command(
                cruise_speed,
                rejoin_vy,
                self.config,
            )
            if self.map3_dual_target_controller:
                command[2] = path_heading_command(
                    self.latest_path_reference,
                    gain=1.8,
                    max_rate=self.config.max_yaw_rate,
                )
        self.active_mode = mode
        return command

    def _advance(self, command: np.ndarray) -> None:
        self._require_ready()
        assert self.config is not None
        assert self.sdk is not None
        for _ in range(self.config.policy_period):
            if (
                self.collision
                or self.map_violation
                or self.reached_goal
                or self.elapsed >= self.config.duration
            ):
                break
            self._update_bodies(self.elapsed)
            previous_xy = self.data.qpos[:2].copy()
            actual_command = self.sdk.move(command, self.sim_dt)
            self.yaw = advance(self.model, self.data, self.gait, self.yaw, actual_command, self.sim_dt)
            self.elapsed += self.sim_dt
            self._update_bodies(self.elapsed)
            robot_xy = self.data.qpos[:2].copy()
            if self.map_planner is not None and not self.map_planner.is_traversable_world(
                float(robot_xy[0]), float(robot_xy[1]),
            ):
                self.map_violation = True
            next_clearances = [
                collision_clearance(
                    robot_xy,
                    state[0],
                    obstacle,
                    robot_radius=self.global_robot_radius,
                )
                for state, obstacle in zip(self._current_states(), self.obstacle_specs)
                if state[2]
            ]
            next_proxy_clearances = [
                bounding_circle_clearance(
                    robot_xy,
                    state[0],
                    obstacle,
                    robot_radius=self.global_robot_radius,
                )
                for state, obstacle in zip(self._current_states(), self.obstacle_specs)
                if state[2]
            ]
            if next_clearances:
                current_min = min(next_clearances)
                self.min_clearance = min(self.min_clearance, current_min)
                if current_min < -self.config.collision_tolerance:
                    self.geometric_collision = True
                elif current_min < 0.0:
                    self.near_miss = True
            if next_proxy_clearances:
                current_proxy_min = min(next_proxy_clearances)
                self.min_proxy_clearance = min(
                    self.min_proxy_clearance,
                    current_proxy_min,
                )
                self.proxy_collision = self.proxy_collision or current_proxy_min < 0.0
            contact_now, _, _ = mujoco_obstacle_contacts(
                self.model,
                self.data,
                self.obstacle_geom_ids,
            )
            self.physical_contact = self.physical_contact or contact_now
            self.collision = self.collision or contact_now
            if contact_now:
                self.near_miss = False
            self.proxy_only = bool(
                self.proxy_collision
                and not self.physical_contact
                and not self.geometric_collision
            )
            self.path_length += float(np.linalg.norm(robot_xy - previous_xy))
            if self.active_mode == "AVOID":
                self.avoid_duration += self.sim_dt
            if self.active_mode == "WAIT_YIELD":
                self.wait_duration += self.sim_dt
            else:
                self.wait_duration = 0.0
            self.reached_goal = float(np.linalg.norm(robot_xy - self.goal_world)) <= self.config.goal_tolerance

    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None):
        super().reset(seed=seed)
        if seed is not None:
            self.master_rng = np.random.default_rng(seed)
        episode_seed = int(self.master_rng.integers(0, 2**31 - 1))
        curriculum_index = self.episode_count
        if options and "curriculum_index" in options:
            curriculum_index = int(options["curriculum_index"])
        config = sample_episode_config(
            self.profile,
            self.master_rng,
            episode_seed,
            route_length=self.route_length,
            scenario_mode=self.scenario_mode,
            curriculum_index=curriculum_index,
        )
        self.episode_count += 1
        config = replace(
            config,
            duration=self.duration,
            mppi_samples=self.mppi_samples,
            mppi_horizon=self.mppi_horizon,
            cruise_mode=self.cruise_mode,
            robot_profile=self.robot_profile.name,
            kinematic_proxy=self.kinematic_proxy,
        )
        map_path = None
        map_pedestrian_motion = None
        map_scene_seed = None
        if self.map_planner is not None:
            map_path, map_obstacles, map_pedestrian_motion, map_scene_seed = self._make_map_episode(
                episode_seed, curriculum_index,
            )
            config = replace(
                config,
                goal_x=float(map_path[-1, 0]),
                goal_y=float(map_path[-1, 1]),
                scenario_mode="candidate_map_balanced_dual_target",
                curriculum_case=f"candidate_map_{map_pedestrian_motion}",
                curriculum_index=int(curriculum_index),
                scenario_obstacles=tuple(
                    obstacle.to_dict() for obstacle in map_obstacles
                ),
            )
        self._setup_episode(config, episode_seed, map_path=map_path)
        observation = self._observe()
        return observation, {
            "episode_seed": episode_seed,
            "config": self.config,
            "robot_profile": self.robot_profile.name,
            "kinematic_proxy": self.kinematic_proxy,
            "candidate_map": self.map_manifest,
            "map_scene_scale": self.map_scene_scale if self.candidate_map is not None else None,
            "map_pedestrian_motion": map_pedestrian_motion,
            "map_curriculum_index": int(curriculum_index) if map_pedestrian_motion else None,
            "map_scene_seed": map_scene_seed,
        }

    def step(self, action: int):
        self._require_ready()
        assert self.global_path is not None
        assert self.latest_path_reference is not None
        immediate_danger = bool(
            self.latest_risk
            and self.latest_ttc < self.config.minimum_reaction_time
        )
        danger_before = bool(immediate_danger)
        waitable_before = bool(self.latest_wait_estimate.waitable > 0.5)
        stationary_wait_risk_before = bool(
            self.latest_wait_estimate.stationary_collision_risk > 0.5
        )
        previous_mode = self.active_mode
        before_path_progress = float(self.latest_path_reference.progress)
        before_path_lateral = abs(float(self.latest_path_reference.lateral_error))
        before_distance = float(np.linalg.norm(self.data.qpos[:2] - self.goal_world))
        command = self._command_for_action(int(action))
        self._advance(command)
        after_distance = float(np.linalg.norm(self.data.qpos[:2] - self.goal_world))
        after_path_reference = self.global_path.reference(self.data.qpos[:2], self.yaw)
        path_progress_delta = float(after_path_reference.progress - before_path_progress)
        progress = before_distance - after_distance
        reward = float(
            3.0 * progress
            + 2.0 * path_progress_delta
            - 0.025 * abs(float(after_path_reference.lateral_error))
            - 0.008
        )
        if previous_mode != self.active_mode:
            reward -= 0.025
        if (
            before_path_lateral > 0.20
            and abs(float(after_path_reference.lateral_error)) < before_path_lateral
        ):
            reward += 0.015
        # Keep the learned layer conservative around the existing rule-based
        # safety floor: early AVOID is allowed, but it should cost something
        # when no danger is predicted, otherwise PPO can learn to take long
        # detours simply because collisions are more heavily penalized than
        # unnecessary avoidance.
        if not danger_before and int(action) in (1, 2):
            reward -= 0.035
        elif danger_before and int(action) in (0, 3):
            reward -= 0.050
        elif not danger_before and int(action) == 0:
            reward += 0.004
        # WAIT_YIELD is useful only when a predicted path crossing justifies
        # giving the moving obstacle time to clear. Reward safe waiting while
        # charging unnecessary waiting, so PPO learns when to yield instead
        # of learning to stop everywhere.
        path_blocked_now = float(self.latest_wait_estimate.path_blocked_now) > 0.0
        path_blocked_future = float(self.latest_wait_estimate.path_blocked_future) > 0.0
        wait_allowed = self._wait_is_allowed()
        if int(action) == 4:
            # Waiting is a short courtesy maneuver.  It receives no recurring
            # positive reward: otherwise PPO can exploit WAIT as a cheap way
            # to avoid both collision and timeout.  The small entry bonus is
            # offset by a per-step cost and a no-progress cost.
            if waitable_before and not stationary_wait_risk_before and not immediate_danger:
                if previous_mode != "WAIT_YIELD":
                    # One-time entry reward makes the useful option visible
                    # to PPO, while the step/no-progress costs prevent a
                    # policy from parking indefinitely.
                    reward += self.config.wait_entry_bonus
                reward -= self.config.wait_step_penalty
                if progress < self.config.minimum_progress * 0.25:
                    reward -= self.config.wait_no_progress_penalty
            else:
                reward -= self.config.wait_invalid_penalty
        elif (
            waitable_before
            and not stationary_wait_risk_before
            and not immediate_danger
            and int(action) in (0, 1, 2, 3)
        ):
            # If the tracker predicts a moving blocker will clear shortly,
            # choosing a detour or continuing through the corridor is less
            # efficient than yielding.  This is deliberately a soft reward
            # preference; the hard safety shield still owns imminent risk.
            reward -= (
                self.config.wait_wrong_action_penalty
                if int(action) in (1, 2)
                else self.config.wait_wrong_cruise_penalty
            )
        elif path_blocked_now and not danger_before and int(action) in (0, 3):
            # This is reward shaping from the planner estimate, not a hard
            # action override: PPO can still choose to proceed, but it learns
            # that waiting is preferable when the current corridor is blocked.
            reward -= 0.06
        if (
            previous_mode == "WAIT_YIELD"
            and self.active_mode != "WAIT_YIELD"
            and not self.latest_wait_estimate.waitable > 0.5
            and not self.latest_wait_estimate.stationary_collision_risk > 0.5
        ):
            # Defer the bonus until the robot actually resumes route
            # progress.  Merely leaving WAIT is not sufficient evidence of a
            # successful yield/recovery sequence.
            self.wait_recovery_deadline = self.elapsed + 1.50
        if self.wait_recovery_deadline is not None:
            recovered = bool(
                progress > self.config.minimum_progress * 0.25
                or path_progress_delta > 0.002
            )
            if recovered:
                reward += self.config.wait_clear_bonus
                self.wait_recovery_deadline = None
            elif self.elapsed >= self.wait_recovery_deadline:
                reward -= self.config.wait_no_progress_penalty
                self.wait_recovery_deadline = None
        terminated = False
        truncated = False
        goal_is_valid = bool(
            self.reached_goal
            and not self.map_violation
            and not self.geometric_collision
            and self.min_clearance
            >= self.config.required_clearance - self.config.collision_tolerance
        )
        # Termination order follows the episode definition: physical contact
        # has priority, then reaching the goal, then timeout.  Geometry/proxy
        # diagnostics are deliberately soft and must not mask a later goal or
        # timeout event.  This prevents a harmless proxy warning from turning
        # a route that actually reaches the goal into an artificial deadlock.
        if self.collision:
            reward -= self.collision_penalty
            terminated = True
        elif self.map_violation:
            reward -= self.collision_penalty
            terminated = True
        elif self.reached_goal:
            reward += 5.0 if goal_is_valid else -0.30
            terminated = True
        elif self.elapsed >= self.config.duration - 1e-9:
            reward -= self.timeout_penalty
            truncated = True
        elif self.geometric_collision:
            # Exact rectangle overlap beyond tolerance is a serious safety
            # diagnostic, but it is kept non-terminal so PPO can learn from
            # the transition without confusing it with MuJoCo contact.
            reward -= 0.20
        elif self.proxy_only:
            # The outer-circle proxy is deliberately only a soft warning.
            reward -= 0.02
        if self.elapsed - self.progress_reference_time >= self.config.stagnation_window:
            window_progress = self.progress_reference_distance - after_distance
            if window_progress < self.config.minimum_progress:
                reward -= 0.20
                self.stagnation_windows += 1
            else:
                self.stagnation_windows = 0
            self.progress_reference_time = self.elapsed
            self.progress_reference_distance = after_distance
        actual_speed = float(np.linalg.norm(self.sdk.velocity[:2]))
        global_progress = self.initial_goal_distance - after_distance
        goal_buffer = max(
            self.config.deadlock_goal_buffer,
            1.5 * self.config.goal_tolerance,
        )
        # ``deadlock`` is deliberately stricter than a single stagnation
        # window.  A robot can temporarily make little goal progress while
        # moving around an obstacle; that is a slow timeout, not a deadlock.
        self.deadlock_latched = bool(
            not self.collision
            and not self.reached_goal
            and self.stagnation_windows >= self.config.deadlock_stagnation_windows
            and global_progress < self.config.deadlock_global_progress_threshold
            and actual_speed < self.config.deadlock_speed_threshold
            and after_distance > goal_buffer
        )
        observation = self._observe()
        info = {
            "mode": self.active_mode,
            "action_name": ACTION_NAMES[int(action)],
            "action_overridden": bool(self.last_action_overridden),
            "wait_allowed": bool(wait_allowed),
            "wait_duration": float(self.wait_duration),
            "wait_recovery_pending": bool(
                self.wait_recovery_deadline is not None
            ),
            "risk": bool(self.latest_risk),
            "collision": bool(self.collision),
            "map_violation": bool(self.map_violation),
            "geometric_collision": bool(self.geometric_collision),
            "proxy_collision": bool(self.proxy_collision),
            "proxy_only": bool(self.proxy_only),
            "physical_contact": bool(self.physical_contact),
            "near_miss": bool(self.near_miss),
            "reached_goal": bool(self.reached_goal),
            "timeout": bool(truncated),
            "deadlock": bool(self.deadlock_latched),
            "min_clearance": float(self.min_clearance),
            "min_proxy_clearance": float(self.min_proxy_clearance),
            "elapsed_time": float(self.elapsed),
            "path_length": float(self.path_length),
            "distance_to_goal": float(after_distance),
            "path_lateral_error": float(self.latest_path_reference.lateral_error),
            "path_heading_error": float(self.latest_path_reference.heading_error),
            "path_remaining_distance": float(self.latest_path_reference.remaining_distance),
            "path_progress": float(self.latest_path_reference.progress),
            "path_blocked_now": float(self.latest_wait_estimate.path_blocked_now),
            "path_blocked_future": float(self.latest_wait_estimate.path_blocked_future),
            "time_to_path_block": float(self.latest_wait_estimate.time_to_path_block),
            "time_to_path_clear": float(self.latest_wait_estimate.time_to_path_clear),
            "blocking_duration": float(self.latest_wait_estimate.blocking_duration),
            "wait_cost": float(self.latest_wait_estimate.wait_cost),
            "detour_cost": float(self.latest_wait_estimate.detour_cost),
            "episode_seed": int(self.episode_seed),
            "robot_profile": self.robot_profile.name,
            "kinematic_proxy": bool(self.kinematic_proxy),
            "robot_planner_radius": float(self.global_robot_radius),
        }
        if terminated or truncated:
            info["success"] = goal_is_valid and not self.collision and not self.map_violation
        return observation, reward, terminated, truncated, info

    def close(self) -> None:
        self.model = None
        self.data = None
