#!/usr/bin/env python3
"""Decision-layer demo: predict a moving-box collision before calling MPPI.

The stack in this file is deliberately separated into three layers:

1. ``SimulatedLidar`` provides a 2D scan from MuJoCo rangefinders.
2. ``MultiTargetTracker`` estimates several obstacle motions and their
   time-to-closest-approach values.
3. ``DecisionLayer`` and PPO choose CRUISE, side-specific AVOID, or
   WAIT_YIELD. AVOID calls the local MPPI planner; CRUISE uses goal-x
   hysteresis and a continuous nominal velocity curve, while WAIT_YIELD sends
   a zero velocity target through the simulated SDK until the next decision.

The ``SimulatedD1SDK`` now sits between MPPI and the kinematic base, applying
command saturation, delay, and acceleration limits. The robot base and gait
are still visualized kinematically. MPPI outputs a planar velocity command;
this does not claim torque-level quadruped control,
whole-body MPC, foothold planning, step-over control, traversability
estimation, or real-robot dynamic-stability validation. It tests the
dynamic-obstacle tracking, collision-prediction, decision-switching, and
local-planning interface before connecting the stack to a stabilized
controller.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field, fields, replace
import json
from pathlib import Path
import time
from typing import Any

import mujoco
import mujoco.viewer
import numpy as np

from run_d1_edu_obstacle_avoidance_viewer import (
    NOMINAL,
    Obstacle,
    build_scene,
)
from run_d1_edu_mppi_obstacle_viewer import (
    ObstacleMPPI,
    SimulatedLidar,
    advance,
    local_goal,
)
from run_d1_edu_mppi_velocity_viewer import GaitLayer
from d1_edu_sim_sdk import SimulatedD1SDK
from d1_edu_scenario import ScenarioObstacle
from d1_edu_nav2_global_path import (
    GlobalPathReference,
    Nav2GlobalPath,
    WaitYieldEstimate,
    estimate_wait_yield,
    gate_wait_yield,
    path_lateral_command,
    terminal_path_risk,
)
from d1_edu_decision_metrics import summarize_decision_trace
from d1_edu_footprint import calibrate_robot_footprint, footprint_summary
from d1_edu_tracker_models import IMMObstaclePredictor


def rotation_world_from_local(yaw: float) -> np.ndarray:
    """Return the 2D rotation that maps robot-local vectors to world."""
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s], [s, c]], dtype=float)


def local_to_world_velocity(velocity_local: np.ndarray, yaw: float) -> np.ndarray:
    return rotation_world_from_local(yaw) @ np.asarray(velocity_local, dtype=float)


def world_to_local_position(
    position_world: np.ndarray,
    robot_xy_world: np.ndarray,
    yaw: float,
) -> np.ndarray:
    return rotation_world_from_local(yaw).T @ (
        np.asarray(position_world, dtype=float) - np.asarray(robot_xy_world, dtype=float)
    )


def world_to_local_velocity(velocity_world: np.ndarray, yaw: float) -> np.ndarray:
    return rotation_world_from_local(yaw).T @ np.asarray(velocity_world, dtype=float)


@dataclass
class EpisodeConfig:
    """All parameters needed to replay one dynamic-obstacle episode."""

    box_x: float = 1.25
    box_y: float = -1.30
    box_vx: float = 0.0
    box_vy: float = 0.38
    box_size_x: float = 0.25
    box_size_y: float = 0.25
    robot_cruise_speed: float = 0.22
    duration: float = 40.0
    # Retained so older replay JSON files remain loadable.  The active
    # benchmark never extends an episode after timeout.
    timeout_extension: float = 10.0
    stagnation_window: float = 1.50
    minimum_progress: float = 0.06
    deadlock_stagnation_windows: int = 3
    deadlock_speed_threshold: float = 0.04
    deadlock_global_progress_threshold: float = 0.15
    deadlock_goal_buffer: float = 0.30
    recovery_duration: float = 1.20
    risk_confirmation_updates: int = 2
    rejoin_gain: float = 0.35
    rejoin_deadband: float = 0.10
    rejoin_max_lateral_speed: float = 0.10
    rejoin_delay: float = 0.35
    rejoin_ramp_time: float = 1.50
    global_path_lookahead: float = 0.80
    global_path_lateral_gain: float = 0.60
    global_path_lateral_deadband: float = 0.05
    global_path_max_lateral_speed: float = 0.22
    global_path_grid_resolution: float = 0.10
    global_path_search_padding: float = 1.50
    global_path_obstacle_margin: float = 0.12
    global_path_robot_radius: float = 0.25
    global_path_soft_cost_radius: float = 0.12
    global_path_cost_weight: float = 1.0
    global_path_replan_period: float = 2.0
    cruise_goal_slowdown_radius: float = 0.75
    cruise_min_approach_speed: float = 0.06
    cruise_reverse_activation_x: float = -0.08
    cruise_reverse_release_x: float = 0.08
    cruise_max_reverse_speed: float = 0.12
    cruise_reverse_gain: float = 0.80
    # WAIT_YIELD is a short, moving-obstacle courtesy action, not a generic
    # stop command.  The learned policy is penalized for holding it too long.
    wait_max_duration: float = 1.80
    wait_max_clear_time: float = 2.00
    wait_min_track_confidence: int = 4
    wait_min_lateral_speed: float = 0.12
    wait_max_longitudinal_speed: float = 0.35
    wait_max_clear_fraction: float = 0.85
    wait_cooldown: float = 0.80
    wait_min_obstacle_speed: float = 0.05
    # The reward is intentionally duration-sensitive.  A one-step courtesy
    # wait can still be useful, but holding WAIT must lose to making progress
    # once the crossing has cleared.
    wait_step_penalty: float = 0.050
    wait_no_progress_penalty: float = 0.080
    wait_invalid_penalty: float = 0.20
    wait_entry_bonus: float = 0.12
    wait_clear_bonus: float = 0.45
    wait_wrong_action_penalty: float = 0.25
    wait_wrong_cruise_penalty: float = 0.18
    timeout_penalty: float = 5.0
    # ``forward_only`` reproduces the pre-fix CRUISE behavior.  The default
    # ``goal_hysteresis`` is the revised controller with goal-x slowdown,
    # neutral hysteresis, reverse correction, and SDK acceleration limiting.
    cruise_mode: str = "goal_hysteresis"
    # Small geometric tolerance for numerical/mesh-model uncertainty.  A
    # clearance of -0.001 m is reported as near-miss, not collision.
    collision_tolerance: float = 0.01
    required_clearance: float = 0.0
    clearance_uncertainty: float = 0.0
    obstacle_position_uncertainty: float = 0.0
    obstacle_velocity_uncertainty: float = 0.0
    lidar_noise_std: float = 0.0
    lidar_dropout_rate: float = 0.0
    command_delay: float = 0.08
    max_vx: float = 0.36
    max_vy: float = 0.25
    max_yaw_rate: float = 0.60
    max_ax: float = 0.60
    max_ay: float = 0.50
    max_yaw_acceleration: float = 1.50

    # Evaluation and reproducibility knobs.  They keep the required public
    # fields above small while allowing a viewer to replay a benchmark case.
    goal_x: float = 3.20
    goal_y: float = 0.0
    goal_tolerance: float = 0.30
    lidar_max_range: float = 4.0
    truth_horizon: float = 4.0
    trajectory_prediction_horizon: float = 3.0
    trajectory_sample_dt: float = 0.25
    minimum_reaction_time: float = 1.20
    policy_period: int = 12
    mppi_samples: int = 256
    mppi_horizon: int = 60
    mppi_seed: int | None = None
    tracking_failure_updates: int = 8
    route_length: float = 3.20
    max_obstacle_speed: float = 7.0
    scenario_mode: str = "legacy"
    curriculum_case: str = ""
    curriculum_index: int = -1
    scenario_obstacles: tuple[dict[str, Any], ...] = ()
    lidar_model: str = "legacy_61_ray"
    lidar_ray_count: int = 61
    lidar_fov: float = np.pi
    lidar_scan_rate_hz: float = 10.0


@dataclass(frozen=True)
class EpisodeResult:
    """Quantitative result for one episode.

    TP/TN/FP/FN are counts over policy updates, not merely episode booleans;
    this makes aggregate benchmark recall and false-alarm rates meaningful.
    """

    success: bool
    collision: bool
    reached_goal: bool
    timeout: bool
    min_clearance: float
    elapsed_time: float
    path_length: float
    decision_triggered: bool
    avoid_duration: float
    true_positive: int
    true_negative: int
    false_positive: int
    false_negative: int
    seed: int
    failure_reason: str
    tracking_failure: bool = False
    final_pose: tuple[float, float] = (0.0, 0.0)
    final_actual_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    mode_history: tuple[str, ...] = ()
    distance_to_goal: float = 0.0
    recovery_triggered: bool = False
    recovery_active_duration: float = 0.0
    trace: tuple[dict[str, Any], ...] = ()
    proxy_collision: bool = False
    min_proxy_clearance: float = float("inf")
    physical_contact: bool = False
    collision_obstacle_index: int | None = None
    contact_geom_name: str | None = None
    contact_distance: float = float("inf")
    contact_position: tuple[float, float, float] | None = None
    near_miss: bool = False
    geometric_collision: bool = False
    proxy_only: bool = False
    decision_metrics: dict[str, Any] = field(default_factory=dict)
    footprint: dict[str, Any] = field(default_factory=dict)


@dataclass
class DynamicBox:
    """Kinematic trajectory for the free-joint box in the test scene."""

    model: mujoco.MjModel
    x: float = 1.25
    y0: float = -1.30
    vx: float = 0.0
    vy: float = 0.38
    z: float = 0.25

    def __post_init__(self) -> None:
        joint_id = self.model.joint("moving_obstacle_freejoint").id
        self.qposadr = int(self.model.jnt_qposadr[joint_id])

    def update(self, data: mujoco.MjData, elapsed: float) -> float:
        x = self.x + self.vx * elapsed
        y = self.y0 + self.vy * elapsed
        data.qpos[self.qposadr:self.qposadr + 3] = [x, y, self.z]
        data.qpos[self.qposadr + 3:self.qposadr + 7] = [1.0, 0.0, 0.0, 0.0]
        return y


@dataclass
class DynamicObstacleBody:
    """Bind one generated analytic trajectory to a MuJoCo free joint."""

    model: mujoco.MjModel
    trajectory: ScenarioObstacle
    index: int = 0

    def __post_init__(self) -> None:
        joint_name = (
            "moving_obstacle_freejoint"
            if self.index == 0
            else f"moving_obstacle_{self.index:02d}_freejoint"
        )
        joint_id = self.model.joint(joint_name).id
        self.qposadr = int(self.model.jnt_qposadr[joint_id])

    def update(self, data: mujoco.MjData, elapsed: float) -> tuple[np.ndarray, np.ndarray, bool]:
        position, velocity, active = self.trajectory.state(elapsed)
        if active:
            z = float(self.trajectory.height)
            data.qpos[self.qposadr:self.qposadr + 3] = [position[0], position[1], z]
        else:
            # Keep an inactive sudden-appearance object outside the LiDAR and
            # route workspace while preserving its free-joint state.
            data.qpos[self.qposadr:self.qposadr + 3] = [100.0, 100.0, -10.0]
        data.qpos[self.qposadr + 3:self.qposadr + 7] = [1.0, 0.0, 0.0, 0.0]
        return position, velocity, active


class LidarTracker:
    """Track one forward LiDAR cluster with constant-velocity prediction.

    Positions and velocities exposed by this class are always in world XY.
    ``ray_indices`` keeps the original scan order when available; without it,
    a small NumPy Euclidean clustering fallback is used.
    """

    def __init__(
        self,
        obstacle_half_length: float = 0.25,
        obstacle_half_width: float = 0.25,
        robot_safety_radius: float = 0.25,
        extra_safety_margin: float = 0.10,
        cluster_jump: float = 0.20,
        min_cluster_points: int = 3,
        max_track_age: float = 0.45,
        velocity_alpha: float = 0.30,
    ) -> None:
        self.obstacle_half_length = obstacle_half_length
        self.obstacle_half_width = obstacle_half_width
        self.robot_safety_radius = robot_safety_radius
        self.extra_safety_margin = extra_safety_margin
        self.cluster_jump = cluster_jump
        self.min_cluster_points = min_cluster_points
        self.max_track_age = max_track_age
        self.velocity_alpha = velocity_alpha

        self.velocity = np.zeros(2, dtype=float)
        self.confidence = 0
        self.observation: np.ndarray | None = None
        self.observation_timestamp: float | None = None
        self.last_measurement_world: np.ndarray | None = None
        self.last_measurement_time: float | None = None
        self.miss_count = 0

    @property
    def effective_safe_distance(self) -> float:
        """Bounding-circle safety distance used by collision prediction."""
        obstacle_radius = float(np.hypot(
            self.obstacle_half_length,
            self.obstacle_half_width,
        ))
        return self.robot_safety_radius + obstacle_radius + self.extra_safety_margin

    def _ordered_clusters(
        self,
        points_local: np.ndarray,
        ray_indices: np.ndarray,
    ) -> list[np.ndarray]:
        order = np.argsort(ray_indices)
        points = points_local[order]
        rays = ray_indices[order]
        if len(points) == 0:
            return []

        clusters: list[np.ndarray] = []
        start = 0
        for index in range(1, len(points)):
            ray_gap = int(rays[index] - rays[index - 1])
            point_jump = float(np.linalg.norm(points[index] - points[index - 1]))
            if ray_gap > 1 or point_jump > self.cluster_jump:
                clusters.append(points[start:index])
                start = index
        clusters.append(points[start:])
        return clusters

    def _euclidean_clusters(self, points_local: np.ndarray) -> list[np.ndarray]:
        if len(points_local) == 0:
            return []
        remaining = set(range(len(points_local)))
        clusters: list[np.ndarray] = []
        while remaining:
            seed = remaining.pop()
            group = [seed]
            queue = [seed]
            while queue:
                current = queue.pop()
                neighbors = [
                    index for index in remaining
                    if np.linalg.norm(points_local[index] - points_local[current])
                    <= self.cluster_jump
                ]
                for index in neighbors:
                    remaining.remove(index)
                    queue.append(index)
                    group.append(index)
            clusters.append(points_local[group])
        return clusters

    def _candidate_clusters(
        self,
        points_local: np.ndarray,
        ray_indices: np.ndarray | None,
    ) -> list[np.ndarray]:
        if points_local.size == 0:
            return []
        points_local = np.asarray(points_local, dtype=float).reshape(-1, 2)
        forward_mask = (
            (points_local[:, 0] > 0.35)
            & (points_local[:, 0] < 2.60)
            & (np.abs(points_local[:, 1]) < 1.80)
        )
        forward = points_local[forward_mask]
        if ray_indices is not None:
            ray_indices = np.asarray(ray_indices)[forward_mask]
            clusters = self._ordered_clusters(forward, ray_indices)
        else:
            clusters = self._euclidean_clusters(forward)
        return [cluster for cluster in clusters if len(cluster) >= self.min_cluster_points]

    def _select_cluster(
        self,
        clusters: list[np.ndarray],
        robot_xy: np.ndarray,
        yaw: float,
        elapsed: float,
    ) -> np.ndarray | None:
        if not clusters:
            return None
        rotation = rotation_world_from_local(yaw)
        candidates: list[tuple[float, float, np.ndarray]] = []
        for cluster in clusters:
            observed_local = np.median(cluster, axis=0)
            observed_world = robot_xy + rotation @ observed_local
            range_to_cluster = float(np.linalg.norm(observed_local))
            if self.last_measurement_world is None or self.last_measurement_time is None:
                track_distance = 0.0
            else:
                dt = max(0.0, elapsed - self.last_measurement_time)
                expected_world = self.last_measurement_world + self.velocity * dt
                track_distance = float(np.linalg.norm(observed_world - expected_world))
            # With a prior track, continuity dominates; range breaks ties.
            score = track_distance + 0.15 * range_to_cluster
            candidates.append((score, range_to_cluster, observed_world))
        candidates.sort(key=lambda candidate: (candidate[0], candidate[1]))
        return candidates[0][2]

    def _handle_miss(self, elapsed: float) -> np.ndarray | None:
        self.miss_count += 1
        self.confidence = max(0, self.confidence - 1)
        if self.last_measurement_world is None or self.last_measurement_time is None:
            self.observation = None
            self.observation_timestamp = None
            return None

        age = elapsed - self.last_measurement_time
        if 0.0 <= age <= self.max_track_age:
            # Short LiDAR dropout: retain a clearly labelled prediction.
            self.observation = self.last_measurement_world + self.velocity * age
            self.observation_timestamp = elapsed
        else:
            self.observation = None
            self.observation_timestamp = None
        return self.observation

    def update(
        self,
        points_local: np.ndarray,
        robot_xy: np.ndarray,
        yaw: float,
        elapsed: float,
        ray_indices: np.ndarray | None = None,
    ) -> np.ndarray | None:
        clusters = self._candidate_clusters(points_local, ray_indices)
        observed_world = self._select_cluster(clusters, robot_xy, yaw, elapsed)
        if observed_world is None:
            return self._handle_miss(elapsed)

        if self.last_measurement_world is not None and self.last_measurement_time is not None:
            dt = elapsed - self.last_measurement_time
            if dt > 1e-6:
                measured = (observed_world - self.last_measurement_world) / dt
                if np.linalg.norm(measured) < 2.0:
                    self.velocity = (
                        (1.0 - self.velocity_alpha) * self.velocity
                        + self.velocity_alpha * measured
                    )

        self.observation = observed_world
        self.observation_timestamp = elapsed
        self.last_measurement_world = observed_world.copy()
        self.last_measurement_time = elapsed
        self.miss_count = 0
        self.confidence = min(8, self.confidence + 1)
        return self.observation

    def collision_prediction(
        self,
        robot_xy: np.ndarray,
        robot_velocity_world: np.ndarray,
        horizon: float = 4.0,
        safe_distance: float | None = None,
    ) -> tuple[bool, float, float]:
        """Return risk, time-to-closest-approach and closest distance.

        ``time_to_closest`` is not an exact collision time. It is the time at
        which two objects under a constant-velocity assumption are closest.
        A collision risk is reported only when that closest distance is below
        the effective bounding-circle safety distance.
        """
        if self.observation is None or self.confidence < 2:
            return False, float("inf"), float("inf")

        relative_position = self.observation - np.asarray(robot_xy, dtype=float)
        relative_velocity = self.velocity - np.asarray(robot_velocity_world, dtype=float)
        velocity_squared = float(relative_velocity @ relative_velocity)
        if velocity_squared < 1e-8:
            time_to_closest = 0.0
        else:
            time_to_closest = float(np.clip(
                -(relative_position @ relative_velocity) / velocity_squared,
                0.0,
                horizon,
            ))
        closest_distance = float(
            np.linalg.norm(relative_position + relative_velocity * time_to_closest)
        )
        threshold = self.effective_safe_distance if safe_distance is None else safe_distance
        return closest_distance < threshold, time_to_closest, closest_distance


@dataclass
class TrackedObstacle:
    """One world-frame obstacle track maintained by MultiTargetTracker."""

    track_id: int
    position_world: np.ndarray
    velocity_world: np.ndarray
    confidence: int
    last_update: float
    miss_count: int = 0
    acceleration_world: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=float))
    history: list[tuple[float, np.ndarray]] = field(default_factory=list)


class MultiTargetTracker:
    """Greedy multi-cluster tracker for a small number of LiDAR targets.

    This is deliberately lightweight: clusters are formed from ordered LiDAR
    hits, then associated to constant-velocity tracks.  It is sufficient for
    the high-level decision layer and keeps track IDs stable through short
    dropouts without pretending to be a full SLAM data-association system.
    """

    def __init__(
        self,
        obstacle_half_length: float = 0.25,
        obstacle_half_width: float = 0.25,
        robot_safety_radius: float = 0.25,
        extra_safety_margin: float = 0.10,
        prediction_position_uncertainty: float = 0.0,
        prediction_velocity_uncertainty: float = 0.0,
        association_distance: float = 0.65,
        max_track_age: float = 0.45,
        velocity_alpha: float = 0.30,
        max_tracks: int = 8,
    ) -> None:
        self.obstacle_half_length = obstacle_half_length
        self.obstacle_half_width = obstacle_half_width
        self.robot_safety_radius = robot_safety_radius
        self.extra_safety_margin = extra_safety_margin
        self.prediction_position_uncertainty = max(0.0, float(prediction_position_uncertainty))
        self.prediction_velocity_uncertainty = max(0.0, float(prediction_velocity_uncertainty))
        self.association_distance = association_distance
        self.max_track_age = max_track_age
        self.velocity_alpha = velocity_alpha
        self.max_tracks = max_tracks
        self.tracks: dict[int, TrackedObstacle] = {}
        self.next_track_id = 0
        self.selected_track: TrackedObstacle | None = None
        self._clusterer = LidarTracker(
            obstacle_half_length=obstacle_half_length,
            obstacle_half_width=obstacle_half_width,
            robot_safety_radius=robot_safety_radius,
            extra_safety_margin=extra_safety_margin,
            max_track_age=max_track_age,
            velocity_alpha=velocity_alpha,
        )
        self._imm_predictor = IMMObstaclePredictor()

    @property
    def effective_safe_distance(self) -> float:
        obstacle_radius = float(np.hypot(
            self.obstacle_half_length,
            self.obstacle_half_width,
        ))
        return (
            self.robot_safety_radius
            + obstacle_radius
            + self.extra_safety_margin
            + self.prediction_position_uncertainty
        )

    @property
    def observation(self) -> np.ndarray | None:
        return None if self.selected_track is None else self.selected_track.position_world

    @property
    def velocity(self) -> np.ndarray:
        if self.selected_track is None:
            return np.zeros(2, dtype=float)
        return self.selected_track.velocity_world

    @property
    def confidence(self) -> int:
        return 0 if self.selected_track is None else self.selected_track.confidence

    @property
    def track_count(self) -> int:
        return len(self.tracks)

    def _detections_world(
        self,
        points_local: np.ndarray,
        robot_xy: np.ndarray,
        yaw: float,
        ray_indices: np.ndarray | None,
    ) -> list[np.ndarray]:
        clusters = self._clusterer._candidate_clusters(points_local, ray_indices)
        rotation = rotation_world_from_local(yaw)
        return [robot_xy + rotation @ np.median(cluster, axis=0) for cluster in clusters]

    def update(
        self,
        points_local: np.ndarray,
        robot_xy: np.ndarray,
        yaw: float,
        elapsed: float,
        ray_indices: np.ndarray | None = None,
    ) -> list[TrackedObstacle]:
        detections = self._detections_world(points_local, robot_xy, yaw, ray_indices)
        unmatched_tracks = set(self.tracks)
        matched_detections: set[int] = set()

        # Greedy nearest-neighbour association is deterministic and adequate
        # for the small number of separated boxes used in this benchmark.
        candidates: list[tuple[float, int, int]] = []
        for detection_index, detection in enumerate(detections):
            for track_id, track in self.tracks.items():
                dt = max(0.0, elapsed - track.last_update)
                predicted = track.position_world + track.velocity_world * dt
                distance = float(np.linalg.norm(detection - predicted))
                candidates.append((distance, detection_index, track_id))
        for distance, detection_index, track_id in sorted(candidates):
            if (
                distance > self.association_distance
                or detection_index in matched_detections
                or track_id not in unmatched_tracks
            ):
                continue
            track = self.tracks[track_id]
            dt = elapsed - track.last_update
            if dt > 1e-6:
                measured_velocity = (detections[detection_index] - track.position_world) / dt
                if np.linalg.norm(measured_velocity) < 2.0:
                    previous_velocity = track.velocity_world.copy()
                    track.velocity_world = (
                        (1.0 - self.velocity_alpha) * track.velocity_world
                        + self.velocity_alpha * measured_velocity
                    )
                    measured_acceleration = (track.velocity_world - previous_velocity) / dt
                    track.acceleration_world = np.clip(
                        0.70 * track.acceleration_world
                        + 0.30 * measured_acceleration,
                        -3.0,
                        3.0,
                    )
            track.position_world = detections[detection_index].copy()
            track.history.append((float(elapsed), track.position_world.copy()))
            track.history = track.history[-12:]
            track.last_update = elapsed
            track.miss_count = 0
            track.confidence = min(8, track.confidence + 1)
            matched_detections.add(detection_index)
            unmatched_tracks.remove(track_id)

        for detection_index, detection in enumerate(detections):
            if detection_index in matched_detections or len(self.tracks) >= self.max_tracks:
                continue
            track_id = self.next_track_id
            self.next_track_id += 1
            self.tracks[track_id] = TrackedObstacle(
                track_id=track_id,
                position_world=detection.copy(),
                velocity_world=np.zeros(2, dtype=float),
                confidence=1,
                last_update=elapsed,
                history=[(float(elapsed), detection.copy())],
            )

        for track_id in list(unmatched_tracks):
            track = self.tracks[track_id]
            age = elapsed - track.last_update
            track.position_world = track.position_world + track.velocity_world * max(age, 0.0)
            track.miss_count += 1
            track.confidence = max(0, track.confidence - 1)
            if age > self.max_track_age:
                del self.tracks[track_id]

        return list(self.tracks.values())

    def predict_trajectories(
        self,
        robot_xy: np.ndarray,
        robot_velocity_world: np.ndarray,
        yaw: float,
        horizon: float = 3.0,
        sample_dt: float = 0.25,
    ) -> list[dict[str, Any]]:
        """Predict every track with a bounded constant-acceleration model.

        The returned trajectory is expressed in a fixed frame anchored at the
        current robot pose.  This is the same frame used by the local MPPI
        rollout, so the prediction can be consumed without silently mixing
        world and body coordinates.
        """
        if horizon <= 0.0 or sample_dt <= 0.0:
            raise ValueError("prediction horizon and sample_dt must be positive")
        sample_count = max(1, int(np.ceil(horizon / sample_dt)))
        times = np.minimum(
            np.arange(sample_count + 1, dtype=float) * sample_dt,
            horizon,
        )
        # ``np.minimum`` can repeat the endpoint when horizon is not a
        # multiple of sample_dt.  Keep the trajectory strictly increasing.
        times = np.unique(np.concatenate((times, np.asarray([horizon]))))
        robot_xy = np.asarray(robot_xy, dtype=float)
        robot_velocity_world = np.asarray(robot_velocity_world, dtype=float)
        obstacle_radius = float(np.hypot(
            self.obstacle_half_length,
            self.obstacle_half_width,
        ))
        physical_distance = self.robot_safety_radius + obstacle_radius
        risk_distance = (
            physical_distance
            + self.extra_safety_margin
            + self.prediction_position_uncertainty
            + self.prediction_velocity_uncertainty * horizon
        )
        heading_world = np.array([np.cos(yaw), np.sin(yaw)], dtype=float)
        predictions: list[dict[str, Any]] = []

        for track in self.tracks.values():
            # Do not extrapolate an acceleration estimate from only one
            # displacement update.  At low confidence the constant-velocity
            # model is safer; the acceleration model becomes active once the
            # track has accumulated enough observations.
            acceleration_world = (
                track.acceleration_world
                if track.confidence >= 4
                else np.zeros(2, dtype=float)
            )
            imm_prediction = self._imm_predictor.predict(
                track.position_world,
                track.velocity_world,
                acceleration_world,
                times,
                history=track.history,
                position_uncertainty=self.prediction_position_uncertainty,
                velocity_uncertainty=self.prediction_velocity_uncertainty,
            )
            obstacle_positions_world = imm_prediction.trajectory
            robot_positions_world = (
                robot_xy[None, :]
                + times[:, None] * robot_velocity_world[None, :]
            )
            relative_positions_world = obstacle_positions_world - robot_positions_world
            center_distances = np.linalg.norm(relative_positions_world, axis=1)
            # Keep the decision layer on the fused mean plus its established
            # risk margin.  The uncertainty tube is intentionally consumed
            # by MPPI only; subtracting it here as well double-counts risk
            # and turns every moving target into an apparent blocker.
            clearances = center_distances - physical_distance
            closest_index = int(np.argmin(center_distances))
            closest_distance = float(center_distances[closest_index])
            time_to_closest = float(times[closest_index])
            risk_margin = (
                self.extra_safety_margin
                + self.prediction_position_uncertainty
                + self.prediction_velocity_uncertainty * times
            )
            risk_mask = clearances <= risk_margin
            risk = bool(np.any(risk_mask))
            ttc = float(times[np.flatnonzero(risk_mask)[0]]) if risk else float("inf")
            min_clearance = float(np.min(clearances))
            closest_relative = relative_positions_world[closest_index]
            closest_norm = float(np.linalg.norm(closest_relative))
            if closest_norm < 1e-8:
                intersection_angle_deg = 0.0
            else:
                cosine = float(np.clip(
                    (heading_world @ closest_relative) / closest_norm,
                    -1.0,
                    1.0,
                ))
                # 0 degrees means the predicted closest point is straight
                # ahead; 90 degrees is a lateral crossing; 180 is behind.
                intersection_angle_deg = float(np.degrees(np.arccos(cosine)))
            ttc_score = (
                0.0
                if not np.isfinite(ttc)
                else float(1.0 - np.clip(ttc / horizon, 0.0, 1.0))
            )
            clearance_score = float(np.clip(
                (risk_distance - min_clearance) / max(risk_distance, 1e-6),
                0.0,
                1.0,
            ))
            angle_score = float(np.clip(
                np.cos(np.radians(intersection_angle_deg)),
                0.0,
                1.0,
            ))
            risk_score = float(
                0.50 * ttc_score
                + 0.35 * clearance_score
                + 0.15 * angle_score
            )
            trajectory_local = np.asarray([
                world_to_local_position(position, robot_xy, yaw)
                for position in obstacle_positions_world
            ])
            predictions.append({
                "track": track,
                "position_world": track.position_world.copy(),
                "position_local": world_to_local_position(track.position_world, robot_xy, yaw),
                "velocity_local": world_to_local_velocity(track.velocity_world, yaw),
                "velocity_world": track.velocity_world.copy(),
                "acceleration_local": world_to_local_velocity(acceleration_world, yaw),
                "acceleration_world": acceleration_world.copy(),
                "trajectory_world": obstacle_positions_world,
                "trajectory_uncertainty_world": imm_prediction.uncertainty_radius.copy(),
                "uncertainty_radius": float(np.max(imm_prediction.uncertainty_radius)),
                "prediction_model": "IMM(CV+CA+CTRV)",
                "model_weights": dict(imm_prediction.model_weights),
                "turn_rate": float(imm_prediction.turn_rate),
                "trajectory_local": trajectory_local,
                "prediction_times": times.copy(),
                "prediction_horizon": float(horizon),
                "prediction_sample_dt": float(sample_dt),
                "risk": risk,
                "ttc": ttc,
                "time_to_closest": time_to_closest,
                "closest_distance": closest_distance,
                "min_clearance": min_clearance,
                "intersection_angle_deg": intersection_angle_deg,
                "risk_score": risk_score,
            })
        return predictions

    def rank_risks(
        self,
        predictions: list[dict[str, Any]],
    ) -> list[dict[str, Any]]:
        """Rank targets by danger, then TTC, clearance, and crossing angle."""
        ranked = sorted(predictions, key=lambda item: (
            not bool(item["risk"]),
            float(item["ttc"]) if np.isfinite(item["ttc"]) else float("inf"),
            float(item["min_clearance"]),
            float(item["intersection_angle_deg"]),
            -float(item["risk_score"]),
        ))
        for rank, prediction in enumerate(ranked, start=1):
            prediction["risk_rank"] = rank
        return ranked

    def predictions(
        self,
        robot_xy: np.ndarray,
        robot_velocity_world: np.ndarray,
        yaw: float,
        horizon: float = 3.0,
        sample_dt: float = 0.25,
    ) -> list[dict[str, Any]]:
        """Return ranked 2–3 second predictions for all active tracks."""
        return self.rank_risks(self.predict_trajectories(
            robot_xy,
            robot_velocity_world,
            yaw,
            horizon=horizon,
            sample_dt=sample_dt,
        ))

    def collision_prediction(
        self,
        robot_xy: np.ndarray,
        robot_velocity_world: np.ndarray,
        yaw: float = 0.0,
        horizon: float = 3.0,
        safe_distance: float | None = None,
    ) -> tuple[bool, float, float]:
        predictions = self.predictions(robot_xy, robot_velocity_world, yaw, horizon)
        if safe_distance is not None:
            for prediction in predictions:
                prediction["risk"] = prediction["closest_distance"] < safe_distance
        usable_predictions = [
            prediction for prediction in predictions
            if prediction["track"].confidence >= 2
        ]
        if not usable_predictions:
            self.selected_track = None
            return False, float("inf"), float("inf")
        selected = usable_predictions[0]
        self.selected_track = selected["track"]
        if self.selected_track.confidence < 2:
            return False, float("inf"), float("inf")
        return (
            bool(selected["risk"]),
            float(selected["time_to_closest"]),
            float(selected["closest_distance"]),
        )


class DecisionLayer:
    """Hysteretic CRUISE/AVOID switch based on predicted geometric risk."""

    def __init__(
        self,
        enter_safe_distance: float = 0.70,
        exit_safe_distance: float = 0.95,
        safe_updates_required: int = 4,
        min_confidence: int = 2,
        risk_updates_required: int = 2,
    ) -> None:
        if exit_safe_distance <= enter_safe_distance:
            raise ValueError("exit_safe_distance must be larger than enter_safe_distance")
        if risk_updates_required <= 0:
            raise ValueError("risk_updates_required must be positive")
        self.enter_safe_distance = enter_safe_distance
        self.exit_safe_distance = exit_safe_distance
        self.safe_updates_required = safe_updates_required
        self.min_confidence = min_confidence
        self.risk_updates_required = risk_updates_required
        self.mode = "CRUISE"
        self.safe_streak = 0
        self.risk_streak = 0

    def update(
        self,
        risk: bool,
        time_to_closest: float,
        closest_distance: float,
        confidence: int,
    ) -> str:
        if risk:
            self.safe_streak = 0
            self.risk_streak += 1
            if self.mode == "AVOID" or self.risk_streak >= self.risk_updates_required:
                self.mode = "AVOID"
            return self.mode

        self.risk_streak = 0

        if self.mode == "AVOID":
            time_is_invalid = not np.isfinite(time_to_closest)
            distance_is_safe = closest_distance > self.exit_safe_distance
            tracking_is_usable = confidence >= self.min_confidence
            if (distance_is_safe or time_is_invalid) and (
                tracking_is_usable or time_is_invalid
            ):
                self.safe_streak += 1
            else:
                self.safe_streak = 0
            if self.safe_streak >= self.safe_updates_required:
                self.mode = "CRUISE"
                self.safe_streak = 0
        return self.mode


@dataclass(frozen=True)
class SimulationResult:
    final_pose: tuple[float, float]
    min_clearance: float
    mode_history: tuple[str, ...]
    avoid_policy_updates: int
    final_actual_velocity: tuple[float, float, float]


def initialize(model: mujoco.MjModel) -> tuple[mujoco.MjData, GaitLayer]:
    data = mujoco.MjData(model)
    gait = GaitLayer(model)
    data.qpos[2] = 0.335
    data.qpos[gait.joint_qpos] = NOMINAL
    mujoco.mj_forward(model, data)
    return data, gait


def approximate_clearance(
    robot_xy: np.ndarray,
    box_xy: np.ndarray,
    obstacle: Obstacle,
) -> float:
    """Backward-compatible display clearance with the old extra margin."""
    return collision_clearance(robot_xy, box_xy, obstacle) - 0.10


def bounding_circle_clearance(
    robot_xy: np.ndarray,
    box_xy: np.ndarray,
    obstacle: Obstacle,
) -> float:
    """Conservative outer-circle clearance in metres.

    This signal is useful as a safety proxy and for conservative planning,
    but it is not an exact rectangle-collision test.  In particular, the
    obstacle's half-diagonal treats empty corner space as occupied.
    """
    center_distance = float(np.linalg.norm(np.asarray(robot_xy) - np.asarray(box_xy)))
    robot_radius = 0.25
    obstacle_radius = float(np.hypot(obstacle.half_length, obstacle.half_width))
    return center_distance - robot_radius - obstacle_radius


def rectangular_clearance(
    robot_xy: np.ndarray,
    box_xy: np.ndarray,
    obstacle: Obstacle,
    robot_radius: float = 0.25,
) -> float:
    """Exact clearance for a circular robot footprint vs. an axis-aligned box.

    The generated scenario boxes have identity orientation, so the shortest
    distance from the robot center to the rectangle is the Euclidean distance
    outside the rectangle.  Subtracting the robot footprint radius gives a
    much less conservative geometric collision measure than the obstacle
    bounding circle.
    """
    robot = np.asarray(robot_xy, dtype=float)
    box = np.asarray(box_xy, dtype=float)
    delta = np.abs(robot - box)
    outside = np.maximum(
        delta - np.array([obstacle.half_length, obstacle.half_width], dtype=float),
        0.0,
    )
    return float(np.linalg.norm(outside) - float(robot_radius))


def collision_clearance(
    robot_xy: np.ndarray,
    box_xy: np.ndarray,
    obstacle: Obstacle,
) -> float:
    """Benchmark collision clearance using the exact box footprint model.

    ``bounding_circle_clearance`` remains available for the conservative
    safety proxy.  Keeping this public name for the benchmark's geometric
    clearance avoids silently mixing the two metrics again.
    """
    return rectangular_clearance(robot_xy, box_xy, obstacle)


def mujoco_obstacle_contacts(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    obstacle_geom_ids: tuple[int, ...],
) -> tuple[bool, int | None, float]:
    """Return whether MuJoCo reports robot-obstacle contact at this state.

    The current episode remains kinematic, so this is telemetry/validation,
    not the primary success criterion.  Floor and obstacle-obstacle contacts
    are excluded.
    """
    contact, obstacle_index, distance, _, _ = mujoco_obstacle_contact_details(
        model,
        data,
        obstacle_geom_ids,
    )
    return contact, obstacle_index, distance


def mujoco_obstacle_contact_details(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    obstacle_geom_ids: tuple[int, ...],
) -> tuple[bool, int | None, float, str | None, tuple[float, float, float] | None]:
    """Return contact status, obstacle index, distance, geom name and point."""
    obstacle_ids = set(int(value) for value in obstacle_geom_ids)
    floor_id = int(model.geom("floor").id)
    best_distance = float("inf")
    best_index: int | None = None
    best_other_geom_name: str | None = None
    best_contact_position: tuple[float, float, float] | None = None
    for contact_index in range(int(data.ncon)):
        contact = data.contact[contact_index]
        geom1 = int(contact.geom1)
        geom2 = int(contact.geom2)
        if geom1 in obstacle_ids:
            obstacle_geom = geom1
            other_geom = geom2
        elif geom2 in obstacle_ids:
            obstacle_geom = geom2
            other_geom = geom1
        else:
            continue
        if other_geom in obstacle_ids or other_geom == floor_id:
            continue
        obstacle_index = obstacle_geom_ids.index(obstacle_geom)
        distance = float(contact.dist)
        if distance < best_distance:
            best_distance = distance
            best_index = obstacle_index
            best_other_geom_name = str(model.geom(other_geom).name)
            best_contact_position = tuple(
                float(value) for value in contact.pos
            )
    return (
        best_index is not None,
        best_index,
        best_distance,
        best_other_geom_name,
        best_contact_position,
    )


def apply_lidar_model(
    scan: np.ndarray,
    rng: np.random.Generator,
    noise_std: float,
    dropout_rate: float,
    max_range: float,
) -> np.ndarray:
    """Return the noisy scan used by the decision layer only."""
    raw = np.asarray(scan, dtype=float)
    noisy = np.full(raw.shape, max_range, dtype=float)
    hit = raw < max_range - 1e-3
    if not np.any(hit):
        return noisy

    values = raw[hit].copy()
    if noise_std > 0.0:
        values += rng.normal(0.0, noise_std, size=values.shape)
    values = np.clip(values, 0.02, max_range)
    if dropout_rate > 0.0:
        keep = rng.random(values.shape) >= dropout_rate
        values = np.where(keep, values, max_range)
    noisy[hit] = values
    return noisy


def ground_truth_prediction(
    robot_xy: np.ndarray,
    robot_velocity_world: np.ndarray,
    box_xy: np.ndarray,
    box_velocity_world: np.ndarray,
    obstacle: Obstacle,
    horizon: float,
    required_clearance: float,
) -> tuple[bool, float, float]:
    """Predict geometric danger from true MuJoCo/trajectory state."""
    relative_position = np.asarray(box_xy, dtype=float) - np.asarray(robot_xy, dtype=float)
    relative_velocity = np.asarray(box_velocity_world, dtype=float) - np.asarray(
        robot_velocity_world,
        dtype=float,
    )
    velocity_squared = float(relative_velocity @ relative_velocity)
    if velocity_squared < 1e-10:
        time_to_closest = 0.0
    else:
        time_to_closest = float(np.clip(
            -(relative_position @ relative_velocity) / velocity_squared,
            0.0,
            horizon,
        ))
    closest_distance = float(
        np.linalg.norm(relative_position + relative_velocity * time_to_closest)
    )
    threshold = 0.25 + float(np.hypot(obstacle.half_length, obstacle.half_width))
    threshold += max(0.0, required_clearance)
    return closest_distance < threshold, time_to_closest, closest_distance


def _validate_episode_config(config: EpisodeConfig) -> None:
    if config.duration <= 0.0:
        raise ValueError("duration must be positive")
    if config.stagnation_window <= 0.0:
        raise ValueError("stagnation_window must be positive")
    if config.minimum_progress < 0.0 or config.recovery_duration <= 0.0:
        raise ValueError("minimum_progress must be non-negative and recovery_duration positive")
    if config.cruise_mode not in {"forward_only", "goal_hysteresis"}:
        raise ValueError("cruise_mode must be 'forward_only' or 'goal_hysteresis'")
    if config.collision_tolerance < 0.0:
        raise ValueError("collision_tolerance cannot be negative")
    if config.deadlock_stagnation_windows <= 0:
        raise ValueError("deadlock_stagnation_windows must be positive")
    if config.deadlock_speed_threshold < 0.0:
        raise ValueError("deadlock_speed_threshold cannot be negative")
    if config.deadlock_global_progress_threshold < 0.0:
        raise ValueError("deadlock_global_progress_threshold cannot be negative")
    if config.deadlock_goal_buffer < 0.0:
        raise ValueError("deadlock_goal_buffer cannot be negative")
    if config.risk_confirmation_updates <= 0:
        raise ValueError("risk_confirmation_updates must be positive")
    if config.rejoin_gain < 0.0 or config.rejoin_deadband < 0.0:
        raise ValueError("rejoin_gain and rejoin_deadband must be non-negative")
    if config.rejoin_max_lateral_speed <= 0.0:
        raise ValueError("rejoin_max_lateral_speed must be positive")
    if config.rejoin_delay < 0.0 or config.rejoin_ramp_time <= 0.0:
        raise ValueError("rejoin_delay must be non-negative and rejoin_ramp_time positive")
    if config.global_path_lookahead <= 0.0:
        raise ValueError("global_path_lookahead must be positive")
    if config.global_path_lateral_gain < 0.0 or config.global_path_lateral_deadband < 0.0:
        raise ValueError("global path gain and deadband must be non-negative")
    if config.global_path_max_lateral_speed <= 0.0:
        raise ValueError("global_path_max_lateral_speed must be positive")
    if config.global_path_replan_period <= 0.0:
        raise ValueError("global_path_replan_period must be positive")
    if config.cruise_goal_slowdown_radius <= config.goal_tolerance:
        raise ValueError("cruise_goal_slowdown_radius must exceed goal_tolerance")
    if config.cruise_min_approach_speed < 0.0:
        raise ValueError("cruise_min_approach_speed cannot be negative")
    if config.cruise_reverse_activation_x >= 0.0:
        raise ValueError("cruise_reverse_activation_x must be negative")
    if config.cruise_reverse_release_x <= 0.0:
        raise ValueError("cruise_reverse_release_x must be positive")
    if config.cruise_reverse_release_x < abs(config.cruise_reverse_activation_x):
        raise ValueError("reverse release must be outside the reverse activation deadband")
    if config.cruise_max_reverse_speed <= 0.0 or config.cruise_reverse_gain <= 0.0:
        raise ValueError("CRUISE reverse speed and gain must be positive")
    if config.required_clearance < 0.0:
        raise ValueError("required_clearance cannot be negative")
    if config.wait_max_duration <= 0.0 or config.wait_max_clear_time <= 0.0:
        raise ValueError("WAIT duration and clear horizon must be positive")
    if config.wait_min_track_confidence < 1:
        raise ValueError("wait_min_track_confidence must be positive")
    if config.wait_min_lateral_speed < 0.0 or config.wait_max_longitudinal_speed < 0.0:
        raise ValueError("WAIT velocity gates cannot be negative")
    if not 0.0 <= config.wait_max_clear_fraction <= 1.0:
        raise ValueError("wait_max_clear_fraction must be in [0, 1]")
    if config.wait_cooldown < 0.0:
        raise ValueError("wait_cooldown cannot be negative")
    if not 0.0 <= config.lidar_dropout_rate <= 1.0:
        raise ValueError("lidar_dropout_rate must be in [0, 1]")
    if config.lidar_noise_std < 0.0 or config.command_delay < 0.0:
        raise ValueError("sensor noise and command delay cannot be negative")
    if config.route_length <= 0.0:
        raise ValueError("route_length must be positive")
    if config.max_obstacle_speed <= 0.0 or config.max_obstacle_speed > 7.0:
        raise ValueError("max_obstacle_speed must be in (0, 7]")
    if config.lidar_ray_count <= 0 or config.lidar_max_range <= 0.0:
        raise ValueError("LiDAR ray count and range must be positive")
    if config.goal_tolerance <= 0.0:
        raise ValueError("goal_tolerance must be positive")
    if config.trajectory_prediction_horizon <= 0.0:
        raise ValueError("trajectory_prediction_horizon must be positive")
    if config.trajectory_sample_dt <= 0.0:
        raise ValueError("trajectory_sample_dt must be positive")
    if config.policy_period <= 0 or config.mppi_samples <= 0 or config.mppi_horizon <= 0:
        raise ValueError("policy and MPPI sizes must be positive")


def make_decision_observation(
    goal_local_vector: np.ndarray,
    robot_velocity_local: np.ndarray,
    obstacle_position_local: np.ndarray | None,
    obstacle_velocity_local: np.ndarray | None,
    confidence: int,
    risk: bool,
    time_to_closest: float,
    closest_distance: float,
    elapsed: float,
    duration: float,
    tracked_obstacles: list[dict[str, Any]] | None = None,
    max_targets: int = 3,
    path_reference: GlobalPathReference | None = None,
    wait_estimate: WaitYieldEstimate | None = None,
) -> np.ndarray:
    """Build a fixed-size observation from controller-visible quantities.

    The first four values describe the goal and actual robot velocity.  Each
    of the next three ranked target slots contains local position, local
    velocity, confidence, TTC, closest distance, minimum clearance,
    intersection angle, and risk score.  Empty slots are zero-padded.
    """
    obstacle_position = (
        np.zeros(2, dtype=float)
        if obstacle_position_local is None
        else np.asarray(obstacle_position_local, dtype=float)
    )
    obstacle_velocity = (
        np.zeros(2, dtype=float)
        if obstacle_velocity_local is None
        else np.asarray(obstacle_velocity_local, dtype=float)
    )
    if tracked_obstacles is None:
        tracked_obstacles = [{
            "position_local": obstacle_position,
            "velocity_local": obstacle_velocity,
            "track": type("TrackView", (), {"confidence": confidence})(),
            "time_to_closest": time_to_closest,
            "closest_distance": closest_distance,
        }]
    values = [
        np.clip(goal_local_vector[0] / 3.2, -1.0, 1.0),
        np.clip(goal_local_vector[1] / 2.0, -1.0, 1.0),
        np.clip(robot_velocity_local[0] / 0.4, -1.0, 1.0),
        np.clip(robot_velocity_local[1] / 0.4, -1.0, 1.0),
    ]
    for index in range(max_targets):
        if index < len(tracked_obstacles):
            target = tracked_obstacles[index]
            position = np.asarray(target["position_local"], dtype=float)
            velocity = np.asarray(target["velocity_local"], dtype=float)
            target_confidence = float(target["track"].confidence)
            target_ttc = float(target["time_to_closest"])
            target_closest = float(target["closest_distance"])
            target_ttc = float(target.get("ttc", target_ttc))
            target_clearance = float(target.get(
                "min_clearance",
                target_closest - 0.25 - float(np.hypot(0.25, 0.25)),
            ))
            target_angle = float(target.get("intersection_angle_deg", 90.0))
            target_risk_score = float(target.get("risk_score", float(risk)))
            normalized_ttc = 1.0 if not np.isfinite(target_ttc) else np.clip(target_ttc / 3.0, 0.0, 1.0)
            normalized_closest = 1.0 if not np.isfinite(target_closest) else np.clip(target_closest / 2.0, 0.0, 1.0)
            normalized_clearance = np.clip((target_clearance + 1.0) / 2.0, 0.0, 1.0)
            normalized_angle = np.clip(target_angle / 180.0, 0.0, 1.0)
            values.extend([
                np.clip(position[0] / 3.0, -1.0, 1.0),
                np.clip(position[1] / 2.0, -1.0, 1.0),
                np.clip(velocity[0] / 0.8, -1.0, 1.0),
                np.clip(velocity[1] / 0.8, -1.0, 1.0),
                np.clip(target_confidence / 8.0, 0.0, 1.0),
                normalized_ttc,
                normalized_closest,
                normalized_clearance,
                normalized_angle,
                np.clip(target_risk_score, 0.0, 1.0),
            ])
        else:
            values.extend([0.0] * 10)
    values.extend([
        float(risk),
        np.clip(elapsed / max(duration, 1e-6), 0.0, 1.0),
    ])
    if path_reference is not None:
        values.extend([
            np.clip(path_reference.lateral_error / 2.0, -1.0, 1.0),
            np.clip(path_reference.heading_error / np.pi, -1.0, 1.0),
            np.clip(path_reference.remaining_fraction, 0.0, 1.0),
            np.clip(path_reference.target_local[1] / 2.0, -1.0, 1.0),
            np.clip(path_reference.progress, 0.0, 1.0),
            np.clip(path_reference.reference_speed / 0.4, 0.0, 1.0),
            np.clip(path_reference.terminal_risk, 0.0, 1.0),
        ])
    wait = wait_estimate or WaitYieldEstimate()
    values.extend([
        np.clip(wait.path_blocked_now, 0.0, 1.0),
        np.clip(wait.path_blocked_future, 0.0, 1.0),
        np.clip(wait.time_to_path_block, 0.0, 1.0),
        np.clip(wait.time_to_path_clear, 0.0, 1.0),
        np.clip(wait.blocking_duration, 0.0, 1.0),
        np.clip(wait.wait_cost, 0.0, 1.0),
        np.clip(wait.detour_cost, 0.0, 1.0),
        np.clip(wait.waitable, 0.0, 1.0),
        np.clip(wait.stationary_collision_risk, 0.0, 1.0),
    ])
    return np.asarray(values, dtype=float)


def gradual_rejoin_lateral_command(
    goal_local_y: float,
    elapsed_since_rejoin_start: float | None,
    config: EpisodeConfig,
) -> tuple[float, float]:
    """Return a bounded, smoothly ramped lateral command toward the goal.

    Rejoining is intentionally disabled until the robot has been clear of the
    obstacle for ``rejoin_delay``.  It then uses a smoothstep ramp so the
    robot does not jump sideways immediately after AVOID -> CRUISE.  The
    command remains proportional to the local goal error and is clamped to a
    conservative lateral-speed limit.

    The returned tuple is ``(command, ramp_factor)``.  Keeping this policy
    pure makes the timing and saturation behavior easy to test independently
    of MuJoCo.
    """
    if elapsed_since_rejoin_start is None:
        return 0.0, 0.0
    if abs(float(goal_local_y)) <= config.rejoin_deadband:
        return 0.0, 1.0

    ramp_age = float(elapsed_since_rejoin_start) - config.rejoin_delay
    if ramp_age <= 0.0:
        return 0.0, 0.0
    normalized_age = float(np.clip(ramp_age / config.rejoin_ramp_time, 0.0, 1.0))
    ramp_factor = normalized_age * normalized_age * (3.0 - 2.0 * normalized_age)
    rejoin_limit = min(config.max_vy, config.rejoin_max_lateral_speed)
    target = float(np.clip(
        config.rejoin_gain * float(goal_local_y),
        -rejoin_limit,
        rejoin_limit,
    ))
    return target * ramp_factor, ramp_factor


def goal_tracking_cruise_command(
    goal_local_vector: np.ndarray,
    cruise_speed: float,
    lateral_command: float,
    config: EpisodeConfig,
    reverse_active: bool = False,
) -> tuple[np.ndarray, bool]:
    """Build a goal-aware nominal CRUISE command.

    CRUISE remains a high-level nominal controller, but it must not keep
    driving forward after the robot has passed the goal.  The local goal
    vector is expressed in the robot body frame, so ``goal_local_vector[0]``
    is positive when the goal is ahead and negative after an overshoot.

    The speed profile is intentionally conservative:

    * stop inside the existing goal tolerance;
    * use a smoothstep speed curve instead of a hard speed switch;
    * use hysteresis around the reverse transition to prevent chatter;
    * cap reverse speed so the command cannot create a large oscillation.

    ``reverse_active`` is state carried by the caller.  Reverse enters when
    the goal is behind the robot by ``cruise_reverse_activation_x`` and exits
    only after the goal is ahead by ``cruise_reverse_release_x``.  The SDK
    still applies the final acceleration limit after this target command.
    """
    goal = np.asarray(goal_local_vector, dtype=float).reshape(-1)
    if goal.size < 2:
        raise ValueError("goal_local_vector must contain at least x and y")
    distance = float(np.linalg.norm(goal[:2]))
    goal_x = float(goal[0])
    nominal_speed = float(np.clip(cruise_speed, 0.0, config.max_vx))

    if distance <= config.goal_tolerance:
        reverse_active = False
        forward_speed = 0.0
    else:
        if reverse_active and goal_x >= config.cruise_reverse_release_x:
            reverse_active = False
        elif not reverse_active and goal_x <= config.cruise_reverse_activation_x:
            reverse_active = True

        if reverse_active and goal_x < config.cruise_reverse_activation_x:
            reverse_error = max(0.0, -goal_x)
            reverse_speed = min(
                config.cruise_max_reverse_speed,
                config.cruise_reverse_gain * reverse_error,
            )
            forward_speed = -reverse_speed
        elif goal_x <= config.cruise_reverse_release_x:
            # Neutral band: lateral goal correction may continue, but the
            # base must not alternate between full forward and reverse.
            forward_speed = 0.0
        else:
            slowdown_span = config.cruise_goal_slowdown_radius - config.goal_tolerance
            distance_phase = float(np.clip(
                (distance - config.goal_tolerance) / slowdown_span,
                0.0,
                1.0,
            ))
            distance_scale = distance_phase * distance_phase * (3.0 - 2.0 * distance_phase)
            alignment_span = config.cruise_goal_slowdown_radius - config.cruise_reverse_release_x
            alignment_phase = float(np.clip(
                (goal_x - config.cruise_reverse_release_x) / alignment_span,
                0.0,
                1.0,
            ))
            alignment_scale = alignment_phase * alignment_phase * (3.0 - 2.0 * alignment_phase)
            approach_floor = min(nominal_speed, config.cruise_min_approach_speed)
            forward_speed = (
                approach_floor * alignment_scale
                + (nominal_speed - approach_floor) * distance_scale * alignment_scale
            )

    command = np.array([
        forward_speed,
        float(np.clip(lateral_command, -config.max_vy, config.max_vy)),
        0.0,
    ], dtype=float)
    return command, reverse_active


def forward_only_cruise_command(
    cruise_speed: float,
    lateral_command: float,
    config: EpisodeConfig,
) -> tuple[np.ndarray, bool]:
    """Reproduce the pre-fix CRUISE policy for controlled comparison."""
    return np.array([
        float(np.clip(cruise_speed, 0.0, config.max_vx)),
        float(np.clip(lateral_command, -config.max_vy, config.max_vy)),
        0.0,
    ], dtype=float), False


def run_episode(
    config: EpisodeConfig,
    seed: int,
    *,
    render: bool = False,
    verbose_policy: bool = False,
    decision_policy: object | None = None,
    record_trace: bool = False,
) -> EpisodeResult:
    """Run one reproducible dynamic-obstacle episode.

    The control path is intentionally kept as:

    ``Decision Layer -> MPPI -> SimulatedD1SDK.move -> advance``.

    MuJoCo is used for the robot/box scene and LiDAR generation, while the
    current base update remains kinematic.  Collision and clearance metrics
    use the true box trajectory and robot pose, never the noisy scan.
    """
    _validate_episode_config(config)
    if decision_policy is not None:
        reset_episode = getattr(decision_policy, "reset_episode", None)
        if reset_episode is not None:
            reset_episode()
    episode_rng = np.random.default_rng(seed)
    if config.scenario_obstacles:
        trajectories = [
            ScenarioObstacle.from_mapping(dict(mapping))
            for mapping in config.scenario_obstacles
        ]
    else:
        trajectories = [ScenarioObstacle(
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
    obstacle_specs = [Obstacle(
        x=trajectory.x,
        y=trajectory.y,
        half_length=trajectory.half_length,
        half_width=trajectory.half_width,
        height=trajectory.height,
        kind=trajectory.kind,
    ) for trajectory in trajectories]
    primary_obstacle = obstacle_specs[0]
    model, _ = build_scene(
        with_lidar=True,
        dynamic_obstacles=obstacle_specs,
        lidar_ray_count=config.lidar_ray_count,
        lidar_fov=config.lidar_fov,
        lidar_max_range=config.lidar_max_range,
        world_length=config.route_length,
    )
    data, gait = initialize(model)
    footprint = calibrate_robot_footprint(model, data)
    obstacle_bodies = [
        DynamicObstacleBody(model, trajectory, index=index)
        for index, trajectory in enumerate(trajectories)
    ]
    obstacle_geom_ids = tuple(
        int(model.geom(
            "moving_obstacle_geom"
            if index == 0
            else f"moving_obstacle_{index:02d}_geom"
        ).id)
        for index in range(len(obstacle_specs))
    )
    lidar = SimulatedLidar(
        model,
        ray_count=config.lidar_ray_count,
        max_range=config.lidar_max_range,
        fov=config.lidar_fov,
    )
    # Use one shared multi-target tracker for both the decision layer and the
    # MPPI prediction cost.  The conservative maximum footprint keeps the
    # tracker safe when a scenario contains boxes of different sizes.
    tracker = MultiTargetTracker(
        obstacle_half_length=max(obstacle.half_length for obstacle in obstacle_specs),
        obstacle_half_width=max(obstacle.half_width for obstacle in obstacle_specs),
        prediction_position_uncertainty=config.obstacle_position_uncertainty,
        prediction_velocity_uncertainty=config.obstacle_velocity_uncertainty,
    )
    decision = DecisionLayer(
        enter_safe_distance=tracker.effective_safe_distance,
        exit_safe_distance=0.95,
        risk_updates_required=config.risk_confirmation_updates,
    )
    planner_seed = config.mppi_seed if config.mppi_seed is not None else seed
    planner = ObstacleMPPI(
        horizon=config.mppi_horizon,
        samples=config.mppi_samples,
        dynamic_obstacle_radius=float(np.hypot(
            primary_obstacle.half_length,
            primary_obstacle.half_width,
        )),
        rng_seed=planner_seed,
        clearance_uncertainty=config.clearance_uncertainty,
        dynamic_position_uncertainty=config.obstacle_position_uncertainty,
        dynamic_velocity_uncertainty=config.obstacle_velocity_uncertainty,
        command_delay=config.command_delay,
        robot_radius=footprint.planner_radius,
    )
    simulated_sdk = SimulatedD1SDK(
        max_velocity=np.array([
            config.max_vx,
            config.max_vy,
            config.max_yaw_rate,
        ]),
        max_acceleration=np.array([
            config.max_ax,
            config.max_ay,
            config.max_yaw_acceleration,
        ]),
        command_delay=config.command_delay,
    )

    sim_dt = model.opt.timestep * 4.0
    goal_world = np.array([config.goal_x, config.goal_y], dtype=float)
    command = np.array([config.robot_cruise_speed, 0.0, 0.0], dtype=float)
    yaw = 0.0
    elapsed = 0.0
    step = 0
    global_path = Nav2GlobalPath(
        lookahead=config.global_path_lookahead,
        grid_resolution=config.global_path_grid_resolution,
        search_padding=config.global_path_search_padding,
        soft_cost_radius=config.global_path_soft_cost_radius,
        cost_weight=config.global_path_cost_weight,
        reference_speed_max=config.max_vx,
        reference_speed_min=min(config.cruise_min_approach_speed, config.max_vx),
    )
    global_path.plan(
        data.qpos[:2],
        goal_world,
        obstacles=obstacle_specs,
        robot_radius=config.global_path_robot_radius,
        obstacle_margin=config.global_path_obstacle_margin,
    )
    path_reference = global_path.reference(data.qpos[:2], yaw)
    last_global_replan = 0.0
    for obstacle_body in obstacle_bodies:
        obstacle_body.update(data, 0.0)
    mujoco.mj_forward(model, data)
    initial_states = [trajectory.state(0.0) for trajectory in trajectories]
    initial_clearances = [
        collision_clearance(data.qpos[:2], state[0], obstacle_spec)
        for state, obstacle_spec in zip(initial_states, obstacle_specs)
        if state[2]
    ]
    initial_proxy_clearances = [
        bounding_circle_clearance(data.qpos[:2], state[0], obstacle_spec)
        for state, obstacle_spec in zip(initial_states, obstacle_specs)
        if state[2]
    ]
    min_clearance = min(initial_clearances) if initial_clearances else float("inf")
    min_proxy_clearance = (
        min(initial_proxy_clearances) if initial_proxy_clearances else float("inf")
    )
    path_length = 0.0
    avoid_duration = 0.0
    mode_history: list[str] = []
    decision_triggered = False
    true_positive = 0
    true_negative = 0
    false_positive = 0
    false_negative = 0
    tracking_miss_streak = 0
    tracking_failure = False
    collision = False
    geometric_collision = (
        min_clearance < -config.collision_tolerance
    )
    proxy_collision = min_proxy_clearance < 0.0
    near_miss = (
        min_clearance < 0.0
        and min_clearance >= -config.collision_tolerance
    )
    # MuJoCo contact is the physical collision ground truth.  The exact
    # clearance and outer-circle proxy are retained as separate diagnostics.
    (
        physical_contact,
        initial_contact_obstacle_index,
        initial_contact_distance,
        initial_contact_geom_name,
        initial_contact_position,
    ) = mujoco_obstacle_contact_details(
        model,
        data,
        obstacle_geom_ids,
    )
    collision = bool(physical_contact)
    collision_obstacle_index: int | None = initial_contact_obstacle_index
    contact_distance = float(initial_contact_distance)
    contact_geom_name: str | None = initial_contact_geom_name
    contact_position: tuple[float, float, float] | None = initial_contact_position
    near_miss = bool(near_miss and not physical_contact)
    proxy_only = bool(
        proxy_collision and not physical_contact and not geometric_collision
    )
    reached_goal = False
    last_diagnostic_step = -1
    recovery_until: float | None = None
    recovery_used = False
    recovery_active_duration = 0.0
    rejoin_started_at: float | None = None
    cruise_reverse_active = False
    progress_history: list[tuple[float, float]] = []
    trace_rows: list[dict[str, Any]] = []
    wait_estimate = WaitYieldEstimate()
    wait_duration = 0.0
    wait_cooldown_until = 0.0

    def current_obstacle_states(
        at_time: float,
    ) -> list[tuple[np.ndarray, np.ndarray, bool]]:
        return [trajectory.state(at_time) for trajectory in trajectories]

    def one_step() -> None:
        nonlocal command, yaw, elapsed, step, min_clearance, path_length
        nonlocal avoid_duration, decision_triggered, true_positive, true_negative
        nonlocal false_positive, false_negative, tracking_miss_streak
        nonlocal tracking_failure, collision, geometric_collision
        nonlocal proxy_collision, physical_contact
        nonlocal near_miss, proxy_only
        nonlocal min_proxy_clearance, reached_goal, last_diagnostic_step
        nonlocal collision_obstacle_index
        nonlocal contact_distance, contact_geom_name, contact_position
        nonlocal recovery_until, recovery_used, recovery_active_duration
        nonlocal rejoin_started_at
        nonlocal cruise_reverse_active
        nonlocal progress_history
        nonlocal path_reference, last_global_replan, wait_estimate
        nonlocal wait_duration, wait_cooldown_until

        for obstacle_body in obstacle_bodies:
            obstacle_body.update(data, elapsed)
        mujoco.mj_forward(model, data)

        if step % config.policy_period == 0:
            raw_scan = lidar.scan(data)
            scan = apply_lidar_model(
                raw_scan,
                episode_rng,
                config.lidar_noise_std,
                config.lidar_dropout_rate,
                config.lidar_max_range,
            )
            points = lidar.point_cloud(scan)
            ray_indices = lidar.hit_ray_indices(scan)
            robot_xy = data.qpos[:2].copy()
            robot_velocity_world = simulated_sdk.getWorldVelocity(yaw)[:2]
            current_states = current_obstacle_states(elapsed)
            if elapsed - last_global_replan >= config.global_path_replan_period - 1e-9:
                dynamic_specs = [
                    Obstacle(
                        x=float(state[0][0]),
                        y=float(state[0][1]),
                        half_length=obstacle.half_length,
                        half_width=obstacle.half_width,
                        height=obstacle.height,
                    )
                    for state, obstacle in zip(current_states, obstacle_specs)
                    if state[2]
                ]
                global_path.plan(
                    robot_xy,
                    goal_world,
                    obstacles=dynamic_specs,
                    robot_radius=config.global_path_robot_radius,
                    obstacle_margin=config.global_path_obstacle_margin,
                )
                last_global_replan = elapsed
            path_reference = global_path.reference(robot_xy, yaw)
            truth_predictions = [
                ground_truth_prediction(
                    robot_xy,
                    robot_velocity_world,
                    state[0],
                    state[1],
                    obstacle_spec,
                    config.truth_horizon,
                    config.required_clearance,
                )
                for state, obstacle_spec in zip(current_states, obstacle_specs)
                if state[2]
            ]
            if truth_predictions:
                truth_risk = any(prediction[0] for prediction in truth_predictions)
                truth_time = min(
                    prediction[1] for prediction in truth_predictions if prediction[0]
                ) if truth_risk else min(prediction[1] for prediction in truth_predictions)
                truth_closest = min(prediction[2] for prediction in truth_predictions)
            else:
                truth_risk, truth_time, truth_closest = False, float("inf"), float("inf")

            tracker.update(points, robot_xy, yaw, elapsed, ray_indices)
            risk, time_to_closest, closest = tracker.collision_prediction(
                robot_xy,
                robot_velocity_world,
                yaw=yaw,
                horizon=config.trajectory_prediction_horizon,
            )
            tracked_predictions = tracker.predictions(
                robot_xy,
                robot_velocity_world,
                yaw=yaw,
                horizon=config.trajectory_prediction_horizon,
                sample_dt=config.trajectory_sample_dt,
            )
            path_reference = replace(
                path_reference,
                terminal_risk=terminal_path_risk(
                    global_path,
                    tracked_predictions,
                    robot_radius=config.global_path_robot_radius,
                    obstacle_radius=float(np.hypot(
                        tracker.obstacle_half_length,
                        tracker.obstacle_half_width,
                    )),
                ),
            )
            wait_estimate = estimate_wait_yield(
                global_path,
                tracked_predictions,
                robot_xy,
                robot_radius=config.global_path_robot_radius,
                obstacle_radius=float(np.hypot(
                    tracker.obstacle_half_length,
                    tracker.obstacle_half_width,
                )),
                lookahead_distance=max(
                    config.global_path_lookahead * 3.0,
                    config.global_path_robot_radius * 4.0,
                ),
                max_wait_seconds=config.wait_max_clear_time,
                min_obstacle_speed=config.wait_min_obstacle_speed,
            )
            wait_estimate = gate_wait_yield(
                global_path,
                wait_estimate,
                tracked_predictions,
                min_track_confidence=config.wait_min_track_confidence,
                min_lateral_speed=config.wait_min_lateral_speed,
                max_longitudinal_speed=config.wait_max_longitudinal_speed,
                max_clear_fraction=config.wait_max_clear_fraction,
            )
            previous_mode = decision.mode
            obstacle_position_world = tracker.observation
            obstacle_velocity_world = tracker.velocity.copy()
            obstacle_position_local = None
            obstacle_velocity_local = None
            if obstacle_position_world is not None:
                obstacle_position_local = world_to_local_position(
                    obstacle_position_world,
                    robot_xy,
                    yaw,
                )
                obstacle_velocity_local = world_to_local_velocity(
                    obstacle_velocity_world,
                    yaw,
                )

            baseline_mode = decision.update(
                risk=risk,
                time_to_closest=time_to_closest,
                closest_distance=closest,
                confidence=tracker.confidence,
            )
            if decision_policy is None:
                action = None
                requested_action = None
                action_overridden = False
                mode = baseline_mode
            else:
                observation = make_decision_observation(
                    local_goal(data, yaw, goal_world),
                    simulated_sdk.getBodyVelocity()[:2],
                    obstacle_position_local,
                    obstacle_velocity_local,
                    tracker.confidence,
                    risk,
                    time_to_closest,
                    closest,
                    elapsed,
                    config.duration,
                    tracked_obstacles=tracked_predictions,
                    path_reference=path_reference,
                    wait_estimate=wait_estimate,
                )
                if hasattr(decision_policy, "act_with_context"):
                    action = int(decision_policy.act_with_context(
                        observation,
                        {
                            "config": config,
                            "elapsed": float(elapsed),
                            "robot_xy": robot_xy.copy(),
                            "robot_velocity_world": robot_velocity_world.copy(),
                            "yaw": float(yaw),
                            "goal_world": goal_world.copy(),
                            "robot_radius": float(footprint.planner_radius),
                            "global_path": global_path,
                            "path_reference": path_reference,
                            "trajectories": tuple(trajectories),
                            "obstacle_specs": tuple(obstacle_specs),
                            "current_states": tuple(current_states),
                            "truth_predictions": tuple(truth_predictions),
                            "truth_risk": bool(truth_risk),
                            "truth_time": float(truth_time),
                            "truth_closest": float(truth_closest),
                            "tracked_predictions": tuple(tracked_predictions),
                        },
                    ))
                else:
                    action = int(decision_policy.act(observation))
                requested_action = action
                action_overridden = False
                if action not in (0, 1, 2, 3, 4):
                    raise ValueError("decision policy action must be one of 0, 1, 2, 3, 4")
                trusted_ground_truth = bool(
                    getattr(decision_policy, "uses_ground_truth", False)
                )
                # A scan-derived safety shield remains active during PPO
                # training/evaluation so a bad policy cannot intentionally
                # ignore an already predicted collision.
                immediate_risk = bool(
                    risk and time_to_closest < config.minimum_reaction_time
                )
                if immediate_risk and not trusted_ground_truth:
                    # Pass away from the currently observed obstacle side;
                    # this keeps the policy from selecting the geometrically
                    # wrong side during an already-dangerous update.
                    obstacle_side = 1.0 if (
                        obstacle_position_local is None
                        or obstacle_position_local[1] <= 0.0
                    ) else -1.0
                    action = 1 if obstacle_side > 0.0 else 2
                wait_allowed = bool(
                    wait_estimate.waitable > 0.5
                    and wait_estimate.stationary_collision_risk < 0.5
                    and wait_duration < config.wait_max_duration
                    and elapsed >= wait_cooldown_until
                )
                if action == 4 and not wait_allowed and not trusted_ground_truth:
                    # Never turn a persistent blocker or a stationary-robot
                    # collision threat into an indefinite stop.  The model's
                    # WAIT request is converted to a side-specific MPPI
                    # avoidance command and recorded as an override.
                    obstacle_side = 1.0 if (
                        obstacle_position_local is None
                        or obstacle_position_local[1] <= 0.0
                    ) else -1.0
                    action = 1 if obstacle_side > 0.0 else 2
                    action_overridden = True
                # The existing decision layer is a hard safety floor: PPO may
                # request an earlier avoidance, but it cannot cancel an
                # already active AVOID state or its hysteresis.
                if trusted_ground_truth:
                    mode = (
                        "AVOID" if action in (1, 2) else
                        "WAIT_YIELD" if action == 4 else
                        "CRUISE"
                    )
                elif immediate_risk:
                    mode = "AVOID"
                elif action in (1, 2):
                    mode = "AVOID"
                    decision.mode = mode
                    decision.safe_streak = 0
                elif action == 4:
                    mode = "WAIT_YIELD"
                elif baseline_mode == "AVOID":
                    mode = "AVOID"
                else:
                    mode = "CRUISE"
            mode_history.append(mode)
            decision_triggered = decision_triggered or mode != "CRUISE"

            if previous_mode in ("AVOID", "WAIT_YIELD") and mode == "CRUISE":
                # A new avoidance episode should choose its side from the new
                # observation.  More importantly, CRUISE is now allowed to
                # steer back toward the goal instead of inheriting a stale
                # lateral commitment.  Start a delayed, smooth rejoin rather
                # than applying the full lateral correction immediately.
                planner.avoid_side = None
                rejoin_started_at = elapsed
                progress_history.clear()
            if previous_mode == "WAIT_YIELD" and mode != "WAIT_YIELD":
                wait_cooldown_until = elapsed + config.wait_cooldown
            elif mode == "AVOID":
                # Keep an already-started rejoin timer across a brief return
                # to AVOID.  The risk decision still has priority, but when
                # the current prediction is safe we can keep making a gentle
                # return instead of restarting the lateral excursion.
                pass

            # Confusion-matrix labels are generated at each policy update.
            positive_action = mode in ("AVOID", "WAIT_YIELD")
            if truth_risk and positive_action:
                true_positive += 1
            elif not truth_risk and not positive_action:
                true_negative += 1
            elif not truth_risk and positive_action:
                false_positive += 1
            elif truth_risk and truth_time >= config.minimum_reaction_time:
                # A near-zero time-to-closest point is already too late to
                # call a missed early warning; only the actionable danger
                # interval contributes to FN.
                false_negative += 1

            visible_active_obstacle = False
            for state in current_states:
                if not state[2]:
                    continue
                local_position = world_to_local_position(state[0], robot_xy, yaw)
                visible_active_obstacle = visible_active_obstacle or (
                    0.35 < local_position[0] < min(config.lidar_max_range, 2.60)
                    and abs(local_position[1]) < 1.80
                )
            if visible_active_obstacle and tracker.track_count == 0:
                tracking_miss_streak += 1
                if tracking_miss_streak >= config.tracking_failure_updates:
                    tracking_failure = True
            elif tracker.track_count > 0:
                tracking_miss_streak = 0

            # Keep one deliberately small recovery mechanism.  If MPPI has
            # remained in AVOID but the robot has not reduced its goal
            # distance over a short window, reverse the lateral exploration
            # side once and hold it briefly.  This is not a third decision
            # mode: CRUISE/AVOID and the safety shield remain unchanged.
            rejoin_factor = 0.0
            rejoin_age: float | None = None
            if mode == "AVOID":
                distance_now = float(np.linalg.norm(robot_xy - goal_world))
                progress_history.append((elapsed, distance_now))
                if (
                    not recovery_used
                    and len(progress_history) >= 2
                    and elapsed - progress_history[0][0]
                    >= config.stagnation_window - 1e-9
                    and progress_history[0][1] - distance_now
                    < config.minimum_progress
                ):
                    current_side = planner.avoid_side
                    if current_side not in (-1.0, 1.0):
                        current_side = (
                            -1.0
                            if obstacle_position_local is not None
                            and obstacle_position_local[1] > 0.0
                            else 1.0
                        )
                    planner.avoid_side = -float(current_side)
                    recovery_until = elapsed + config.recovery_duration
                    recovery_used = True
            elif mode == "WAIT_YIELD":
                # Waiting is still executed through the SDK. The SDK applies
                # acceleration and command-delay limits, so the base does
                # not teleport to zero velocity.
                planner.avoid_side = None
                cruise_reverse_active = False
                rejoin_age = None
            else:
                progress_history.clear()

            if mode == "AVOID":
                cruise_reverse_active = False
                recovery_active = (
                    recovery_until is not None and elapsed < recovery_until
                )
                if (
                    not recovery_active
                    and decision_policy is not None
                    and action in (1, 2)
                ):
                    planner.avoid_side = 1.0 if action == 1 else -1.0
                command = planner.update(
                    np.zeros(3),
                    points,
                    path_reference.target_local,
                    terminal_goal=local_goal(data, yaw, goal_world),
                    reference_speed=path_reference.reference_speed,
                    terminal_risk=path_reference.terminal_risk,
                    # The list is the authoritative multi-target prediction;
                    # leave the legacy single-target parameters empty to avoid
                    # charging the selected target twice in the MPPI cost.
                    dynamic_obstacle_position_local=None,
                    dynamic_obstacle_velocity_local=None,
                    current_time=elapsed,
                    dynamic_trajectories_local=[
                        (
                            np.asarray(prediction["trajectory_local"], dtype=float),
                            np.asarray(prediction["velocity_local"], dtype=float),
                            float(np.hypot(
                                tracker.obstacle_half_length,
                                tracker.obstacle_half_width,
                            )),
                            np.asarray(
                                prediction.get("trajectory_uncertainty_world", [0.0]),
                                dtype=float,
                            ),
                        )
                        for prediction in tracked_predictions
                        if prediction["track"].confidence >= 2
                    ],
                    trajectory_sample_dt=config.trajectory_sample_dt,
                )
                path_vy = path_lateral_command(
                    path_reference,
                    gain=config.global_path_lateral_gain,
                    deadband=config.global_path_lateral_deadband,
                    max_speed=min(config.max_vy, config.global_path_max_lateral_speed),
                )
                if not risk and rejoin_started_at is not None:
                    goal_local_vector = local_goal(data, yaw, goal_world)
                    rejoin_age = elapsed - rejoin_started_at
                    rejoin_vy, rejoin_factor = gradual_rejoin_lateral_command(
                        goal_local_vector[1],
                        rejoin_age,
                        config,
                    )
                    # Preserve MPPI's forward/safety decision while replacing
                    # only a now-unnecessary lateral excursion with the
                    # smooth global-path rejoin command.
                    command[1] = path_vy if abs(path_vy) > 0.0 else rejoin_vy
            elif mode == "WAIT_YIELD":
                command = np.zeros(3, dtype=float)
                rejoin_age = None
            else:
                cruise_speed = min(
                    config.robot_cruise_speed,
                    path_reference.reference_speed,
                )
                if decision_policy is not None and action == 3:
                    cruise_speed *= 0.55
                goal_local_vector = local_goal(data, yaw, goal_world)
                rejoin_age = (
                    None
                    if rejoin_started_at is None
                    else elapsed - rejoin_started_at
                )
                rejoin_vy, rejoin_factor = gradual_rejoin_lateral_command(
                    goal_local_vector[1],
                    rejoin_age,
                    config,
                )
                path_vy = path_lateral_command(
                    path_reference,
                    gain=config.global_path_lateral_gain,
                    deadband=config.global_path_lateral_deadband,
                    max_speed=min(config.max_vy, config.global_path_max_lateral_speed),
                )
                goal_lateral_vy = path_vy if abs(path_vy) > 0.0 else rejoin_vy
                if config.cruise_mode == "forward_only":
                    command, cruise_reverse_active = forward_only_cruise_command(
                        cruise_speed,
                        goal_lateral_vy,
                        config,
                    )
                else:
                    command, cruise_reverse_active = goal_tracking_cruise_command(
                        goal_local_vector,
                        cruise_speed,
                        goal_lateral_vy,
                        config,
                        reverse_active=cruise_reverse_active,
                    )

            if mode == "WAIT_YIELD":
                wait_duration += sim_dt * config.policy_period
            else:
                wait_duration = 0.0

            if record_trace:
                trace_rows.append({
                    "t": float(elapsed),
                    "robot": [float(robot_xy[0]), float(robot_xy[1])],
                    "yaw": float(yaw),
                    "goal": [float(goal_world[0]), float(goal_world[1])],
                    "global_path": [
                        [float(point[0]), float(point[1])]
                        for point in global_path.path
                    ],
                    "obstacles": [
                        {
                            "id": trajectory.obstacle_id,
                            "kind": trajectory.kind,
                            "motion_kind": trajectory.motion_kind or trajectory.kind,
                            "x": float(state[0][0]),
                            "y": float(state[0][1]),
                            "active": bool(state[2]),
                        }
                        for trajectory, state in zip(trajectories, current_states)
                    ],
                    "mode": mode,
                    "action": None if action is None else int(action),
                    "requested_action": requested_action,
                    "action_overridden": bool(action_overridden),
                    "action_name": (
                        "BASELINE" if action is None else
                        ("CRUISE", "AVOID_LEFT", "AVOID_RIGHT", "SLOWDOWN", "WAIT_YIELD")[int(action)]
                    ),
                    "truth_risk": bool(truth_risk),
                    "predicted_risk": bool(risk),
                    "truth_ttc": float(truth_time),
                    "predicted_ttc": float(time_to_closest),
                    "tracker_confidence": int(tracker.confidence),
                    "prediction_models": [
                        str(prediction.get("prediction_model", "CV"))
                        for prediction in tracked_predictions
                    ],
                    "prediction_uncertainty_radius": [
                        float(prediction.get("uncertainty_radius", 0.0))
                        for prediction in tracked_predictions
                    ],
                    "command": [float(value) for value in command],
                    "actual_velocity": [
                        float(value) for value in simulated_sdk.velocity
                    ],
                    "min_clearance": float(min_clearance),
                    "min_proxy_clearance": float(min_proxy_clearance),
                    "collision": bool(collision),
                    "geometric_collision": bool(geometric_collision),
                    "proxy_collision": bool(proxy_collision),
                    "physical_contact": bool(physical_contact),
                    "near_miss": bool(near_miss),
                    "recovery_triggered": bool(recovery_used),
                    "recovery_active": bool(
                        recovery_until is not None and elapsed < recovery_until
                    ),
                    "rejoin_factor": float(rejoin_factor),
                    "rejoin_age": None if rejoin_age is None else float(rejoin_age),
                    "cruise_reverse_active": bool(cruise_reverse_active),
                    "path_lateral_error": float(path_reference.lateral_error),
                    "path_heading_error": float(path_reference.heading_error),
                    "path_remaining_distance": float(path_reference.remaining_distance),
                    "path_progress": float(path_reference.progress),
                    "reference_speed": float(path_reference.reference_speed),
                    "terminal_risk": float(path_reference.terminal_risk),
                    "path_blocked_now": float(wait_estimate.path_blocked_now),
                    "path_blocked_future": float(wait_estimate.path_blocked_future),
                    "time_to_path_block": float(wait_estimate.time_to_path_block),
                    "time_to_path_clear": float(wait_estimate.time_to_path_clear),
                    "blocking_duration": float(wait_estimate.blocking_duration),
                    "wait_cost": float(wait_estimate.wait_cost),
                    "detour_cost": float(wait_estimate.detour_cost),
                    "waitable": float(wait_estimate.waitable),
                    "stationary_collision_risk": float(
                        wait_estimate.stationary_collision_risk
                    ),
                    "wait_duration": float(wait_duration),
                    "path_target_local": [
                        float(value) for value in path_reference.target_local
                    ],
                })

            if verbose_policy:
                obstacle_text = "none" if obstacle_position_world is None else (
                    f"({obstacle_position_world[0]:+.2f},"
                    f"{obstacle_position_world[1]:+.2f})"
                )
                print(
                    f"t={elapsed:5.2f}s decision={mode:6s} "
                    f"truth={'RISK' if truth_risk else 'SAFE':4s} "
                    f"lidar_hits={len(points):02d} confidence={tracker.confidence} "
                    f"obstacle_world={obstacle_text} "
                    f"truth_ttc={truth_time:4.2f}s truth_closest={truth_closest:4.2f}m "
                    f"cmd=(vx={command[0]:+.2f}, vy={command[1]:+.2f}) "
                    f"actual=(vx={simulated_sdk.velocity[0]:+.2f},"
                    f"vy={simulated_sdk.velocity[1]:+.2f})"
                )
                last_diagnostic_step = step
            if mode != previous_mode and not verbose_policy and step != last_diagnostic_step:
                last_diagnostic_step = step

        previous_xy = data.qpos[:2].copy()
        actual_command = simulated_sdk.move(command, sim_dt)
        yaw = advance(model, data, gait, yaw, actual_command, sim_dt)
        for obstacle_body in obstacle_bodies:
            obstacle_body.update(data, elapsed + sim_dt)
        mujoco.mj_forward(model, data)
        robot_xy = data.qpos[:2].copy()
        next_states = current_obstacle_states(elapsed + sim_dt)
        next_clearances = [
            (
                index,
                collision_clearance(robot_xy, state[0], obstacle_specs[index]),
            )
            for index, state in enumerate(next_states)
            if state[2]
        ]
        next_proxy_clearances = [
            (
                index,
                bounding_circle_clearance(
                    robot_xy,
                    state[0],
                    obstacle_specs[index],
                ),
            )
            for index, state in enumerate(next_states)
            if state[2]
        ]
        if next_clearances:
            next_obstacle_index, next_min_clearance = min(
                next_clearances,
                key=lambda item: item[1],
            )
            min_clearance = min(min_clearance, next_min_clearance)
            if next_min_clearance < -config.collision_tolerance:
                geometric_collision = True
            elif next_min_clearance < 0.0:
                near_miss = True
        if next_proxy_clearances:
            _, next_min_proxy_clearance = min(
                next_proxy_clearances,
                key=lambda item: item[1],
            )
            min_proxy_clearance = min(
                min_proxy_clearance,
                next_min_proxy_clearance,
            )
            proxy_collision = proxy_collision or next_min_proxy_clearance < 0.0
        (
            contact_now,
            contact_obstacle_index,
            current_contact_distance,
            current_contact_geom_name,
            current_contact_position,
        ) = mujoco_obstacle_contact_details(
            model,
            data,
            obstacle_geom_ids,
        )
        physical_contact = physical_contact or contact_now
        if contact_now:
            collision = True
            collision_obstacle_index = contact_obstacle_index
            if current_contact_distance <= contact_distance:
                contact_distance = float(current_contact_distance)
                contact_geom_name = current_contact_geom_name
                contact_position = current_contact_position
            near_miss = False
        proxy_only = bool(
            proxy_collision and not physical_contact and not geometric_collision
        )
        path_length += float(np.linalg.norm(robot_xy - previous_xy))
        if decision.mode == "AVOID":
            avoid_duration += sim_dt
        if recovery_until is not None and elapsed < recovery_until:
            recovery_active_duration += min(sim_dt, recovery_until - elapsed)
        elapsed += sim_dt
        if record_trace and collision:
            trace_rows.append({
                "t": float(elapsed),
                "robot": [float(robot_xy[0]), float(robot_xy[1])],
                "yaw": float(yaw),
                "goal": [float(goal_world[0]), float(goal_world[1])],
                "global_path": [
                    [float(point[0]), float(point[1])]
                    for point in global_path.path
                ],
                "obstacles": [
                    {
                        "id": trajectory.obstacle_id,
                        "kind": trajectory.kind,
                        "motion_kind": trajectory.motion_kind or trajectory.kind,
                        "x": float(state[0][0]),
                        "y": float(state[0][1]),
                        "active": bool(state[2]),
                    }
                    for trajectory, state in zip(trajectories, next_states)
                ],
                "mode": decision.mode,
                "min_clearance": float(min_clearance),
                "min_proxy_clearance": float(min_proxy_clearance),
                "collision": True,
                "geometric_collision": bool(geometric_collision),
                "proxy_collision": bool(proxy_collision),
                "physical_contact": bool(physical_contact),
                "near_miss": bool(near_miss),
                    "collision_obstacle_index": collision_obstacle_index,
                    "contact_distance": float(contact_distance),
                    "contact_geom_name": contact_geom_name,
                    "contact_position": contact_position,
                "recovery_triggered": bool(recovery_used),
                    "recovery_active": bool(
                        recovery_until is not None and elapsed < recovery_until
                    ),
                    "cruise_reverse_active": bool(cruise_reverse_active),
                    "path_lateral_error": float(path_reference.lateral_error),
                    "path_heading_error": float(path_reference.heading_error),
                    "path_remaining_distance": float(path_reference.remaining_distance),
                    "path_progress": float(path_reference.progress),
                    "reference_speed": float(path_reference.reference_speed),
                    "terminal_risk": float(path_reference.terminal_risk),
                    "path_target_local": [
                        float(value) for value in path_reference.target_local
                    ],
                })
        step += 1
        reached_goal = reached_goal or (
            float(np.linalg.norm(robot_xy - goal_world)) <= config.goal_tolerance
        )

    def run_loop(viewer: mujoco.viewer.Handle | None = None) -> None:
        while (
            elapsed < config.duration
            and not collision
            and not reached_goal
            and (viewer is None or viewer.is_running())
        ):
            one_step()
            if viewer is not None:
                viewer.sync()
                time.sleep(sim_dt)

    if render:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Free camera: it stays where it is until the user moves it.
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            viewer.cam.trackbodyid = -1
            viewer.cam.distance = 3.4
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -18
            viewer.cam.lookat[:] = [1.10, 0.0, 0.25]
            run_loop(viewer)
    else:
        run_loop()

    timeout = not reached_goal and elapsed >= config.duration - 1e-9
    success = (
        not collision
        and not geometric_collision
        and reached_goal
        and not timeout
        and min_clearance >= config.required_clearance - config.collision_tolerance
        and not tracking_failure
    )
    near_miss = bool(near_miss and not physical_contact)
    proxy_only = bool(
        proxy_collision and not physical_contact and not geometric_collision
    )
    if success:
        failure_reason = "none"
    elif collision:
        failure_reason = "collision"
    elif geometric_collision:
        failure_reason = "geometry_overlap"
    elif min_clearance < config.required_clearance - config.collision_tolerance:
        failure_reason = "clearance"
    elif tracking_failure:
        failure_reason = "tracking"
    elif timeout:
        failure_reason = "timeout"
    elif near_miss:
        failure_reason = "near_miss"
    else:
        failure_reason = "goal"

    distance_to_goal = float(np.linalg.norm(data.qpos[:2] - goal_world))
    decision_metrics = summarize_decision_trace(
        trace_rows,
        timeout=timeout,
        distance_to_goal=distance_to_goal,
        goal_tolerance=config.goal_tolerance,
        collision=collision,
    )

    return EpisodeResult(
        success=success,
        collision=collision,
        reached_goal=reached_goal,
        timeout=timeout,
        min_clearance=float(min_clearance),
        elapsed_time=float(elapsed),
        path_length=float(path_length),
        decision_triggered=decision_triggered,
        avoid_duration=float(avoid_duration),
        true_positive=true_positive,
        true_negative=true_negative,
        false_positive=false_positive,
        false_negative=false_negative,
        seed=int(seed),
        failure_reason=failure_reason,
        tracking_failure=tracking_failure,
        final_pose=(float(data.qpos[0]), float(data.qpos[1])),
        final_actual_velocity=tuple(float(value) for value in simulated_sdk.velocity),
        mode_history=tuple(mode_history),
        distance_to_goal=distance_to_goal,
        recovery_triggered=recovery_used,
        recovery_active_duration=float(recovery_active_duration),
        trace=tuple(trace_rows),
        proxy_collision=bool(proxy_collision),
        min_proxy_clearance=float(min_proxy_clearance),
        physical_contact=bool(physical_contact),
        collision_obstacle_index=collision_obstacle_index,
        contact_geom_name=contact_geom_name,
        contact_distance=float(contact_distance),
        contact_position=contact_position,
        near_miss=bool(near_miss),
        geometric_collision=bool(geometric_collision),
        proxy_only=bool(proxy_only),
        decision_metrics=decision_metrics,
        footprint=footprint_summary(footprint),
    )


def run(
    duration: float,
    headless: bool,
    box_speed: float,
    box_x: float,
    verbose_policy: bool = False,
    sdk_delay: float = 0.08,
    decision_policy: object | None = None,
) -> SimulationResult:
    config = EpisodeConfig(
        duration=duration,
        box_vy=box_speed,
        box_x=box_x,
        command_delay=sdk_delay,
    )
    episode = run_episode(
        config,
        seed=0,
        render=not headless,
        verbose_policy=verbose_policy,
        decision_policy=decision_policy,
    )
    print(
        f"final_pose=({episode.final_pose[0]:+.3f}, {episode.final_pose[1]:+.3f}) "
        f"avoid_policy_updates={sum(mode == 'AVOID' for mode in episode.mode_history)} "
        f"min_clearance={episode.min_clearance:+.3f}m"
    )
    if episode.min_clearance > 0.0:
        print("status=PASS: decision layer selected a collision-free behavior")
    else:
        print("status=FAIL: decision-layer route entered the approximate safety radius")

    return SimulationResult(
        final_pose=episode.final_pose,
        min_clearance=episode.min_clearance,
        mode_history=episode.mode_history,
        avoid_policy_updates=sum(mode == "AVOID" for mode in episode.mode_history),
        final_actual_velocity=episode.final_actual_velocity,
    )


def _config_from_json(path: str | Path) -> EpisodeConfig:
    with Path(path).open("r", encoding="utf-8") as handle:
        payload = json.load(handle)
    if "config" in payload:
        payload = payload["config"]
    allowed = {field.name for field in fields(EpisodeConfig)}
    return EpisodeConfig(**{key: value for key, value in payload.items() if key in allowed})


def _seed_from_json(path: str | Path) -> int | None:
    with Path(path).open("r", encoding="utf-8") as handle:
        payload = json.load(handle)
    value = payload.get("seed")
    return None if value is None else int(value)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration", type=float, default=40.0)
    parser.add_argument(
        "--box-speed",
        type=float,
        default=0.38,
        help="lateral box speed in m/s",
    )
    parser.add_argument(
        "--box-x",
        type=float,
        default=1.25,
        help="box forward distance in m; increase it for a non-collision case",
    )
    parser.add_argument(
        "--verbose-policy",
        action="store_true",
        help="print one diagnostic line per policy update",
    )
    parser.add_argument(
        "--sdk-delay",
        type=float,
        default=0.08,
        help="simulated SDK command delay in seconds",
    )
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument(
        "--replay-config",
        type=str,
        default=None,
        help="benchmark failure JSON produced in output/failures",
    )
    parser.add_argument(
        "--ppo-model",
        type=str,
        default=None,
        help="load a Stable-Baselines3 PPO .zip model for the decision layer",
    )
    parser.add_argument(
        "--ppo-stochastic",
        action="store_true",
        help="sample actions from the SB3 policy instead of using deterministic actions",
    )
    args = parser.parse_args()
    decision_policy = None
    if args.ppo_model:
        from d1_edu_sb3_policy import SB3DecisionPolicy

        decision_policy = SB3DecisionPolicy(
            args.ppo_model,
            deterministic=not args.ppo_stochastic,
        )
    if args.replay_config:
        config = _config_from_json(args.replay_config)
        replay_seed = args.seed
        if replay_seed is None:
            replay_seed = _seed_from_json(args.replay_config) or 0
        run_episode(
            config,
            seed=replay_seed,
            render=not args.headless,
            verbose_policy=True,
            decision_policy=decision_policy,
        )
    else:
        run(
            args.duration,
            args.headless,
            args.box_speed,
            args.box_x,
            args.verbose_policy,
            args.sdk_delay,
            decision_policy,
        )


if __name__ == "__main__":
    main()
