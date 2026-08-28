#!/usr/bin/env python3
"""Gymnasium environment for high-level D1 Edu PPO decisions.

The action space controls only the high-level decision.  MPPI still creates
the local velocity command and the simulated SDK still applies velocity,
acceleration, and delay limits.
"""

from __future__ import annotations

from dataclasses import replace
from typing import Any

import gymnasium as gym
from gymnasium import spaces
import mujoco
import numpy as np

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
    path_lateral_command,
    terminal_path_risk,
)
from run_d1_edu_mppi_obstacle_viewer import (
    ObstacleMPPI,
    SimulatedLidar,
    advance,
)
from run_d1_edu_obstacle_avoidance_viewer import Obstacle, build_scene
from d1_edu_scenario import ScenarioObstacle
from d1_edu_sim_sdk import SimulatedD1SDK
from d1_edu_footprint import calibrate_robot_footprint


ACTION_NAMES = ("CRUISE", "AVOID_LEFT", "AVOID_RIGHT", "SLOWDOWN", "WAIT_YIELD")
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
        scenario_mode: str = "multi_target",
        seed: int = 0,
        duration: float = 24.0,
        route_length: float = 3.20,
        mppi_samples: int = 32,
        mppi_horizon: int = 30,
        collision_penalty: float = 20.0,
        timeout_penalty: float = 4.0,
        cruise_mode: str = "goal_hysteresis",
    ) -> None:
        super().__init__()
        self.profile = profile
        self.scenario_mode = scenario_mode
        self.base_seed = int(seed)
        self.duration = float(duration)
        self.route_length = float(route_length)
        self.mppi_samples = int(mppi_samples)
        self.mppi_horizon = int(mppi_horizon)
        self.collision_penalty = float(collision_penalty)
        self.timeout_penalty = float(timeout_penalty)
        self.cruise_mode = str(cruise_mode)
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

    def _setup_episode(self, config: EpisodeConfig, episode_seed: int) -> None:
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
        self.model, _ = build_scene(
            with_lidar=True,
            dynamic_obstacles=self.obstacle_specs,
            lidar_ray_count=config.lidar_ray_count,
            lidar_fov=config.lidar_fov,
            lidar_max_range=config.lidar_max_range,
            world_length=config.route_length,
        )
        self.data, self.gait = initialize(self.model)
        self.footprint = calibrate_robot_footprint(self.model, self.data)
        self.obstacle_bodies = [
            DynamicObstacleBody(self.model, trajectory, index=index)
            for index, trajectory in enumerate(self.trajectories)
        ]
        obstacle_geom_ids = tuple(
            int(self.model.geom(
                "moving_obstacle_geom"
                if index == 0
                else f"moving_obstacle_{index:02d}_geom"
            ).id)
            for index in range(len(self.obstacle_specs))
        )
        self.obstacle_geom_ids = obstacle_geom_ids
        self.lidar = SimulatedLidar(
            self.model,
            ray_count=config.lidar_ray_count,
            max_range=config.lidar_max_range,
            fov=config.lidar_fov,
        )
        self.tracker = MultiTargetTracker(
            obstacle_half_length=max(obstacle.half_length for obstacle in self.obstacle_specs),
            obstacle_half_width=max(obstacle.half_width for obstacle in self.obstacle_specs),
            prediction_position_uncertainty=config.obstacle_position_uncertainty,
            prediction_velocity_uncertainty=config.obstacle_velocity_uncertainty,
        )
        self.decision = DecisionLayer(
            enter_safe_distance=self.tracker.effective_safe_distance,
            exit_safe_distance=0.95,
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
        self.global_path.plan(
            self.data.qpos[:2],
            self.goal_world,
            obstacles=self.obstacle_specs,
            robot_radius=config.global_path_robot_radius,
            obstacle_margin=config.global_path_obstacle_margin,
        )
        self.last_global_replan = 0.0
        self.sim_dt = self.model.opt.timestep * 4.0
        self.yaw = 0.0
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
        self.proxy_collision = False
        self.physical_contact = False
        self.near_miss = False
        self.reached_goal = False
        self.path_length = 0.0
        # Put the MuJoCo obstacle geoms at t=0 before collecting both
        # geometric diagnostics and physical contact telemetry.
        self._update_bodies(0.0)
        initial_clearances = [
            collision_clearance(self.data.qpos[:2], state[0], obstacle)
            for state, obstacle in zip(self._current_states(), self.obstacle_specs)
            if state[2]
        ]
        self.min_clearance = min(initial_clearances) if initial_clearances else float("inf")
        initial_proxy_clearances = [
            bounding_circle_clearance(self.data.qpos[:2], state[0], obstacle)
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
        if self.elapsed - self.last_global_replan >= self.config.global_path_replan_period - 1e-9:
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
        self.active_mode = mode
        return command

    def _advance(self, command: np.ndarray) -> None:
        self._require_ready()
        assert self.config is not None
        assert self.sdk is not None
        for _ in range(self.config.policy_period):
            if self.collision or self.reached_goal or self.elapsed >= self.config.duration:
                break
            self._update_bodies(self.elapsed)
            previous_xy = self.data.qpos[:2].copy()
            actual_command = self.sdk.move(command, self.sim_dt)
            self.yaw = advance(self.model, self.data, self.gait, self.yaw, actual_command, self.sim_dt)
            self.elapsed += self.sim_dt
            self._update_bodies(self.elapsed)
            robot_xy = self.data.qpos[:2].copy()
            next_clearances = [
                collision_clearance(robot_xy, state[0], obstacle)
                for state, obstacle in zip(self._current_states(), self.obstacle_specs)
                if state[2]
            ]
            next_proxy_clearances = [
                bounding_circle_clearance(robot_xy, state[0], obstacle)
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
        )
        self._setup_episode(config, episode_seed)
        observation = self._observe()
        return observation, {"episode_seed": episode_seed, "config": config}

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
        }
        if terminated or truncated:
            info["success"] = goal_is_valid and not self.collision
        return observation, reward, terminated, truncated, info

    def close(self) -> None:
        self.model = None
        self.data = None
