#!/usr/bin/env python3
"""Minimal velocity-space MPPI obstacle avoidance for the D1 Edu MuJoCo model.

The planner samples body-frame [vx, vy, yaw_rate] sequences, rolls them out
with a kinematic base model, and adds clearance/collision costs from LiDAR
points and an optional constant-velocity dynamic-obstacle prediction. The
selected command is then rendered with the existing D1 gait layer. This is an
MPPI planning-layer prototype, not yet a 12-joint whole-body torque controller.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import time

import mujoco
import mujoco.viewer
import numpy as np

from run_d1_edu_obstacle_avoidance_viewer import (
    NOMINAL,
    Obstacle,
    build_scene,
)
from run_d1_edu_mppi_velocity_viewer import GaitLayer


ROOT = Path(__file__).resolve().parents[1]


def yaw_to_quat(yaw: float) -> np.ndarray:
    return np.array([np.cos(yaw / 2.0), 0.0, 0.0, np.sin(yaw / 2.0)])


class ObstacleMPPI:
    """Sampling-based local planner with a box-clearance cost."""

    def __init__(
        self,
        plan_dt: float = 0.10,
        horizon: int = 60,
        samples: int = 256,
        dynamic_obstacle_radius: float = float(np.hypot(0.25, 0.25)),
        rng_seed: int = 23,
        clearance_uncertainty: float = 0.0,
        dynamic_position_uncertainty: float = 0.0,
        dynamic_velocity_uncertainty: float = 0.0,
        command_delay: float = 0.0,
        robot_radius: float = 0.25,
    ):
        self.plan_dt = plan_dt
        self.horizon = horizon
        self.samples = samples
        self.temperature = 1.8
        self.rng = np.random.default_rng(rng_seed)
        # Keep yaw fixed in this first LiDAR test so the scan frame and the
        # lateral avoidance behavior are easy to inspect.
        self.noise_std = np.array([0.07, 0.18, 0.0])
        self.lower = np.array([0.04, -0.34, 0.0])
        self.upper = np.array([0.36, 0.34, 0.0])
        self.mean = np.zeros((self.horizon, 3), dtype=float)
        self.mean[:, 0] = 0.22
        self.goal = np.array([3.20, 0.0])
        if robot_radius <= 0.0:
            raise ValueError("robot_radius must be positive")
        self.robot_radius = float(robot_radius)
        self.safety_margin = 0.30
        # A dynamic obstacle is represented by its center/observed centroid.
        # The bounding radius covers the D1-side footprint of the 0.5 x 0.5 m
        # test box; the safety margin is applied outside this radius.
        self.dynamic_obstacle_radius = dynamic_obstacle_radius
        self.dynamic_safety_margin = 0.10
        # These terms turn point-estimate geometry into a conservative tube:
        # sensor/range uncertainty, obstacle prediction error, and motion
        # during the command interface delay are all charged in the rollout.
        self.clearance_uncertainty = max(0.0, float(clearance_uncertainty))
        self.dynamic_position_uncertainty = max(0.0, float(dynamic_position_uncertainty))
        self.dynamic_velocity_uncertainty = max(0.0, float(dynamic_velocity_uncertainty))
        self.command_delay = max(0.0, float(command_delay))
        self.update_count = 0
        self.last_costs = None
        self.avoid_side = None

    def _rollout_cost(
        self,
        state: np.ndarray,
        commands: np.ndarray,
        lidar_points: np.ndarray,
        goal: np.ndarray,
        dynamic_obstacle_position_local: np.ndarray | None = None,
        dynamic_obstacle_velocity_local: np.ndarray | None = None,
        current_time: float = 0.0,
        dynamic_obstacles_local: list[tuple[np.ndarray, np.ndarray, float]] | None = None,
        dynamic_trajectories_local: list[tuple[np.ndarray, np.ndarray, float]] | None = None,
        trajectory_sample_dt: float = 0.25,
        terminal_goal: np.ndarray | None = None,
        reference_speed: float | None = None,
        terminal_risk: float = 0.0,
    ) -> np.ndarray:
        """Evaluate sampled sequences in one fixed robot-local frame.

        ``dynamic_obstacle_position_local`` and
        ``dynamic_obstacle_velocity_local`` are both expressed at
        ``current_time`` in the robot frame at the beginning of this rollout.
        The robot state is rolled forward in that same fixed local frame, so
        the dynamic obstacle is predicted independently for every horizon step.
        """
        n = commands.shape[0]
        x = np.full(n, state[0], dtype=float)
        y = np.full(n, state[1], dtype=float)
        yaw = np.full(n, state[2], dtype=float)
        total = np.zeros(n, dtype=float)
        previous = np.zeros((n, 3), dtype=float)
        side_target = None
        if self.avoid_side is not None and lidar_points.size:
            observed_half_width = float(np.max(np.abs(lidar_points[:, 1])))
            side_target = self.avoid_side * min(0.95, max(0.78, observed_half_width + 0.35))

        for t in range(self.horizon):
            command = commands[:, t, :]
            vx, vy, yaw_rate = command.T
            c, s = np.cos(yaw), np.sin(yaw)
            x += (c * vx - s * vy) * self.plan_dt
            y += (s * vx + c * vy) * self.plan_dt
            yaw += yaw_rate * self.plan_dt
            rollout_elapsed = (t + 1) * self.plan_dt
            robot_speed = np.hypot(vx, vy)
            delayed_motion_margin = self.command_delay * robot_speed

            if lidar_points.size:
                distance = np.hypot(
                    x[:, None] - lidar_points[None, :, 0],
                    y[:, None] - lidar_points[None, :, 1],
                )
                clearance = distance.min(axis=1) - self.robot_radius
                effective_margin = (
                    self.safety_margin
                    + self.clearance_uncertainty
                    + delayed_motion_margin
                )
                risk = np.maximum(effective_margin - clearance, 0.0)
                total += 220.0 * risk**2
                total += 1200.0 * (clearance < 0.0)

            dynamic_predictions = list(dynamic_obstacles_local or [])
            if dynamic_obstacle_position_local is not None:
                dynamic_velocity = (
                    np.zeros(2, dtype=float)
                    if dynamic_obstacle_velocity_local is None
                    else np.asarray(dynamic_obstacle_velocity_local, dtype=float)
                )
                dynamic_predictions.append((
                    np.asarray(dynamic_obstacle_position_local, dtype=float),
                    dynamic_velocity,
                    self.dynamic_obstacle_radius,
                ))
            if dynamic_predictions:
                rollout_time = current_time + rollout_elapsed
                for dynamic_position, dynamic_velocity, dynamic_radius in dynamic_predictions:
                    predicted_obstacle = (
                        np.asarray(dynamic_position, dtype=float)
                        + np.asarray(dynamic_velocity, dtype=float) * (rollout_time - current_time)
                    )
                    dynamic_distance = np.hypot(
                        x - predicted_obstacle[0],
                        y - predicted_obstacle[1],
                    )
                    dynamic_clearance = (
                        dynamic_distance
                        - self.robot_radius
                        - float(dynamic_radius)
                    )
                    dynamic_margin = (
                        self.dynamic_safety_margin
                        + self.dynamic_position_uncertainty
                        + self.dynamic_velocity_uncertainty * rollout_elapsed
                        + delayed_motion_margin
                    )
                    dynamic_risk = np.maximum(
                        dynamic_margin - dynamic_clearance,
                        0.0,
                    )
                    total += 260.0 * dynamic_risk**2
                    total += 1500.0 * (dynamic_clearance < 0.0)

            # Ranked tracker trajectories are already expressed in the fixed
            # local frame at the start of this rollout.  Interpolate them at
            # every MPPI step; after the prediction horizon, continue with the
            # last supplied velocity.  The legacy constant-velocity list above
            # remains available for callers that do not provide trajectories.
            if dynamic_trajectories_local:
                rollout_elapsed = (t + 1) * self.plan_dt
                for item in dynamic_trajectories_local:
                    if len(item) == 3:
                        trajectory, dynamic_velocity, dynamic_radius = item
                        trajectory_uncertainty = 0.0
                    else:
                        trajectory, dynamic_velocity, dynamic_radius, trajectory_uncertainty = item
                    trajectory = np.asarray(trajectory, dtype=float).reshape(-1, 2)
                    if trajectory.size == 0:
                        continue
                    trajectory_times = np.arange(len(trajectory), dtype=float) * trajectory_sample_dt
                    uncertainty_values = np.asarray(
                        trajectory_uncertainty,
                        dtype=float,
                    ).reshape(-1)
                    if rollout_elapsed <= trajectory_times[-1]:
                        predicted_obstacle = np.array([
                            np.interp(rollout_elapsed, trajectory_times, trajectory[:, axis])
                            for axis in range(2)
                        ])
                    else:
                        predicted_obstacle = (
                            trajectory[-1]
                            + np.asarray(dynamic_velocity, dtype=float)
                            * (rollout_elapsed - trajectory_times[-1])
                        )
                    if uncertainty_values.size == 0:
                        predicted_uncertainty = 0.0
                    elif uncertainty_values.size == 1:
                        predicted_uncertainty = float(uncertainty_values[0])
                    else:
                        uncertainty_times = np.linspace(
                            0.0,
                            trajectory_times[-1],
                            uncertainty_values.size,
                        )
                        predicted_uncertainty = float(np.interp(
                            rollout_elapsed,
                            uncertainty_times,
                            uncertainty_values,
                        ))
                    dynamic_distance = np.hypot(
                        x - predicted_obstacle[0],
                        y - predicted_obstacle[1],
                    )
                    dynamic_clearance = (
                        dynamic_distance
                        - self.robot_radius
                        - float(dynamic_radius)
                        - predicted_uncertainty
                    )
                    dynamic_margin = (
                        self.dynamic_safety_margin
                        + self.dynamic_position_uncertainty
                        + self.dynamic_velocity_uncertainty * rollout_elapsed
                        + delayed_motion_margin
                    )
                    dynamic_risk = np.maximum(
                        dynamic_margin - dynamic_clearance,
                        0.0,
                    )
                    total += 260.0 * dynamic_risk**2
                    total += 1500.0 * (dynamic_clearance < 0.0)

            # Keep the route goal-directed, upright in heading, and smooth.
            total += 0.12 * (goal[0] - x) ** 2
            total += 0.70 * y**2
            # Break the perfectly symmetric left/right choice only for this
            # deterministic demo; the real planner can remove this bias.
            total += 0.90 * np.maximum(-y, 0.0) ** 2
            if self.avoid_side is not None and (
                lidar_points.size
                or dynamic_obstacle_position_local is not None
                or dynamic_obstacles_local
                or dynamic_trajectories_local
            ):
                # Commit to the selected passing side early enough to absorb
                # the simulated SDK delay and acceleration limit.
                total += 12.0 * np.maximum(-self.avoid_side * y, 0.0) ** 2
                if side_target is not None:
                    total += 18.00 * (side_target - y) ** 2
            total += 0.20 * yaw**2
            total += 0.45 * vy**2 + 0.30 * yaw_rate**2
            if reference_speed is not None:
                total += 0.50 * (vx - float(reference_speed)) ** 2
            total += 0.015 * vx**2
            total += 0.12 * np.sum((command - previous) ** 2, axis=1)
            previous = command

        terminal = goal if terminal_goal is None else np.asarray(terminal_goal, dtype=float)
        terminal_weight = 0.80 * (1.0 - 0.40 * float(np.clip(terminal_risk, 0.0, 1.0)))
        total += terminal_weight * (terminal[0] - x) ** 2
        total += 1.80 * (terminal[1] - y) ** 2
        return total

    def update(
        self,
        state: np.ndarray,
        lidar_points: np.ndarray,
        goal: np.ndarray,
        dynamic_obstacle_position_local: np.ndarray | None = None,
        dynamic_obstacle_velocity_local: np.ndarray | None = None,
        current_time: float = 0.0,
        dynamic_obstacles_local: list[tuple[np.ndarray, np.ndarray, float]] | None = None,
        dynamic_trajectories_local: list[tuple[np.ndarray, np.ndarray, float]] | None = None,
        trajectory_sample_dt: float = 0.25,
        terminal_goal: np.ndarray | None = None,
        reference_speed: float | None = None,
        terminal_risk: float = 0.0,
    ) -> np.ndarray:
        """Select a local velocity command using current and predicted obstacles."""
        self.goal = goal.copy()
        if (
            (
                lidar_points.size
                or dynamic_obstacle_position_local is not None
                or dynamic_obstacles_local
                or dynamic_trajectories_local
            )
            and self.avoid_side is None
        ):
            # The first scan is symmetric in this test scene.  Commit to the
            # positive lateral side once, so replanning does not alternate
            # between two equally good sides at every scan update.
            self.avoid_side = 1.0
        if not lidar_points.size:
            # Once the obstacle leaves the scan, warm-start the lateral
            # sequence toward the goal instead of retaining the avoidance
            # offset indefinitely.
            desired_vy = np.clip(goal[1] / (self.horizon * self.plan_dt), -0.25, 0.25)
            self.mean[:, 1] = 0.30 * self.mean[:, 1] + 0.70 * desired_vy
        noise = self.rng.normal(
            0.0, self.noise_std, size=(self.samples, self.horizon, 3)
        )
        commands = np.clip(self.mean[None, :, :] + noise, self.lower, self.upper)
        costs = self._rollout_cost(
            state,
            commands,
            lidar_points,
            goal,
            dynamic_obstacle_position_local,
            dynamic_obstacle_velocity_local,
            current_time,
            dynamic_obstacles_local,
            dynamic_trajectories_local,
            trajectory_sample_dt,
            terminal_goal,
            reference_speed,
            terminal_risk,
        )
        shifted = costs - costs.min()
        weights = np.exp(-shifted / self.temperature)
        weights /= weights.sum() + 1e-12

        self.mean += np.sum(weights[:, None, None] * noise, axis=0)
        self.mean = np.clip(self.mean, self.lower, self.upper)
        command = self.mean[0].copy()
        self.mean[:-1] = self.mean[1:]
        self.mean[-1] = self.mean[-2]
        self.update_count += 1
        self.last_costs = (float(costs.min()), float(costs.mean()))
        return command


class SimulatedLidar:
    """Read the MuJoCo rangefinders as a horizontal 2D LiDAR scan."""

    def __init__(
        self,
        model: mujoco.MjModel,
        ray_count: int = 61,
        max_range: float = 4.0,
        fov: float = np.pi,
    ):
        self.model = model
        endpoint = not np.isclose(fov, 2.0 * np.pi)
        self.angles = np.linspace(-fov / 2.0, fov / 2.0, ray_count, endpoint=endpoint)
        self.max_range = max_range
        self.sensor_adr = np.array([
            model.sensor_adr[model.sensor(f"lidar_range_{i:02d}").id]
            for i in range(ray_count)
        ])

    def scan(self, data: mujoco.MjData) -> np.ndarray:
        values = data.sensordata[self.sensor_adr].copy()
        return np.where(values > 0.0, np.minimum(values, self.max_range), self.max_range)

    def hit_ray_indices(self, scan: np.ndarray) -> np.ndarray:
        """Return hit indices so the tracker can preserve LiDAR ray order."""
        return np.flatnonzero(scan < self.max_range - 1e-3)

    def point_cloud(self, scan: np.ndarray) -> np.ndarray:
        hit = scan < self.max_range - 1e-3
        return np.column_stack((
            scan[hit] * np.cos(self.angles[hit]),
            scan[hit] * np.sin(self.angles[hit]),
        ))


def initialize(model: mujoco.MjModel) -> tuple[mujoco.MjData, GaitLayer]:
    data = mujoco.MjData(model)
    gait = GaitLayer(model)
    data.qpos[2] = 0.335
    data.qpos[gait.joint_qpos] = NOMINAL
    mujoco.mj_forward(model, data)
    return data, gait


def local_goal(
    data: mujoco.MjData,
    yaw: float,
    goal_world: np.ndarray | None = None,
) -> np.ndarray:
    """Express the fixed world goal in the current body frame."""
    target = np.array([3.20, 0.0]) if goal_world is None else np.asarray(goal_world)
    delta = target - data.qpos[:2]
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([c * delta[0] + s * delta[1], -s * delta[0] + c * delta[1]])


def ground_truth_clearance(x: float, y: float, obstacle: Obstacle) -> float:
    """Evaluation-only clearance; this is never passed into the planner."""
    dx = max(abs(x - obstacle.x) - obstacle.half_length, 0.0)
    dy = max(abs(y - obstacle.y) - obstacle.half_width, 0.0)
    distance = np.hypot(dx, dy)
    if abs(x - obstacle.x) <= obstacle.half_length and abs(y - obstacle.y) <= obstacle.half_width:
        distance = 0.0
    return float(distance - 0.25)


def advance(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    gait: GaitLayer,
    yaw: float,
    command: np.ndarray,
    dt: float,
) -> float:
    """Apply one selected body command to the kinematic visualization."""
    vx, vy, yaw_rate = command
    c, s = np.cos(yaw), np.sin(yaw)
    data.qpos[0] += (c * vx - s * vy) * dt
    data.qpos[1] += (s * vx + c * vy) * dt
    yaw += yaw_rate * dt
    data.qpos[3:7] = yaw_to_quat(yaw)
    gait.phase = (gait.phase + 0.012 * (dt / model.opt.timestep)) % (2.0 * np.pi)
    data.qpos[gait.joint_qpos] = gait.target(command)
    mujoco.mj_forward(model, data)
    return yaw


def run_headless(duration: float) -> None:
    model, obstacle = build_scene(with_lidar=True)
    data, gait = initialize(model)
    lidar = SimulatedLidar(model)
    planner = ObstacleMPPI()
    sim_dt = model.opt.timestep * 4.0
    policy_period = 12
    command = np.array([0.22, 0.0, 0.0])
    yaw = 0.0
    elapsed = 0.0
    step = 0
    min_clearance = float("inf")
    last_report = -1.0
    points = np.empty((0, 2))

    while elapsed < duration:
        if step % policy_period == 0:
            # LiDAR points are expressed in the current body frame, so every
            # local rollout starts at the current body-frame origin.
            state = np.zeros(3)
            scan = lidar.scan(data)
            points = lidar.point_cloud(scan)
            command = planner.update(state, points, local_goal(data, yaw))
        yaw = advance(model, data, gait, yaw, command, sim_dt)
        min_clearance = min(min_clearance, ground_truth_clearance(data.qpos[0], data.qpos[1], obstacle))
        if data.qpos[0] > last_report + 0.5:
            print(
                f"x={data.qpos[0]:+.2f} y={data.qpos[1]:+.2f} "
                f"lidar_hits={len(points):02d} "
                f"cmd=(vx={command[0]:+.2f}, vy={command[1]:+.2f}, "
                f"yaw_rate={command[2]:+.2f})"
            )
            last_report = float(data.qpos[0])
        elapsed += sim_dt
        step += 1

    print(
        f"planner_updates={planner.update_count} "
        f"final_pose=({data.qpos[0]:+.3f}, {data.qpos[1]:+.3f}) "
        f"min_clearance={min_clearance:+.3f}m"
    )
    if min_clearance > 0.0 and data.qpos[0] > obstacle.x + obstacle.half_length:
        print("status=PASS: MPPI selected a collision-free route around the obstacle")
    else:
        print("status=FAIL: MPPI route did not clear the obstacle")


def run_viewer(duration: float) -> None:
    model, obstacle = build_scene(with_lidar=True)
    data, gait = initialize(model)
    lidar = SimulatedLidar(model)
    planner = ObstacleMPPI()
    sim_dt = model.opt.timestep * 4.0
    policy_period = 12
    command = np.array([0.22, 0.0, 0.0])
    yaw = 0.0
    elapsed = 0.0
    step = 0
    last_report = -1.0
    points = np.empty((0, 2))

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        viewer.cam.trackbodyid = -1
        viewer.cam.distance = 3.1
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -18
        viewer.cam.lookat[:] = [1.15, 0.0, 0.22]

        while viewer.is_running() and elapsed < duration:
            if step % policy_period == 0:
                state = np.zeros(3)
                scan = lidar.scan(data)
                points = lidar.point_cloud(scan)
                command = planner.update(state, points, local_goal(data, yaw))
            yaw = advance(model, data, gait, yaw, command, sim_dt)
            if data.qpos[0] > last_report + 0.5:
                print(
                    f"x={data.qpos[0]:+.2f} y={data.qpos[1]:+.2f} "
                    f"lidar_hits={len(points):02d} "
                    f"command=(vx={command[0]:+.2f}, vy={command[1]:+.2f}, "
                    f"yaw_rate={command[2]:+.2f})"
                )
                last_report = float(data.qpos[0])
            viewer.sync()
            elapsed += sim_dt
            step += 1
            time.sleep(sim_dt)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration", type=float, default=18.0)
    args = parser.parse_args()
    if args.headless:
        run_headless(args.duration)
    else:
        run_viewer(args.duration)


if __name__ == "__main__":
    main()
