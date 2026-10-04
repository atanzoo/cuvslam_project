#!/usr/bin/env python3
"""Minimal D1 Edu obstacle-avoidance visualization.

This is intentionally a planning-layer demo. It can build either a static
box or a free-joint box for the higher-level LiDAR/decision example. The robot
pose and gait are visualized kinematically because the current D1 Edu gait
layer is not yet a fully stabilized torque-level controller.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import time

import mujoco
import mujoco.viewer
import numpy as np

from run_d1_edu_mppi_velocity_viewer import GaitLayer


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models" / "d1_edu" / "d1_edu.xml"
NOMINAL = np.array([0.0, 0.65, -1.25] * 4)


@dataclass(frozen=True)
class Obstacle:
    x: float = 1.25
    y: float = 0.0
    half_length: float = 0.24
    half_width: float = 0.40
    height: float = 0.30
    # ``box`` preserves the historical model. ``person`` is a tall
    # cylindrical pedestrian proxy used by the dynamic multi-target scene.
    kind: str = "box"


class MinimalAvoider:
    """A small reactive policy: shift left, pass, then return to center."""

    def __init__(self, obstacle: Obstacle):
        self.obstacle = obstacle
        self.state = "CRUISE"
        self.target_y = 0.0
        self.trigger_distance = 0.90
        self.pass_distance = obstacle.half_length + 0.45
        self.safe_y = obstacle.half_width + 0.25 + 0.12

    def command(self, x: float, y: float) -> np.ndarray:
        dx = self.obstacle.x - x

        if self.state == "CRUISE" and 0.0 < dx < self.trigger_distance:
            self.state = "SHIFT_LEFT"
            self.target_y = self.obstacle.y + self.safe_y
        elif self.state == "SHIFT_LEFT" and dx < -self.pass_distance:
            self.state = "RETURN_CENTER"
            self.target_y = 0.0
        elif self.state == "RETURN_CENTER" and abs(y - self.target_y) < 0.04:
            self.state = "CRUISE_AFTER_OBSTACLE"

        # Proportional lateral motion gives the obstacle-avoidance behavior.
        vy = np.clip(1.35 * (self.target_y - y), -0.34, 0.34)
        vx = 0.20 if self.state in {"SHIFT_LEFT", "RETURN_CENTER"} else 0.22
        return np.array([vx, vy, 0.0], dtype=float)


def _yaw_pitch_quat(yaw: float, pitch: float = np.pi / 2.0) -> np.ndarray:
    """Quaternion that points a site's local +Z axis into the XY plane."""
    q_yaw = np.array([np.cos(yaw / 2.0), 0.0, 0.0, np.sin(yaw / 2.0)])
    q_pitch = np.array([np.cos(pitch / 2.0), 0.0, np.sin(pitch / 2.0), 0.0])
    w1, x1, y1, z1 = q_yaw
    w2, x2, y2, z2 = q_pitch
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def build_scene(
    with_lidar: bool = False,
    dynamic_obstacle: bool = False,
    dynamic_obstacle_config: Obstacle | None = None,
    dynamic_obstacles: list[Obstacle] | None = None,
    static_obstacles: list[Obstacle] | None = None,
    lidar_ray_count: int = 61,
    lidar_fov: float = np.pi,
    lidar_max_range: float = 4.0,
    world_length: float | None = None,
    model_path: str | Path | None = None,
) -> tuple[mujoco.MjModel, Obstacle]:
    """Load the D1 model and append visible, collidable obstacle geometry.

    When ``with_lidar`` is true, 61 horizontal MuJoCo rangefinder sensors are
    added as a 180-degree, 3-degree-resolution 2D LiDAR mounted on the base.
    When ``dynamic_obstacle`` is true, the box is attached to a free joint so a
    caller can move it by updating its seven free-joint qpos values.
    Static obstacles are world-attached boxes, used for known occupied map cells.
    """
    if dynamic_obstacles is not None:
        if not dynamic_obstacles:
            raise ValueError("dynamic_obstacles cannot be empty")
        obstacle = dynamic_obstacles[0]
        dynamic_specs = dynamic_obstacles
    elif dynamic_obstacle:
        obstacle = dynamic_obstacle_config or Obstacle(
            y=-1.30,
            half_length=0.25,
            half_width=0.25,
            height=0.25,
        )
        dynamic_specs = [obstacle]
    else:
        obstacle = Obstacle()
        dynamic_specs = []
    selected_model_path = MODEL_PATH if model_path is None else Path(model_path)
    if not selected_model_path.is_file():
        raise FileNotFoundError(f"robot model not found: {selected_model_path}")
    spec = mujoco.MjSpec.from_file(str(selected_model_path))
    if world_length is not None:
        floor = spec.geom("floor")
        if floor is None:
            floor = spec.worldbody.add_geom()
            floor.name = "floor"
            floor.type = mujoco.mjtGeom.mjGEOM_PLANE
            floor.pos[:] = [0.0, 0.0, 0.0]
            floor.size[:2] = [max(20.0, world_length + 5.0)] * 2
            floor.size[2] = 0.1
            floor.contype = 1
            floor.conaffinity = 1
        floor.size[:2] = [max(20.0, world_length + 5.0)] * 2
        floor.size[2] = max(float(floor.size[2]), 0.1)

    if dynamic_specs:
        for index, obstacle_spec in enumerate(dynamic_specs):
            moving_body = spec.worldbody.add_body()
            moving_body.name = "moving_obstacle" if index == 0 else f"moving_obstacle_{index:02d}"
            moving_body.pos[:] = [obstacle_spec.x, obstacle_spec.y, obstacle_spec.height]
            freejoint = moving_body.add_freejoint()
            freejoint.name = (
                "moving_obstacle_freejoint"
                if index == 0
                else f"moving_obstacle_{index:02d}_freejoint"
            )

            box = moving_body.add_geom()
            box.name = "moving_obstacle_geom" if index == 0 else f"moving_obstacle_{index:02d}_geom"
            is_person = obstacle_spec.kind == "person"
            box.type = (
                mujoco.mjtGeom.mjGEOM_CYLINDER
                if is_person else mujoco.mjtGeom.mjGEOM_BOX
            )
            box.pos[:] = [0.0, 0.0, 0.0]
            if is_person:
                person_radius = max(obstacle_spec.half_length, obstacle_spec.half_width)
                box.size[:2] = [person_radius, obstacle_spec.height]
                box.rgba[:] = [0.20, 0.78, 0.42, 1.0]
            else:
                box.size[:] = [
                    obstacle_spec.half_length,
                    obstacle_spec.half_width,
                    obstacle_spec.height,
                ]
                box.rgba[:] = [0.88, 0.16, 0.08, 1.0]
            box.contype = 1
            box.conaffinity = 1
            box.friction[:] = [1.0, 0.01, 0.01]
            box.solref[:] = [0.005, 1.0]
            box.solimp[:3] = [0.99, 0.999, 1e-5]
    else:
        box = spec.worldbody.add_geom()
        box.name = "avoidance_obstacle"
        box.type = mujoco.mjtGeom.mjGEOM_BOX
        box.pos[:] = [obstacle.x, obstacle.y, obstacle.height]
        box.size[:] = [obstacle.half_length, obstacle.half_width, obstacle.height]
        box.rgba[:] = [0.88, 0.16, 0.08, 1.0]
        box.contype = 1
        box.conaffinity = 1
        box.friction[:] = [1.0, 0.01, 0.01]
        box.solref[:] = [0.005, 1.0]
        box.solimp[:3] = [0.99, 0.999, 1e-5]

        # A transparent warning region makes the trigger distance visible.
        warning = spec.worldbody.add_geom()
        warning.name = "avoidance_warning_region"
        warning.type = mujoco.mjtGeom.mjGEOM_BOX
        warning.pos[:] = [obstacle.x - 0.45, obstacle.y, 0.006]
        warning.size[:] = [0.45, obstacle.half_width + 0.25, 0.006]
        warning.rgba[:] = [1.0, 0.65, 0.05, 0.16]
        warning.contype = 0
        warning.conaffinity = 0

    for index, obstacle_spec in enumerate(static_obstacles or []):
        wall = spec.worldbody.add_geom()
        wall.name = f"candidate_map_wall_{index:04d}_geom"
        wall.type = mujoco.mjtGeom.mjGEOM_BOX
        wall.pos[:] = [obstacle_spec.x, obstacle_spec.y, obstacle_spec.height]
        wall.size[:] = [
            obstacle_spec.half_length,
            obstacle_spec.half_width,
            obstacle_spec.height,
        ]
        wall.rgba[:] = [0.20, 0.22, 0.26, 1.0]
        wall.contype = 1
        wall.conaffinity = 1
        wall.friction[:] = [1.0, 0.01, 0.01]
        wall.solref[:] = [0.005, 1.0]
        wall.solimp[:3] = [0.99, 0.999, 1e-5]

    if with_lidar:
        base = spec.worldbody.first_body()
        if lidar_ray_count <= 0 or lidar_max_range <= 0.0:
            raise ValueError("LiDAR ray count and range must be positive")
        endpoint = not np.isclose(lidar_fov, 2.0 * np.pi)
        for index, angle in enumerate(np.linspace(
            -lidar_fov / 2.0,
            lidar_fov / 2.0,
            lidar_ray_count,
            endpoint=endpoint,
        )):
            site = base.add_site()
            site.name = f"lidar_ray_{index:02d}"
            site.type = mujoco.mjtGeom.mjGEOM_SPHERE
            site.pos[:] = [0.0, 0.0, 0.08]
            site.size[:1] = [0.004]
            site.rgba[:] = [0.2, 0.8, 1.0, 0.0]
            site.quat[:] = _yaw_pitch_quat(float(angle))

            sensor = spec.add_sensor()
            sensor.name = f"lidar_range_{index:02d}"
            sensor.type = mujoco.mjtSensor.mjSENS_RANGEFINDER
            sensor.objtype = mujoco.mjtObj.mjOBJ_SITE
            sensor.objname = site.name
            sensor.intprm[0] = 1
            sensor.cutoff = lidar_max_range

    return spec.compile(), obstacle


def initialize(model: mujoco.MjModel) -> tuple[mujoco.MjData, GaitLayer]:
    data = mujoco.MjData(model)
    gait = GaitLayer(model)
    data.qpos[2] = 0.335
    data.qpos[gait.joint_qpos] = NOMINAL
    mujoco.mj_forward(model, data)
    return data, gait


def step_kinematic(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    gait: GaitLayer,
    avoider: MinimalAvoider,
    dt: float,
) -> tuple[np.ndarray, float]:
    x, y = float(data.qpos[0]), float(data.qpos[1])
    command = avoider.command(x, y)

    # The current demo uses a world-frame velocity command for clarity.
    data.qpos[0] += command[0] * dt
    data.qpos[1] += command[1] * dt
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    gait.phase = (gait.phase + 0.012 * (dt / model.opt.timestep)) % (2.0 * np.pi)
    data.qpos[gait.joint_qpos] = gait.target(command)
    mujoco.mj_forward(model, data)
    return command, float(data.qpos[0])


def clearance(x: float, y: float, obstacle: Obstacle) -> float:
    """Approximate distance from the robot center to the obstacle boundary."""
    dx = max(abs(x - obstacle.x) - obstacle.half_length, 0.0)
    dy = max(abs(y - obstacle.y) - obstacle.half_width, 0.0)
    return float(np.hypot(dx, dy) - 0.25)


def run_headless(duration: float) -> None:
    model, obstacle = build_scene()
    data, gait = initialize(model)
    avoider = MinimalAvoider(obstacle)
    dt = model.opt.timestep * 4.0
    elapsed = 0.0
    min_clearance = float("inf")
    last_state = avoider.state

    while elapsed < duration:
        command, _ = step_kinematic(model, data, gait, avoider, dt)
        min_clearance = min(min_clearance, clearance(data.qpos[0], data.qpos[1], obstacle))
        if avoider.state != last_state:
            print(
                f"t={elapsed:5.2f}s state={avoider.state:18s} "
                f"pose=({data.qpos[0]:+.2f}, {data.qpos[1]:+.2f}) "
                f"cmd=({command[0]:+.2f}, {command[1]:+.2f}, {command[2]:+.2f})"
            )
            last_state = avoider.state
        elapsed += dt

    print(
        f"final_pose=({data.qpos[0]:+.3f}, {data.qpos[1]:+.3f}) "
        f"min_clearance={min_clearance:+.3f}m"
    )
    if min_clearance > 0.0:
        print("status=PASS: minimal obstacle avoidance cleared the box")
    else:
        print("status=FAIL: path entered the approximate safety radius")


def run_viewer(duration: float) -> None:
    model, obstacle = build_scene()
    data, gait = initialize(model)
    avoider = MinimalAvoider(obstacle)
    dt = model.opt.timestep * 4.0
    elapsed = 0.0
    last_state = avoider.state

    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Free camera: it stays where it is until the user moves it.
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        viewer.cam.trackbodyid = -1
        viewer.cam.distance = 3.1
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -18
        viewer.cam.lookat[:] = [1.15, 0.0, 0.22]

        while viewer.is_running() and elapsed < duration:
            command, _ = step_kinematic(model, data, gait, avoider, dt)
            if avoider.state != last_state:
                print(
                    f"t={elapsed:5.2f}s state={avoider.state} "
                    f"pose=({data.qpos[0]:+.2f}, {data.qpos[1]:+.2f}) "
                    f"command=(vx={command[0]:+.2f}, vy={command[1]:+.2f})"
                )
                last_state = avoider.state
            viewer.sync()
            elapsed += dt
            time.sleep(dt)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true", help="run the path test without opening a window")
    parser.add_argument("--duration", type=float, default=16.0)
    args = parser.parse_args()
    if args.headless:
        run_headless(args.duration)
    else:
        run_viewer(args.duration)


if __name__ == "__main__":
    main()
