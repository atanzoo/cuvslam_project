#!/usr/bin/env python3
"""Velocity-level MPPI with a conservative D1 Edu diagonal gait layer."""

from pathlib import Path
import time

import mujoco
import mujoco.viewer
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models" / "d1_edu" / "d1_edu.xml"
NOMINAL = np.array([0.0, 0.65, -1.25] * 4)


class GaitLayer:
    def __init__(self, model: mujoco.MjModel):
        self.model = model
        # Select the twelve robot hinge joints explicitly.  This also keeps
        # the gait layer valid when a scene adds a freejoint obstacle body.
        hinge_ids = np.flatnonzero(model.jnt_type == mujoco.mjtJoint.mjJNT_HINGE)
        if len(hinge_ids) != 12:
            raise ValueError(f"expected 12 robot hinge joints, found {len(hinge_ids)}")
        self.joint_qpos = model.jnt_qposadr[hinge_ids]
        self.joint_qvel = model.jnt_dofadr[hinge_ids]
        self.phase = 0.0
        self.kp = 220.0
        self.kd = 8.0
        self.max_torque = 48.0

    def target(self, command: np.ndarray, phase: float | None = None) -> np.ndarray:
        vx, _, yaw_rate = command
        phase = self.phase if phase is None else phase
        q = NOMINAL.copy()
        stride = 0.12 * np.clip(abs(vx) / 0.35, 0.0, 1.0)
        lift = 0.075 * np.clip(abs(vx) / 0.35, 0.0, 1.0)
        turn_ratio = np.clip(abs(yaw_rate) / 0.6, 0.0, 1.0)
        turn = 0.12 * turn_ratio
        for leg, pair_phase in ((0, phase), (1, phase + np.pi), (2, phase + np.pi), (3, phase)):
            j = 3 * leg
            direction = 1.0 if vx >= 0.0 else -1.0
            # One cycle: stance from 0..pi, swing from pi..2pi.
            wrapped = pair_phase % (2.0 * np.pi)
            # Inside legs take shorter steps; outside legs take longer steps.
            left_side = 1.0 if leg in (0, 3) else -1.0
            stride_scale = 1.0 - 0.42 * np.sign(yaw_rate) * left_side * turn_ratio
            stride_leg = stride * np.clip(stride_scale, 0.55, 1.45)
            x_rel = direction * 0.5 * stride_leg * np.cos(wrapped)
            swing = max(0.0, -np.sin(wrapped))
            z_rel = -0.335 + lift * swing
            l1, l2 = 0.200, 0.21366
            cos_knee = np.clip((x_rel * x_rel + z_rel * z_rel - l1 * l1 - l2 * l2) / (2.0 * l1 * l2), -1.0, 1.0)
            knee = -np.arccos(cos_knee)
            phi = np.arctan2(x_rel, -z_rel)
            hip = phi - np.arctan2(l2 * np.sin(knee), l1 + l2 * np.cos(knee))
            q[j + 1] = hip
            q[j + 2] = knee
            q[j] = turn * left_side * np.sign(yaw_rate)
        return q

    def torque(self, data: mujoco.MjData, target: np.ndarray) -> np.ndarray:
        tau = self.kp * (target - data.qpos[self.joint_qpos]) - self.kd * data.qvel[self.joint_qvel]
        return np.clip(tau, -self.max_torque, self.max_torque)


class VelocityMPPI:
    def __init__(self, model: mujoco.MjModel, gait: GaitLayer):
        self.model = model
        self.gait = gait
        self.horizon = 12
        self.samples = 32
        self.temperature = 0.12
        self.rng = np.random.default_rng(11)
        self.noise_std = np.array([0.10, 0.03, 0.16])
        self.mean = np.zeros((self.horizon, 3))
        # A nonzero warm start makes forward progress visible within the
        # short local horizon; MPPI then refines this command online.
        self.mean[:, 0] = 0.22

    def rollout_cost(self, state: mujoco.MjData, commands: np.ndarray, goal_x: float, phase: float) -> float:
        data = mujoco.MjData(self.model)
        mujoco.mj_copyData(data, self.model, state)
        total = 0.0
        physics_steps_per_sample = 5
        for step, command in enumerate(commands):
            for substep in range(physics_steps_per_sample):
                target = self.gait.target(command, phase + 0.012 * (step * physics_steps_per_sample + substep))
                data.ctrl[:] = self.gait.torque(data, target)
                mujoco.mj_step(self.model, data)
            x, y, z = data.qpos[:3]
            quat = data.qpos[3:7]
            total += 2.5 * (goal_x - x) ** 2 + 2.0 * y**2
            total += 18.0 * float(np.sum(quat[1:] ** 2))
            total += 8.0 * (z - 0.335) ** 2
            total += 0.04 * float(np.sum(command**2))
            if z < 0.20 or z > 0.70:
                total += 200.0
        return float(total)

    def update(self, state: mujoco.MjData, goal_x: float) -> np.ndarray:
        noise = self.rng.normal(0.0, self.noise_std, size=(self.samples, self.horizon, 3))
        commands = self.mean[None, :, :] + noise
        commands[:, :, 0] = np.clip(commands[:, :, 0], -0.30, 0.45)
        commands[:, :, 1] = np.clip(commands[:, :, 1], -0.15, 0.15)
        commands[:, :, 2] = np.clip(commands[:, :, 2], -0.60, 0.60)
        costs = np.array([self.rollout_cost(state, c, goal_x, self.gait.phase) for c in commands])
        weights = np.exp(-(costs - costs.min()) / self.temperature)
        weights /= weights.sum() + 1e-12
        self.mean += np.sum(weights[:, None, None] * noise, axis=0)
        self.mean[:, 0] = np.clip(self.mean[:, 0], -0.30, 0.45)
        self.mean[:, 1] = np.clip(self.mean[:, 1], -0.15, 0.15)
        self.mean[:, 2] = np.clip(self.mean[:, 2], -0.60, 0.60)
        command = self.mean[0].copy()
        self.mean[:-1] = self.mean[1:]
        self.mean[-1] = self.mean[-2]
        return command


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    data.qpos[2] = 0.335
    data.qpos[model.jnt_qposadr[1:]] = NOMINAL
    mujoco.mj_forward(model, data)

    gait = GaitLayer(model)
    planner = VelocityMPPI(model, gait)
    command = np.zeros(3)
    goal_x = 0.8
    steps = 0
    warmup = 500
    policy_period = 5

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 1.5
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20
        while viewer.is_running():
            if steps < warmup:
                command[:] = 0.0
            elif steps % policy_period == 0:
                command = planner.update(data, goal_x)
            target = gait.target(command)
            data.ctrl[:] = gait.torque(data, target)
            mujoco.mj_step(model, data)
            gait.phase = (gait.phase + 0.012) % (2.0 * np.pi)
            viewer.sync()
            steps += 1
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()
