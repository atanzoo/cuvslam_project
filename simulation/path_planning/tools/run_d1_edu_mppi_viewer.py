#!/usr/bin/env python3
"""Minimal whole-body MPPI demo for the official Agibot D1 Edu model.

Run from the repository root on macOS with:
    ./.venv/bin/mjpython simulation/path_planning/tools/run_d1_edu_mppi_viewer.py
"""

from pathlib import Path
import time

import mujoco
import mujoco.viewer
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models" / "d1_edu" / "d1_edu.xml"


class MPPI:
    def __init__(self, model: mujoco.MjModel, horizon: int = 15, samples: int = 32):
        self.model = model
        self.horizon = horizon
        self.samples = samples
        self.nu = model.nu
        self.rng = np.random.default_rng(7)
        # First stabilize the hardware-like low-level loop; MPPI explores
        # around this tracked target instead of trying to catch a falling body.
        self.kp = 120.0
        self.kd = 4.0
        self.max_torque = 40.0
        self.temperature = 0.25
        self.noise_std = np.array([0.04, 0.08, 0.08] * 4)
        self.joint_qpos = model.jnt_qposadr[1:]
        self.joint_qvel = model.jnt_dofadr[1:]
        self.joint_low = model.jnt_range[1:, 0]
        self.joint_high = model.jnt_range[1:, 1]
        self.nominal = np.array([0.0, 0.65, -1.25] * 4)
        self.mean = np.repeat(self.nominal[None, :], horizon, axis=0)
        self.phase = 0.0

    def gait_reference(self) -> np.ndarray:
        """A conservative diagonal-pair seed for online MPPI refinement."""
        ref = np.repeat(self.nominal[None, :], self.horizon, axis=0)
        for t in range(self.horizon):
            phase = self.phase + 0.32 * t
            swing_a = np.sin(phase)
            swing_b = np.sin(phase + np.pi)
            for leg, swing in ((0, swing_a), (1, swing_b), (2, swing_b), (3, swing_a)):
                j = 3 * leg
                # Negative hip angle moves the foot forward in this model.
                ref[t, j + 1] = self.nominal[j + 1] - 0.12 * swing
                ref[t, j + 2] = self.nominal[j + 2] + 0.10 * max(0.0, swing)
        return ref

    def torque(self, data: mujoco.MjData, target: np.ndarray) -> np.ndarray:
        tau = self.kp * (target - data.qpos[self.joint_qpos]) - self.kd * data.qvel[self.joint_qvel]
        return np.clip(tau, -self.max_torque, self.max_torque)

    def rollout_cost(self, state: mujoco.MjData, actions: np.ndarray, goal_x: float) -> float:
        data = mujoco.MjData(self.model)
        mujoco.mj_copyData(data, self.model, state)
        total = 0.0
        for action in actions:
            action = np.clip(action, self.joint_low, self.joint_high)
            data.ctrl[:] = self.torque(data, action)
            mujoco.mj_step(self.model, data)
            x, y, z = data.qpos[:3]
            quat = data.qpos[3:7]
            total += 2.0 * (goal_x - x) ** 2 + 2.0 * y**2
            total += 5.0 * (z - 0.335) ** 2
            total += 12.0 * float(np.sum(quat[1:] ** 2))
            total += 0.30 * max(0.0, -float(data.qvel[0]))
            total -= 0.18 * min(0.35, max(0.0, float(data.qvel[0])))
            total += 0.015 * float(np.sum((action - self.nominal) ** 2))
            if z < 0.16 or z > 0.75:
                total += 100.0
        return float(total)

    def update(self, state: mujoco.MjData, goal_x: float) -> np.ndarray:
        gait = self.gait_reference()
        self.mean = 0.65 * self.mean + 0.35 * gait
        noise = self.rng.normal(0.0, self.noise_std, size=(self.samples, self.horizon, self.nu))
        trajectories = np.clip(self.mean[None, :, :] + noise, self.joint_low, self.joint_high)
        costs = np.array([self.rollout_cost(state, traj, goal_x) for traj in trajectories])
        scaled = -(costs - costs.min()) / max(self.temperature, 1e-6)
        scaled -= scaled.max()
        weights = np.exp(scaled)
        weights /= weights.sum() + 1e-12
        self.mean += np.sum(weights[:, None, None] * noise, axis=0)
        self.mean = np.clip(self.mean, self.joint_low, self.joint_high)
        first = self.mean[0].copy()
        self.mean[:-1] = self.mean[1:]
        self.mean[-1] = self.nominal
        self.phase = (self.phase + 0.08) % (2.0 * np.pi)
        return first


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    data.qpos[2] = 0.335
    joint_qpos = model.jnt_qposadr[1:]
    data.qpos[joint_qpos] = np.array([0.0, 0.65, -1.25] * 4)
    mujoco.mj_forward(model, data)

    controller = MPPI(model)
    goal_x = 0.8
    action = controller.nominal.copy()
    policy_steps = 0
    warmup_steps = 500
    sim_steps_per_policy = 5

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 1.5
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20
        while viewer.is_running():
            if policy_steps < warmup_steps:
                action = controller.nominal.copy()
            elif policy_steps % sim_steps_per_policy == 0:
                action = controller.update(data, goal_x)
            data.ctrl[:] = controller.torque(data, action)
            mujoco.mj_step(model, data)
            viewer.sync()
            policy_steps += 1
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()
