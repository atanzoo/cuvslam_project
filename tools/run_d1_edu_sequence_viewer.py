#!/usr/bin/env python3
"""D1 Edu visual sequence: forward 5s, moving left curve 5s, forward 5s."""

from pathlib import Path
import time

import mujoco
import mujoco.viewer
import numpy as np

from run_d1_edu_mppi_velocity_viewer import GaitLayer


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models" / "d1_edu" / "d1_edu.xml"
NOMINAL = np.array([0.0, 0.65, -1.25] * 4)


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    gait = GaitLayer(model)
    joint_qpos = model.jnt_qposadr[1:]
    data.qpos[2] = 0.335
    data.qpos[joint_qpos] = NOMINAL
    mujoco.mj_forward(model, data)

    yaw = 0.0
    elapsed = 0.0
    turn_duration = 5.0
    total_duration = 5.0 + turn_duration + 5.0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Free camera: use the trackpad/mouse to orbit and inspect posture.
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        viewer.cam.trackbodyid = -1
        viewer.cam.distance = 2.8
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20
        viewer.cam.lookat[:] = [0.9, 0.0, 0.2]
        while viewer.is_running() and elapsed < total_duration:
            dt = model.opt.timestep * 4.0
            if elapsed < 5.0:
                command = np.array([0.22, 0.0, 0.0])
                data.qpos[0] += 0.20 * dt
            elif elapsed < 5.0 + turn_duration:
                # Moving left curve: forward velocity and yaw rate together.
                command = np.array([0.20, 0.0, 0.35])
                yaw += 0.35 * dt
                data.qpos[0] += 0.20 * np.cos(yaw) * dt
                data.qpos[1] += 0.20 * np.sin(yaw) * dt
            else:
                command = np.array([0.22, 0.0, 0.0])
                data.qpos[0] += 0.20 * np.cos(yaw) * dt
                data.qpos[1] += 0.20 * np.sin(yaw) * dt

            gait.phase = (gait.phase + 0.012 * 4.0) % (2.0 * np.pi)
            data.qpos[3:7] = [np.cos(yaw / 2.0), 0.0, 0.0, np.sin(yaw / 2.0)]
            data.qpos[joint_qpos] = gait.target(command)
            mujoco.mj_forward(model, data)

            viewer.sync()
            elapsed += dt
            time.sleep(dt)


if __name__ == "__main__":
    main()
