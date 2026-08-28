#!/usr/bin/env python3
"""Stable visual walk animation for the D1 Edu model on macOS.

This is a kinematic preview: it visualizes the intended movement without
letting an untuned contact controller destabilize the robot.
"""

from pathlib import Path
import argparse
import time

import mujoco
import mujoco.viewer
import numpy as np

from run_d1_edu_mppi_velocity_viewer import GaitLayer


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models" / "d1_edu" / "d1_edu.xml"
NOMINAL = np.array([0.0, 0.65, -1.25] * 4)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("forward", "left", "right"), default="forward")
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    gait = GaitLayer(model)
    command = {
        "forward": np.array([0.22, 0.0, 0.0]),
        "left": np.array([0.12, 0.0, 0.8]),
        "right": np.array([0.12, 0.0, -0.8]),
    }[args.mode]
    joint_qpos = model.jnt_qposadr[1:]
    speed = 0.20
    yaw_rate = float(command[2])
    yaw = 0.0
    sim_time = 0.0

    data.qpos[2] = 0.335
    data.qpos[joint_qpos] = NOMINAL
    mujoco.mj_forward(model, data)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        viewer.cam.trackbodyid = -1
        viewer.cam.distance = 1.5
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20
        viewer.cam.lookat[:] = [0.0, 0.0, 0.2]
        while viewer.is_running():
            dt = model.opt.timestep * 4.0
            sim_time += dt
            gait.phase = (gait.phase + 0.012 * 4.0) % (2.0 * np.pi)
            target = gait.target(command)

            if args.mode == "forward":
                data.qpos[0] += speed * dt
            else:
                yaw += yaw_rate * dt
                data.qpos[0] += speed * np.cos(yaw) * dt
                data.qpos[1] += speed * np.sin(yaw) * dt
            data.qpos[3:7] = [np.cos(yaw / 2.0), 0.0, 0.0, np.sin(yaw / 2.0)]
            data.qpos[joint_qpos] = target
            mujoco.mj_forward(model, data)
            # Hard-lock the visualization camera; mouse/trackpad gestures
            # must not make the whole scene rotate.
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            viewer.cam.trackbodyid = -1
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -20
            viewer.cam.distance = 1.5
            viewer.cam.lookat[:] = [0.0, 0.0, 0.2]
            viewer.sync()
            time.sleep(dt)


if __name__ == "__main__":
    main()
