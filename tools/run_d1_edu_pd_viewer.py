#!/usr/bin/env python3
"""Open D1 Edu in the MuJoCo viewer with a simple joint-space PD hold."""

from pathlib import Path
import time

import mujoco
import mujoco.viewer
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models" / "d1_edu" / "d1_edu.xml"


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    # With the current nominal joint angles, the feet are about 0.335 m
    # below the converted free-joint origin. Start with the feet on the floor.
    data.qpos[2] = 0.335

    nominal = [0.0, 0.65, -1.25] * 4
    joint_qpos = model.jnt_qposadr[1:]
    joint_qvel = model.jnt_dofadr[1:]
    data.qpos[joint_qpos] = np.asarray(nominal)
    target_q = data.qpos.copy()
    mujoco.mj_forward(model, data)

    kp, kd = 45.0, 1.5
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 1.4
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20
        while viewer.is_running():
            data.ctrl[:] = kp * (target_q[joint_qpos] - data.qpos[joint_qpos]) - kd * data.qvel[joint_qvel]
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()
