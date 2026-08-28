#!/usr/bin/env python3
"""Basic D1 Edu gait smoke test before adding MPPI."""

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
    parser.add_argument("--mode", choices=("stand", "forward", "turn-left", "turn-right"), default="stand")
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    data.qpos[2] = 0.335
    data.qpos[model.jnt_qposadr[1:]] = NOMINAL
    mujoco.mj_forward(model, data)

    gait = GaitLayer(model)
    commands = {
        "stand": np.array([0.0, 0.0, 0.0]),
        "forward": np.array([0.16, 0.0, 0.0]),
        "turn-left": np.array([0.0, 0.0, 0.35]),
        "turn-right": np.array([0.0, 0.0, -0.35]),
    }
    command = commands[args.mode]
    warmup_steps = 500
    steps = 0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 1.5
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20
        while viewer.is_running():
            active_command = np.zeros(3) if steps < warmup_steps else command
            target = gait.target(active_command)
            data.ctrl[:] = gait.torque(data, target)
            mujoco.mj_step(model, data)
            gait.phase = (gait.phase + 0.012) % (2.0 * np.pi)
            viewer.sync()
            steps += 1
            time.sleep(model.opt.timestep)


if __name__ == "__main__":
    main()
