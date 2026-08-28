#!/usr/bin/env python3
"""Load and smoke-test the converted D1 Edu MuJoCo model on the local machine."""

from pathlib import Path

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models" / "d1_edu" / "d1_edu.xml"


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    # Start above the floor with a conservative four-leg crouch.
    # Place the feet close to the added floor for the nominal joint pose.
    data.qpos[2] = 0.335
    nominal = {
        "FL_ABAD_JOINT": 0.0,
        "FL_HIP_JOINT": 0.65,
        "FL_KNEE_JOINT": -1.25,
        "FR_ABAD_JOINT": 0.0,
        "FR_HIP_JOINT": 0.65,
        "FR_KNEE_JOINT": -1.25,
        "RR_ABAD_JOINT": 0.0,
        "RR_HIP_JOINT": 0.65,
        "RR_KNEE_JOINT": -1.25,
        "RL_ABAD_JOINT": 0.0,
        "RL_HIP_JOINT": 0.65,
        "RL_KNEE_JOINT": -1.25,
    }
    for joint_name, angle in nominal.items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        data.qpos[model.jnt_qposadr[joint_id]] = angle

    mujoco.mj_forward(model, data)
    initial_height = float(data.qpos[2])
    target_q = data.qpos.copy()
    joint_qpos = model.jnt_qposadr[1:]
    joint_qvel = model.jnt_dofadr[1:]
    kp, kd = 45.0, 1.5
    for _ in range(500):
        data.ctrl[:] = kp * (target_q[joint_qpos] - data.qpos[joint_qpos]) - kd * data.qvel[joint_qvel]
        mujoco.mj_step(model, data)

    if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
        raise RuntimeError("MuJoCo state became non-finite")

    print(f"model={MODEL_PATH}")
    print(f"nq={model.nq} nv={model.nv} nu={model.nu} bodies={model.nbody} joints={model.njnt}")
    print(f"initial_base_height={initial_height:.3f} final_base_height={data.qpos[2]:.3f}")
    print("status=PASS: D1 Edu MJCF loaded and stepped locally with joint PD")


if __name__ == "__main__":
    main()
