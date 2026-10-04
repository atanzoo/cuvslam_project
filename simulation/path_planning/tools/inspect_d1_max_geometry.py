#!/usr/bin/env python3
"""Measure the imported D1 Max MuJoCo geometry without running locomotion.

This is a static/default-pose audit for Scheme A A2.  It reports the exact XY
extents of the loaded collision geoms where MuJoCo exposes mesh vertices, and
keeps the result separate from the planner footprint contract.  A measured
default pose is not a gait envelope and must not be used as real-robot
evidence.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import json
import math
from typing import Any

import mujoco
import numpy as np

from d1_robot_profile import D1_MAX_PROXY_PROFILE


DEFAULT_MJCF = Path(D1_MAX_PROXY_PROFILE.model_asset or "")
DEFAULT_REPORT = Path("research/path_planning/D1_MAX_GEOMETRY_AUDIT_2026-09-03.json")


def _box_points(model: mujoco.MjModel, data: mujoco.MjData, geom_id: int) -> np.ndarray:
    half_size = np.asarray(model.geom_size[geom_id], dtype=float)
    local = np.asarray(
        [
            [x, y, z]
            for x in (-half_size[0], half_size[0])
            for y in (-half_size[1], half_size[1])
            for z in (-half_size[2], half_size[2])
        ],
        dtype=float,
    )
    rotation = np.asarray(data.geom_xmat[geom_id], dtype=float).reshape(3, 3)
    return (local @ rotation.T) + np.asarray(data.geom_xpos[geom_id], dtype=float)


def _mesh_points(model: mujoco.MjModel, data: mujoco.MjData, geom_id: int) -> np.ndarray:
    mesh_id = int(model.geom_dataid[geom_id])
    vertex_start = int(model.mesh_vertadr[mesh_id])
    vertex_count = int(model.mesh_vertnum[mesh_id])
    vertices = np.asarray(
        model.mesh_vert[vertex_start:vertex_start + vertex_count],
        dtype=float,
    )
    rotation = np.asarray(data.geom_xmat[geom_id], dtype=float).reshape(3, 3)
    return (vertices @ rotation.T) + np.asarray(data.geom_xpos[geom_id], dtype=float)


def _geom_points(model: mujoco.MjModel, data: mujoco.MjData, geom_id: int) -> np.ndarray:
    geom_type = int(model.geom_type[geom_id])
    if geom_type == int(mujoco.mjtGeom.mjGEOM_BOX):
        return _box_points(model, data, geom_id)
    if geom_type == int(mujoco.mjtGeom.mjGEOM_MESH):
        return _mesh_points(model, data, geom_id)
    # The imported model currently contains only boxes and meshes.  Keep a
    # conservative fallback so future approved imports are still inspectable.
    center = np.asarray(data.geom_xpos[geom_id], dtype=float)
    radius = float(model.geom_rbound[geom_id])
    offsets = np.asarray(
        [[x, y, 0.0] for x in (-radius, radius) for y in (-radius, radius)],
        dtype=float,
    )
    return center + offsets


def _bounds(points: np.ndarray) -> dict[str, list[float]]:
    planar = np.asarray(points, dtype=float)[:, :2]
    return {
        "min": [float(value) for value in planar.min(axis=0)],
        "max": [float(value) for value in planar.max(axis=0)],
        "size": [float(value) for value in planar.max(axis=0) - planar.min(axis=0)],
    }


def _conservative_sweep(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    collision_ids: list[int],
    base_id: int,
    *,
    samples: int,
    seed: int,
) -> dict[str, Any]:
    """Estimate a joint-limit envelope using geom bounding spheres.

    This intentionally does not claim to model a gait.  It samples every
    actuated hinge at both limits and then samples uniformly inside the
    official MuJoCo joint ranges.  ``geom_rbound`` makes the result
    conservative but can be larger than the exact mesh footprint.
    """
    if samples < 0:
        raise ValueError("sweep sample count must be non-negative")

    hinge_ids = [
        joint_id
        for joint_id in range(int(model.njnt))
        if int(model.jnt_type[joint_id]) == int(mujoco.mjtJoint.mjJNT_HINGE)
        and int(model.jnt_limited[joint_id])
    ]
    qpos_default = np.asarray(data.qpos, dtype=float).copy()
    qpos_addresses = [int(model.jnt_qposadr[joint_id]) for joint_id in hinge_ids]
    joint_ranges = np.asarray(model.jnt_range[hinge_ids], dtype=float)
    sample_qpos: list[np.ndarray] = []
    for index, address in enumerate(qpos_addresses):
        for value in joint_ranges[index]:
            qpos = qpos_default.copy()
            qpos[address] = float(value)
            sample_qpos.append(qpos)
    if samples:
        rng = np.random.default_rng(seed)
        random_values = rng.uniform(
            joint_ranges[:, 0],
            joint_ranges[:, 1],
            size=(samples, len(hinge_ids)),
        )
        for values in random_values:
            qpos = qpos_default.copy()
            qpos[qpos_addresses] = values
            sample_qpos.append(qpos)

    base_origin = np.asarray(data.geom_xpos[base_id][:2], dtype=float)
    lower = np.full(2, np.inf, dtype=float)
    upper = np.full(2, -np.inf, dtype=float)
    radius = 0.0
    radius_sample = -1
    for sample_index, qpos in enumerate(sample_qpos):
        data.qpos[:] = qpos
        mujoco.mj_forward(model, data)
        centers = np.asarray(data.geom_xpos[collision_ids, :2], dtype=float)
        radii = np.asarray(model.geom_rbound[collision_ids], dtype=float)
        lower = np.minimum(lower, np.min(centers - radii[:, None], axis=0))
        upper = np.maximum(upper, np.max(centers + radii[:, None], axis=0))
        sample_radius = float(
            np.max(np.linalg.norm(centers - base_origin, axis=1) + radii)
        )
        if sample_radius > radius:
            radius = sample_radius
            radius_sample = sample_index

    data.qpos[:] = qpos_default
    mujoco.mj_forward(model, data)
    if radius_sample < 0:
        return {
            "method": "no joint-limit samples",
            "seed": int(seed),
            "random_samples": int(samples),
            "evaluated_samples": 0,
        }
    return {
        "method": "joint limits plus uniform hinge-range samples using geom_rbound",
        "seed": int(seed),
        "random_samples": int(samples),
        "limit_samples": int(2 * len(hinge_ids)),
        "evaluated_samples": len(sample_qpos),
        "hinge_joint_count": len(hinge_ids),
        "xy_bounds": {
            "min": [float(value) for value in lower],
            "max": [float(value) for value in upper],
            "size": [float(value) for value in upper - lower],
        },
        "planar_radius_m": float(radius),
        "planner_radius_candidate_m": float(math.ceil(radius * 100.0) / 100.0),
        "radius_sample_index": int(radius_sample),
    }


def measure_d1_max_geometry(
    model_path: str | Path,
    *,
    sweep_samples: int = 256,
    sweep_seed: int = 0,
) -> dict[str, Any]:
    """Return a JSON-serializable default-pose and joint-envelope audit."""
    path = Path(model_path)
    if not path.is_file():
        raise FileNotFoundError(f"D1 Max MJCF not found: {path}")

    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    collision_ids = [
        geom_id
        for geom_id in range(int(model.ngeom))
        if str(model.geom(geom_id).name).endswith("_collision")
        and not str(model.geom(geom_id).name).startswith("moving_obstacle")
    ]
    if not collision_ids:
        raise ValueError("D1 Max MJCF has no robot collision geoms")

    base_id = next(
        (
            geom_id
            for geom_id in collision_ids
            if str(model.geom(geom_id).name) == "BASE_LINK_collision"
        ),
        None,
    )
    if base_id is None:
        raise ValueError("D1 Max MJCF has no BASE_LINK_collision geom")

    base_origin = np.asarray(data.geom_xpos[base_id][:2], dtype=float)
    all_points: list[np.ndarray] = []
    geom_records: list[dict[str, Any]] = []
    for geom_id in collision_ids:
        points = _geom_points(model, data, geom_id)
        all_points.append(points)
        planar = points[:, :2]
        center = np.asarray(data.geom_xpos[geom_id][:2], dtype=float)
        geom_records.append(
            {
                "name": str(model.geom(geom_id).name),
                "body": str(model.body(model.geom_bodyid[geom_id]).name),
                "type": int(model.geom_type[geom_id]),
                "center_xy": [float(value) for value in center],
                "xy_bounds": _bounds(points),
                "center_offset_m": float(np.linalg.norm(center - base_origin)),
                "planar_radius_m": float(np.linalg.norm(planar - base_origin, axis=1).max()),
                "rbound_m": float(model.geom_rbound[geom_id]),
                "contype": int(model.geom_contype[geom_id]),
                "conaffinity": int(model.geom_conaffinity[geom_id]),
            }
        )

    base_points = _geom_points(model, data, base_id)
    robot_points = np.vstack(all_points)
    base_radius = float(np.linalg.norm(base_points[:, :2] - base_origin, axis=1).max())
    robot_radius = float(np.linalg.norm(robot_points[:, :2] - base_origin, axis=1).max())
    model_type = int(model.geom_type[base_id])
    sweep = _conservative_sweep(
        model,
        data,
        collision_ids,
        base_id,
        samples=sweep_samples,
        seed=sweep_seed,
    )

    return {
        "model": str(path),
        "profile": D1_MAX_PROXY_PROFILE.name,
        "source_revision": D1_MAX_PROXY_PROFILE.source_revision,
        "pose": {
            "kind": "MuJoCo default qpos after mj_forward",
            "base_origin_xy": [float(value) for value in base_origin],
            "base_origin_z": float(data.geom_xpos[base_id][2]),
        },
        "model_dimensions": {
            "nq": int(model.nq),
            "nv": int(model.nv),
            "nu": int(model.nu),
            "nbody": int(model.nbody),
            "ngeom": int(model.ngeom),
            "nmesh": int(model.nmesh),
            "robot_collision_geom_count": len(collision_ids),
        },
        "base_collision": {
            "name": str(model.geom(base_id).name),
            "type": model_type,
            "type_name": "box" if model_type == int(mujoco.mjtGeom.mjGEOM_BOX) else "non-box",
            "proxy_detected": model_type == int(mujoco.mjtGeom.mjGEOM_BOX),
            "xy_bounds": _bounds(base_points),
            "planar_radius_m": base_radius,
            "rbound_m": float(model.geom_rbound[base_id]),
        },
        "robot_collision": {
            "xy_bounds": _bounds(robot_points),
            "planar_radius_m": robot_radius,
            "planner_radius_candidate_m": float(math.ceil(robot_radius * 100.0) / 100.0),
        },
        "joint_limit_envelope": sweep,
        "collision_geoms": geom_records,
        "limitations": [
            "Default-pose measurement is not a gait envelope.",
            "Joint-limit envelope uses conservative geom_rbound samples and is not a complete reachable-set proof.",
            "Joint-limit envelope is not a measured walking footprint and must not be used as a final planner radius.",
            "BASE_LINK uses a box proxy because the official STL exceeds the MuJoCo mesh face limit.",
            "No ground plane, standing controller, contact calibration, or D1 Max gait acceptance is included.",
            "Planner radius candidate is an audit value only and is not approved for training.",
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mjcf", type=Path, default=DEFAULT_MJCF)
    parser.add_argument("--output", type=Path, default=DEFAULT_REPORT)
    parser.add_argument("--sweep-samples", type=int, default=256)
    parser.add_argument("--sweep-seed", type=int, default=0)
    args = parser.parse_args()
    result = measure_d1_max_geometry(
        args.mjcf,
        sweep_samples=args.sweep_samples,
        sweep_seed=args.sweep_seed,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(
        "PASS D1 Max geometry audit: "
        f"base_radius={result['base_collision']['planar_radius_m']:.6f}m "
        f"robot_radius={result['robot_collision']['planar_radius_m']:.6f}m "
        f"candidate={result['robot_collision']['planner_radius_candidate_m']:.2f}m"
    )
    print(f"report={args.output}")


if __name__ == "__main__":
    main()
