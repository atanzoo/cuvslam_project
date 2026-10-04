#!/usr/bin/env python3
"""MuJoCo-derived planar footprint calibration for the D1 Edu prototype.

The planner uses a bounded base-footprint radius.  The simulator still keeps
the articulated collision geoms and MuJoCo contact as the final ground truth.
This module makes the two numbers explicit instead of silently using 0.25 m.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import mujoco
import numpy as np


@dataclass(frozen=True)
class RobotFootprintCalibration:
    """Planar extents measured from collision geoms at the current pose."""

    planner_radius: float
    base_extent: float
    articulated_extent: float
    collision_geoms: dict[str, float]


def calibrate_robot_footprint(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    planner_min_radius: float = 0.32,
    planner_max_radius: float = 0.35,
) -> RobotFootprintCalibration:
    """Measure a conservative planner radius from robot collision geoms.

    ``geom_rbound`` is the mesh bounding radius around each geom's current
    center.  Adding its planar center offset from the free-base origin gives
    a conservative footprint for the current articulated pose.  The planner
    radius is clamped to the requested 0.32--0.35 m interval and is based on
    the base collision geom; the larger articulated extent is reported for
    diagnostics and not used to redefine physical contact.
    """
    if planner_min_radius <= 0.0 or planner_max_radius < planner_min_radius:
        raise ValueError("invalid planner footprint radius bounds")
    mujoco.mj_forward(model, data)
    base_xy = np.asarray(data.qpos[:2], dtype=float)
    extents: dict[str, float] = {}
    base_extents: list[float] = []
    for geom_id in range(int(model.ngeom)):
        name = str(model.geom(geom_id).name)
        if not name.endswith("_collision") or name.startswith("moving_obstacle"):
            continue
        center_offset = float(np.linalg.norm(
            np.asarray(data.geom_xpos[geom_id][:2], dtype=float) - base_xy
        ))
        extent = center_offset + float(model.geom_rbound[geom_id])
        if not np.isfinite(extent):
            continue
        extents[name] = extent
        if name == "BASE_LINK_collision":
            base_extents.append(extent)
    if not extents:
        raise ValueError("no robot collision geoms found")
    base_extent = max(base_extents) if base_extents else max(extents.values())
    articulated_extent = max(extents.values())
    planner_radius = float(np.clip(
        base_extent,
        planner_min_radius,
        planner_max_radius,
    ))
    return RobotFootprintCalibration(
        planner_radius=planner_radius,
        base_extent=float(base_extent),
        articulated_extent=float(articulated_extent),
        collision_geoms=extents,
    )


def footprint_summary(calibration: RobotFootprintCalibration) -> dict[str, Any]:
    """Return a JSON-friendly calibration payload for episode telemetry."""
    return {
        "planner_radius": float(calibration.planner_radius),
        "base_extent": float(calibration.base_extent),
        "articulated_extent": float(calibration.articulated_extent),
        "collision_geoms": {
            str(name): float(value)
            for name, value in calibration.collision_geoms.items()
        },
    }
