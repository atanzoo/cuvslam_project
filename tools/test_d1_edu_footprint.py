#!/usr/bin/env python3
"""Regression tests for MuJoCo-derived D1 planar footprint calibration."""

from __future__ import annotations

from pathlib import Path

import mujoco

from d1_edu_footprint import calibrate_robot_footprint
from run_d1_edu_mppi_obstacle_viewer import ObstacleMPPI


ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(ROOT / "models/d1_edu/d1_edu.xml"))
    data = mujoco.MjData(model)
    calibration = calibrate_robot_footprint(model, data)
    assert 0.32 <= calibration.planner_radius <= 0.35
    assert calibration.base_extent > 0.31
    assert calibration.articulated_extent >= calibration.base_extent
    assert "BASE_LINK_collision" in calibration.collision_geoms
    planner = ObstacleMPPI(robot_radius=calibration.planner_radius)
    assert planner.robot_radius == calibration.planner_radius
    print(
        "PASS footprint calibration: "
        f"planner={calibration.planner_radius:.4f}m "
        f"base={calibration.base_extent:.4f}m "
        f"articulated={calibration.articulated_extent:.4f}m"
    )


if __name__ == "__main__":
    main()
