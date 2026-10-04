#!/usr/bin/env python3
"""Checks for the local D1 Max geometry audit gate."""

from __future__ import annotations

from pathlib import Path
import unittest

from inspect_d1_max_geometry import measure_d1_max_geometry


MODEL = Path("external/Agibot_D1_Max/d1_max.xml")


@unittest.skipUnless(MODEL.is_file(), "local ignored D1 Max MJCF is not present")
class D1MaxGeometryAuditTest(unittest.TestCase):
    def test_default_pose_geometry_is_measured(self) -> None:
        result = measure_d1_max_geometry(MODEL)
        self.assertEqual(result["profile"], "d1_max_proxy")
        self.assertEqual(result["model_dimensions"]["robot_collision_geom_count"], 29)
        self.assertTrue(result["base_collision"]["proxy_detected"])
        self.assertAlmostEqual(result["base_collision"]["planar_radius_m"], 0.476076, places=5)
        self.assertAlmostEqual(result["robot_collision"]["planar_radius_m"], 0.480335, places=5)
        self.assertEqual(result["robot_collision"]["planner_radius_candidate_m"], 0.49)


if __name__ == "__main__":
    unittest.main()
