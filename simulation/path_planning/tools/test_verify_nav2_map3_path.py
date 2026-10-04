#!/usr/bin/env python3
"""Offline verifier unit tests; the JSON fixtures are not Nav2 evidence."""

from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

from d1_nav2_map_export import StaticBox, export_nav2_map
from verify_nav2_map3_path import verify_path


class VerifyNav2Map3PathTest(unittest.TestCase):
    def _fixture(self, root: Path) -> tuple[Path, Path, Path]:
        (root / "source.pgm").write_bytes(b"P5\n40 40\n255\n" + bytes([254] * 1600))
        (root / "source.yaml").write_text(
            "image: source.pgm\nmode: trinary\nresolution: 0.1\n"
            "origin: [0, 0, 0]\nnegate: 0\n"
            "occupied_thresh: 0.65\nfree_thresh: 0.25\n",
            encoding="utf-8",
        )
        output = root / "export"
        export_nav2_map(
            root / "source.yaml", output,
            box=StaticBox(2.0, 2.0, 0.25, 0.25),
            scene_scale=1,
            scene_id="synthetic",
        )
        return output / "map.yaml", output / "manifest.json", root / "path.json"

    @staticmethod
    def _path(path_file: Path, points: list[list[float]], scene_id: str = "synthetic") -> None:
        path_file.write_text(json.dumps({
            "status": "real_nav2_planner_path_unverified_geometry",
            "scene_id": scene_id,
            "action": "compute_path_to_pose",
            "frame_id": "map",
            "goal": points[-1],
            "points": points,
        }), encoding="utf-8")

    def test_safe_dense_path_passes_geometry_only(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            map_yaml, manifest, path_file = self._fixture(Path(temp))
            self._path(path_file, [[0.8, 0.8], [3.2, 0.8]])
            result = verify_path(map_yaml, manifest, path_file, expected_start=(0.8, 0.8))
            self.assertTrue(result["all_samples_traversable"])
            self.assertGreater(result["min_fixed_edge_distance_m"], 0.61)
            self.assertIn("not_closed_loop", result["status"])
            self.assertLess(result["start_error_m"], 0.10)

    def test_fixed_box_and_identity_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            map_yaml, manifest, path_file = self._fixture(Path(temp))
            self._path(path_file, [[0.8, 2.0], [3.2, 2.0]])
            with self.assertRaisesRegex(AssertionError, "unsafe"):
                verify_path(map_yaml, manifest, path_file)
            self._path(path_file, [[0.8, 0.8], [3.2, 0.8]], scene_id="wrong")
            with self.assertRaisesRegex(ValueError, "identities differ"):
                verify_path(map_yaml, manifest, path_file)


if __name__ == "__main__":
    unittest.main()
