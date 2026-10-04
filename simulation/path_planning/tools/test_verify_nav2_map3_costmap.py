#!/usr/bin/env python3
"""Small fail-closed checks for saved Nav2 costmap evidence."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from verify_nav2_map3_costmap import verify_costmap


class VerifyNav2Map3CostmapTest(unittest.TestCase):
    def _fixture(self, root: Path) -> tuple[Path, Path, Path, Path, Path]:
        pgm = root / "map.pgm"
        pgm.write_bytes(b"P5\n3 3\n255\n" + bytes((205, 254, 254, 254, 254, 254, 254, 254, 0)))
        yaml = root / "map.yaml"
        yaml.write_text(
            "image: map.pgm\nmode: trinary\nresolution: 1\norigin: [0, 0, 0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.19\n",
            encoding="utf-8",
        )
        manifest = root / "manifest.json"
        manifest.write_text(json.dumps({
            "scene_id": "test_scene", "width": 3, "height": 3,
            "map_yaml_sha256": hashlib.sha256(yaml.read_bytes()).hexdigest(),
            "map_pgm_sha256": hashlib.sha256(pgm.read_bytes()).hexdigest(),
        }), encoding="utf-8")
        raw = root / "costmap.bin"
        # Costmap has bottom-up rows; PGM has top-down rows.
        raw.write_bytes(bytes((0, 0, 254, 0, 0, 0, 255, 0, 0)))
        snapshot = root / "costmap.json"
        snapshot.write_text(json.dumps({
            "status": "real_nav2_global_costmap_sampled_not_closed_loop",
            "scene_id": "test_scene", "width": 3, "height": 3,
            "resolution": 1, "origin_xy": [0, 0],
            "unknown_cells": 1, "fixed_center_cost": 254, "start_cost": 0,
            "costmap_sha256": hashlib.sha256(raw.read_bytes()).hexdigest(),
        }), encoding="utf-8")
        route = root / "path.json"
        route.write_text(json.dumps({
            "status": "real_nav2_planner_path_unverified_geometry",
            "scene_id": "test_scene", "points": [[0.5, 1.5], [1.5, 1.5]],
        }), encoding="utf-8")
        return yaml, manifest, snapshot, raw, route

    def test_safe_path_and_unknown_orientation(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            result = verify_costmap(*self._fixture(Path(temporary)))
            self.assertEqual(result["source_unknown_cells_blocked"], 1)
            self.assertEqual(result["source_occupied_cells_blocked"], 1)
            self.assertGreater(result["dense_costmap_samples_clear"], 2)

    def test_unknown_made_free_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            yaml, manifest, snapshot, raw, route = self._fixture(Path(temporary))
            values = bytearray(raw.read_bytes())
            values[6] = 0
            raw.write_bytes(values)
            data = json.loads(snapshot.read_text(encoding="utf-8"))
            data["costmap_sha256"] = hashlib.sha256(values).hexdigest()
            snapshot.write_text(json.dumps(data), encoding="utf-8")
            with self.assertRaisesRegex(AssertionError, "Unknown became traversable"):
                verify_costmap(yaml, manifest, snapshot, raw, route)

    def test_path_through_occupied_cell_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            yaml, manifest, snapshot, raw, route = self._fixture(Path(temporary))
            data = json.loads(route.read_text(encoding="utf-8"))
            data["points"] = [[1.5, 0.5], [2.5, 0.5]]
            route.write_text(json.dumps(data), encoding="utf-8")
            with self.assertRaisesRegex(AssertionError, "entered cost 254"):
                verify_costmap(yaml, manifest, snapshot, raw, route)


if __name__ == "__main__":
    unittest.main()
