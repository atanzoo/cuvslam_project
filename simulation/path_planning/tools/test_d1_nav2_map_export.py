#!/usr/bin/env python3
"""Focused offline tests; these do not claim a Nav2 runtime test."""

from __future__ import annotations

import hashlib
from pathlib import Path
import tempfile
import unittest

from d1_candidate_map import load_candidate_map
from d1_nav2_map_export import StaticBox, export_nav2_map, rasterize_fixed_box


class Nav2MapExportTest(unittest.TestCase):
    def _source(self, root: Path, *, unknown: bool = False) -> Path:
        pixels = bytearray([254] * 16)
        if unknown:
            pixels[3 * 4 + 2] = 205
        (root / "source.pgm").write_bytes(b"P5\n4 4\n255\n" + pixels)
        yaml_file = root / "source.yaml"
        yaml_file.write_text(
            "image: source.pgm\nmode: trinary\nresolution: 1\n"
            "origin: [1, 2, 0]\nnegate: 0\n"
            "occupied_thresh: 0.65\nfree_thresh: 0.25\n",
            encoding="utf-8",
        )
        return yaml_file

    def test_scale_and_lower_row_overlay_preserve_source(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = self._source(root)
            original = hashlib.sha256((root / "source.pgm").read_bytes()).hexdigest()
            output = root / "export"
            manifest = export_nav2_map(
                source, output,
                box=StaticBox(2.5, 2.5, 0.25, 0.25),
                scene_scale=1,
                scene_id="bottom_row",
            )
            loaded = load_candidate_map(output / "map.yaml")
            self.assertEqual((loaded.width, loaded.height), (4, 4))
            self.assertEqual(loaded.metadata.origin, (1.0, 2.0, 0.0))
            self.assertEqual(loaded.pixel(1, 3), 0)
            self.assertEqual(loaded.pixel(1, 0), 254)
            self.assertEqual(manifest["static_box_cells"]["occupied_cells_added"], 1)
            self.assertEqual(manifest["read_back_validation"], "all_pixels_and_metadata_exact")
            self.assertEqual(hashlib.sha256((root / "source.pgm").read_bytes()).hexdigest(), original)

    def test_threefold_geometry_and_unknown_preserved(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = self._source(root, unknown=True)
            output = root / "export"
            manifest = export_nav2_map(
                source, output,
                box=StaticBox(5.5, 9.5, 0.25, 0.25),
                scene_scale=3,
                scene_id="scale3",
            )
            loaded = load_candidate_map(output / "map.yaml")
            self.assertEqual((loaded.width, loaded.height), (12, 12))
            self.assertEqual(loaded.metadata.resolution, 1.0)
            self.assertEqual(loaded.metadata.origin, (3.0, 6.0, 0.0))
            self.assertEqual(loaded.pixel(6, 11), 205)
            self.assertEqual(loaded.pixel(2, 8), 0)
            self.assertEqual(manifest["rasterization"], "nearest_neighbor_cell_replication")
            self.assertEqual(loaded.metadata.free_thresh, 0.19)
            self.assertEqual(manifest["source_free_thresh"], 0.25)
            self.assertEqual(manifest["nav2_trinary_class_validation"], "all_unique_pixels_match_candidate_classes")

    def test_unknown_or_outside_box_is_rejected_without_output(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = self._source(root, unknown=True)
            candidate = load_candidate_map(source)
            with self.assertRaisesRegex(ValueError, "unknown"):
                rasterize_fixed_box(candidate, StaticBox(3.5, 2.5, 0.25, 0.25))
            output = root / "invalid"
            with self.assertRaisesRegex(ValueError, "outside"):
                export_nav2_map(
                    source, output,
                    box=StaticBox(1.1, 2.5, 0.25, 0.25),
                    scene_scale=1,
                    scene_id="invalid",
                )
            self.assertFalse(output.exists())

    def test_existing_output_is_never_overwritten(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = self._source(root)
            output = root / "export"
            export_nav2_map(source, output, box=StaticBox(2.5, 2.5, 0.25, 0.25), scene_scale=1)
            before = hashlib.sha256((output / "map.pgm").read_bytes()).hexdigest()
            with self.assertRaises(FileExistsError):
                export_nav2_map(source, output, box=StaticBox(3.5, 3.5, 0.25, 0.25), scene_scale=1)
            self.assertEqual(hashlib.sha256((output / "map.pgm").read_bytes()).hexdigest(), before)


if __name__ == "__main__":
    unittest.main()
