#!/usr/bin/env python3
"""Export a scenario-specific, read-only-source map for a future Nav2 run.

This does not launch ROS or validate a Nav2 costmap. The caller must pass the
fixed actor from the exact episode identity; pedestrians are never rasterized.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from d1_candidate_map import CandidateMap, load_candidate_map


@dataclass(frozen=True)
class StaticBox:
    x: float
    y: float
    half_length: float
    half_width: float

    def validate(self) -> None:
        values = (self.x, self.y, self.half_length, self.half_width)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("static box values must be finite")
        if self.half_length <= 0.0 or self.half_width <= 0.0:
            raise ValueError("static box half extents must be positive")


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _box_cell_bounds(candidate: CandidateMap, box: StaticBox) -> tuple[int, int, int, int]:
    """Return inclusive PGM (left, right, top, bottom) intersection bounds."""
    box.validate()
    origin_x, origin_y, _ = candidate.scaled_origin
    resolution = candidate.scaled_resolution
    min_x, max_x = box.x - box.half_length, box.x + box.half_length
    min_y, max_y = box.y - box.half_width, box.y + box.half_width
    end_x = origin_x + candidate.width * resolution
    end_y = origin_y + candidate.height * resolution
    if not (origin_x < min_x < max_x < end_x and origin_y < min_y < max_y < end_y):
        raise ValueError("static box touches or extends outside the map bounds")

    left = math.floor((min_x - origin_x) / resolution)
    right = math.ceil((max_x - origin_x) / resolution) - 1
    bottom_from_origin = math.floor((min_y - origin_y) / resolution)
    top_from_origin = math.ceil((max_y - origin_y) / resolution) - 1
    top = candidate.height - 1 - top_from_origin
    bottom = candidate.height - 1 - bottom_from_origin
    return left, right, top, bottom


def rasterize_fixed_box(candidate: CandidateMap, box: StaticBox) -> tuple[bytes, dict[str, int]]:
    """Mark every cell intersecting a known-free, axis-aligned static box."""
    left, right, top, bottom = _box_cell_bounds(candidate, box)
    image = np.frombuffer(candidate.pixels, dtype=np.uint8).copy().reshape(
        candidate.height, candidate.width,
    )
    covered = image[top:bottom + 1, left:right + 1]
    if not all(candidate.classify_pixel(int(value)) == "free" for value in np.unique(covered)):
        raise ValueError("static box intersects occupied or unknown source cells")
    changed_cells = int(covered.size)
    covered[:] = 0
    return image.tobytes(), {
        "left": left,
        "right": right,
        "top": top,
        "bottom": bottom,
        "occupied_cells_added": changed_cells,
    }


def nav2_free_threshold(candidate: CandidateMap, pixels: bytes) -> float:
    """Choose YAML threshold so Nav2 trinary preserves the source cell classes.

    The candidate-map reader reserves pixel 205 as Unknown, but Nav2 Foxy's
    trinary map server applies the YAML free threshold numerically. At 0.25,
    pixel 205 is interpreted as free (occupancy 50/255).
    """
    threshold = candidate.metadata.free_thresh
    if 205 in pixels:
        if candidate.metadata.negate != 0:
            raise ValueError("unknown pixel 205 with negate=1 needs a different map encoding")
        threshold = min(threshold, 0.19)
    for pixel in set(pixels):
        occupancy = pixel / 255.0 if candidate.metadata.negate else 1.0 - pixel / 255.0
        nav2_class = (
            "occupied" if occupancy > candidate.metadata.occupied_thresh else
            "free" if occupancy < threshold else "unknown"
        )
        if nav2_class != candidate.classify_pixel(pixel):
            raise ValueError(
                f"Nav2 trinary would classify pixel {pixel} as {nav2_class}, "
                f"candidate map expects {candidate.classify_pixel(pixel)}"
            )
    return threshold


def export_nav2_map(
    source_yaml: str | Path,
    output_dir: str | Path,
    *,
    box: StaticBox,
    scene_scale: int = 3,
    scene_id: str = "unspecified",
) -> dict[str, object]:
    """Create a new map.yaml/map.pgm/manifest.json without modifying input."""
    if scene_scale < 1:
        raise ValueError("scene_scale must be a positive integer")
    if not scene_id or any(char in scene_id for char in "\n\r\t"):
        raise ValueError("scene_id must be a nonempty single-line identifier")
    candidate = load_candidate_map(
        source_yaml, scene_scale=scene_scale, preserve_resolution=True,
    )
    source_yaml_hash = _sha256(candidate.yaml_path)
    source_pgm_hash = _sha256(candidate.pgm_path)
    output = Path(output_dir).resolve()
    if output.exists():
        raise FileExistsError(f"output directory already exists: {output}")
    pixels, overlay = rasterize_fixed_box(candidate, box)
    metadata = candidate.metadata
    free_threshold = nav2_free_threshold(candidate, pixels)
    origin = candidate.scaled_origin
    yaml_text = (
        "image: map.pgm\n"
        f"mode: {metadata.mode}\n"
        f"resolution: {candidate.scaled_resolution:.12g}\n"
        f"origin: [{origin[0]:.12g}, {origin[1]:.12g}, {origin[2]:.12g}]\n"
        f"negate: {metadata.negate}\n"
        f"occupied_thresh: {metadata.occupied_thresh:.12g}\n"
        f"free_thresh: {free_threshold:.12g}\n"
    )
    image_bytes = f"P5\n{candidate.width} {candidate.height}\n255\n".encode("ascii") + pixels
    output.mkdir(parents=True, exist_ok=False)
    pgm_file = output / "map.pgm"
    yaml_file = output / "map.yaml"
    pgm_file.write_bytes(image_bytes)
    yaml_file.write_text(yaml_text, encoding="utf-8")
    reloaded = load_candidate_map(yaml_file)
    if (
        reloaded.width != candidate.width
        or reloaded.height != candidate.height
        or reloaded.pixels != pixels
        or reloaded.metadata.resolution != candidate.scaled_resolution
        or reloaded.metadata.origin != origin
        or reloaded.metadata.negate != metadata.negate
        or reloaded.metadata.occupied_thresh != metadata.occupied_thresh
        or reloaded.metadata.free_thresh != free_threshold
    ):
        raise RuntimeError(f"exported map read-back differs from generated cells: {output}")
    if _sha256(candidate.yaml_path) != source_yaml_hash or _sha256(candidate.pgm_path) != source_pgm_hash:
        raise RuntimeError("source map changed during export")
    manifest: dict[str, object] = {
        "status": "map_export_only_not_nav2_validated",
        "scene_id": scene_id,
        "source_yaml": str(candidate.yaml_path),
        "source_yaml_sha256": source_yaml_hash,
        "source_pgm_sha256": source_pgm_hash,
        "scene_scale": scene_scale,
        "rasterization": "nearest_neighbor_cell_replication",
        "width": candidate.width,
        "height": candidate.height,
        "resolution": candidate.scaled_resolution,
        "origin": list(origin),
        "static_box": {
            "x": box.x,
            "y": box.y,
            "half_length": box.half_length,
            "half_width": box.half_width,
        },
        "static_box_cells": overlay,
        "source_free_thresh": metadata.free_thresh,
        "nav2_free_thresh": free_threshold,
        "nav2_trinary_class_validation": "all_unique_pixels_match_candidate_classes",
        "read_back_validation": "all_pixels_and_metadata_exact",
        "map_yaml_sha256": _sha256(yaml_file),
        "map_pgm_sha256": _sha256(pgm_file),
    }
    (output / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-map", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--scene-scale", type=int, default=3)
    parser.add_argument("--scene-id", required=True)
    parser.add_argument(
        "--static-box", type=float, nargs=4, required=True,
        metavar=("X", "Y", "HALF_LENGTH", "HALF_WIDTH"),
    )
    args = parser.parse_args()
    manifest = export_nav2_map(
        args.source_map,
        args.output_dir,
        box=StaticBox(*args.static_box),
        scene_scale=args.scene_scale,
        scene_id=args.scene_id,
    )
    print(json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
