#!/usr/bin/env python3
"""Verify a real Nav2 global costmap and dense path samples for one map-3 scene."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

from d1_candidate_map import load_candidate_map


def verify_costmap(
    map_yaml: str | Path,
    manifest_json: str | Path,
    costmap_json: str | Path,
    costmap_bin: str | Path,
    path_json: str | Path,
    *,
    sample_spacing: float = 0.025,
) -> dict[str, object]:
    if not 0 < sample_spacing <= 0.05:
        raise ValueError("sample_spacing must be within (0, 0.05] m")
    manifest = json.loads(Path(manifest_json).read_text(encoding="utf-8"))
    snapshot = json.loads(Path(costmap_json).read_text(encoding="utf-8"))
    route = json.loads(Path(path_json).read_text(encoding="utf-8"))
    if len({manifest.get("scene_id"), snapshot.get("scene_id"), route.get("scene_id")}) != 1:
        raise ValueError("map, costmap, and path scene identities differ")
    if snapshot.get("status") not in (
        "real_nav2_global_costmap_sampled_not_closed_loop",
        "real_nav2_local_costmap_sampled_not_closed_loop",
    ):
        raise ValueError("costmap is not identified as a real Nav2 snapshot")
    if route.get("status") != "real_nav2_planner_path_unverified_geometry":
        raise ValueError("path is not identified as a real Nav2 planner result")
    candidate = load_candidate_map(map_yaml)
    if (candidate.width, candidate.height) != (snapshot["width"], snapshot["height"]):
        raise ValueError("costmap dimensions differ from the exported map")
    if (candidate.width, candidate.height) != (manifest["width"], manifest["height"]):
        raise ValueError("map dimensions differ from the manifest")
    if hashlib.sha256(candidate.yaml_path.read_bytes()).hexdigest() != manifest["map_yaml_sha256"]:
        raise ValueError("map YAML hash differs from the manifest")
    if hashlib.sha256(candidate.pgm_path.read_bytes()).hexdigest() != manifest["map_pgm_sha256"]:
        raise ValueError("map PGM hash differs from the manifest")
    if abs(float(snapshot["resolution"]) - candidate.scaled_resolution) > 1e-8:
        raise ValueError("costmap resolution differs from the exported map")
    if any(abs(float(a) - float(b)) > 1e-8 for a, b in zip(snapshot["origin_xy"], candidate.scaled_origin[:2])):
        raise ValueError("costmap origin differs from the exported map")
    raw = Path(costmap_bin).read_bytes()
    if len(raw) != candidate.width * candidate.height or hashlib.sha256(raw).hexdigest() != snapshot["costmap_sha256"]:
        raise ValueError("costmap bytes/hash differ from the snapshot")
    if snapshot["unknown_cells"] == 0 or snapshot["fixed_center_cost"] < 253 or snapshot["start_cost"] >= 253:
        raise AssertionError("costmap has lost Unknown/fixed obstacle or blocked start")

    # PGM rows run top-to-bottom; Nav2 costmap rows run bottom-to-top.
    unknown = occupied = 0
    for cost_row in range(candidate.height):
        pgm_row = candidate.height - 1 - cost_row
        pgm_offset = pgm_row * candidate.width
        cost_offset = cost_row * candidate.width
        for column in range(candidate.width):
            pixel_class = candidate.classify_pixel(candidate.pixels[pgm_offset + column])
            cost = raw[cost_offset + column]
            if pixel_class == "unknown":
                unknown += 1
                if cost < 253:
                    raise AssertionError(f"source Unknown became traversable at costmap ({column}, {cost_row}): {cost}")
            elif pixel_class == "occupied":
                occupied += 1
                if cost < 253:
                    raise AssertionError(f"source Occupied became traversable at costmap ({column}, {cost_row}): {cost}")

    points = route.get("points", [])
    if len(points) < 2 or any(
        len(point) != 2 or not all(math.isfinite(float(value)) for value in point)
        for point in points
    ):
        raise ValueError("path needs finite XY pairs")
    samples = 0
    for start, end in zip(points[:-1], points[1:]):
        length = math.hypot(end[0] - start[0], end[1] - start[1])
        steps = max(1, math.ceil(length / sample_spacing))
        for index in range(steps + 1):
            t = index / steps
            x = float(start[0] + t * (end[0] - start[0]))
            y = float(start[1] + t * (end[1] - start[1]))
            column = math.floor((x - snapshot["origin_xy"][0]) / snapshot["resolution"])
            row = math.floor((y - snapshot["origin_xy"][1]) / snapshot["resolution"])
            if not (0 <= column < candidate.width and 0 <= row < candidate.height):
                raise AssertionError(f"Nav2 path left the costmap at ({x}, {y})")
            cost = raw[row * candidate.width + column]
            if cost >= 253:
                raise AssertionError(f"Nav2 path entered cost {cost} at ({x}, {y})")
            samples += 1
    return {
        "status": "real_nav2_costmap_and_path_geometry_pass_only_not_closed_loop",
        "scene_id": manifest["scene_id"],
        "source_unknown_cells_blocked": unknown,
        "source_occupied_cells_blocked": occupied,
        "nav2_unknown_cells": snapshot["unknown_cells"],
        "path_segments": len(points) - 1,
        "dense_costmap_samples_clear": samples,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-yaml", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--costmap-json", type=Path, required=True)
    parser.add_argument("--costmap-bin", type=Path, required=True)
    parser.add_argument("--nav2-path", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(verify_costmap(
        args.map_yaml, args.manifest, args.costmap_json, args.costmap_bin, args.nav2_path,
    ), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
