#!/usr/bin/env python3
"""Check a real Nav2 path against the exported map and fixed-box geometry."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from d1_candidate_map import load_candidate_map
from d1_map_planner import CandidateMapPlanner


def verify_path(
    map_yaml: str | Path,
    manifest_json: str | Path,
    path_json: str | Path,
    *,
    sample_spacing: float = 0.025,
    robot_radius: float = 0.49,
    obstacle_margin: float = 0.12,
    expected_start: tuple[float, float] | None = None,
) -> dict[str, object]:
    if sample_spacing <= 0.0 or robot_radius <= 0.0 or obstacle_margin < 0.0:
        raise ValueError("spacing, radius and margin must be valid")
    manifest = json.loads(Path(manifest_json).read_text(encoding="utf-8"))
    result = json.loads(Path(path_json).read_text(encoding="utf-8"))
    if result.get("scene_id") != manifest.get("scene_id"):
        raise ValueError("Nav2 path and map scene identities differ")
    if result.get("status") != "real_nav2_planner_path_unverified_geometry":
        raise ValueError("path has not been identified as a Nav2 action result")
    if result.get("action") != "compute_path_to_pose" or result.get("frame_id") != "map":
        raise ValueError("path is not a map-frame Nav2 planner result")
    path = np.asarray(result.get("points", []), dtype=float)
    if path.ndim != 2 or path.shape[1] != 2 or len(path) < 2 or not np.isfinite(path).all():
        raise ValueError("path must contain at least two finite XY points")
    requested_goal = np.asarray(result.get("goal", []), dtype=float)
    if requested_goal.shape != (2,) or not np.isfinite(requested_goal).all():
        raise ValueError("Nav2 result must record a finite requested goal")
    goal_error = float(np.linalg.norm(path[-1] - requested_goal))
    if goal_error > 0.10:
        raise AssertionError(f"Nav2 path ends {goal_error:.3f} m from requested goal")
    start_error = None
    if expected_start is not None:
        expected = np.asarray(expected_start, dtype=float)
        if expected.shape != (2,) or not np.isfinite(expected).all():
            raise ValueError("expected_start must be finite XY")
        start_error = float(np.linalg.norm(path[0] - expected))
        if start_error > 0.10:
            raise AssertionError(f"Nav2 path starts {start_error:.3f} m from expected start")
    candidate = load_candidate_map(map_yaml)
    if (candidate.width, candidate.height) != (manifest["width"], manifest["height"]):
        raise ValueError("exported map dimensions do not match manifest")
    if (
        candidate.metadata.resolution != manifest["resolution"]
        or list(candidate.metadata.origin) != manifest["origin"]
        or hashlib.sha256(candidate.yaml_path.read_bytes()).hexdigest() != manifest["map_yaml_sha256"]
        or hashlib.sha256(candidate.pgm_path.read_bytes()).hexdigest() != manifest["map_pgm_sha256"]
    ):
        raise ValueError("exported map identity does not match manifest")
    planner = CandidateMapPlanner(
        candidate, robot_radius_m=robot_radius, obstacle_margin_m=obstacle_margin,
    )
    sampled = [path[0]]
    for first, second in zip(path[:-1], path[1:]):
        distance = float(np.linalg.norm(second - first))
        steps = max(1, math.ceil(distance / sample_spacing))
        sampled.extend(np.linspace(first, second, steps + 1)[1:])
    points = np.asarray(sampled, dtype=float)
    traversable = np.asarray([
        planner.is_traversable_world(float(x), float(y)) for x, y in points
    ])
    box = manifest["static_box"]
    edge_distance = np.hypot(
        np.maximum(np.abs(points[:, 0] - float(box["x"])) - float(box["half_length"]), 0.0),
        np.maximum(np.abs(points[:, 1] - float(box["y"])) - float(box["half_width"]), 0.0),
    )
    minimum_edge = float(np.min(edge_distance))
    if not traversable.all() or minimum_edge < robot_radius + obstacle_margin:
        raise AssertionError(
            f"Nav2 path unsafe: traversable={int(np.count_nonzero(traversable))}/{len(points)}, "
            f"min_fixed_edge={minimum_edge:.4f} m, required={robot_radius + obstacle_margin:.4f} m"
        )
    return {
        "status": "nav2_path_geometry_pass_only_not_closed_loop",
        "scene_id": manifest["scene_id"],
        "nav2_path_points": len(path),
        "dense_samples": len(points),
        "min_fixed_edge_distance_m": minimum_edge,
        "required_distance_m": robot_radius + obstacle_margin,
        "all_samples_traversable": True,
        "path_length_m": float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))),
        "goal_error_m": goal_error,
        "start_error_m": start_error,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-yaml", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--nav2-path", type=Path, required=True)
    parser.add_argument("--start-x", type=float)
    parser.add_argument("--start-y", type=float)
    args = parser.parse_args()
    if (args.start_x is None) != (args.start_y is None):
        parser.error("--start-x and --start-y must be provided together")
    print(json.dumps(
        verify_path(
            args.map_yaml, args.manifest, args.nav2_path,
            expected_start=(args.start_x, args.start_y) if args.start_x is not None else None,
        ),
        indent=2,
        sort_keys=True,
    ))


if __name__ == "__main__":
    main()
