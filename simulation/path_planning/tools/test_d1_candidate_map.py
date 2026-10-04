#!/usr/bin/env python3
"""Static tests for candidate-map loading and scaled coordinate semantics."""

from __future__ import annotations

import math
from pathlib import Path

from d1_candidate_map import load_candidate_map, occupied_rectangles


ROOT = Path(__file__).resolve().parents[1]
MAP_YAML = ROOT / "../../real_robot/cuvslam/evidence/maps/20260924_092630_current_cuvslam_a2m12/map.yaml"


def test_candidate_map_metadata_and_counts() -> None:
    candidate = load_candidate_map(MAP_YAML)
    assert (candidate.width, candidate.height) == (199, 308)
    assert candidate.metadata.resolution == 0.05
    assert candidate.metadata.origin == (-3.89, -5.34, 0.0)
    assert candidate.counts() == {"unknown": 28502, "occupied": 4385, "free": 28405}
    assert len(candidate.largest_free_component()) == 27222


def test_coordinate_round_trip_preserves_row_orientation() -> None:
    candidate = load_candidate_map(MAP_YAML)
    for column, row in ((0, 0), (11, 0), (100, 153), (198, 307)):
        x, y = candidate.cell_to_world(column, row)
        assert candidate.world_to_cell(x, y) == (column, row)


def test_scene_scale_does_not_change_pixels_or_topology() -> None:
    candidate = load_candidate_map(MAP_YAML, scene_scale=1.5)
    assert candidate.scene_scale == 1.5
    assert math.isclose(candidate.scaled_resolution, 0.075, rel_tol=0.0, abs_tol=1e-12)
    assert all(
        math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-12)
        for actual, expected in zip(candidate.scaled_origin, (-5.835, -8.01, 0.0))
    )
    assert all(
        math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-12)
        for actual, expected in zip(candidate.world_extent, (-5.835, -8.01, 9.09, 15.09))
    )
    assert candidate.counts() == {"unknown": 28502, "occupied": 4385, "free": 28405}
    assert len(candidate.largest_free_component()) == 27222


def test_five_x_scene_preserves_five_centimeter_resolution() -> None:
    candidate = load_candidate_map(MAP_YAML, scene_scale=5.0, preserve_resolution=True)
    assert (candidate.width, candidate.height) == (995, 1540)
    assert (candidate.source_width, candidate.source_height) == (199, 308)
    assert math.isclose(candidate.scaled_resolution, 0.05, rel_tol=0.0, abs_tol=1e-12)
    assert all(
        math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-12)
        for actual, expected in zip(candidate.world_extent, (-19.45, -26.70, 30.30, 50.30))
    )
    assert candidate.pixel(5 * 41 + 2, 5 * 73 + 4) == candidate.pixel(5 * 41, 5 * 73)
    assert candidate.counts() == {"unknown": 712550, "occupied": 109625, "free": 710125}
    assert candidate.manifest()["source_pixel_counts"] == {
        "unknown": 28502, "occupied": 4385, "free": 28405,
    }


def test_occupied_wall_rectangles_do_not_promote_unknown_to_walls() -> None:
    candidate = load_candidate_map(MAP_YAML, scene_scale=5.0, preserve_resolution=True)
    walls = occupied_rectangles(candidate)
    assert 0 < len(walls) < 2_000
    for wall in walls:
        assert wall.half_length > 0.0
        assert wall.half_width > 0.0
        cell = candidate.world_to_cell(wall.x, wall.y)
        assert candidate.classify_pixel(candidate.pixel(*cell)) == "occupied"


def test_invalid_scale_is_rejected() -> None:
    for scale in (0.0, -1.0, float("inf")):
        try:
            load_candidate_map(MAP_YAML, scene_scale=scale)
        except ValueError as error:
            assert "scene_scale" in str(error)
        else:
            raise AssertionError("invalid scene scale was accepted")


def main() -> None:
    for test in (
        test_candidate_map_metadata_and_counts,
        test_coordinate_round_trip_preserves_row_orientation,
        test_scene_scale_does_not_change_pixels_or_topology,
        test_five_x_scene_preserves_five_centimeter_resolution,
        test_occupied_wall_rectangles_do_not_promote_unknown_to_walls,
        test_invalid_scale_is_rejected,
    ):
        test()
    print("PASS candidate map static tests")


if __name__ == "__main__":
    main()
