#!/usr/bin/env python3
"""Static tests for conservative candidate-map routing."""

from __future__ import annotations

from pathlib import Path

from d1_candidate_map import load_candidate_map
from d1_dynamic_risk import assess_swept_arc_risk, compare_candidate_trajectories, ctrv_prediction
from d1_map_planner import CandidateMapPlanner
from d1_map_scenario import generate_map_scenario


ROOT = Path(__file__).resolve().parents[1]
MAP_YAML = ROOT / "../../real_robot/cuvslam/evidence/maps/20260924_092630_current_cuvslam_a2m12/map.yaml"


def _planner(*, scene_scale: float = 1.5, preserve_resolution: bool = False) -> CandidateMapPlanner:
    candidate = load_candidate_map(
        MAP_YAML, scene_scale=scene_scale, preserve_resolution=preserve_resolution,
    )
    return CandidateMapPlanner(candidate, robot_radius_m=0.3359, obstacle_margin_m=0.12)


def test_inflated_map_has_a_large_traversable_component() -> None:
    planner = _planner()
    assert planner.inflation_cells == 7
    assert planner.traversable_cell_count() > 1000
    assert len(planner.largest_traversable_component()) > 1000


def test_five_x_preserved_resolution_route_uses_fixed_robot_geometry() -> None:
    planner = _planner(scene_scale=5.0, preserve_resolution=True)
    assert planner.inflation_cells == 10
    assert planner.robot_radius_m == 0.3359
    assert planner.obstacle_margin_m == 0.12
    assert planner.traversable_cell_count() > 400_000
    start, goal = planner.safe_route_endpoints()
    route = planner.plan(start, goal)
    assert route.length_m > 50.0
    assert all(planner.is_traversable_cell(*cell) for cell in route.cells)


def test_a_star_route_stays_inside_inflated_free_space() -> None:
    planner = _planner()
    start, goal = planner.safe_route_endpoints()
    route = planner.plan(start, goal)
    assert len(route.cells) > 2
    assert route.length_m > 1.0
    assert all(planner.is_traversable_cell(*cell) for cell in route.cells)
    assert route.robot_radius_m == 0.3359
    assert route.obstacle_margin_m == 0.12


def test_scenario_validation_reports_segment_and_swept_arc_boundaries() -> None:
    planner = _planner()
    scenario = generate_map_scenario(
        planner.candidate_map,
        seed=240924,
        curriculum_index=4,
        scenario_family="turning",
        schedule_index=4,
    )
    result = planner.validate_scenario(scenario)
    assert "valid" in result
    assert "trajectory_validation" in result
    assert result["trajectory_validation"] == "grid_path_between_waypoints"
    assert result["swept_arc_validation"] == "not_run"
    assert result["dynamic_safety_ready"] is False
    assert len(result["actors"]) == 1


def test_unknown_cell_is_not_traversable() -> None:
    planner = _planner()
    candidate = planner.candidate_map
    unknown = next(
        (column, row)
        for row in range(candidate.height)
        for column in range(candidate.width)
        if candidate.classify_pixel(candidate.pixel(column, row)) == "unknown"
    )
    assert not planner.is_traversable_cell(*unknown)
    assert not planner.is_traversable_world(*candidate.cell_to_world(*unknown))
    assert not planner.is_traversable_world(-1_000.0, -1_000.0)


def test_turning_swept_arc_detects_intersection_and_ttc() -> None:
    times = (0.0, 0.5, 1.0, 1.5, 2.0)
    obstacle = ctrv_prediction((1.0, 0.0), (0.0, 1.0), 1.0, times)
    robot = tuple((1.0, 0.0) for _ in times)
    result = assess_swept_arc_risk(
        robot,
        obstacle,
        times,
        robot_radius_m=0.20,
        obstacle_radius_m=0.20,
        uncertainty_radius_m=(0.0,) * len(times),
    )
    assert result.swept_arc_intersects is True
    assert result.ttc_s == 0.0
    assert result.predicted_min_distance_m == 0.0


def test_left_right_candidate_comparison_prefers_clear_side() -> None:
    times = (0.0, 1.0, 2.0)
    obstacle = tuple((1.0, 0.0) for _ in times)
    candidates = {
        "PASS_LEFT": tuple((0.0, 2.0) for _ in times),
        "PASS_RIGHT": tuple((0.0, 0.1) for _ in times),
    }
    results = compare_candidate_trajectories(
        candidates,
        obstacle,
        times,
        robot_radius_m=0.10,
        obstacle_radius_m=0.10,
    )
    assert results["PASS_LEFT"].swept_arc_intersects is False
    assert results["PASS_RIGHT"].swept_arc_intersects is True


def main() -> None:
    test_inflated_map_has_a_large_traversable_component()
    test_five_x_preserved_resolution_route_uses_fixed_robot_geometry()
    test_a_star_route_stays_inside_inflated_free_space()
    test_scenario_validation_reports_segment_and_swept_arc_boundaries()
    test_unknown_cell_is_not_traversable()
    print("PASS map planner static tests")


if __name__ == "__main__":
    main()
