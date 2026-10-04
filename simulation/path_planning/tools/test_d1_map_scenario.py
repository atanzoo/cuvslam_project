#!/usr/bin/env python3
"""Tests for deterministic map-anchored dynamic scenario manifests."""

from __future__ import annotations

from pathlib import Path

from d1_candidate_map import load_candidate_map
from d1_map_scenario import SCENARIO_FAMILIES, generate_map_scenario


ROOT = Path(__file__).resolve().parents[1]
MAP_YAML = ROOT / "../../real_robot/cuvslam/evidence/maps/20260924_092630_current_cuvslam_a2m12/map.yaml"


def _candidate_map():
    return load_candidate_map(MAP_YAML, scene_scale=1.5)


def test_all_required_families_are_deterministic_and_map_anchored() -> None:
    candidate = _candidate_map()
    for index, family in enumerate(SCENARIO_FAMILIES):
        first = generate_map_scenario(
            candidate,
            seed=240924,
            curriculum_index=index,
            scenario_family=family,
            schedule_index=100 + index,
        )
        second = generate_map_scenario(
            candidate,
            seed=240924,
            curriculum_index=index,
            scenario_family=family,
            schedule_index=100 + index,
        )
        assert first.to_dict() == second.to_dict()
        assert first.curriculum_index == index
        assert first.schedule_index == 100 + index
        for point in (first.start, first.goal, *(actor.start for actor in first.actors)):
            column, row = candidate.world_to_cell(*point)
            assert candidate.classify_pixel(candidate.pixel(column, row)) == "free"
        assert first.map_manifest["scene_scale"] == 1.5
        assert first.geometry_validation == "anchors_known_free_only"


def test_multi_person_occlusion_contains_grouped_people() -> None:
    scenario = generate_map_scenario(
        _candidate_map(),
        seed=7,
        curriculum_index=7,
        scenario_family="multi_person_occlusion",
        schedule_index=7,
    )
    assert len(scenario.actors) == 3
    assert {actor.occlusion_group for actor in scenario.actors} == {"group_00"}
    assert {actor.motion_kind for actor in scenario.actors} == {
        "crossing", "same_direction", "static_occluder",
    }


def test_default_family_comes_from_curriculum_index() -> None:
    scenario = generate_map_scenario(
        _candidate_map(), seed=3, curriculum_index=9, schedule_index=12,
    )
    assert scenario.scenario_family == SCENARIO_FAMILIES[1]
    assert scenario.schedule_index == 12


def test_planner_component_can_supply_route_and_anchor_policy() -> None:
    from d1_map_planner import CandidateMapPlanner

    candidate = _candidate_map()
    planner = CandidateMapPlanner(candidate, robot_radius_m=0.3359, obstacle_margin_m=0.12)
    scenario = generate_map_scenario(
        candidate,
        seed=240924,
        curriculum_index=0,
        scenario_family="crossing",
        schedule_index=0,
        route_endpoints=planner.safe_route_endpoints(),
        anchor_cells=planner.largest_traversable_component(),
    )
    assert scenario.route_source == "planner_traversable_component"
    assert scenario.geometry_validation == "anchors_traversable_component"
    for point in (scenario.start, scenario.goal, *(actor.start for actor in scenario.actors)):
        column, row = candidate.world_to_cell(*point)
        assert planner.is_traversable_cell(column, row)


def main() -> None:
    test_all_required_families_are_deterministic_and_map_anchored()
    test_multi_person_occlusion_contains_grouped_people()
    test_default_family_comes_from_curriculum_index()
    test_planner_component_can_supply_route_and_anchor_policy()
    print("PASS map scenario static tests")


if __name__ == "__main__":
    main()
