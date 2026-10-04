#!/usr/bin/env python3
"""Smoke tests for the Stable-Baselines3 D1 decision environment."""

from __future__ import annotations

import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace

import numpy as np

from d1_edu_sb3_env import D1DecisionEnv, OBSERVATION_DIM
from d1_edu_nav2_global_path import GlobalPathReference, path_heading_command
from run_d1_edu_decision_layer_sb3 import (
    EpisodeOutcomeTrend,
    _candidate_map_identity,
    _validate_model_metadata,
)


ROOT = Path(__file__).resolve().parents[1]
MAP_YAML = ROOT / "../../real_robot/cuvslam/evidence/maps/20260924_092630_current_cuvslam_a2m12/map.yaml"


def test_episode_outcome_trend_counts_rates_per_step_window() -> None:
    trend = EpisodeOutcomeTrend(window_steps=5)
    trend.record_episode({"success": True, "collision": False, "timeout": False})
    trend.record_episode({"success": False, "collision": True, "timeout": False})
    trend.record_episode({"success": False, "collision": False, "timeout": True})

    first = trend.advance(5)
    assert len(first) == 1
    assert first[0]["start_step"] == 0
    assert first[0]["end_step"] == 5
    assert first[0]["episodes"] == 3
    assert first[0]["success_rate"] == 1 / 3
    assert first[0]["collision_rate"] == 1 / 3
    assert first[0]["timeout_rate"] == 1 / 3

    trend.record_episode({"success": True, "collision": False, "timeout": False})
    second = trend.advance(10)
    assert second[0]["start_step"] == 5
    assert second[0]["end_step"] == 10
    assert second[0]["success_rate"] == 1.0
    assert second[0]["collision_rate"] == 0.0
    assert second[0]["timeout_rate"] == 0.0


def test_map_checkpoint_identity_rejects_a_different_scale() -> None:
    args = SimpleNamespace(
        robot_profile="d1_max_proxy",
        kinematic_proxy=True,
        candidate_map_yaml=MAP_YAML,
        map_scene_scale=3.0,
    )
    identity = _candidate_map_identity(args)
    assert identity is not None
    with TemporaryDirectory(prefix="d1-map-checkpoint-") as temp_dir:
        model_path = Path(temp_dir) / "policy.zip"
        metadata_path = Path(temp_dir) / "policy.metadata.json"
        metadata = {
            "robot_profile": "d1_max_proxy",
            "kinematic_proxy": True,
            "candidate_map": identity,
        }
        metadata_path.write_text(json.dumps(metadata), encoding="utf-8")
        _validate_model_metadata(model_path, args)
        metadata["candidate_map"]["scene_scale"] = 1.5
        metadata_path.write_text(json.dumps(metadata), encoding="utf-8")
        try:
            _validate_model_metadata(model_path, args)
        except ValueError as error:
            assert "identity mismatch" in str(error)
        else:
            raise AssertionError("checkpoint from a different map scale was accepted")


def test_path_heading_command_is_bounded_and_respects_deadband() -> None:
    reference = GlobalPathReference(
        target_local=np.array([0.5, 0.5]),
        lateral_error=0.0,
        heading_error=0.5,
        remaining_distance=1.0,
        progress=0.5,
        nearest_index=0,
        remaining_fraction=0.5,
        reference_speed=0.22,
    )
    assert path_heading_command(reference, gain=2.0, max_rate=0.6) == 0.6
    reference = GlobalPathReference(
        **{**reference.__dict__, "heading_error": 0.01},
    )
    assert path_heading_command(reference, gain=2.0, max_rate=0.6) == 0.0


def main() -> None:
    env = D1DecisionEnv(
        profile="baseline",
        scenario_mode="multi_target",
        seed=17,
        duration=0.40,
        mppi_samples=4,
        mppi_horizon=5,
    )
    observation, info = env.reset(seed=17)
    assert observation.shape == (OBSERVATION_DIM,)
    assert env.observation_space.contains(observation)
    assert env.action_space.contains(0)
    assert env.action_space.n == 5
    next_observation, reward, terminated, truncated, step_info = env.step(0)
    assert next_observation.shape == (OBSERVATION_DIM,)
    assert np.isfinite(reward)
    assert isinstance(terminated, bool)
    assert isinstance(truncated, bool)
    assert "episode_seed" in step_info
    assert len(info["config"].scenario_obstacles) == 3
    assert info["robot_profile"] == "d1_edu"
    assert info["kinematic_proxy"] is False
    env.close()

    max_env = D1DecisionEnv(
        profile="baseline",
        robot_profile="d1_max_proxy",
        kinematic_proxy=True,
        scenario_mode="multi_target",
        seed=19,
        duration=0.20,
        mppi_samples=4,
        mppi_horizon=5,
    )
    max_observation, max_info = max_env.reset(seed=19)
    assert max_observation.shape == (OBSERVATION_DIM,)
    assert max_env.observation_space.contains(max_observation)
    assert max_info["robot_profile"] == "d1_max_proxy"
    assert max_info["kinematic_proxy"] is True
    assert max_info["config"].robot_profile == "d1_max_proxy"
    assert max_info["config"].kinematic_proxy is True
    assert max_info["config"].global_path_robot_radius >= 0.49
    assert max_env.map3_dual_target_controller is False
    max_env.step(0)
    max_env.close()

    map_env = D1DecisionEnv(
        profile="baseline",
        robot_profile="d1_max_proxy",
        kinematic_proxy=True,
        scenario_mode="people_balanced_multi_target",
        seed=23,
        duration=50.0,
        route_length=10.0,
        mppi_samples=4,
        mppi_horizon=5,
        candidate_map_yaml=MAP_YAML,
        map_scene_scale=3.0,
    )
    map_env.route_length = 10.0
    map_observation, map_info = map_env.reset(
        seed=23, options={"curriculum_index": 0},
    )
    assert map_observation.shape == (OBSERVATION_DIM,)
    assert map_env.observation_space.contains(map_observation)
    assert map_info["candidate_map"]["scene_scale"] == 3.0
    assert (map_env.candidate_map.width, map_env.candidate_map.height) == (597, 924)
    assert len(map_env.map_wall_geom_ids) == len(map_env.map_wall_obstacles)
    assert map_env.map_planner is not None
    assert np.isclose(map_env.global_robot_radius, 0.49)
    assert map_env.map_planner.inflation_cells == 13
    assert map_env.map_route_planner is not None
    assert map_env.map_route_planner.inflation_cells > map_env.map_planner.inflation_cells
    assert np.isclose(map_info["candidate_map"]["route_center_extra_buffer_m"], 0.25)
    assert map_info["candidate_map"]["map_wall_rectangle_count"] == len(
        map_env.map_wall_geom_ids
    )
    assert all(
        map_env.map_planner.is_traversable_world(float(point[0]), float(point[1]))
        for point in map_env.active_map_path
    )
    assert map_env.map_planner.is_traversable_world(
        float(map_env.data.qpos[0]), float(map_env.data.qpos[1]),
    )
    assert map_info["map_pedestrian_motion"] == "crossing"
    assert map_info["map_curriculum_index"] == 0
    assert map_info["config"].scenario_mode == "candidate_map_balanced_dual_target"
    assert np.isclose(
        np.linalg.norm(np.diff(map_env.active_map_path, axis=0), axis=1).sum(),
        10.0,
        atol=1e-6,
    )
    assert len(map_info["config"].scenario_obstacles) == 2
    fixed_object, first_actor = map_info["config"].scenario_obstacles
    assert fixed_object["kind"] == "object"
    assert fixed_object["motion_kind"] == "static"
    assert first_actor["kind"] == "person"
    assert first_actor["motion_kind"] == "crossing"
    first_path = map_env.active_map_path.copy()
    first_actor = dict(first_actor)
    map_observation, repeated_info = map_env.reset(
        seed=23, options={"curriculum_index": 0},
    )
    assert np.array_equal(first_path, map_env.active_map_path)
    assert first_actor == repeated_info["config"].scenario_obstacles[1]
    seen_motions: list[str] = []
    mirrored_offsets: dict[int, list[float]] = {}
    for curriculum_index in range(10):
        _, seeded_info = map_env.reset(
            seed=100 + (curriculum_index % 5),
            options={"curriculum_index": curriculum_index},
        )
        seen_motions.append(seeded_info["map_pedestrian_motion"])
        assert seeded_info["config"].scenario_mode == "candidate_map_balanced_dual_target"
        assert len(seeded_info["config"].scenario_obstacles) == 2
        object_spec, actor_spec = seeded_info["config"].scenario_obstacles
        assert object_spec["kind"] == "object" and object_spec["motion_kind"] == "static"
        assert actor_spec["kind"] == "person"
        route_cumulative = np.concatenate((
            [0.0],
            np.cumsum(np.linalg.norm(np.diff(map_env.active_map_path, axis=0), axis=1)),
        ))
        for elapsed in np.arange(0.0, 50.01, 0.10):
            position, _, active = map_env.trajectories[1].state(float(elapsed))
            if active:
                assert map_env.map_planner.is_traversable_world(
                    float(position[0]), float(position[1]),
                )
                fixed_position = np.array([object_spec["x"], object_spec["y"]])
                assert np.linalg.norm(position - fixed_position) > (
                    np.hypot(actor_spec["half_length"], actor_spec["half_width"])
                    + np.hypot(object_spec["half_length"], object_spec["half_width"])
                )
        assert all(
            map_env.map_planner.is_traversable_world(float(point[0]), float(point[1]))
            for point in map_env.active_map_path
        )
        assert np.isclose(route_cumulative[-1], 10.0, atol=1e-6)
        actor = map_env.trajectories[1]
        if actor.motion_kind == "crossing":
            center_position = actor.state(actor.disappear_time / 2.0)[0]
            velocity = np.array([actor.vx, actor.vy])
            projected_position = center_position
        else:
            projected_position = np.array([actor.x, actor.y])
            velocity = np.zeros(2)
        nearest_index = int(np.argmin(np.linalg.norm(
            map_env.active_map_path - projected_position, axis=1,
        )))
        _, _, normal = map_env._route_frame_at(
            map_env.active_map_path, route_cumulative,
            float(route_cumulative[nearest_index]),
        )
        signed = float(
            np.dot(velocity, normal) if actor.motion_kind == "crossing"
            else np.dot(projected_position - map_env.active_map_path[nearest_index], normal)
        )
        mirrored_offsets.setdefault(curriculum_index, []).append(signed)
    assert seen_motions == [
        "crossing", "same_direction", "accelerating", "turning", "irregular",
        "crossing", "same_direction", "accelerating", "turning", "irregular",
    ]
    for family_index in range(5):
        assert mirrored_offsets[family_index][0] * mirrored_offsets[family_index + 5][0] < 0.0, (
            family_index,
            mirrored_offsets[family_index][0],
            mirrored_offsets[family_index + 5][0],
        )
    _, map_reward, map_terminated, map_truncated, map_step_info = map_env.step(0)
    assert np.isfinite(map_reward)
    assert isinstance(map_terminated, bool)
    assert isinstance(map_truncated, bool)
    assert map_step_info["map_violation"] is False
    map_env.close()
    test_episode_outcome_trend_counts_rates_per_step_window()
    test_map_checkpoint_identity_rejects_a_different_scale()
    test_path_heading_command_is_bounded_and_respects_deadband()
    print("PASS SB3 environment reset/step smoke test")
    print("PASS D1 Max kinematic-proxy environment reset/step smoke test")
    print("PASS 3x 10 m candidate-map D1 Max proxy dual-target motion curriculum smoke test")
    print("PASS per-5k training success/timeout/collision trend aggregation")
    print("PASS map hash/scale checkpoint compatibility guard test")
    print("status=PASS: Stable-Baselines3 D1 Edu and D1 Max proxy environments are callable")


if __name__ == "__main__":
    main()
