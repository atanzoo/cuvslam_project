#!/usr/bin/env python3
"""Tests for long-route obstacle generation and RPLIDAR A2 scene wiring."""

from __future__ import annotations

from dataclasses import replace
import sys

import numpy as np

from d1_edu_scenario import (
    SCENARIO_KINDS,
    generate_long_route_scenario,
    generate_multi_target_scenario,
    generate_people_balanced_multi_target_scenario,
    generate_people_random_multi_target_scenario,
)
from run_d1_edu_decision_layer_benchmark import sample_episode_config
from run_d1_edu_decision_layer_viewer import (
    MultiTargetTracker,
    make_decision_observation,
    run_episode,
)
from run_d1_edu_mppi_obstacle_viewer import ObstacleMPPI
from run_d1_edu_obstacle_avoidance_viewer import Obstacle
from d1_edu_nav2_global_path import (
    Nav2GlobalPath,
    estimate_wait_yield,
    path_lateral_command,
)


def test_scenario_is_seeded_and_capped() -> None:
    first = generate_long_route_scenario(123, route_length=30.0, max_obstacle_speed=7.0)
    second = generate_long_route_scenario(123, route_length=30.0, max_obstacle_speed=7.0)
    assert first.to_dict() == second.to_dict()
    assert first.route_length == 30.0
    assert {obstacle.kind for obstacle in first.obstacles} == set(SCENARIO_KINDS)
    for obstacle in first.obstacles:
        for elapsed in np.linspace(0.0, 120.0, 25):
            speed = np.linalg.norm(obstacle.state(float(elapsed))[1])
            assert speed <= 7.0 + 1e-9


def test_long_route_config_uses_rplidar_a2_profile() -> None:
    config = sample_episode_config(
        "baseline",
        np.random.default_rng(0),
        123,
        route_length=30.0,
        scenario_mode="long_route",
    )
    assert config.goal_x == 30.0
    assert config.scenario_mode == "long_route"
    assert config.lidar_model == "RPLIDAR_A2M8"
    assert config.lidar_ray_count == 800
    assert np.isclose(config.lidar_fov, 2.0 * np.pi)
    assert config.lidar_max_range == 12.0
    assert len(config.scenario_obstacles) == 6


def test_multi_obstacle_mujoco_smoke() -> None:
    config = sample_episode_config(
        "baseline",
        np.random.default_rng(1),
        456,
        route_length=30.0,
        scenario_mode="long_route",
    )
    config = replace(
        config,
        duration=0.50,
        goal_x=0.8,
        mppi_samples=4,
        mppi_horizon=5,
    )
    result = run_episode(config, seed=456)
    assert result.elapsed_time > 0.0
    assert len(result.mode_history) > 0


def test_multi_target_scenario_is_seeded_and_has_multiple_boxes() -> None:
    first = generate_multi_target_scenario(321, obstacle_count=3)
    second = generate_multi_target_scenario(321, obstacle_count=3)
    assert first.to_dict() == second.to_dict()
    assert len(first.obstacles) == 3
    assert {obstacle.kind for obstacle in first.obstacles} == {
        "static", "crossing", "same_direction",
    }


def test_irregular_multi_target_has_non_linear_motion_and_uncertainty() -> None:
    first = generate_multi_target_scenario(321, obstacle_count=3, irregular=True)
    second = generate_multi_target_scenario(321, obstacle_count=3, irregular=True)
    assert first.to_dict() == second.to_dict()
    assert {obstacle.kind for obstacle in first.obstacles} == {
        "static", "crossing", "irregular",
    }
    irregular = next(obstacle for obstacle in first.obstacles if obstacle.kind == "irregular")
    samples = np.asarray([irregular.state(float(t))[0] for t in (0.0, 0.5, 1.0, 1.5)])
    assert np.linalg.norm(samples[2] - 2.0 * samples[1] + samples[0]) > 1e-3
    config = sample_episode_config(
        "baseline",
        np.random.default_rng(7),
        321,
        scenario_mode="irregular_multi_target",
    )
    assert config.clearance_uncertainty > 0.0
    assert config.obstacle_velocity_uncertainty > 0.0


def test_people_multi_target_has_a_dynamic_pedestrian_proxy() -> None:
    first = generate_multi_target_scenario(
        987,
        route_length=10.0,
        obstacle_count=3,
        irregular=True,
        include_person=True,
    )
    second = generate_multi_target_scenario(
        987,
        route_length=10.0,
        obstacle_count=3,
        irregular=True,
        include_person=True,
    )
    assert first.to_dict() == second.to_dict()
    person = next(obstacle for obstacle in first.obstacles if obstacle.kind == "person")
    assert person.height == 0.85
    assert person.half_length == person.half_width == 0.18
    positions = np.asarray([person.state(float(t))[0] for t in (0.0, 1.0, 2.0, 3.0)])
    assert np.linalg.norm(positions[-1] - positions[0]) > 0.10
    config = sample_episode_config(
        "baseline",
        np.random.default_rng(9),
        987,
        route_length=10.0,
        scenario_mode="people_multi_target",
    )
    assert config.scenario_mode == "people_multi_target"
    assert any(item["kind"] == "person" for item in config.scenario_obstacles)


def test_people_multi_target_mujoco_smoke() -> None:
    config = sample_episode_config(
        "baseline",
        np.random.default_rng(10),
        988,
        route_length=10.0,
        scenario_mode="people_multi_target",
    )
    config = replace(config, duration=0.50, goal_x=0.8, mppi_samples=4, mppi_horizon=5)
    result = run_episode(config, seed=988, record_trace=True)
    assert result.elapsed_time > 0.0
    assert len(result.trace) >= 1
    assert any(item["kind"] == "person" for item in result.trace[0]["obstacles"])


def test_random_people_object_scene_has_two_people_and_random_motion() -> None:
    first = generate_people_random_multi_target_scenario(4321, route_length=10.0)
    second = generate_people_random_multi_target_scenario(4321, route_length=10.0)
    assert first.to_dict() == second.to_dict()
    assert len(first.obstacles) == 3
    assert [obstacle.kind for obstacle in first.obstacles].count("person") == 2
    assert [obstacle.kind for obstacle in first.obstacles].count("object") == 1
    motion_kinds = {obstacle.motion_kind for obstacle in first.obstacles}
    assert motion_kinds <= {
        "static", "crossing", "same_direction", "accelerating", "turning", "irregular",
    }
    config = sample_episode_config(
        "baseline",
        np.random.default_rng(12),
        4321,
        route_length=10.0,
        scenario_mode="people_random_multi_target",
    )
    assert len(config.scenario_obstacles) == 3
    assert sum(item["kind"] == "person" for item in config.scenario_obstacles) == 2
    assert sum(item["kind"] == "object" for item in config.scenario_obstacles) == 1


def test_random_people_object_mujoco_smoke() -> None:
    config = sample_episode_config(
        "baseline",
        np.random.default_rng(13),
        4322,
        route_length=10.0,
        scenario_mode="people_random_multi_target",
    )
    config = replace(config, duration=0.50, goal_x=0.8, mppi_samples=4, mppi_horizon=5)
    result = run_episode(config, seed=4322, record_trace=True)
    assert result.elapsed_time > 0.0
    assert len(result.trace) >= 1
    assert sum(item["kind"] == "person" for item in result.trace[0]["obstacles"]) == 2


def test_balanced_people_object_scene_cycles_motion_and_mirrors() -> None:
    expected = {
        "static", "crossing", "same_direction",
        "accelerating", "turning", "irregular",
    }
    per_target = [set(), set(), set()]
    first_cycle_y: list[list[float]] = []
    mirrored_cycle_y: list[list[float]] = []
    for index in range(12):
        scenario = generate_people_balanced_multi_target_scenario(
            5000 + index,
            route_length=10.0,
            curriculum_index=index,
        )
        assert len(scenario.obstacles) == 3
        for target_index, obstacle in enumerate(scenario.obstacles):
            per_target[target_index].add(str(obstacle.motion_kind))
        positions = [float(obstacle.y) for obstacle in scenario.obstacles]
        (first_cycle_y if index < 6 else mirrored_cycle_y).append(positions)
    assert all(motions == expected for motions in per_target)
    assert np.sign(first_cycle_y[0][0]) == -np.sign(mirrored_cycle_y[0][0])

    config = sample_episode_config(
        "full_randomization",
        np.random.default_rng(14),
        5000,
        route_length=10.0,
        scenario_mode="people_balanced_multi_target",
        curriculum_index=7,
    )
    assert config.scenario_mode == "people_balanced_multi_target"
    assert len(config.scenario_obstacles) == 3


def test_multi_target_tracker_associates_two_clusters() -> None:
    tracker = MultiTargetTracker()
    points = np.array([
        [1.00, 0.30], [1.02, 0.30], [1.04, 0.30],
        [1.00, -0.50], [1.02, -0.50], [1.04, -0.50],
    ])
    rays = np.array([10, 11, 12, 30, 31, 32])
    tracker.update(points, np.zeros(2), 0.0, 0.0, rays)
    tracker.update(points + np.array([0.02, 0.0]), np.zeros(2), 0.0, 0.1, rays)
    assert tracker.track_count == 2
    predictions = tracker.predictions(np.zeros(2), np.zeros(2), 0.0)
    assert len(predictions) == 2
    assert {prediction["track"].track_id for prediction in predictions} == {0, 1}
    risk, _, closest = tracker.collision_prediction(
        np.zeros(2), np.array([0.2, 0.0]), 0.0
    )
    assert risk
    assert closest < tracker.effective_safe_distance


def test_tracker_predicts_three_seconds_and_ranks_risk_fields() -> None:
    tracker = MultiTargetTracker()
    points = np.array([
        [1.00, 0.00], [1.02, 0.00], [1.04, 0.00],
        [1.00, 0.80], [1.02, 0.80], [1.04, 0.80],
    ])
    rays = np.array([10, 11, 12, 30, 31, 32])
    tracker.update(points, np.zeros(2), 0.0, 0.0, rays)
    tracker.update(points, np.zeros(2), 0.0, 0.1, rays)
    ranked = tracker.predictions(
        np.zeros(2), np.array([0.20, 0.0]), 0.0,
        horizon=2.5,
        sample_dt=0.5,
    )
    assert len(ranked) == 2
    assert ranked[0]["risk_rank"] == 1
    assert ranked[1]["risk_rank"] == 2
    for prediction in ranked:
        assert prediction["trajectory_world"].shape == (6, 2)
        assert prediction["trajectory_local"].shape == (6, 2)
        assert np.isclose(prediction["prediction_times"][-1], 2.5)
        assert 0.0 <= prediction["intersection_angle_deg"] <= 180.0
        assert 0.0 <= prediction["risk_score"] <= 1.0
    assert ranked[0]["ttc"] <= ranked[1]["ttc"]
    observation = make_decision_observation(
        np.array([2.0, 0.0]),
        np.zeros(2),
        ranked[0]["position_local"],
        ranked[0]["velocity_local"],
        ranked[0]["track"].confidence,
        ranked[0]["risk"],
        ranked[0]["time_to_closest"],
        ranked[0]["closest_distance"],
        0.0,
        10.0,
        tracked_obstacles=ranked,
    )
    assert observation.shape == (45,)


def test_nav2_global_path_reference_guides_rejoin() -> None:
    global_path = Nav2GlobalPath(lookahead=0.80, spacing=0.10)
    path = global_path.plan(np.array([0.0, 0.0]), np.array([3.2, 0.0]))
    assert path.shape[1] == 2
    reference = global_path.reference(np.array([1.0, -1.0]), yaw=0.0)
    assert np.isclose(reference.lateral_error, -1.0, atol=1e-6)
    assert reference.target_local[1] > 0.0
    assert path_lateral_command(
        reference,
        gain=0.60,
        deadband=0.05,
        max_speed=0.22,
    ) > 0.0
    curved_path = np.array([
        [0.0, 0.0], [1.0, 0.0], [2.0, 0.5], [3.2, 0.0],
    ])
    global_path.set_path(curved_path)
    curved_reference = global_path.reference(np.array([1.5, 0.25]), yaw=0.0)
    assert 0.0 <= curved_reference.progress <= 1.0
    assert curved_reference.remaining_distance >= 0.0


def test_nav2_costmap_planner_detours_around_obstacle() -> None:
    planner = Nav2GlobalPath(
        lookahead=0.80,
        spacing=0.10,
        grid_resolution=0.10,
        search_padding=1.0,
        soft_cost_radius=0.12,
        cost_weight=1.0,
    )
    path = planner.plan(
        np.array([0.0, 0.0]),
        np.array([3.2, 0.0]),
        obstacles=[Obstacle(
            x=1.6,
            y=0.0,
            half_length=0.25,
            half_width=0.50,
            height=0.30,
        )],
        robot_radius=0.25,
        obstacle_margin=0.12,
    )
    assert planner.plan_status == "astar_costmap"
    assert planner.costmap.size > 0
    assert planner.occupied.any()
    assert planner.total_length > 3.2
    assert float(np.max(np.abs(path[:, 1]))) >= 0.80


def test_path_reference_extends_ppo_observation() -> None:
    global_path = Nav2GlobalPath(lookahead=0.80, spacing=0.10)
    global_path.plan(np.array([0.0, 0.0]), np.array([3.2, 0.0]))
    reference = global_path.reference(np.array([0.0, 0.0]), yaw=0.0)
    observation = make_decision_observation(
        np.array([2.0, 0.0]),
        np.zeros(2),
        None,
        None,
        0,
        False,
        float("inf"),
        float("inf"),
        0.0,
        10.0,
        path_reference=reference,
    )
    assert observation.shape == (52,)


def test_wait_yield_estimate_detects_predicted_path_crossing() -> None:
    planner = Nav2GlobalPath(lookahead=0.80, spacing=0.10)
    planner.plan(np.array([0.0, 0.0]), np.array([3.0, 0.0]))
    track = type("TrackView", (), {"confidence": 4})()
    prediction = {
        "track": track,
        "trajectory_world": np.array([
            [1.0, 1.0],
            [1.0, 0.5],
            [1.0, 0.0],
            [1.0, -0.5],
            [1.0, -1.0],
        ]),
        "prediction_times": np.array([0.0, 0.5, 1.0, 1.5, 2.0]),
    }
    estimate = estimate_wait_yield(planner, [prediction], np.array([0.0, 0.0]))
    assert estimate.path_blocked_now == 0.0
    assert estimate.path_blocked_future == 1.0
    assert 0.0 < estimate.time_to_path_block < 1.0
    assert estimate.blocking_duration > 0.0


def test_wait_yield_only_allows_a_moving_blocker_that_clears() -> None:
    planner = Nav2GlobalPath(lookahead=0.80, spacing=0.10)
    planner.plan(np.array([0.0, 0.0]), np.array([3.0, 0.0]))
    track = type("TrackView", (), {"confidence": 4})()
    times = np.arange(0.0, 2.01, 0.25)
    moving_prediction = {
        "track": track,
        "velocity_world": np.array([0.0, 0.60]),
        "trajectory_world": np.column_stack((
            np.full(times.shape, 1.0),
            0.50 + 0.60 * times,
        )),
        "prediction_times": times,
    }
    estimate = estimate_wait_yield(
        planner,
        [moving_prediction],
        np.array([0.0, 0.0]),
        max_wait_seconds=2.0,
    )
    assert estimate.path_blocked_now == 1.0
    assert estimate.waitable == 1.0
    assert estimate.stationary_collision_risk == 0.0

    static_prediction = dict(moving_prediction)
    static_prediction["velocity_world"] = np.zeros(2)
    static_estimate = estimate_wait_yield(
        planner,
        [static_prediction],
        np.array([0.0, 0.0]),
        max_wait_seconds=2.0,
    )
    assert static_estimate.waitable == 0.0


def test_mppi_accepts_multiple_dynamic_predictions() -> None:
    planner = ObstacleMPPI(horizon=6, samples=12, rng_seed=2)
    command = planner.update(
        np.zeros(3),
        np.empty((0, 2)),
        np.array([2.0, 0.0]),
        dynamic_obstacles_local=[
            (np.array([1.0, 0.45]), np.array([0.0, -0.1]), 0.35),
            (np.array([1.2, -0.45]), np.array([0.0, 0.1]), 0.35),
        ],
    )
    assert command.shape == (3,)
    assert np.all(np.isfinite(command))


def test_mppi_accepts_ranked_trajectory_predictions() -> None:
    planner = ObstacleMPPI(horizon=8, samples=12, rng_seed=3)
    trajectory = np.array([
        [1.0, 0.45], [0.95, 0.40], [0.90, 0.35],
        [0.85, 0.30], [0.80, 0.25],
    ])
    command = planner.update(
        np.zeros(3),
        np.empty((0, 2)),
        np.array([2.0, 0.0]),
        dynamic_trajectories_local=[
            (trajectory, np.array([-0.2, -0.2]), 0.35),
        ],
        trajectory_sample_dt=0.25,
    )
    assert command.shape == (3,)
    assert np.all(np.isfinite(command))


def test_mppi_robust_clearance_accepts_uncertainty_and_delay() -> None:
    planner = ObstacleMPPI(
        horizon=8,
        samples=12,
        rng_seed=4,
        clearance_uncertainty=0.06,
        dynamic_position_uncertainty=0.05,
        dynamic_velocity_uncertainty=0.16,
        command_delay=0.08,
    )
    command = planner.update(
        np.zeros(3),
        np.empty((0, 2)),
        np.array([2.0, 0.0]),
        dynamic_trajectories_local=[(
            np.array([[1.0, 0.20], [0.95, 0.10], [0.90, 0.00]]),
            np.array([-0.1, -0.2]),
            0.35,
        )],
    )
    assert command.shape == (3,)
    assert np.all(np.isfinite(command))


def main() -> None:
    tests = [
        test_scenario_is_seeded_and_capped,
        test_long_route_config_uses_rplidar_a2_profile,
        test_multi_obstacle_mujoco_smoke,
        test_multi_target_scenario_is_seeded_and_has_multiple_boxes,
        test_irregular_multi_target_has_non_linear_motion_and_uncertainty,
        test_people_multi_target_has_a_dynamic_pedestrian_proxy,
        test_people_multi_target_mujoco_smoke,
        test_random_people_object_scene_has_two_people_and_random_motion,
        test_random_people_object_mujoco_smoke,
        test_balanced_people_object_scene_cycles_motion_and_mirrors,
        test_multi_target_tracker_associates_two_clusters,
        test_tracker_predicts_three_seconds_and_ranks_risk_fields,
        test_nav2_global_path_reference_guides_rejoin,
        test_nav2_costmap_planner_detours_around_obstacle,
        test_path_reference_extends_ppo_observation,
        test_wait_yield_estimate_detects_predicted_path_crossing,
        test_wait_yield_only_allows_a_moving_blocker_that_clears,
        test_mppi_accepts_multiple_dynamic_predictions,
        test_mppi_accepts_ranked_trajectory_predictions,
        test_mppi_robust_clearance_accepts_uncertainty_and_delay,
    ]
    for test in tests:
        test()
        print(f"PASS {test.__name__}")
    print("status=PASS: long-route scenario suite")


if __name__ == "__main__":
    main()
