#!/usr/bin/env python3
"""Regression tests for the D1 Edu episode and Monte Carlo benchmark APIs."""

from __future__ import annotations

import csv
from dataclasses import asdict
import json
from pathlib import Path
import tempfile

import numpy as np

from d1_edu_sim_sdk import SimulatedD1SDK
from run_d1_edu_decision_layer_benchmark import run_benchmark, sample_episode_config
from run_d1_edu_decision_layer_viewer import (
    EpisodeConfig,
    bounding_circle_clearance,
    forward_only_cruise_command,
    apply_lidar_model,
    collision_clearance,
    goal_tracking_cruise_command,
    gradual_rejoin_lateral_command,
    run_episode,
)
from run_d1_edu_obstacle_avoidance_viewer import Obstacle


def _fast_config(**overrides: object) -> EpisodeConfig:
    values: dict[str, object] = {
        "duration": 6.0,
        "goal_x": 1.40,
        "goal_tolerance": 0.25,
        "mppi_samples": 32,
        "mppi_horizon": 30,
        "command_delay": 0.0,
    }
    values.update(overrides)
    return EpisodeConfig(**values)


def test_fixed_seed_reproducible() -> None:
    config = _fast_config(box_x=2.80, box_y=1.30, box_vy=0.20)
    first = run_episode(config, seed=173)
    second = run_episode(config, seed=173)
    assert asdict(first) == asdict(second)


def test_safe_scene_is_tn() -> None:
    result = run_episode(
        _fast_config(box_x=2.80, box_y=1.30, box_vy=0.20, goal_x=0.90),
        seed=1,
    )
    assert result.false_positive == 0
    assert result.true_negative > 0
    assert "AVOID" not in result.mode_history


def test_rejoin_is_delayed_and_smoothly_ramped() -> None:
    config = _fast_config(
        rejoin_gain=0.50,
        rejoin_deadband=0.05,
        rejoin_max_lateral_speed=0.10,
        rejoin_delay=0.30,
        rejoin_ramp_time=1.00,
    )
    assert gradual_rejoin_lateral_command(1.0, 0.0, config) == (0.0, 0.0)
    assert gradual_rejoin_lateral_command(1.0, 0.30, config) == (0.0, 0.0)
    middle, middle_factor = gradual_rejoin_lateral_command(1.0, 0.80, config)
    final, final_factor = gradual_rejoin_lateral_command(1.0, 1.30, config)
    assert 0.0 < middle < final <= config.rejoin_max_lateral_speed
    assert 0.0 < middle_factor < final_factor <= 1.0
    assert gradual_rejoin_lateral_command(0.04, 1.30, config) == (0.0, 1.0)


def test_rejoin_timer_survives_safe_avoid_hysteresis() -> None:
    result = run_episode(
        _fast_config(
            box_x=1.25,
            box_y=-1.30,
            box_vy=0.38,
            goal_x=1.40,
        ),
        seed=2,
        record_trace=True,
    )
    trace = list(result.trace)
    rejoin_points = [point for point in trace if point.get("rejoin_factor", 0.0) > 0.0]
    assert rejoin_points
    # Re-entering AVOID during a safe hysteresis interval must not erase the
    # rejoin state; the command remains bounded by the configured limit.
    assert max(abs(point["command"][1]) for point in rejoin_points) <= 0.10 + 1e-9


def test_cruise_keeps_forward_speed_far_from_goal() -> None:
    config = _fast_config(robot_cruise_speed=0.22, max_vx=0.36)
    command, active = goal_tracking_cruise_command(
        np.array([2.0, 0.0]),
        config.robot_cruise_speed,
        0.0,
        config,
    )
    assert command[0] == config.robot_cruise_speed
    assert not active


def test_cruise_slows_down_near_goal() -> None:
    config = _fast_config(robot_cruise_speed=0.22)
    command, active = goal_tracking_cruise_command(
        np.array([0.50, 0.0]),
        config.robot_cruise_speed,
        0.0,
        config,
    )
    assert 0.0 < command[0] < config.robot_cruise_speed
    assert not active


def test_cruise_stops_inside_goal_tolerance() -> None:
    config = _fast_config(robot_cruise_speed=0.22)
    command, active = goal_tracking_cruise_command(
        np.array([0.10, 0.10]),
        config.robot_cruise_speed,
        0.0,
        config,
    )
    assert np.allclose(command, np.zeros(3))
    assert not active


def test_cruise_reverses_after_goal_overshoot() -> None:
    config = _fast_config(robot_cruise_speed=0.22)
    command, active = goal_tracking_cruise_command(
        np.array([-0.40, 0.0]),
        config.robot_cruise_speed,
        0.0,
        config,
    )
    assert -config.cruise_max_reverse_speed <= command[0] < 0.0
    assert active


def test_cruise_hysteresis_has_neutral_band() -> None:
    config = _fast_config(robot_cruise_speed=0.22)
    reverse, active = goal_tracking_cruise_command(
        np.array([-0.20, 0.40]), config.robot_cruise_speed, 0.0, config
    )
    neutral, active = goal_tracking_cruise_command(
        np.array([-0.02, 0.40]), config.robot_cruise_speed, 0.0, config,
        reverse_active=active,
    )
    assert reverse[0] < 0.0
    assert active
    assert np.isclose(neutral[0], 0.0)


def test_cruise_exits_reverse_only_after_release_threshold() -> None:
    config = _fast_config(robot_cruise_speed=0.22)
    _, active = goal_tracking_cruise_command(
        np.array([-0.20, 0.40]), config.robot_cruise_speed, 0.0, config
    )
    _, still_active = goal_tracking_cruise_command(
        np.array([0.04, 0.40]), config.robot_cruise_speed, 0.0, config,
        reverse_active=active,
    )
    forward, inactive = goal_tracking_cruise_command(
        np.array([0.12, 0.40]), config.robot_cruise_speed, 0.0, config,
        reverse_active=still_active,
    )
    assert still_active
    assert not inactive
    assert forward[0] > 0.0


def test_cruise_forward_curve_changes_smoothly() -> None:
    config = _fast_config(robot_cruise_speed=0.22)
    speeds = [
        goal_tracking_cruise_command(
            np.array([goal_x, 0.0]), config.robot_cruise_speed, 0.0, config
        )[0][0]
        for goal_x in (0.08, 0.12, 0.20, 0.35, 0.50, 0.75, 1.20)
    ]
    assert speeds[0] == 0.0
    assert all(left <= right + 1e-9 for left, right in zip(speeds, speeds[1:]))
    assert speeds[-1] == config.robot_cruise_speed


def test_forward_only_cruise_reproduces_positive_forward_command() -> None:
    config = _fast_config(robot_cruise_speed=0.22, cruise_mode="forward_only")
    command, active = forward_only_cruise_command(
        config.robot_cruise_speed,
        0.0,
        config,
    )
    assert np.isclose(command[0], config.robot_cruise_speed)
    assert command[0] > 0.0
    assert not active


def test_crossing_scene_triggers_tp() -> None:
    result = run_episode(
        _fast_config(box_x=1.25, box_y=-1.30, box_vy=0.38, goal_x=1.40),
        seed=2,
    )
    assert result.decision_triggered
    assert result.true_positive > 0


def test_collision_scene_is_recorded() -> None:
    result = run_episode(
        _fast_config(
            box_x=0.80,
            box_y=0.0,
            box_vx=0.0,
            box_vy=0.0,
            max_vy=0.001,
            max_ay=0.001,
            goal_x=1.50,
        ),
        seed=3,
    )
    assert result.collision
    assert result.failure_reason == "collision"


def test_proxy_only_is_not_physical_collision() -> None:
    result = run_episode(
        _fast_config(
            # The conservative outer-circle proxy overlaps here, while the
            # exact axis-aligned box footprint and MuJoCo geoms remain apart.
            box_x=0.60,
            box_y=0.0,
            box_vx=0.0,
            box_vy=0.0,
            robot_cruise_speed=0.0,
            goal_x=0.90,
            duration=0.25,
        ),
        seed=33,
    )
    assert result.proxy_collision
    assert result.proxy_only
    assert not result.collision
    assert not result.physical_contact
    assert not result.geometric_collision


def test_timeout_is_recorded() -> None:
    result = run_episode(
        _fast_config(robot_cruise_speed=0.0, duration=0.25, goal_x=0.90),
        seed=4,
    )
    assert result.timeout
    assert not result.reached_goal
    assert result.failure_reason == "timeout"
    assert not result.success


def test_stagnation_recovery_triggers_once_without_new_mode() -> None:
    result = run_episode(
        _fast_config(
            duration=3.0,
            goal_x=1.50,
            box_x=0.90,
            box_y=-0.50,
            box_vx=0.0,
            box_vy=0.20,
            max_vx=0.001,
            max_vy=0.001,
            stagnation_window=0.50,
            minimum_progress=0.01,
            recovery_duration=0.50,
            mppi_samples=8,
            mppi_horizon=10,
        ),
        seed=101,
    )
    assert result.recovery_triggered
    assert result.recovery_active_duration > 0.0
    assert set(result.mode_history) <= {"CRUISE", "AVOID"}


def test_episode_trace_records_robot_and_obstacle_paths() -> None:
    result = run_episode(
        _fast_config(duration=0.50, goal_x=0.90, box_x=2.80, box_y=1.30),
        seed=102,
        record_trace=True,
    )
    assert result.trace
    sample = result.trace[0]
    assert {"t", "robot", "goal", "obstacles", "mode"} <= sample.keys()
    assert len(sample["robot"]) == 2
    assert sample["obstacles"]


def test_clearance_formula_uses_exact_rectangle_geometry() -> None:
    obstacle = Obstacle(half_length=0.30, half_width=0.40)
    # The robot center is outside the box by (0.70, 0.60), so the nearest
    # rectangle point is (0.30, 0.40) away in each axis.
    expected = np.hypot(0.70, 0.60) - 0.25
    actual = collision_clearance(np.array([0.0, 0.0]), np.array([1.0, 1.0]), obstacle)
    np.testing.assert_allclose(actual, expected)


def test_bounding_circle_proxy_is_separate_from_exact_clearance() -> None:
    obstacle = Obstacle(half_length=0.30, half_width=0.40)
    exact = collision_clearance(np.array([0.0, 0.0]), np.array([1.0, 0.0]), obstacle)
    proxy = bounding_circle_clearance(np.array([0.0, 0.0]), np.array([1.0, 0.0]), obstacle)
    assert exact > proxy


def test_simulated_sdk_limits_and_delay() -> None:
    sdk = SimulatedD1SDK(
        max_velocity=np.array([0.20, 0.10, 0.30]),
        max_acceleration=np.array([1.0, 1.0, 1.0]),
        command_delay=0.05,
    )
    first = sdk.move(np.array([1.0, 0.0, 0.0]), 0.01)
    assert np.allclose(first, 0.0)
    second = sdk.move(np.array([1.0, 0.0, 0.0]), 0.01)
    assert second[0] <= 0.02 + 1e-9
    for _ in range(30):
        actual = sdk.move(np.array([1.0, 0.0, 0.0]), 0.01)
    assert 0.0 < actual[0] <= 0.20


def test_lidar_noise_does_not_change_ground_truth_collision() -> None:
    common = {
        "box_x": 0.45,
        "box_y": 0.0,
        "box_vx": 0.0,
        "box_vy": 0.0,
        "max_vy": 0.001,
        "max_ay": 0.001,
        "goal_x": 1.50,
        "duration": 0.40,
    }
    clean = run_episode(_fast_config(**common, lidar_noise_std=0.0), seed=5)
    noisy = run_episode(
        _fast_config(**common, lidar_noise_std=0.20, lidar_dropout_rate=0.90),
        seed=5,
    )
    assert clean.collision == noisy.collision == True


def test_lidar_model_is_bounded() -> None:
    scan = np.array([0.5, 1.0, 4.0])
    noisy = apply_lidar_model(
        scan,
        np.random.default_rng(0),
        noise_std=0.2,
        dropout_rate=0.0,
        max_range=4.0,
    )
    assert np.all(noisy >= 0.02)
    assert np.all(noisy <= 4.0)
    assert noisy[2] == 4.0


def test_csv_and_summary_json_are_written() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        csv_path = Path(temporary) / "benchmark.csv"
        summary = run_benchmark(1, "baseline", 7, csv_path)
        summary_path = Path(temporary) / "d1_decision_benchmark_summary.json"
        assert csv_path.exists()
        assert summary_path.exists()
        with csv_path.open(newline="", encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
        assert len(rows) == 1
        assert "min_clearance" in rows[0]
        assert float(rows[0]["duration"]) == 40.0
        assert "extended_collision" not in rows[0]
        payload = json.loads(summary_path.read_text(encoding="utf-8"))
        assert payload["trials"] == 1
        assert summary["trials"] == 1


def test_benchmark_profiles_use_40_second_episodes() -> None:
    rng = np.random.default_rng(7)
    for profile in ("baseline", "sensor_noise", "full_randomization"):
        config = sample_episode_config(profile, rng, 7, scenario_mode="legacy")
        assert config.duration == 40.0


def main() -> None:
    tests = [
        test_fixed_seed_reproducible,
        test_safe_scene_is_tn,
        test_cruise_keeps_forward_speed_far_from_goal,
        test_cruise_slows_down_near_goal,
        test_cruise_stops_inside_goal_tolerance,
        test_cruise_reverses_after_goal_overshoot,
        test_cruise_hysteresis_has_neutral_band,
        test_cruise_exits_reverse_only_after_release_threshold,
        test_cruise_forward_curve_changes_smoothly,
        test_forward_only_cruise_reproduces_positive_forward_command,
        test_crossing_scene_triggers_tp,
        test_collision_scene_is_recorded,
        test_proxy_only_is_not_physical_collision,
        test_timeout_is_recorded,
        test_stagnation_recovery_triggers_once_without_new_mode,
        test_episode_trace_records_robot_and_obstacle_paths,
        test_clearance_formula_uses_exact_rectangle_geometry,
        test_bounding_circle_proxy_is_separate_from_exact_clearance,
        test_simulated_sdk_limits_and_delay,
        test_lidar_noise_does_not_change_ground_truth_collision,
        test_lidar_model_is_bounded,
        test_csv_and_summary_json_are_written,
        test_benchmark_profiles_use_40_second_episodes,
    ]
    for test in tests:
        test()
        print(f"PASS {test.__name__}")
    print("status=PASS: decision-layer Monte Carlo benchmark suite")


if __name__ == "__main__":
    main()
