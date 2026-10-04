#!/usr/bin/env python3
"""Headless regression tests for the D1 Edu dynamic-obstacle decision stack."""

from __future__ import annotations

import numpy as np

from run_d1_edu_decision_layer_viewer import (
    DecisionLayer,
    LidarTracker,
    local_to_world_velocity,
    run,
)
from run_d1_edu_mppi_obstacle_viewer import ObstacleMPPI
from d1_edu_sim_sdk import SimulatedD1SDK


def test_velocity_frame_conversion() -> None:
    velocity_world = local_to_world_velocity(np.array([0.20, 0.0]), np.pi / 2.0)
    np.testing.assert_allclose(velocity_world, [0.0, 0.20], atol=1e-8)


def test_yaw_corrected_collision_prediction() -> None:
    tracker = LidarTracker()
    points = np.array([[1.00, -0.01], [1.00, 0.0], [1.00, 0.01]])
    yaw = np.pi / 2.0
    tracker.update(points, np.zeros(2), yaw, 0.0, np.array([1, 2, 3]))
    tracker.update(points, np.zeros(2), yaw, 0.1, np.array([1, 2, 3]))
    robot_velocity_world = local_to_world_velocity(np.array([0.20, 0.0]), yaw)
    risk, time_to_closest, closest = tracker.collision_prediction(
        np.zeros(2),
        robot_velocity_world,
    )
    assert risk
    assert time_to_closest > 0.0
    assert closest < tracker.effective_safe_distance


def test_cluster_continuity() -> None:
    tracker = LidarTracker()
    first = np.array([[1.00, 0.48], [1.02, 0.50], [1.04, 0.52]])
    tracker.update(first, np.zeros(2), 0.0, 0.0, np.array([1, 2, 3]))

    # The distractor is closer, but continuity should keep the previous box.
    second = np.array([
        [0.60, -0.80], [0.62, -0.82], [0.64, -0.84],
        [1.03, 0.54], [1.05, 0.56], [1.07, 0.58],
    ])
    selected = tracker.update(
        second,
        np.zeros(2),
        0.0,
        0.1,
        np.array([10, 11, 12, 1, 2, 3]),
    )
    assert selected is not None and selected[1] > 0.40


def test_short_and_long_lidar_dropout() -> None:
    tracker = LidarTracker()
    points = np.array([[1.00, 0.0], [1.02, 0.01], [1.04, -0.01]])
    tracker.update(points, np.zeros(2), 0.0, 0.0, np.array([1, 2, 3]))
    tracker.update(points + [0.0, 0.02], np.zeros(2), 0.0, 0.1, np.array([1, 2, 3]))

    short_prediction = tracker.update(np.empty((0, 2)), np.zeros(2), 0.0, 0.2)
    assert short_prediction is not None
    assert tracker.miss_count == 1

    expired = tracker.update(np.empty((0, 2)), np.zeros(2), 0.0, 1.0)
    assert expired is None
    assert tracker.observation is None


def test_decision_hysteresis() -> None:
    decision = DecisionLayer(safe_updates_required=3, risk_updates_required=1)
    assert decision.update(True, 0.4, 0.3, 3) == "AVOID"
    assert decision.update(False, 1.0, 0.80, 3) == "AVOID"
    assert decision.update(False, 1.0, 1.00, 3) == "AVOID"
    assert decision.update(False, 1.0, 1.00, 3) == "AVOID"
    assert decision.update(False, np.inf, np.inf, 0) == "CRUISE"


def test_decision_requires_confirmed_risk_before_avoid() -> None:
    decision = DecisionLayer(risk_updates_required=2)
    assert decision.update(True, 3.0, 0.90, 3) == "CRUISE"
    assert decision.update(True, 3.0, 0.90, 3) == "AVOID"


def test_dynamic_mppi_zero_velocity_is_static() -> None:
    planner = ObstacleMPPI(plan_dt=0.10, horizon=4, samples=8)
    commands = np.zeros((1, planner.horizon, 3))
    state = np.zeros(3)
    goal = np.array([1.0, 0.0])
    position = np.array([1.0, 0.0])
    static_now = planner._rollout_cost(
        state,
        commands,
        np.empty((0, 2)),
        goal,
        position,
        np.zeros(2),
        current_time=0.0,
    )
    static_later = planner._rollout_cost(
        state,
        commands,
        np.empty((0, 2)),
        goal,
        position,
        np.zeros(2),
        current_time=10.0,
    )
    np.testing.assert_allclose(static_now, static_later, atol=1e-10)


def test_simulated_sdk_delay_saturation_and_acceleration() -> None:
    sdk = SimulatedD1SDK(
        max_velocity=np.array([0.20, 0.10, 0.30]),
        max_acceleration=np.array([1.0, 1.0, 1.0]),
        command_delay=0.05,
    )
    first = sdk.move(np.array([1.0, 0.0, 0.0]), 0.01)
    assert np.allclose(first, 0.0)
    for _ in range(20):
        actual = sdk.move(np.array([1.0, 0.0, 0.0]), 0.01)
    assert 0.0 < actual[0] <= 0.20
    assert actual[1] == 0.0


def test_headless_safe_and_crossing_scenarios() -> None:
    safe = run(
        duration=8.0,
        headless=True,
        box_speed=0.38,
        box_x=2.60,
    )
    assert "AVOID" not in safe.mode_history

    crossing = run(
        duration=8.0,
        headless=True,
        box_speed=0.38,
        box_x=1.25,
    )
    assert "AVOID" in crossing.mode_history
    assert crossing.mode_history[-1] == "CRUISE"
    assert crossing.min_clearance > 0.0


def main() -> None:
    tests = [
        test_velocity_frame_conversion,
        test_yaw_corrected_collision_prediction,
        test_cluster_continuity,
        test_short_and_long_lidar_dropout,
        test_decision_hysteresis,
        test_dynamic_mppi_zero_velocity_is_static,
        test_simulated_sdk_delay_saturation_and_acceleration,
        test_headless_safe_and_crossing_scenarios,
    ]
    for test in tests:
        test()
        print(f"PASS {test.__name__}")
    print("status=PASS: decision-layer regression suite")


if __name__ == "__main__":
    main()
