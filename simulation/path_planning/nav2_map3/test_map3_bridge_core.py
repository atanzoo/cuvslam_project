#!/usr/bin/env python3
"""No-ROS tests for the Map-3 simulated command safety boundary."""

import unittest

from map3_bridge_core import SimCommandGuard, ZERO_COMMAND


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class SimCommandGuardTest(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.guard = SimCommandGuard(
            scene_id="scene-1", map_bounds=(-5.0, 5.0, -5.0, 5.0),
            clock=self.clock,
        )

    def pose(self, sequence=0, **overrides):
        message = dict(scene_id="scene-1", sequence=sequence,
                       x=0.0, y=0.0, yaw=0.0, vx=0.0, vy=0.0, wz=0.0)
        message.update(overrides)
        self.guard.accept_pose(message)

    def test_disarmed_until_fresh_pose_and_arm(self):
        self.guard.accept_command(0.2, 0.0, 0.1)
        self.assertEqual(tuple(self.guard.read_command()["velocity_body"]), ZERO_COMMAND)
        with self.assertRaisesRegex(RuntimeError, "fresh simulation pose"):
            self.guard.arm("scene-1")
        self.pose()
        self.guard.arm("scene-1")
        self.assertTrue(self.guard.read_command()["armed"])
        self.assertEqual(tuple(self.guard.read_command()["velocity_body"]), ZERO_COMMAND)
        self.guard.accept_command(0.2, 0.0, 0.1)
        self.assertEqual(self.guard.read_command()["velocity_body"], [0.2, 0.0, 0.1])

    def test_pose_timeout_latches_zero(self):
        self.pose()
        self.assertIsNotNone(self.guard.pose_snapshot())
        self.guard.arm("scene-1")
        self.guard.accept_command(0.2, 0.0, 0.0)
        self.clock.now = 0.31
        self.assertIsNone(self.guard.pose_snapshot())
        result = self.guard.read_command()
        self.assertFalse(result["armed"])
        self.assertEqual(result["fault_reason"], "stale_simulation_pose")
        self.assertEqual(tuple(result["velocity_body"]), ZERO_COMMAND)
        self.pose(sequence=1)
        self.guard.accept_command(0.2, 0.0, 0.0)
        self.assertEqual(tuple(self.guard.read_command()["velocity_body"]), ZERO_COMMAND)

    def test_command_timeout_latches_zero(self):
        self.pose()
        self.guard.arm("scene-1")
        self.guard.accept_command(0.2, 0.0, 0.0)
        self.clock.now = 0.25
        self.pose(sequence=1)
        self.clock.now = 0.31
        result = self.guard.read_command()
        self.assertEqual(result["fault_reason"], "stale_nav2_command")
        self.assertEqual(tuple(result["velocity_body"]), ZERO_COMMAND)

    def test_wrong_scene_nonmonotonic_pose_and_outside_map_rejected(self):
        with self.assertRaisesRegex(ValueError, "scene identity"):
            self.pose(scene_id="other")
        self.pose()
        with self.assertRaisesRegex(ValueError, "increase"):
            self.pose(sequence=0)
        with self.assertRaisesRegex(ValueError, "outside"):
            self.pose(sequence=1, x=5.0)
        self.assertEqual(self.guard.pose_sequence, 0)

    def test_overspeed_or_invalid_command_disarms(self):
        self.pose()
        self.guard.arm("scene-1")
        self.guard.accept_command(0.31, 0.0, 0.0)
        self.assertEqual(self.guard.read_command()["fault_reason"], "invalid_or_overspeed_nav2_command")
        self.assertEqual(tuple(self.guard.read_command()["velocity_body"]), ZERO_COMMAND)
        self.guard.arm("scene-1")
        self.guard.accept_command(float("nan"), 0.0, 0.0)
        self.assertEqual(tuple(self.guard.read_command()["velocity_body"]), ZERO_COMMAND)

    def test_explicit_stop_is_zero(self):
        self.pose()
        self.guard.arm("scene-1")
        self.guard.accept_command(0.2, 0.0, 0.1)
        self.guard.disarm("goal_cancelled")
        result = self.guard.read_command()
        self.assertEqual(result["fault_reason"], "goal_cancelled")
        self.assertEqual(tuple(result["velocity_body"]), ZERO_COMMAND)


if __name__ == "__main__":
    unittest.main()
