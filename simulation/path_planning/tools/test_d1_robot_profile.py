#!/usr/bin/env python3
"""Unit tests for the staged D1 robot-profile contract."""

from __future__ import annotations

import unittest

from d1_robot_profile import (
    D1_EDU_PROFILE,
    D1_MAX_PROXY_PROFILE,
    available_robot_profiles,
    get_robot_profile,
    require_runnable_profile,
)


class RobotProfileTests(unittest.TestCase):
    def test_d1_edu_is_the_compatibility_baseline(self) -> None:
        profile = require_runnable_profile("d1_edu")
        self.assertEqual(profile, D1_EDU_PROFILE)
        self.assertEqual(
            profile.model_asset,
            "simulation/path_planning/models/d1_edu/d1_edu.xml",
        )

    def test_d1_max_is_explicitly_not_ready(self) -> None:
        profile = get_robot_profile("d1_max_proxy")
        self.assertEqual(profile, D1_MAX_PROXY_PROFILE)
        self.assertFalse(profile.runnable)
        self.assertEqual(profile.model_asset, "external/Agibot_D1_Max/d1_max.xml")
        self.assertEqual(
            profile.source_revision,
            "83bdebb04ae140f751e7c5e9e2c8be44aad62473",
        )
        with self.assertRaisesRegex(RuntimeError, "not runnable yet"):
            require_runnable_profile("d1_max_proxy")

    def test_d1_max_alias_is_not_a_second_model(self) -> None:
        self.assertIs(get_robot_profile("d1_max"), D1_MAX_PROXY_PROFILE)
        self.assertEqual(available_robot_profiles(), ("d1_edu", "d1_max_proxy"))

    def test_unknown_profile_fails_with_choices(self) -> None:
        with self.assertRaisesRegex(ValueError, "d1_edu.*d1_max_proxy"):
            get_robot_profile("d1_ultra")


if __name__ == "__main__":
    unittest.main()
