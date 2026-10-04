#!/usr/bin/env python3
"""Unit checks for the ROS-independent odometry contract validator."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest


MODULE_PATH = Path(__file__).with_name("odom_contract_guard.py")
SPEC = importlib.util.spec_from_file_location("odom_contract_guard", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def make_message(*, covariance=(1.0, 0.0, 0.0, 0.0, 0.0, 1.0), stamp=1):
    twist = SimpleNamespace(
        linear=SimpleNamespace(x=0.1, y=0.0, z=0.0),
        angular=SimpleNamespace(x=0.0, y=0.0, z=0.2),
    )
    full_covariance = [0.0] * 36
    full_covariance[0] = covariance[0]
    full_covariance[35] = covariance[-1]
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id="odom",
            stamp=SimpleNamespace(sec=stamp, nanosec=0),
        ),
        child_frame_id="base_link",
        twist=SimpleNamespace(twist=twist, covariance=full_covariance),
    )


class OdomContractGuardTest(unittest.TestCase):
    def test_accepts_valid_selected_twist(self):
        valid, reason = MODULE.validate_message(
            make_message(), required_twist_indices=(0, 5)
        )
        self.assertTrue(valid, reason)

    def test_rejects_zero_covariance(self):
        valid, reason = MODULE.validate_message(
            make_message(covariance=(0.0,) * 6), required_twist_indices=(0, 5)
        )
        self.assertFalse(valid)
        self.assertIn("covariance", reason)

    def test_rejects_wrong_frame(self):
        message = make_message()
        message.header.frame_id = "map"
        valid, reason = MODULE.validate_message(
            message, required_twist_indices=(0, 5)
        )
        self.assertFalse(valid)
        self.assertIn("frame_id", reason)

    def test_rejects_out_of_order_timestamp(self):
        valid, reason = MODULE.validate_message(
            make_message(stamp=3), required_twist_indices=(0, 5), last_stamp_ns=3_000_000_000
        )
        self.assertFalse(valid)
        self.assertIn("timestamp", reason)


if __name__ == "__main__":
    unittest.main()
