import math
import unittest

from record_nav2_commands import summarize


def sample(t, vx, vy=0.0, wz=0.0):
    return {"receipt_elapsed_s": t, "velocity_body": [vx, vy, wz]}


class CommandEvidenceTests(unittest.TestCase):
    def test_safe_commands_and_receipt_timing(self):
        result = summarize([sample(0, 0.28), sample(0.1, 0.29, wz=-0.4), sample(0.2, 0)])
        self.assertTrue(result["command_bounds_pass"])
        self.assertEqual(result["max_abs_velocity_body"], [0.29, 0, 0.4])
        self.assertAlmostEqual(result["receipt_interval_median_s"], 0.1)

    def test_observed_failure_command_is_rejected(self):
        result = summarize([sample(0, 0.31153398752212524, wz=-0.28992852568626404)])
        self.assertFalse(result["command_bounds_pass"])
        self.assertEqual(result["overspeed_count"], 1)

    def test_tiny_float_overshoot_matches_guard_contract(self):
        self.assertFalse(summarize([sample(0, 0.30000004172325134)])["command_bounds_pass"])
        self.assertTrue(summarize([sample(0, 0.30)])["command_bounds_pass"])

    def test_other_axes_are_checked(self):
        result = summarize([sample(0, 0, vy=-0.03), sample(0.1, 0, wz=0.71)])
        self.assertEqual(result["overspeed_count"], 2)

    def test_nonfinite_and_empty_stream_do_not_qualify(self):
        self.assertFalse(summarize([])["command_bounds_pass"])
        result = summarize([sample(0, math.nan), sample(0.1, math.inf)])
        self.assertEqual(result["nonfinite_count"], 2)
        self.assertFalse(result["command_bounds_pass"])


if __name__ == "__main__":
    unittest.main()
