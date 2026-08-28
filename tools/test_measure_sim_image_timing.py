import unittest

from measure_sim_image_timing import (
    effective_rate_hz,
    intervals_ms,
    pairing_metrics,
    percentile,
)


class SimImageTimingMathTest(unittest.TestCase):
    def test_thirty_hertz_rate(self):
        timestamps = [0, 33_333_333, 66_666_666, 100_000_000]
        self.assertAlmostEqual(effective_rate_hz(timestamps), 30.0, places=5)

    def test_intervals_are_milliseconds(self):
        self.assertEqual(intervals_ms([0, 5_000_000, 20_000_000]), [5.0, 15.0])

    def test_exact_pairing(self):
        fraction, skew, interior = pairing_metrics(
            [10, 20, 30],
            [10, 20, 30],
        )
        self.assertEqual(fraction, 1.0)
        self.assertEqual(skew, 0.0)
        self.assertEqual(interior, 0)

    def test_pairing_ignores_subscription_edge_loss(self):
        fraction, skew, interior = pairing_metrics(
            [0, 10_000_000, 20_000_000],
            [10_000_000, 20_000_000],
        )
        self.assertAlmostEqual(fraction, 2.0 / 3.0)
        self.assertEqual(skew, 0.0)
        self.assertEqual(interior, 0)

    def test_pairing_reports_interior_loss_and_skew(self):
        fraction, skew, interior = pairing_metrics(
            [0, 10_000_000, 20_000_000],
            [0, 20_000_000],
        )
        self.assertAlmostEqual(fraction, 2.0 / 3.0)
        self.assertEqual(skew, 10.0)
        self.assertEqual(interior, 1)

    def test_percentile_interpolates(self):
        self.assertEqual(percentile([0.0, 10.0], 50.0), 5.0)


if __name__ == "__main__":
    unittest.main()
