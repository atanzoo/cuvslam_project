import math
import unittest

from validate_stereo_geometry import percentile, summarize_matches


class StereoGeometryMathTest(unittest.TestCase):
    def test_percentile_interpolates(self):
        self.assertEqual(percentile([0.0, 10.0], 50.0), 5.0)

    def test_rectified_positive_disparity_summary(self):
        result = summarize_matches(
            stamp_ns=123,
            keypoints_left=30,
            keypoints_right=28,
            disparities=[6.0, 7.0, 8.0, 9.0],
            vertical_residuals=[0.0, 0.1, -0.1, 0.0],
        )
        self.assertEqual(result.matches, 4)
        self.assertEqual(result.median_disparity_px, 7.5)
        self.assertLess(result.vertical_p95_px, 0.11)
        self.assertEqual(result.positive_disparity_fraction, 1.0)

    def test_mismatched_measurement_counts_are_rejected(self):
        with self.assertRaises(ValueError):
            summarize_matches(0, 0, 0, [1.0], [])

    def test_empty_percentile_is_nan(self):
        self.assertTrue(math.isnan(percentile([], 50.0)))


if __name__ == "__main__":
    unittest.main()
