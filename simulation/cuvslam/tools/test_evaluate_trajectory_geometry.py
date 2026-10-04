import unittest

from evaluate_trajectory_geometry_bag import percentile


class TrajectoryGeometryTest(unittest.TestCase):
    def test_percentile_interpolates(self):
        self.assertEqual(percentile([0.0, 10.0], 50.0), 5.0)
        self.assertEqual(percentile([3.0], 95.0), 3.0)


if __name__ == "__main__":
    unittest.main()
