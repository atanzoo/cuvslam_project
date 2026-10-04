import unittest

from compare_landmark_bags import nearest_distances, percentile


class CompareLandmarkBagsTest(unittest.TestCase):
    def test_nearest_distances(self):
        self.assertEqual(
            nearest_distances([(0.0, 0.0, 0.0)], [(0.0, 3.0, 4.0)]),
            [5.0],
        )

    def test_percentile(self):
        self.assertEqual(percentile([0.0, 2.0], 50.0), 1.0)


if __name__ == "__main__":
    unittest.main()
