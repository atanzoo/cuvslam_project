import math
import unittest

from evaluate_landmark_geometry_bag import (
    Box,
    pearson_correlation,
    point_to_box_surface,
)
from frame_contract_math import Transform


class LandmarkGeometryTest(unittest.TestCase):
    def setUp(self):
        self.box = Box(
            name="test",
            pose=Transform(),
            size=(2.0, 4.0, 6.0),
        )

    def test_point_outside_box(self):
        self.assertAlmostEqual(
            point_to_box_surface((1.3, 0.0, 0.0), self.box),
            0.3,
        )

    def test_point_inside_box_uses_nearest_face(self):
        self.assertAlmostEqual(
            point_to_box_surface((0.0, 0.0, 0.0), self.box),
            1.0,
        )

    def test_rotated_box(self):
        rotated = Box(
            name="rotated",
            pose=Transform(
                rotation=(
                    0.0,
                    0.0,
                    math.sin(math.pi / 4.0),
                    math.cos(math.pi / 4.0),
                )
            ),
            size=(2.0, 4.0, 6.0),
        )
        self.assertAlmostEqual(
            point_to_box_surface((0.0, 1.2, 0.0), rotated),
            0.2,
            places=6,
        )

    def test_pearson_correlation(self):
        self.assertAlmostEqual(
            pearson_correlation([1.0, 2.0, 3.0], [3.0, 2.0, 1.0]),
            -1.0,
        )


if __name__ == "__main__":
    unittest.main()
