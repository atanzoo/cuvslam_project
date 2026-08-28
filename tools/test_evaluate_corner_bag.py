import unittest

from evaluate_corner_bag import (
    phase_boundary_indices,
    wrapped_degrees,
)


class CornerBagTest(unittest.TestCase):
    def test_phase_boundaries_find_turn(self):
        yaws = [0.0, 0.2, 0.8, 1.2, 45.0, 89.2, 90.1]
        self.assertEqual(phase_boundary_indices(yaws), (2, 5))

    def test_wrapped_degrees(self):
        self.assertAlmostEqual(wrapped_degrees(181.0), -179.0)
        self.assertAlmostEqual(wrapped_degrees(-181.0), 179.0)


if __name__ == "__main__":
    unittest.main()
