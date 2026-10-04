import unittest

from evaluate_observation_geometry_bag import (
    base_to_cloud_frame,
    selected_indices,
    stereo_focal_baseline,
)


class ObservationGeometryTest(unittest.TestCase):
    def test_physical_camera_frame_translation(self):
        transform = base_to_cloud_frame("camera_infra1_frame")
        self.assertEqual(transform.translation, (0.19, 0.025, 0.20))

    def test_optical_override_changes_rotation(self):
        physical = base_to_cloud_frame("camera_infra1_frame", "physical")
        optical = base_to_cloud_frame("camera_infra1_frame", "optical")
        self.assertNotEqual(physical.rotation, optical.rotation)

    def test_selection_includes_endpoints(self):
        indices = selected_indices(100, 5)
        self.assertEqual(indices[0], 0)
        self.assertEqual(indices[-1], 99)
        self.assertEqual(len(indices), 5)

    def test_selection_keeps_all_when_under_limit(self):
        self.assertEqual(selected_indices(3, 30), [0, 1, 2])

    def test_focal_baseline_comes_from_camera_info(self):
        class Info:
            pass

        left = Info()
        left.k = [314.5, 0.0, 0.0, 0.0, 314.5, 0.0, 0.0, 0.0, 1.0]
        right = Info()
        right.p = [314.5, 0.0, 0.0, -15.725, 0.0, 314.5, 0.0, 0.0]
        focal, baseline, product = stereo_focal_baseline(left, right)
        self.assertAlmostEqual(focal, 314.5)
        self.assertAlmostEqual(baseline, 0.05)
        self.assertAlmostEqual(product, 15.725)


if __name__ == "__main__":
    unittest.main()
