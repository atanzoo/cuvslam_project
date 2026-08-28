import math
import unittest

from validate_sim_camera_intrinsics import (
    Calibration,
    camera_checks,
    horizontal_fov,
    load_camera_spec,
    stereo_checks,
)


SDF = (
    "../deployment/isaac_ros/cuvslam_sim_observability/"
    "worlds/indoor_gz_sim.sdf"
)


def calibration(projection_x: float = 0.0) -> Calibration:
    fx = 208.33176612854004
    fy = 208.3317518234253
    return Calibration(
        width=424,
        height=240,
        distortion_model="plumb_bob",
        distortion=(0.0, 0.0, 0.0, 0.0, 0.0),
        intrinsic=(fx, 0.0, 212.0, 0.0, fy, 120.0, 0.0, 0.0, 1.0),
        rectification=(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
        projection=(
            fx,
            0.0,
            212.0,
            projection_x,
            0.0,
            fy,
            120.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ),
    )


class SimCameraIntrinsicsTest(unittest.TestCase):
    def test_sdf_fov_matches_camera_info_focal_length(self):
        spec = load_camera_spec(SDF, "d435i_infra1")
        actual = horizontal_fov(424, 208.33176612854004)
        self.assertLess(abs(actual - spec.horizontal_fov), 1e-5)

    def test_camera_contract_passes_known_sim_calibration(self):
        spec = load_camera_spec(SDF, "d435i_infra1")
        checks = camera_checks("left", spec, calibration(), 1e-5)
        self.assertTrue(all(check.passed for check in checks))

    def test_stereo_contract_requires_encoded_baseline(self):
        left = calibration()
        right = calibration(-208.33176612854004 * 0.05)
        checks = stereo_checks(left, right, 0.05, 1e-5)
        self.assertTrue(all(check.passed for check in checks))

        invalid = stereo_checks(left, calibration(), 0.05, 1e-5)
        self.assertFalse(invalid[-1].passed)

    def test_invalid_focal_length_is_rejected(self):
        with self.assertRaises(ValueError):
            horizontal_fov(424, math.nan)


if __name__ == "__main__":
    unittest.main()
