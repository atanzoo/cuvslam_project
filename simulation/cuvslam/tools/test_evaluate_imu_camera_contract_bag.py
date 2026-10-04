import math
import unittest

from evaluate_imu_camera_contract_bag import (
    ImuSample,
    integrate_angular_velocity,
    interval_counts,
    measured_turn_boundary_indices,
)


class ImuCameraContractTest(unittest.TestCase):
    def test_counts_imu_samples_between_images(self):
        self.assertEqual(
            interval_counts(
                [0, 5, 10, 15, 20, 25, 30],
                [0, 16, 31],
            ),
            [4, 3],
        )

    def test_integrates_z_gyro(self):
        samples = [
            ImuSample(
                timestamp_ns=index * 100_000_000,
                frame_id="imu",
                angular_velocity=(0.0, 0.0, math.pi / 2.0),
                linear_acceleration=(0.0, 0.0, 9.81),
            )
            for index in range(11)
        ]
        integrated = integrate_angular_velocity(
            samples,
            0,
            1_000_000_000,
        )
        self.assertAlmostEqual(integrated[2], 90.0)

    def test_measured_turn_boundaries_do_not_assume_ninety_degrees(self):
        self.assertEqual(
            measured_turn_boundary_indices(
                [0.0, 0.5, 1.2, 40.0, 79.8, 80.0],
            ),
            (1, 4),
        )


if __name__ == "__main__":
    unittest.main()
