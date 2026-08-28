import math
import unittest

from frame_contract_math import (
    Transform,
    base_to_left_optical,
    compose,
    inverse,
    quaternion_from_rpy,
    relative,
    rotate_vector,
    translation_norm,
)


class FrameContractMathTest(unittest.TestCase):
    def test_compose_inverse_is_identity(self):
        transform = Transform(
            translation=(1.0, -2.0, 0.5),
            rotation=quaternion_from_rpy(0.1, -0.2, 0.7),
        )
        result = compose(transform, inverse(transform))
        self.assertAlmostEqual(translation_norm(result), 0.0, places=10)
        self.assertAlmostEqual(abs(result.rotation[3]), 1.0, places=10)

    def test_optical_forward_is_body_forward(self):
        extrinsic = base_to_left_optical()
        optical_forward_in_body = rotate_vector(
            extrinsic.rotation,
            (0.0, 0.0, 1.0),
        )
        for actual, expected in zip(
            optical_forward_in_body,
            (1.0, 0.0, 0.0),
        ):
            self.assertAlmostEqual(actual, expected, places=10)

    def test_camera_arc_is_removed_when_converted_back_to_base(self):
        base_camera = base_to_left_optical()
        world_base_start = Transform()
        world_base_end = Transform(
            rotation=quaternion_from_rpy(0.0, 0.0, math.pi / 2.0)
        )
        world_camera_start = compose(world_base_start, base_camera)
        world_camera_end = compose(world_base_end, base_camera)

        camera_motion = relative(world_camera_start, world_camera_end)
        expected_arc = math.sqrt(0.19**2 + 0.025**2) * math.sqrt(2.0)
        self.assertAlmostEqual(
            translation_norm(camera_motion),
            expected_arc,
            places=10,
        )

        recovered_base_start = compose(world_camera_start, inverse(base_camera))
        recovered_base_end = compose(world_camera_end, inverse(base_camera))
        recovered_base_motion = relative(
            recovered_base_start,
            recovered_base_end,
        )
        self.assertAlmostEqual(
            translation_norm(recovered_base_motion),
            0.0,
            places=10,
        )


if __name__ == "__main__":
    unittest.main()
