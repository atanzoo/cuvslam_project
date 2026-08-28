import math

import pytest

from cuvslam_sim_observability.geometry import (
    Transform,
    compose,
    inverse,
    quaternion_from_rpy,
)


def test_compose_with_inverse_is_identity():
    transform = Transform(
        translation=(1.0, -2.0, 0.5),
        rotation=quaternion_from_rpy(0.1, -0.2, 0.7),
    )
    result = compose(transform, inverse(transform))
    assert result.translation == pytest.approx((0.0, 0.0, 0.0))
    assert result.rotation == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_quaternion_from_yaw():
    result = quaternion_from_rpy(0.0, 0.0, math.pi)
    assert result == pytest.approx((0.0, 0.0, 1.0, 0.0), abs=1e-8)
