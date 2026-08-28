import math

import pytest

from cuvslam_sim_sensor_adapter.calibration import projection_with_baseline


def test_projection_encodes_right_camera_baseline():
    source = [
        208.3317661,
        0.0,
        212.0,
        0.0,
        0.0,
        208.3317518,
        120.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
    ]

    corrected = projection_with_baseline(source, 0.05)

    assert corrected[:3] == source[:3]
    assert corrected[4:] == source[4:]
    assert math.isclose(corrected[3], -10.416588305)
    assert source[3] == 0.0


@pytest.mark.parametrize(
    ("projection", "baseline"),
    [
        ([0.0] * 12, 0.05),
        ([1.0] * 11, 0.05),
        ([1.0] * 12, 0.0),
        ([1.0] * 12, -0.05),
    ],
)
def test_projection_rejects_invalid_calibration(projection, baseline):
    with pytest.raises(ValueError):
        projection_with_baseline(projection, baseline)
