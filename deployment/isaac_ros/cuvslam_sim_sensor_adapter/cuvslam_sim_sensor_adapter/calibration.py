"""Pure calibration helpers for the simulated stereo adapter."""

from __future__ import annotations

import math
from collections.abc import Sequence


def projection_with_baseline(
    projection: Sequence[float],
    baseline_m: float,
) -> list[float]:
    if len(projection) != 12:
        raise ValueError("CameraInfo projection matrix must contain 12 values")

    focal_x = float(projection[0])
    if not math.isfinite(focal_x) or focal_x <= 0.0:
        raise ValueError("CameraInfo focal length P[0] must be positive")
    if not math.isfinite(baseline_m) or baseline_m <= 0.0:
        raise ValueError("Stereo baseline must be positive")

    corrected = [float(value) for value in projection]
    corrected[3] = -focal_x * baseline_m
    return corrected
