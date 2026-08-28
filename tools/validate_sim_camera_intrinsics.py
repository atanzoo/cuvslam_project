#!/usr/bin/env python3
"""Validate live simulated stereo CameraInfo against the Gazebo SDF contract."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import time
from typing import Sequence
import xml.etree.ElementTree as ET


@dataclass(frozen=True)
class CameraSpec:
    name: str
    width: int
    height: int
    horizontal_fov: float


@dataclass(frozen=True)
class Calibration:
    width: int
    height: int
    distortion_model: str
    distortion: tuple[float, ...]
    intrinsic: tuple[float, ...]
    rectification: tuple[float, ...]
    projection: tuple[float, ...]


@dataclass(frozen=True)
class Check:
    name: str
    actual: float
    expected: float
    tolerance: float

    @property
    def error(self) -> float:
        return abs(self.actual - self.expected)

    @property
    def passed(self) -> bool:
        return math.isfinite(self.actual) and self.error <= self.tolerance


def horizontal_fov(width: int, focal_x: float) -> float:
    if width <= 0 or not math.isfinite(focal_x) or focal_x <= 0.0:
        raise ValueError("Image width and focal length must be positive")
    return 2.0 * math.atan(width / (2.0 * focal_x))


def load_camera_spec(sdf_path: str, sensor_name: str) -> CameraSpec:
    root = ET.parse(sdf_path).getroot()
    for sensor in root.findall(".//sensor"):
        if sensor.get("name") != sensor_name:
            continue
        camera = sensor.find("camera")
        if camera is None:
            raise ValueError(f"Sensor {sensor_name} has no camera element")
        return CameraSpec(
            name=sensor_name,
            width=int(camera.findtext("image/width", "")),
            height=int(camera.findtext("image/height", "")),
            horizontal_fov=float(camera.findtext("horizontal_fov", "")),
        )
    raise ValueError(f"Camera sensor {sensor_name} not found in {sdf_path}")


def calibration_from_message(message) -> Calibration:
    return Calibration(
        width=int(message.width),
        height=int(message.height),
        distortion_model=str(message.distortion_model),
        distortion=tuple(message.d),
        intrinsic=tuple(message.k),
        rectification=tuple(message.r),
        projection=tuple(message.p),
    )


def camera_checks(
    label: str,
    spec: CameraSpec,
    calibration: Calibration,
    tolerance: float,
) -> list[Check]:
    k = calibration.intrinsic
    p = calibration.projection
    checks = [
        Check(f"{label}.width", calibration.width, spec.width, 0.0),
        Check(f"{label}.height", calibration.height, spec.height, 0.0),
        Check(
            f"{label}.horizontal_fov",
            horizontal_fov(calibration.width, k[0]),
            spec.horizontal_fov,
            tolerance,
        ),
        Check(f"{label}.K_fx_vs_P_fx", k[0], p[0], tolerance),
        Check(f"{label}.K_fy_vs_P_fy", k[4], p[5], tolerance),
        Check(f"{label}.K_cx_vs_P_cx", k[2], p[2], tolerance),
        Check(f"{label}.K_cy_vs_P_cy", k[5], p[6], tolerance),
        Check(f"{label}.cx", k[2], spec.width / 2.0, tolerance),
        Check(f"{label}.cy", k[5], spec.height / 2.0, tolerance),
    ]
    identity = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    checks.extend(
        Check(f"{label}.R[{index}]", actual, expected, tolerance)
        for index, (actual, expected) in enumerate(
            zip(calibration.rectification, identity)
        )
    )
    checks.extend(
        Check(f"{label}.D[{index}]", actual, 0.0, tolerance)
        for index, actual in enumerate(calibration.distortion)
    )
    return checks


def stereo_checks(
    left: Calibration,
    right: Calibration,
    baseline_m: float,
    tolerance: float,
) -> list[Check]:
    checks = []
    for label, left_value, right_value in (
        ("stereo.fx", left.intrinsic[0], right.intrinsic[0]),
        ("stereo.fy", left.intrinsic[4], right.intrinsic[4]),
        ("stereo.cx", left.intrinsic[2], right.intrinsic[2]),
        ("stereo.cy", left.intrinsic[5], right.intrinsic[5]),
    ):
        checks.append(Check(label, right_value, left_value, tolerance))
    checks.append(
        Check(
            "right.P3",
            right.projection[3],
            -right.projection[0] * baseline_m,
            tolerance,
        )
    )
    return checks


def print_checks(checks: Sequence[Check]) -> int:
    failures = 0
    for check in checks:
        status = "PASS" if check.passed else "FAIL"
        failures += 0 if check.passed else 1
        print(
            f"{status} check={check.name} actual={check.actual:.12g} "
            f"expected={check.expected:.12g} error={check.error:.3g} "
            f"tolerance={check.tolerance:.3g}"
        )
    return failures


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sdf", required=True)
    parser.add_argument("--left-sensor", default="d435i_infra1")
    parser.add_argument("--right-sensor", default="d435i_infra2")
    parser.add_argument(
        "--left-topic",
        default="/d435i/infra1/camera_info",
    )
    parser.add_argument(
        "--right-topic",
        default="/cuvslam/input/infra2/camera_info",
    )
    parser.add_argument("--baseline-m", type=float, default=0.05)
    parser.add_argument("--tolerance", type=float, default=1e-5)
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    left_spec = load_camera_spec(args.sdf, args.left_sensor)
    right_spec = load_camera_spec(args.sdf, args.right_sensor)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo

    rclpy.init()
    node = Node("sim_camera_intrinsics_validator")
    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    messages = {}

    def receive(label):
        def callback(message):
            messages.setdefault(label, calibration_from_message(message))

        return callback

    subscriptions = [
        node.create_subscription(
            CameraInfo,
            args.left_topic,
            receive("left"),
            qos,
        ),
        node.create_subscription(
            CameraInfo,
            args.right_topic,
            receive("right"),
            qos,
        ),
    ]
    deadline = time.monotonic() + args.timeout
    while len(messages) < 2 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    missing = sorted({"left", "right"} - messages.keys())
    if missing:
        print(f"FAIL missing CameraInfo: {', '.join(missing)}")
        failures = len(missing)
    else:
        checks = camera_checks(
            "left",
            left_spec,
            messages["left"],
            args.tolerance,
        )
        checks.extend(
            camera_checks(
                "right",
                right_spec,
                messages["right"],
                args.tolerance,
            )
        )
        checks.extend(
            stereo_checks(
                messages["left"],
                messages["right"],
                args.baseline_m,
                args.tolerance,
            )
        )
        failures = print_checks(checks)

    print(
        f"SUMMARY status={'PASS' if failures == 0 else 'FAIL'} "
        f"failures={failures} sdf={Path(args.sdf)}"
    )
    del subscriptions
    node.destroy_node()
    rclpy.shutdown()
    if failures:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
