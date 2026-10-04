"""Dependency-free rigid-transform math for frame-contract diagnostics."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Tuple


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


@dataclass(frozen=True)
class Transform:
    translation: Vector3 = (0.0, 0.0, 0.0)
    rotation: Quaternion = (0.0, 0.0, 0.0, 1.0)


def normalize(q: Quaternion) -> Quaternion:
    magnitude = math.sqrt(sum(value * value for value in q))
    if magnitude == 0.0:
        raise ValueError("zero-length quaternion")
    return tuple(value / magnitude for value in q)  # type: ignore[return-value]


def quaternion_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return normalize(
        (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )
    )


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return normalize(
        (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )
    )


def rotate_vector(q: Quaternion, vector: Vector3) -> Vector3:
    qx, qy, qz, qw = normalize(q)
    vx, vy, vz = vector
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def compose(parent_child: Transform, child_grandchild: Transform) -> Transform:
    offset = rotate_vector(
        parent_child.rotation,
        child_grandchild.translation,
    )
    return Transform(
        translation=tuple(
            parent + child
            for parent, child in zip(parent_child.translation, offset)
        ),
        rotation=quaternion_multiply(
            parent_child.rotation,
            child_grandchild.rotation,
        ),
    )


def inverse(transform: Transform) -> Transform:
    x, y, z, w = normalize(transform.rotation)
    inverse_rotation = (-x, -y, -z, w)
    translation = rotate_vector(
        inverse_rotation,
        tuple(-value for value in transform.translation),
    )
    return Transform(translation=translation, rotation=inverse_rotation)


def relative(start: Transform, end: Transform) -> Transform:
    return compose(inverse(start), end)


def translation_norm(transform: Transform) -> float:
    return math.sqrt(sum(value * value for value in transform.translation))


def rotation_angle_degrees(transform: Transform) -> float:
    _, _, _, w = normalize(transform.rotation)
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, abs(w)))))


def base_to_left_optical(
    x: float = 0.19,
    y: float = 0.025,
    z: float = 0.20,
) -> Transform:
    # ROS body axes to optical axes: X right, Y down, Z forward.
    return Transform(
        translation=(x, y, z),
        rotation=(-0.5, 0.5, -0.5, 0.5),
    )
