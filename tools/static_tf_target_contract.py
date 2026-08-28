"""SDF-derived truth and candidate edges for static TF target validation."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

from frame_contract_math import (
    Transform,
    compose,
    quaternion_from_rpy,
)


VALIDATION_WORLD = "simulation_validation_world"
TARGET_MODELS = ("feature_box_a", "feature_box_b")
TARGET_LIGHTS = ("ceiling_light",)
SENSOR_FRAMES = (
    "base_link",
    "camera_link",
    "camera_infra1_frame",
    "camera_infra1_optical_frame",
    "camera_infra2_frame",
    "camera_infra2_optical_frame",
    "camera_imu_frame",
    "lidar_link",
)


@dataclass(frozen=True)
class Edge:
    parent: str
    child: str
    transform: Transform


def parse_pose(element: ET.Element | None) -> Transform:
    if element is None or not element.text:
        return Transform()
    values = [float(value) for value in element.text.split()]
    if len(values) != 6:
        raise ValueError(f"expected six SDF pose values, got {values}")
    x, y, z, roll, pitch, yaw = values
    return Transform(
        translation=(x, y, z),
        rotation=quaternion_from_rpy(roll, pitch, yaw),
    )


def required(parent: ET.Element, path: str) -> ET.Element:
    element = parent.find(path)
    if element is None:
        raise ValueError(f"SDF element not found: {path}")
    return element


def load_sdf_truth(path: str | Path) -> dict[str, Transform]:
    world = required(ET.parse(path).getroot(), "world")
    robot = required(world, "model[@name='slam_bot']")
    world_robot = parse_pose(robot.find("pose"))

    frames = {VALIDATION_WORLD: Transform()}
    link_names = (
        "base_link",
        "camera_link",
        "camera_infra1_frame",
        "camera_infra2_frame",
        "camera_imu_frame",
        "laser_frame",
    )
    for name in link_names:
        link = required(robot, f"link[@name='{name}']")
        frames[name] = compose(world_robot, parse_pose(link.find("pose")))

    optical_rotation = Transform(
        rotation=(-0.5, 0.5, -0.5, 0.5),
    )
    frames["camera_infra1_optical_frame"] = compose(
        frames["camera_infra1_frame"],
        optical_rotation,
    )
    frames["camera_infra2_optical_frame"] = compose(
        frames["camera_infra2_frame"],
        optical_rotation,
    )
    frames["lidar_link"] = frames["laser_frame"]

    for name in TARGET_MODELS:
        model = required(world, f"model[@name='{name}']")
        frames[f"validation_target_{name}"] = parse_pose(model.find("pose"))
    for name in TARGET_LIGHTS:
        light = required(world, f"light[@name='{name}']")
        frames[f"validation_target_{name}"] = parse_pose(light.find("pose"))
    return frames


def candidate_edges(truth: dict[str, Transform]) -> tuple[Edge, ...]:
    identity = (0.0, 0.0, 0.0, 1.0)
    optical = (-0.5, 0.5, -0.5, 0.5)
    edges = [
        Edge(VALIDATION_WORLD, "base_link", truth["base_link"]),
        Edge(
            "base_link",
            "camera_link",
            Transform((0.19, 0.0, 0.20), identity),
        ),
        Edge(
            "camera_link",
            "camera_infra1_frame",
            Transform((0.0, 0.025, 0.0), identity),
        ),
        Edge(
            "camera_link",
            "camera_infra2_frame",
            Transform((0.0, -0.025, 0.0), identity),
        ),
        Edge(
            "camera_infra1_frame",
            "camera_infra1_optical_frame",
            Transform(rotation=optical),
        ),
        Edge(
            "camera_infra2_frame",
            "camera_infra2_optical_frame",
            Transform(rotation=optical),
        ),
        Edge(
            "camera_link",
            "camera_imu_frame",
            Transform(rotation=identity),
        ),
        Edge(
            "base_link",
            "lidar_link",
            Transform((0.0, 0.0, 0.13), identity),
        ),
    ]
    for frame, transform in truth.items():
        if frame.startswith("validation_target_"):
            edges.append(Edge(VALIDATION_WORLD, frame, transform))
    return tuple(edges)

