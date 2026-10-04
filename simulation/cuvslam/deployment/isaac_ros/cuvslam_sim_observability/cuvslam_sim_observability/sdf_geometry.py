"""Extract static box outlines and alignment poses from a Gazebo SDF world."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

from .geometry import Transform, compose, quaternion_from_rpy


@dataclass(frozen=True)
class BoxGeometry:
    model_name: str
    collision_name: str
    pose: Transform
    size: tuple[float, float, float]


@dataclass(frozen=True)
class WorldGeometry:
    boxes: tuple[BoxGeometry, ...]
    initial_robot_pose: Transform
    base_camera_pose: Transform


def parse_pose(element: ET.Element | None) -> Transform:
    if element is None or not (element.text or "").strip():
        return Transform()
    values = [float(value) for value in element.text.split()]
    if len(values) != 6:
        raise ValueError(f"expected six SDF pose values, got {values}")
    x, y, z, roll, pitch, yaw = values
    return Transform(
        translation=(x, y, z),
        rotation=quaternion_from_rpy(roll, pitch, yaw),
    )


def parse_size(element: ET.Element | None) -> tuple[float, float, float]:
    if element is None or not (element.text or "").strip():
        raise ValueError("box geometry is missing size")
    values = tuple(float(value) for value in element.text.split())
    if len(values) != 3:
        raise ValueError(f"expected three box size values, got {values}")
    return values  # type: ignore[return-value]


def load_world(path: str | Path) -> WorldGeometry:
    root = ET.parse(path).getroot()
    world = root.find("world")
    if world is None:
        raise ValueError("SDF does not contain a world")

    robot = world.find("model[@name='slam_bot']")
    if robot is None:
        raise ValueError("SDF does not contain slam_bot")
    initial_robot_pose = parse_pose(robot.find("pose"))

    camera_link = robot.find("link[@name='camera_infra1_frame']")
    if camera_link is None:
        raise ValueError("slam_bot does not contain camera_infra1_frame")
    camera_link_pose = parse_pose(camera_link.find("pose"))
    link_to_optical = Transform(
        rotation=(-0.5, 0.5, -0.5, 0.5),
    )
    base_camera_pose = compose(camera_link_pose, link_to_optical)

    boxes: list[BoxGeometry] = []
    for model in world.findall("model"):
        if model.get("name") == "slam_bot":
            continue
        if (model.findtext("static") or "").strip().lower() != "true":
            continue
        model_pose = parse_pose(model.find("pose"))
        for link in model.findall("link"):
            link_pose = parse_pose(link.find("pose"))
            for collision in link.findall("collision"):
                size_element = collision.find("geometry/box/size")
                if size_element is None:
                    continue
                collision_pose = parse_pose(collision.find("pose"))
                boxes.append(
                    BoxGeometry(
                        model_name=model.get("name", "unnamed"),
                        collision_name=collision.get("name", "collision"),
                        pose=compose(
                            compose(model_pose, link_pose),
                            collision_pose,
                        ),
                        size=parse_size(size_element),
                    )
                )
            # The dense-turn feature arc is deliberately visual-only.  Keep
            # it in the Foxglove reference overlay without giving Gazebo
            # LiDAR a collision surface to scan as a fake wall.
            if model.get("name") == "dense_turn_feature_arc":
                for visual in link.findall("visual"):
                    if visual.get("name") != "panel":
                        continue
                    size_element = visual.find("geometry/box/size")
                    if size_element is None:
                        continue
                    visual_pose = parse_pose(visual.find("pose"))
                    boxes.append(
                        BoxGeometry(
                            model_name=model.get("name", "unnamed"),
                            collision_name=visual.get("name", "visual"),
                            pose=compose(
                                compose(model_pose, link_pose),
                                visual_pose,
                            ),
                            size=parse_size(size_element),
                        )
                    )

    if not boxes:
        raise ValueError("SDF does not contain static box collisions")
    return WorldGeometry(
        boxes=tuple(boxes),
        initial_robot_pose=initial_robot_pose,
        base_camera_pose=base_camera_pose,
    )


def box_edge_points(
    size: tuple[float, float, float],
) -> tuple[tuple[float, float, float], ...]:
    half_x, half_y, half_z = (value * 0.5 for value in size)
    corners = (
        (-half_x, -half_y, -half_z),
        (half_x, -half_y, -half_z),
        (half_x, half_y, -half_z),
        (-half_x, half_y, -half_z),
        (-half_x, -half_y, half_z),
        (half_x, -half_y, half_z),
        (half_x, half_y, half_z),
        (-half_x, half_y, half_z),
    )
    edge_indices = (
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    )
    return tuple(corners[index] for edge in edge_indices for index in edge)
