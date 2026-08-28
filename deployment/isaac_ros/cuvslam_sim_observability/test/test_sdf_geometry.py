from pathlib import Path

from cuvslam_sim_observability.sdf_geometry import (
    box_edge_points,
    load_world,
)


def test_baseline_world_geometry():
    world_path = Path(__file__).parents[1] / "worlds" / "indoor_gz_sim.sdf"
    world = load_world(world_path)
    names = {box.model_name for box in world.boxes}
    assert {
        "floor",
        "north_wall",
        "south_wall",
        "east_wall",
        "west_wall",
        "feature_box_a",
        "feature_box_b",
    } <= names
    assert world.initial_robot_pose.translation == (0.0, 0.0, 0.06)
    assert world.base_camera_pose.translation == (0.19, 0.025, 0.20)


def test_box_has_twelve_line_segments():
    assert len(box_edge_points((1.0, 2.0, 3.0))) == 24
