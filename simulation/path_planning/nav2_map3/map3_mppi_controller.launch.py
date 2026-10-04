"""SIMULATION ONLY: isolated Humble Nav2 planner and MPPI controller."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    params_default = Path(__file__).with_name("map3_mppi_params.yaml")
    overrides_default = Path(__file__).with_name("map3_mppi_overrides_none.yaml")
    map_yaml = LaunchConfiguration("map_yaml")
    params_file = LaunchConfiguration("params_file")
    controller_overrides_file = LaunchConfiguration("controller_overrides_file")
    return LaunchDescription([
        DeclareLaunchArgument("map_yaml"),
        DeclareLaunchArgument("params_file", default_value=str(params_default)),
        DeclareLaunchArgument(
            "controller_overrides_file", default_value=str(overrides_default),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map3_mppi_sim_map_to_odom",
            namespace="/",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            output="screen",
        ),
        LifecycleNode(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            namespace="/",
            output="screen",
            parameters=[
                params_file, controller_overrides_file,
                {"yaml_filename": map_yaml, "use_sim_time": False},
            ],
        ),
        LifecycleNode(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            namespace="/",
            output="screen",
            parameters=[params_file, controller_overrides_file, {"use_sim_time": False}],
        ),
        LifecycleNode(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            namespace="/",
            output="screen",
            parameters=[params_file, controller_overrides_file, {"use_sim_time": False}],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="map3_mppi_sim_controller_lifecycle_manager",
            namespace="/",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "autostart": True,
                "node_names": ["map_server", "planner_server", "controller_server"],
            }],
        ),
    ])
