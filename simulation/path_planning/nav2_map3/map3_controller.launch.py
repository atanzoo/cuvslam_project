"""SIMULATION ONLY: isolated Foxy Nav2 planner+controller, no robot drivers."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    params_default = Path(__file__).with_name("map3_controller_params.yaml")
    map_yaml = LaunchConfiguration("map_yaml")
    params_file = LaunchConfiguration("params_file")
    return LaunchDescription([
        DeclareLaunchArgument("map_yaml"),
        DeclareLaunchArgument("params_file", default_value=str(params_default)),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map3_sim_map_to_odom",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            output="screen",
        ),
        LifecycleNode(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[params_file, {"yaml_filename": map_yaml, "use_sim_time": False}],
        ),
        LifecycleNode(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": False}],
        ),
        LifecycleNode(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": False}],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="map3_sim_controller_lifecycle_manager",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "autostart": True,
                "node_names": ["map_server", "planner_server", "controller_server"],
            }],
        ),
    ])
