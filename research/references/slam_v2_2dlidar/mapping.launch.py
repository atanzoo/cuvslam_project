from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    scan_topic = LaunchConfiguration("scan_topic")

    parameters_file = PathJoinSubstitution([
        FindPackageShare("slam_bringup"),
        "config",
        "slam_toolbox.yaml",
    ])

    launch_args = [
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
    ]

    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="async_slam_toolbox_node",
        output="screen",
        parameters=[
            parameters_file,
            {
                "odom_frame": odom_frame,
                "base_frame": base_frame,
                "scan_topic": scan_topic,
            },
        ],
    )

    return LaunchDescription(launch_args + [slam_node])
