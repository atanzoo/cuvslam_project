from pathlib import Path

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration


def native_pose_bridge(context, bridge_executable):
    if LaunchConfiguration("enable_native_pose").perform(context).lower() != "true":
        return []

    native_pose_topic = LaunchConfiguration("native_pose_topic").perform(context)
    if not native_pose_topic.startswith("/"):
        native_pose_topic = "/" + native_pose_topic
    return [
        ExecuteProcess(
            cmd=[
                bridge_executable,
                f"{native_pose_topic}@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
                "--ros-args",
                "-r",
                f"{native_pose_topic}:=/simulation/native_pose_raw",
            ],
            additional_env={
                "IGN_PARTITION": LaunchConfiguration("ign_partition").perform(
                    context
                )
            },
            output="screen",
        )
    ]


def generate_launch_description():
    partition = LaunchConfiguration("ign_partition")
    enable_native_pose = LaunchConfiguration("enable_native_pose")
    bridge_executable = str(
        Path(get_package_prefix("ros_ign_bridge"))
        / "lib"
        / "ros_ign_bridge"
        / "parameter_bridge"
    )

    bridge_args = [
        "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
        "/model/slam_bot/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
        "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        "/d435i/infra1/image_rect_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
        "/d435i/infra1/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        "/d435i/infra2/image_rect_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
        "/d435i/infra2/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        "/d435i/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
        "--ros-args",
        "-r",
        "/model/slam_bot/cmd_vel:=/cmd_vel",
    ]
    return LaunchDescription([
        DeclareLaunchArgument("ign_partition", default_value="cuvslam_d435i_sim"),
        DeclareLaunchArgument(
            "enable_native_pose",
            default_value="false",
            description="Enable evaluation-only native pose bridge for route tests",
        ),
        DeclareLaunchArgument(
            "native_pose_topic",
            default_value="/world/cuvslam_mapping_simple/dynamic_pose/info",
            description="Gazebo Pose_V topic for the selected simulation world",
        ),
        LogInfo(msg=["Gazebo bridge executable=", bridge_executable]),
        LogInfo(msg=["Gazebo -> ROS: D435i stereo/IMU, /scan, /clock, evaluation-only native pose"]),
        LogInfo(msg=["ROS -> Gazebo: /cmd_vel"]),
        LogInfo(msg=["Wheel odometry bridge: disabled by estimator contract"]),
        ExecuteProcess(
            cmd=[bridge_executable] + bridge_args,
            additional_env={"IGN_PARTITION": partition},
            output="screen",
        ),
        OpaqueFunction(
            function=lambda context: native_pose_bridge(context, bridge_executable)
        ),
    ])
