"""Launch Isaac ROS Visual SLAM against the declared simulated D435i topics."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def static_transform(
    name,
    parent_frame,
    child_frame,
    translation=(0.0, 0.0, 0.0),
    rotation=(0.0, 0.0, 0.0, 1.0),
):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        arguments=[
            *(
                str(value) if isinstance(value, (int, float)) else value
                for value in translation
            ),
            *(str(value) for value in rotation),
            parent_frame,
            child_frame,
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )


def generate_launch_description():
    enable_imu_fusion = LaunchConfiguration("enable_imu_fusion")
    enable_debug_mode = LaunchConfiguration("enable_debug_mode")
    enable_slam_visualization = LaunchConfiguration(
        "enable_slam_visualization"
    )
    enable_observations_view = LaunchConfiguration(
        "enable_observations_view"
    )
    enable_mapping = LaunchConfiguration("enable_mapping")
    debug_dump_path = LaunchConfiguration("debug_dump_path")
    camera_x = LaunchConfiguration("camera_x")
    world_sdf = LaunchConfiguration("world_sdf")
    visual_slam_node = ComposableNode(
        name="visual_slam_node",
        package="isaac_ros_visual_slam",
        plugin="nvidia::isaac_ros::visual_slam::VisualSlamNode",
        parameters=[
            {
                "use_sim_time": True,
                "denoise_input_images": False,
                "rectified_images": True,
                "enable_imu_fusion": ParameterValue(
                    enable_imu_fusion,
                    value_type=bool,
                ),
                "gyro_noise_density": 0.000244,
                "gyro_random_walk": 0.000019393,
                "accel_noise_density": 0.001862,
                "accel_random_walk": 0.003,
                "calibration_frequency": 200.0,
                "enable_debug_mode": ParameterValue(
                    enable_debug_mode,
                    value_type=bool,
                ),
                "debug_dump_path": debug_dump_path,
                "enable_slam_visualization": ParameterValue(
                    enable_slam_visualization,
                    value_type=bool,
                ),
                "enable_landmarks_view": ParameterValue(
                    enable_slam_visualization,
                    value_type=bool,
                ),
                "enable_observations_view": ParameterValue(
                    enable_observations_view,
                    value_type=bool,
                ),
                "enable_localization_n_mapping": ParameterValue(
                    enable_mapping,
                    value_type=bool,
                ),
                "map_frame": "map",
                "odom_frame": "odom",
                "base_frame": "base_link",
                "input_base_frame": "base_link",
                "input_left_camera_frame": "camera_infra1_frame",
                "input_right_camera_frame": "",
                "input_imu_frame": "slam_bot/camera_imu_frame/d435i_imu",
                "publish_map_to_odom_tf": True,
                "publish_odom_to_base_tf": True,
                # NVIDIA defines this as the acceptable interval between
                # consecutive synchronized image messages, not the left/right
                # pair skew.  The simulator is nominally 30 Hz but measures
                # about 35 ms between frames, so 40 ms avoids false warnings
                # while still detecting a dropped-frame-sized gap.
                "img_jitter_threshold_ms": 40.0,
                "msg_filter_queue_size": 100,
                "image_qos": "SENSOR_DATA",
            }
        ],
        remappings=[
            ("stereo_camera/left/image", "/d435i/infra1/image_rect_raw"),
            ("stereo_camera/left/camera_info", "/d435i/infra1/camera_info"),
            ("stereo_camera/right/image", "/d435i/infra2/image_rect_raw"),
            (
                "stereo_camera/right/camera_info",
                "/cuvslam/input/infra2/camera_info",
            ),
            ("visual_slam/imu", "/d435i/imu"),
        ],
    )

    container = ComposableNodeContainer(
        name="d435i_sim_visual_slam_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[visual_slam_node],
        output="screen",
    )

    right_camera_info_adapter = Node(
        package="cuvslam_sim_sensor_adapter",
        executable="right_camera_info_adapter",
        name="right_camera_info_adapter",
        parameters=[
            {
                "use_sim_time": True,
                "stereo_baseline_m": 0.05,
            }
        ],
        output="screen",
    )

    static_transforms = [
        static_transform(
            "base_to_camera",
            "base_link",
            "camera_link",
            translation=(camera_x, 0.0, 0.20),
        ),
        static_transform(
            "camera_to_infra1",
            "camera_link",
            "camera_infra1_frame",
            translation=(0.0, 0.025, 0.0),
        ),
        static_transform(
            "camera_to_infra2",
            "camera_link",
            "camera_infra2_frame",
            translation=(0.0, -0.025, 0.0),
        ),
        static_transform(
            "infra1_to_infra1_optical",
            "camera_infra1_frame",
            "camera_infra1_optical_frame",
            rotation=(-0.5, 0.5, -0.5, 0.5),
        ),
        static_transform(
            "infra2_to_infra2_optical",
            "camera_infra2_frame",
            "camera_infra2_optical_frame",
            rotation=(-0.5, 0.5, -0.5, 0.5),
        ),
        static_transform(
            "camera_to_imu",
            "camera_link",
            "camera_imu_frame",
        ),
        static_transform(
            "imu_to_simulated_imu",
            "camera_imu_frame",
            "slam_bot/camera_imu_frame/d435i_imu",
        ),
        static_transform(
            "base_to_lidar",
            "base_link",
            "lidar_link",
            translation=(0.0, 0.0, 0.13),
        ),
        static_transform(
            "lidar_to_simulated_lidar",
            "lidar_link",
            "slam_bot/laser_frame/lidar",
        ),
    ]

    world_outline = Node(
        package="cuvslam_sim_observability",
        executable="world_outline_publisher",
        name="world_outline_publisher",
        parameters=[{"use_sim_time": True, "world_sdf": world_sdf}],
        output="screen",
    )

    native_pose_relay = ExecuteProcess(
        cmd=[
            "python3",
            "/workspaces/isaac_ros-dev/simulation/cuvslam/tools/"
            "simulation_native_pose_relay.py",
            "--expected-start-x",
            "-1.8",
            "--expected-start-y",
            "-1.8",
            "--expected-start-z",
            "0.06",
        ],
        output="screen",
    )

    default_world_sdf = (
        get_package_share_directory("cuvslam_sim_observability")
        + "/worlds/indoor_gz_sim.sdf"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_x",
                default_value="0.19",
                description="Diagnostic base_link to D435i forward offset",
            ),
            DeclareLaunchArgument(
                "enable_mapping",
                default_value="true",
                description="Enable cuVSLAM localization-and-mapping backend",
            ),
            DeclareLaunchArgument(
                "enable_imu_fusion",
                default_value="false",
                description=(
                    "Enable the simulated 200 Hz D435i IMU input for A/B tests"
                ),
            ),
            DeclareLaunchArgument(
                "enable_debug_mode",
                default_value="false",
                description="Write NVIDIA cuVSLAM input debug data for diagnostics",
            ),
            DeclareLaunchArgument(
                "enable_slam_visualization",
                default_value="false",
                description="Publish sparse mapping landmarks for bounded tests",
            ),
            DeclareLaunchArgument(
                "enable_observations_view",
                default_value="false",
                description="Publish current feature observations for diagnostics",
            ),
            DeclareLaunchArgument(
                "debug_dump_path",
                default_value="/tmp/cuvslam_d435i_sim",
                description="Directory for NVIDIA cuVSLAM debug input data",
            ),
            DeclareLaunchArgument(
                "world_sdf",
                default_value=default_world_sdf,
                description="SDF used only for the Foxglove ground-truth overlay",
            ),
            right_camera_info_adapter,
            *static_transforms,
            native_pose_relay,
            world_outline,
            TimerAction(period=1.0, actions=[container]),
        ]
    )
