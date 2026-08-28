"""Real Intel RealSense D435i stereo/VIO launch for the Jetson target.

The RealSense node owns the camera/IMU static TF tree.  This launch therefore
does not add the simulation-only aliases or sensor transforms.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    enable_imu_fusion = LaunchConfiguration("enable_imu_fusion")
    enable_mapping = LaunchConfiguration("enable_mapping")
    enable_debug_mode = LaunchConfiguration("enable_debug_mode")
    enable_slam_visualization = LaunchConfiguration("enable_slam_visualization")
    enable_observations_view = LaunchConfiguration("enable_observations_view")
    denoise_input_images = LaunchConfiguration("denoise_input_images")
    emitter_enabled = LaunchConfiguration("emitter_enabled")
    enable_auto_exposure = LaunchConfiguration("enable_auto_exposure")
    exposure = LaunchConfiguration("exposure")
    gain = LaunchConfiguration("gain")
    debug_dump_path = LaunchConfiguration("debug_dump_path")
    initial_reset = LaunchConfiguration("initial_reset")
    enable_visual_slam = LaunchConfiguration("enable_visual_slam")
    infra_profile = LaunchConfiguration("infra_profile")

    realsense_camera = Node(
        name="camera",
        namespace="camera",
        package="realsense2_camera",
        executable="realsense2_camera_node",
        parameters=[
            {
                "enable_infra1": True,
                "enable_infra2": True,
                "enable_color": False,
                "enable_depth": False,
                # Keep the stereo profile configurable so we can qualify the
                # camera/cuVSLAM path at a lower pixel load without changing
                # the rest of the launch wiring.
                "depth_module.infra_profile": infra_profile,
                "depth_module.profile": infra_profile,
                "enable_gyro": True,
                "enable_accel": True,
                "gyro_fps": 200,
                "accel_fps": 200,
                "unite_imu_method": 2,
                # Keep auto exposure as the safe default for changing indoor
                # scenes.  The R2 runner exposes the emitter as a deliberate
                # lighting-profile choice instead of silently changing it.
                "depth_module.enable_auto_exposure": ParameterValue(
                    enable_auto_exposure, value_type=bool
                ),
                "depth_module.exposure": ParameterValue(
                    exposure, value_type=int
                ),
                "depth_module.gain": ParameterValue(
                    gain, value_type=int
                ),
                "depth_module.emitter_enabled": ParameterValue(
                    emitter_enabled, value_type=int
                ),
                "publish_tf": True,
                "tf_publish_rate": 0.0,
                "initial_reset": ParameterValue(initial_reset, value_type=bool),
            }
        ],
        output="screen",
    )

    visual_slam = ComposableNode(
        name="visual_slam_node",
        package="isaac_ros_visual_slam",
        plugin="nvidia::isaac_ros::visual_slam::VisualSlamNode",
        parameters=[
            {
                "use_sim_time": False,
                # NVIDIA documents this path for noisy / low-light input.
                "denoise_input_images": ParameterValue(
                    denoise_input_images, value_type=bool
                ),
                "rectified_images": True,
                "enable_imu_fusion": ParameterValue(
                    enable_imu_fusion, value_type=bool
                ),
                "gyro_noise_density": 0.000244,
                "gyro_random_walk": 0.000019393,
                "accel_noise_density": 0.001862,
                "accel_random_walk": 0.003,
                "calibration_frequency": 200.0,
                "enable_debug_mode": ParameterValue(
                    enable_debug_mode, value_type=bool
                ),
                "debug_dump_path": debug_dump_path,
                "enable_slam_visualization": ParameterValue(
                    enable_slam_visualization, value_type=bool
                ),
                "enable_landmarks_view": ParameterValue(
                    enable_slam_visualization, value_type=bool
                ),
                "enable_observations_view": ParameterValue(
                    enable_observations_view, value_type=bool
                ),
                "enable_localization_n_mapping": ParameterValue(
                    enable_mapping, value_type=bool
                ),
                "map_frame": "map",
                "odom_frame": "odom",
                "base_frame": "camera_link",
                "input_base_frame": "camera_link",
                "input_left_camera_frame": "camera_infra1_frame",
                # NVIDIA's RealSense example uses this frame.  The current
                # driver also publishes camera_imu_optical_frame on /camera/imu
                # and exposes an identity TF path between the two IMU frames.
                "input_imu_frame": "camera_gyro_optical_frame",
                "publish_map_to_odom_tf": True,
                "publish_odom_to_base_tf": True,
                "img_jitter_threshold_ms": 40.0,
                "msg_filter_queue_size": 100,
                "image_qos": "SENSOR_DATA",
            }
        ],
        remappings=[
            ("stereo_camera/left/image", "/camera/infra1/image_rect_raw"),
            ("stereo_camera/left/camera_info", "/camera/infra1/camera_info"),
            ("stereo_camera/right/image", "/camera/infra2/image_rect_raw"),
            ("stereo_camera/right/camera_info", "/camera/infra2/camera_info"),
            ("visual_slam/imu", "/camera/imu"),
        ],
    )

    container = ComposableNodeContainer(
        name="d435i_real_visual_slam_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[visual_slam],
        output="screen",
        condition=IfCondition(enable_visual_slam),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_imu_fusion", default_value="false"),
            DeclareLaunchArgument("enable_mapping", default_value="false"),
            DeclareLaunchArgument("enable_debug_mode", default_value="false"),
            DeclareLaunchArgument(
                "enable_slam_visualization", default_value="false"
            ),
            DeclareLaunchArgument(
                "enable_observations_view", default_value="false"
            ),
            DeclareLaunchArgument(
                "denoise_input_images", default_value="true"
            ),
            DeclareLaunchArgument(
                "emitter_enabled", default_value="1"
            ),
            DeclareLaunchArgument(
                "enable_auto_exposure", default_value="true"
            ),
            DeclareLaunchArgument("exposure", default_value="8500"),
            DeclareLaunchArgument("gain", default_value="16"),
            DeclareLaunchArgument(
                "debug_dump_path", default_value="/tmp/cuvslam_d435i_real"
            ),
            DeclareLaunchArgument("initial_reset", default_value="true"),
            DeclareLaunchArgument("enable_visual_slam", default_value="true"),
            DeclareLaunchArgument("infra_profile", default_value="640,360,30"),
            realsense_camera,
            container,
        ]
    )
