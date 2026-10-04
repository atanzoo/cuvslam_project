"""Experimental real D435i/cuVSLAM odometry plus RPLIDAR A2M12 mapping.

Ownership contract:

* cuVSLAM owns odom -> base_link only;
* the static publishers own base_link -> camera_link and
  base_link -> rplidar_link;
* sllidar_ros2 publishes /scan;
* slam_toolbox is the sole owner of map -> odom.

The default extrinsics are deliberately zero placeholders.  They make the
first stationary wiring test possible, but they are not accuracy-qualified
and must be replaced before motion or map-quality claims.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _declare(name, default, description):
    return DeclareLaunchArgument(
        name,
        default_value=str(default),
        description=description,
    )


def generate_launch_description():
    workspace_root = LaunchConfiguration("workspace_root")
    serial_port = LaunchConfiguration("serial_port")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    frame_id = LaunchConfiguration("frame_id")
    scan_mode = LaunchConfiguration("scan_mode")
    inverted = LaunchConfiguration("inverted")
    angle_compensate = LaunchConfiguration("angle_compensate")
    initial_reset = LaunchConfiguration("initial_reset")
    enable_imu_fusion = LaunchConfiguration("enable_imu_fusion")
    infra_profile = LaunchConfiguration("infra_profile")
    denoise_input_images = LaunchConfiguration("denoise_input_images")
    emitter_enabled = LaunchConfiguration("emitter_enabled")
    enable_auto_exposure = LaunchConfiguration("enable_auto_exposure")
    exposure = LaunchConfiguration("exposure")
    gain = LaunchConfiguration("gain")
    enable_observations_view = LaunchConfiguration("enable_observations_view")
    enable_lidar_odom = LaunchConfiguration("enable_lidar_odom")
    enable_ekf_shadow = LaunchConfiguration("enable_ekf_shadow")
    enable_scan_tf_gate = LaunchConfiguration("enable_scan_tf_gate")

    camera_x = LaunchConfiguration("camera_x")
    camera_y = LaunchConfiguration("camera_y")
    camera_z = LaunchConfiguration("camera_z")
    camera_yaw = LaunchConfiguration("camera_yaw")
    lidar_x = LaunchConfiguration("lidar_x")
    lidar_y = LaunchConfiguration("lidar_y")
    lidar_z = LaunchConfiguration("lidar_z")
    lidar_yaw = LaunchConfiguration("lidar_yaw")

    real_cuvslam_launch = PathJoinSubstitution(
        [
            FindPackageShare("isaac_ros_visual_slam"),
            "launch",
            "isaac_ros_visual_slam_d435i_real.launch.py",
        ]
    )
    slam_params_file = PathJoinSubstitution(
        [
            workspace_root,
            "real_robot",
            "cuvslam",
            "config",
            "real_d435i_rplidar_slam_toolbox.yaml",
        ]
    )
    lidar_odom_params_file = PathJoinSubstitution(
        [
            workspace_root,
            "real_robot",
            "cuvslam",
            "config",
            "real_d435i_rplidar_lidar_odom.yaml",
        ]
    )
    ekf_shadow_params_file = PathJoinSubstitution(
        [
            workspace_root,
            "real_robot",
            "cuvslam",
            "config",
            "real_d435i_rplidar_ekf_shadow.yaml",
        ]
    )
    odom_contract_guard_script = PathJoinSubstitution(
        [workspace_root, "real_robot", "cuvslam", "tools", "odom_contract_guard.py"]
    )
    scan_tf_gate_script = PathJoinSubstitution(
        [workspace_root, "real_robot", "cuvslam", "tools", "scan_tf_gate.py"]
    )

    camera_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="d435i_base_to_camera_static_tf",
        arguments=[
            camera_x,
            camera_y,
            camera_z,
            camera_yaw,
            "0.0",  # pitch; CLI order is yaw, pitch, roll
            "0.0",  # roll
            "base_link",
            "camera_link",
        ],
        output="screen",
    )

    lidar_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_rplidar_static_tf",
        arguments=[
            lidar_x,
            lidar_y,
            lidar_z,
            lidar_yaw,
            "0.0",  # pitch; CLI order is yaw, pitch, roll
            "0.0",  # roll
            "base_link",
            "rplidar_link",
        ],
        output="screen",
    )

    rplidar = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        parameters=[
            {
                "channel_type": "serial",
                "serial_port": serial_port,
                "serial_baudrate": serial_baudrate,
                "frame_id": frame_id,
                "inverted": inverted,
                "angle_compensate": angle_compensate,
                "scan_mode": scan_mode,
            }
        ],
        remappings=[("/scan", "/scan_raw")],
        output="screen",
    )

    scan_tf_gate = ExecuteProcess(
        cmd=[
            "python3",
            scan_tf_gate_script,
            "--input", "/scan_raw",
            "--output", "/scan",
            "--target-frame", "odom",
            "--max-wait-s", "2.0",
            "--queue-size", "32",
            "--check-hz", "30.0",
        ],
        condition=IfCondition(enable_scan_tf_gate),
        output="screen",
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[slam_params_file],
        output="screen",
    )

    cuvslam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(real_cuvslam_launch),
        launch_arguments={
            "enable_imu_fusion": enable_imu_fusion,
            "enable_mapping": "false",
            "enable_debug_mode": "false",
            "enable_slam_visualization": "false",
            "enable_observations_view": enable_observations_view,
            "denoise_input_images": denoise_input_images,
            "emitter_enabled": emitter_enabled,
            "enable_auto_exposure": enable_auto_exposure,
            "exposure": exposure,
            "gain": gain,
            "debug_dump_path": "/workspaces/isaac_ros_data/experiments/real_d435i_rplidar/debug_dump",
            "initial_reset": initial_reset,
            "enable_visual_slam": "true",
            "infra_profile": infra_profile,
            "base_frame": "base_link",
            "input_base_frame": "base_link",
            "publish_map_to_odom_tf": "false",
            "publish_odom_to_base_tf": "true",
        }.items(),
    )

    # This is deliberately an optional shadow branch.  rf2o publishes an
    # independent local odometry topic but does not publish TF, so cuVSLAM
    # remains the only odom -> base_link owner during the first comparison.
    lidar_odom = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        parameters=[lidar_odom_params_file],
        # Defense in depth: publish_tf=false is the intended contract, and
        # this remap prevents an accidental parameter regression from reaching
        # the production /tf graph.
        remappings=[("/tf", "/shadow/lidar_odom_tf_unused")],
        condition=IfCondition(enable_lidar_odom),
        output="screen",
    )

    # Validate selected twist fields before they enter robot_localization.
    # The guard rejects zero/invalid covariance instead of inventing a tiny
    # value that would make the source appear more certain than it is.
    visual_odom_guard = ExecuteProcess(
        cmd=[
            "python3",
            odom_contract_guard_script,
            "--source",
            "visual",
            "--input",
            "/visual_slam/tracking/odometry",
            "--output",
            "/fusion/visual_odom",
            "--required-twist",
            "0,1,5",
            "--expected-frame",
            "odom",
            "--expected-child-frame",
            "base_link",
        ],
        condition=IfCondition(enable_ekf_shadow),
        output="screen",
    )
    lidar_odom_guard = ExecuteProcess(
        cmd=[
            "python3",
            odom_contract_guard_script,
            "--source",
            "lidar",
            "--input",
            "/lidar/odom",
            "--output",
            "/fusion/lidar_odom",
            "--required-twist",
            "0,5",
            "--expected-frame",
            "odom",
            "--expected-child-frame",
            "base_link",
        ],
        condition=IfCondition(enable_ekf_shadow),
        output="screen",
    )

    # EKF shadow output is remapped to a diagnostic-only topic and has TF
    # publication disabled in its YAML.  It must not replace the active TF
    # owner until a separate promotion review approves that change.
    ekf_shadow = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_shadow",
        parameters=[ekf_shadow_params_file],
        remappings=[
            ("odometry/filtered", "/odometry/filtered_shadow"),
            ("/tf", "/shadow/ekf_tf_unused"),
        ],
        condition=IfCondition(enable_ekf_shadow),
        output="screen",
    )

    return LaunchDescription(
        [
            _declare(
                "workspace_root",
                "/workspaces/isaac_ros-dev",
                "Mounted Isaac ROS workspace root",
            ),
            _declare(
                "serial_port",
                "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_538fdae3c9e4c947b71ec98a5cebbe00-if00-port0",
                "A2M12 serial device",
            ),
            _declare("serial_baudrate", 256000, "A2M12 serial baud rate"),
            _declare("frame_id", "rplidar_link", "LaserScan frame"),
            _declare("scan_mode", "Sensitivity", "A2M12 scan mode"),
            _declare("inverted", "false", "Invert scan direction"),
            _declare("angle_compensate", "true", "Enable angle compensation"),
            _declare("enable_imu_fusion", "true", "Enable cuVSLAM IMU fusion"),
            _declare("infra_profile", "640,360,30", "D435i IR profile"),
            _declare(
                "denoise_input_images",
                "true",
                "Enable cuVSLAM input denoising",
            ),
            _declare("emitter_enabled", "1", "Enable the D435i IR projector"),
            _declare(
                "enable_auto_exposure",
                "true",
                "Enable D435i automatic exposure",
            ),
            _declare("exposure", "8500", "Manual D435i exposure"),
            _declare("gain", "16", "D435i gain"),
            _declare(
                "enable_observations_view",
                "false",
                "Publish cuVSLAM observations for diagnostics",
            ),
            _declare(
                "enable_lidar_odom",
                "false",
                "Start independent RPLIDAR scan-matching odometry",
            ),
            _declare(
                "enable_ekf_shadow",
                "false",
                "Start robot_localization EKF without publishing TF",
            ),
            _declare(
                "enable_scan_tf_gate",
                "true",
                "Wait for timestamped odom TF before releasing /scan",
            ),
            _declare(
                "initial_reset",
                "false",
                "Reset the D435i before opening the stream",
            ),
            _declare("camera_x", 0.0, "Temporary base_link to camera_link x (m)"),
            _declare("camera_y", 0.0, "Temporary base_link to camera_link y (m)"),
            _declare("camera_z", 0.0, "Temporary base_link to camera_link z (m)"),
            _declare("camera_yaw", 0.0, "Temporary base_link to camera_link yaw (rad)"),
            _declare("lidar_x", 0.0, "Temporary base_link to rplidar_link x (m)"),
            _declare("lidar_y", 0.0, "Temporary base_link to rplidar_link y (m)"),
            _declare("lidar_z", 0.0, "Temporary base_link to rplidar_link z (m)"),
            _declare("lidar_yaw", 0.0, "Temporary base_link to rplidar_link yaw (rad)"),
            LogInfo(
                msg=(
                    "WARNING: A2M12 integration uses zero extrinsic placeholders; "
                    "perform only stationary wiring validation until calibrated."
                )
            ),
            camera_static_tf,
            lidar_static_tf,
            rplidar,
            scan_tf_gate,
            TimerAction(period=1.0, actions=[cuvslam, slam_toolbox]),
            TimerAction(
                period=2.0,
                actions=[
                    visual_odom_guard,
                    lidar_odom_guard,
                    lidar_odom,
                    ekf_shadow,
                ],
            ),
        ]
    )
