"""Experimental candidate-map localization only.

Runtime ownership:

* ``candidate_map_server`` owns the candidate ``/map`` data source.
* ``candidate_amcl`` is the only ``map -> odom`` TF publisher in this launch.
* cuVSLAM and the A2M12 runtime are external inputs; this launch does not
  start either sensor path.

The launch deliberately contains only the localization lifecycle group. It is
not a navigation bringup and it does not claim map or localization accuracy.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


EXPERIMENT_ROOT = Path(__file__).resolve().parent
CANDIDATE_MAP = (
    EXPERIMENT_ROOT.parent
    / "evidence"
    / "maps"
    / "20260911_162933_cuvslam_a2m12_emitter_off"
    / "map.yaml"
)


def _argument(name, default, description):
    return DeclareLaunchArgument(
        name,
        default_value=str(default),
        description=description,
    )


def generate_launch_description():
    map_yaml = LaunchConfiguration("map_yaml")
    scan_topic = LaunchConfiguration("scan_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="candidate_map_server",
        output="screen",
        parameters=[
            {
                "yaml_filename": map_yaml,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="candidate_amcl",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "global_frame_id": "map",
                "odom_frame_id": "odom",
                "base_frame_id": "base_link",
                "scan_topic": scan_topic,
                "map_topic": "/map",
                "tf_broadcast": True,
                "set_initial_pose": False,
                "first_map_only": False,
                "robot_model_type": "nav2_amcl::DifferentialMotionModel",
                "laser_model_type": "likelihood_field",
            }
        ],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="candidate_localization_lifecycle_manager",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": ["candidate_map_server", "candidate_amcl"],
            }
        ],
    )

    return LaunchDescription(
        [
            _argument(
                "map_yaml",
                CANDIDATE_MAP,
                "Candidate occupancy-map YAML; override for an isolated review.",
            ),
            _argument(
                "scan_topic",
                "/scan",
                "Existing LaserScan input supplied by the external A2M12 path.",
            ),
            _argument(
                "use_sim_time",
                "false",
                "Use simulated time only for an explicitly labelled replay.",
            ),
            map_server,
            amcl,
            lifecycle_manager,
        ]
    )
