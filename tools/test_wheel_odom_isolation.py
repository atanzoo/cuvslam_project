from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
ACTIVE_RUNTIME_FILES = (
    ROOT / "deployment/slam_gazebo/launch/gz_bridge_portable.launch.py",
    ROOT / "tools/run_container_straight_mapping_trial.sh",
    ROOT / "tools/run_jetson_straight_mapping.sh",
    ROOT / "tools/simulation_imu_turn_profile.py",
    ROOT / "tools/evaluate_in_place_turn_bag.py",
    ROOT / "tools/evaluate_native_corner_bag.py",
    ROOT / "tools/simulation_native_corner_profile.py",
    ROOT / "tools/simulation_native_pose_relay.py",
    ROOT / "tools/evaluate_observation_geometry_bag.py",
    ROOT / "tools/evaluate_landmark_geometry_bag.py",
    ROOT / "deployment/isaac_ros/isaac_ros_visual_slam_d435i_sim.launch.py",
)
FORBIDDEN = (
    "/model/slam_bot/odometry",
    "simulation_truth_distance_profile.py",
    "simulation_truth_corner_profile.py",
    "simulation_truth_roundtrip_profile.py",
)


class WheelOdomIsolationTest(unittest.TestCase):
    def test_active_runtime_has_no_wheel_odom_dependency(self):
        for path in ACTIVE_RUNTIME_FILES:
            text = path.read_text(encoding="utf-8")
            for forbidden in FORBIDDEN:
                self.assertNotIn(forbidden, text, f"{path}: {forbidden}")

    def test_ground_truth_odom_only_appears_in_runtime_rejection(self):
        for path in ACTIVE_RUNTIME_FILES:
            text = path.read_text(encoding="utf-8")
            count = text.count("/ground_truth/odom")
            if path.name == "run_jetson_straight_mapping.sh":
                self.assertEqual(count, 2)
                self.assertIn("prohibited wheel odometry topic", text)
            else:
                self.assertEqual(count, 0, str(path))

    def test_turn_profile_has_no_odometry_message_dependency(self):
        text = (
            ROOT / "tools/simulation_imu_turn_profile.py"
        ).read_text(encoding="utf-8")
        self.assertNotIn("nav_msgs", text)
        self.assertNotIn("Odometry", text)


if __name__ == "__main__":
    unittest.main()
