import ast
from pathlib import Path
import unittest


LAUNCH_PATH = (
    Path(__file__).parents[1]
    / "deployment"
    / "isaac_ros"
    / "isaac_ros_visual_slam_d435i_sim.launch.py"
)


class CuvslamLaunchContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tree = ast.parse(LAUNCH_PATH.read_text(encoding="ascii"))

    def test_visual_slam_uses_native_base_frame(self):
        parameter_sets = []
        for node in ast.walk(self.tree):
            if not isinstance(node, ast.Dict):
                continue
            values = {
                ast.literal_eval(key): value
                for key, value in zip(node.keys, node.values)
                if isinstance(key, ast.Constant)
            }
            if "base_frame" in values:
                parameter_sets.append(values)

        self.assertEqual(len(parameter_sets), 1)
        parameters = parameter_sets[0]
        self.assertEqual(ast.literal_eval(parameters["map_frame"]), "map")
        self.assertEqual(ast.literal_eval(parameters["odom_frame"]), "odom")
        self.assertEqual(
            ast.literal_eval(parameters["base_frame"]),
            "base_link",
        )
        self.assertEqual(
            ast.literal_eval(parameters["input_base_frame"]),
            "base_link",
        )
        self.assertEqual(
            ast.literal_eval(parameters["input_left_camera_frame"]),
            "camera_infra1_frame",
        )
        self.assertEqual(
            ast.literal_eval(parameters["input_right_camera_frame"]),
            "",
        )
        self.assertEqual(
            ast.literal_eval(parameters["input_imu_frame"]),
            "slam_bot/camera_imu_frame/d435i_imu",
        )
        self.assertTrue(
            ast.literal_eval(parameters["publish_map_to_odom_tf"])
        )
        self.assertTrue(
            ast.literal_eval(parameters["publish_odom_to_base_tf"])
        )

    def test_imu_fusion_is_opt_in(self):
        declarations = [
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "DeclareLaunchArgument"
            and ast.literal_eval(node.args[0]) == "enable_imu_fusion"
        ]
        self.assertEqual(len(declarations), 1)
        keywords = {
            keyword.arg: ast.literal_eval(keyword.value)
            for keyword in declarations[0].keywords
            if keyword.arg == "default_value"
        }
        self.assertEqual(keywords["default_value"], "false")

    def test_imu_noise_contract_matches_nvidia_d435i_example(self):
        parameter_sets = []
        for node in ast.walk(self.tree):
            if not isinstance(node, ast.Dict):
                continue
            values = {
                ast.literal_eval(key): value
                for key, value in zip(node.keys, node.values)
                if isinstance(key, ast.Constant)
            }
            if "calibration_frequency" in values:
                parameter_sets.append(values)

        self.assertEqual(len(parameter_sets), 1)
        parameters = parameter_sets[0]
        expected = {
            "gyro_noise_density": 0.000244,
            "gyro_random_walk": 0.000019393,
            "accel_noise_density": 0.001862,
            "accel_random_walk": 0.003,
            "calibration_frequency": 200.0,
        }
        for name, value in expected.items():
            self.assertEqual(ast.literal_eval(parameters[name]), value)

    def test_debug_dump_is_opt_in(self):
        declarations = [
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "DeclareLaunchArgument"
            and ast.literal_eval(node.args[0]) == "enable_debug_mode"
        ]
        self.assertEqual(len(declarations), 1)
        defaults = {
            keyword.arg: ast.literal_eval(keyword.value)
            for keyword in declarations[0].keywords
            if keyword.arg == "default_value"
        }
        self.assertEqual(defaults["default_value"], "false")

    def test_slam_visualization_is_opt_in(self):
        declarations = [
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "DeclareLaunchArgument"
            and ast.literal_eval(node.args[0]) == "enable_slam_visualization"
        ]
        self.assertEqual(len(declarations), 1)
        defaults = {
            keyword.arg: ast.literal_eval(keyword.value)
            for keyword in declarations[0].keywords
            if keyword.arg == "default_value"
        }
        self.assertEqual(defaults["default_value"], "false")

    def test_observations_view_is_opt_in(self):
        declarations = [
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "DeclareLaunchArgument"
            and ast.literal_eval(node.args[0]) == "enable_observations_view"
        ]
        self.assertEqual(len(declarations), 1)
        defaults = {
            keyword.arg: ast.literal_eval(keyword.value)
            for keyword in declarations[0].keywords
            if keyword.arg == "default_value"
        }
        self.assertEqual(defaults["default_value"], "false")

    def test_static_sensor_tree_matches_locked_contract(self):
        expected_edges = {
            ("base_link", "camera_link"),
            ("camera_link", "camera_infra1_frame"),
            ("camera_link", "camera_infra2_frame"),
            ("camera_infra1_frame", "camera_infra1_optical_frame"),
            ("camera_infra2_frame", "camera_infra2_optical_frame"),
            ("camera_link", "camera_imu_frame"),
            ("camera_imu_frame", "slam_bot/camera_imu_frame/d435i_imu"),
            ("base_link", "lidar_link"),
            ("lidar_link", "slam_bot/laser_frame/lidar"),
        }
        actual_edges = set()
        for node in ast.walk(self.tree):
            if not isinstance(node, ast.Call):
                continue
            if not isinstance(node.func, ast.Name):
                continue
            if node.func.id != "static_transform":
                continue
            arguments = [ast.literal_eval(argument) for argument in node.args]
            actual_edges.add((arguments[1], arguments[2]))

        self.assertEqual(actual_edges, expected_edges)

    def test_estimator_waits_for_static_transforms(self):
        timers = [
            node
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "TimerAction"
        ]
        self.assertEqual(len(timers), 1)
        keywords = {
            keyword.arg: ast.literal_eval(keyword.value)
            for keyword in timers[0].keywords
            if keyword.arg == "period"
        }
        self.assertEqual(keywords["period"], 1.0)


if __name__ == "__main__":
    unittest.main()
