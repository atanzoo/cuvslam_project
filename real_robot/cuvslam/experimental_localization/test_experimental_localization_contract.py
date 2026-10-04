#!/usr/bin/env python3
"""Offline contract tests for the experimental localization stage."""

import hashlib
import importlib.util
import math
import re
import sys
import unittest
from types import SimpleNamespace
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
EXPERIMENT = ROOT / "real_robot/cuvslam/experimental_localization"
LAUNCH = EXPERIMENT / "experimental_candidate_map_localization.launch.py"
MONITOR = EXPERIMENT / "experimental_localization_monitor.py"
MAP_DIR = ROOT / "real_robot/cuvslam/evidence/maps/20260924_092630_current_cuvslam_a2m12"
MAP_YAML = MAP_DIR / "map.yaml"
MAP_PGM = MAP_DIR / "map.pgm"


def load_monitor():
    spec = importlib.util.spec_from_file_location("experimental_localization_monitor", MONITOR)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class ExperimentalLocalizationContractTest(unittest.TestCase):
    def test_candidate_map_exists_is_complete_and_hash_matches_readme(self):
        self.assertTrue(MAP_YAML.is_file())
        self.assertTrue(MAP_PGM.is_file())
        yaml_text = MAP_YAML.read_text()
        self.assertIn("image: map.pgm", yaml_text)
        self.assertIn("resolution: 0.05", yaml_text)
        self.assertRegex(yaml_text, r"origin:\s*\[-3\.89, -5\.34, 0\]")
        self.assertEqual(
            hashlib.sha256(MAP_YAML.read_bytes()).hexdigest(),
            "be42f16b9730106c54f0bb75e83b04ec48b7c2c69f49e12c4fb4183ca21feb5d",
        )
        pgm = MAP_PGM.read_bytes()
        self.assertTrue(pgm.startswith(b"P5"))
        header = re.match(rb"P5\s+199\s+308\s+255\s+", pgm)
        self.assertIsNotNone(header)
        assert header is not None
        self.assertEqual(len(pgm) - header.end(), 199 * 308)
        self.assertEqual(
            hashlib.sha256(pgm).hexdigest(),
            "41e913c872bd118fc971b313024339f78e23247aec94fde00a18e51aa62efd74",
        )

    def test_launch_has_only_localization_owners(self):
        text = LAUNCH.read_text()
        for required in (
            'package="nav2_map_server"',
            'executable="map_server"',
            'package="nav2_amcl"',
            'executable="amcl"',
            'package="nav2_lifecycle_manager"',
            'node_names": ["candidate_map_server", "candidate_amcl"]',
            'global_frame_id": "map"',
            'odom_frame_id": "odom"',
            'base_frame_id": "base_link"',
            'tf_broadcast": True',
            'scan_topic": scan_topic',
        ):
            self.assertIn(required, text, required)
        for forbidden in (
            "slam_toolbox",
            "planner_server",
            "controller_server",
            "behavior_server",
            "bt_navigator",
            "velocity_smoother",
            "base_controller",
            "/cmd_vel",
            "cmd_vel",
        ):
            self.assertNotIn(forbidden, text, forbidden)

    def test_monitor_schema_and_read_only_contract(self):
        text = MONITOR.read_text()
        for required in (
            "SCHEMA_VERSION = \"experimental_localization_telemetry.v1\"",
            'STATE_WAIT = "WAIT"',
            'STATE_OK = "OK"',
            'STATE_REVIEW = "REVIEW"',
            'STATE_LOST = "LOST"',
            '"map_to_odom"',
            '"map"',
            '"scan"',
            '"trajectory"',
            '"active_map_warning"',
            '"extrinsic_candidate"',
            '"read_only": True',
            '"/amcl_pose"',
            '"/map"',
            '"/scan"',
            'OccupancyGrid',
            'LaserScan',
            '"/api/render"',
            '"/api/map_grid"',
            'transform_scan_to_map',
            'classify_scan_overlap',
            'nearest_occupied_distance',
            '"map_overlap"',
            'SCAN_DELAY',
            'Time.from_msg(entry["stamp_msg"])',
            'lookup_transform("map", "odom", Time())',
        ):
            self.assertIn(required, text, required)
        self.assertNotIn('create_publisher(', text)
        self.assertNotIn('send_transform', text)
        self.assertNotIn('create_client(', text)

    def test_map_payload_scan_transform_and_exact_tf_delay(self):
        module = load_monitor()
        map_snapshot = module.MapSnapshot()
        map_snapshot.accept_grid(
            width=2,
            height=2,
            resolution_m=0.05,
            origin_x_m=-3.89,
            origin_y_m=-5.34,
            origin_yaw_rad=0.0,
            data=[0, 100, -1, 50],
            stamp_s=12.0,
        )
        metadata = map_snapshot.metadata(now_s=13.0)
        self.assertEqual((metadata["width"], metadata["height"]), (2, 2))
        self.assertEqual(metadata["origin"]["x_m"], -3.89)
        self.assertEqual(metadata["cell_count"], 4)
        self.assertEqual(map_snapshot.grid_payload()["encoding"], "gray_u8_base64")

        classified, overlap = module.classify_scan_overlap(
            [
                {"x_m": -3.815, "y_m": -5.315},
                {"x_m": -3.0, "y_m": -5.0},
            ],
            map_snapshot,
        )
        self.assertEqual(overlap["status"], "OBSERVE")
        self.assertEqual(overlap["matched_near_count"], 1)
        self.assertEqual(overlap["matched_review_count"], 1)
        self.assertEqual(classified[0]["overlap"], "near")
        self.assertEqual(classified[1]["overlap"], "out_of_map")

        points, valid_count = module.transform_scan_to_map(
            [1.0, float("nan"), 2.0],
            angle_min=0.0,
            angle_increment=math.pi / 2.0,
            range_min=0.1,
            range_max=3.0,
            transform_x=1.0,
            transform_y=2.0,
            transform_yaw=math.pi / 2.0,
        )
        self.assertEqual(valid_count, 2)
        self.assertAlmostEqual(points[0]["x_m"], 1.0, places=6)
        self.assertAlmostEqual(points[0]["y_m"], 3.0, places=6)

        scan = module.ScanSnapshot()
        stamp = SimpleNamespace(sec=1, nanosec=0)
        scan.accept_scan(
            stamp_s=1.0,
            stamp_msg=stamp,
            frame_id="rplidar_link",
            ranges=[1.0],
            angle_min=0.0,
            angle_increment=0.1,
            range_min=0.1,
            range_max=10.0,
        )
        scan.resolve(
            lambda _entry: {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0, "tf_stamp_s": 1.0},
            now_s=1.1,
        )
        self.assertEqual(scan.summary(1.1)["status"], module.SCAN_OK)
        scan.accept_scan(
            stamp_s=2.0,
            stamp_msg=stamp,
            frame_id="rplidar_link",
            ranges=[1.0],
            angle_min=0.0,
            angle_increment=0.1,
            range_min=0.1,
            range_max=10.0,
        )
        scan.resolve(lambda _entry: None, now_s=2.1)
        delayed = scan.summary(2.1)
        self.assertEqual(delayed["status"], module.SCAN_DELAY)
        self.assertTrue(delayed["retained_last_valid"])
        self.assertFalse(delayed["exact_timestamp_tf"])

    def test_state_machine_positive_and_negative_cases(self):
        module = load_monitor()
        machine = module.LocalizationStateMachine()
        state, reasons, transition = machine.evaluate(
            now_s=10.0,
            pose_age_s=0.1,
            tf_age_s=0.1,
            covariance={"xy_m2": 0.01, "yaw_rad2": 0.01},
        )
        self.assertEqual((state, transition), (module.STATE_OK, "WAIT->OK"))
        self.assertEqual(reasons, [])

        state, reasons, transition = machine.evaluate(
            now_s=10.0,
            pose_age_s=None,
            tf_age_s=0.1,
            covariance=None,
        )
        self.assertEqual(state, module.STATE_REVIEW)
        self.assertIn("amcl_pose_missing", reasons)
        self.assertEqual(transition, "OK->REVIEW")

        state, reasons, transition = machine.evaluate(
            now_s=10.0,
            pose_age_s=0.1,
            tf_age_s=None,
            covariance={"xy_m2": 0.01, "yaw_rad2": 0.01},
        )
        self.assertEqual(state, module.STATE_REVIEW)
        self.assertIn("required_telemetry_missing", reasons)
        self.assertIsNone(transition)

        state, reasons, transition = machine.evaluate(
            now_s=10.0,
            pose_age_s=0.1,
            tf_age_s=0.1,
            covariance={"xy_m2": 0.01, "yaw_rad2": 0.01},
            pose_frame_id="odom",
        )
        self.assertEqual(state, module.STATE_REVIEW)
        self.assertIn("amcl_pose_frame_invalid", reasons)
        self.assertIsNone(transition)

        state, reasons, transition = machine.evaluate(
            now_s=10.0,
            pose_age_s=3.1,
            tf_age_s=3.1,
            covariance={"xy_m2": 0.01, "yaw_rad2": 0.01},
        )
        self.assertEqual(state, module.STATE_LOST)
        self.assertIn("telemetry_stale", reasons)
        self.assertEqual(transition, "REVIEW->LOST")

        state, reasons, transition = machine.evaluate(
            now_s=10.0,
            pose_age_s=0.1,
            tf_age_s=0.1,
            covariance={"xy_m2": 0.01, "yaw_rad2": 0.01},
        )
        self.assertEqual(state, module.STATE_OK)
        self.assertIn("recovered", reasons)
        self.assertEqual(transition, "LOST->OK")

    def test_quaternion_yaw(self):
        module = load_monitor()
        self.assertAlmostEqual(
            module.quaternion_to_yaw(0.0, 0.0, 0.70710678, 0.70710678),
            1.5707963,
            places=5,
        )

    def test_empty_snapshot_is_wait_and_exposes_schema(self):
        module = load_monitor()
        snapshot = module.MonitorSnapshot()
        payload = snapshot.update(now_s=1.0, tf_available=False, tf_stamp_s=None)
        self.assertEqual(payload["schema_version"], module.SCHEMA_VERSION)
        self.assertEqual(payload["state"], module.STATE_WAIT)
        self.assertFalse(payload["map_to_odom"]["available"])
        self.assertTrue(payload["read_only"])
        self.assertIn("trajectory", payload)
        self.assertIn("active_map_warning", payload)

if __name__ == "__main__":
    unittest.main()
