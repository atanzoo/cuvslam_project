import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from create_sim_camera_rate_variant import (
    CAMERA_SENSORS,
    set_camera_update_rate,
)


SOURCE = (
    Path(__file__).resolve().parents[3]
    / "simulation"
    / "cuvslam"
    / "deployment"
    / "slam_gazebo"
    / "worlds"
    / "cuvslam_mapping_simple.sdf"
)


class SimCameraRateVariantTest(unittest.TestCase):
    def test_only_camera_rates_are_changed(self):
        with tempfile.NamedTemporaryFile(suffix=".sdf") as output:
            changed = set_camera_update_rate(SOURCE, output.name, 20.0)
            self.assertEqual(changed, 2)
            root = ET.parse(output.name).getroot()
            rates = {
                sensor.get("name"): float(sensor.findtext("update_rate"))
                for sensor in root.findall(".//sensor")
            }
        for name in CAMERA_SENSORS:
            self.assertEqual(rates[name], 20.0)
        self.assertEqual(rates["d435i_imu"], 200.0)
        self.assertEqual(rates["lidar"], 10.0)

    def test_invalid_rate_is_rejected(self):
        with self.assertRaises(ValueError):
            set_camera_update_rate(SOURCE, "/tmp/unused.sdf", 0.0)


if __name__ == "__main__":
    unittest.main()
