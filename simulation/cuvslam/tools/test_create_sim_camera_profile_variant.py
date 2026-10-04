import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from create_sim_camera_profile_variant import (
    CAMERA_SENSORS,
    set_camera_image_profile,
)


SOURCE = (
    Path(__file__).resolve().parents[3]
    / "simulation"
    / "cuvslam"
    / "deployment"
    / "isaac_ros"
    / "cuvslam_sim_observability"
    / "worlds"
    / "indoor_gz_sim.sdf"
)


class CameraProfileVariantTest(unittest.TestCase):
    def test_changes_only_both_stereo_image_dimensions(self):
        with tempfile.NamedTemporaryFile(suffix=".sdf") as output:
            changed = set_camera_image_profile(
                SOURCE,
                output.name,
                640,
                360,
            )
            self.assertEqual(changed, 2)
            tree = ET.parse(output.name)
            sensors = {
                sensor.get("name"): sensor
                for sensor in tree.getroot().findall(".//sensor")
            }
            for name in CAMERA_SENSORS:
                self.assertEqual(
                    sensors[name].findtext("./camera/image/width"),
                    "640",
                )
                self.assertEqual(
                    sensors[name].findtext("./camera/image/height"),
                    "360",
                )
                self.assertEqual(
                    sensors[name].findtext("update_rate"),
                    "30",
                )
                self.assertEqual(
                    sensors[name].findtext("./camera/horizontal_fov"),
                    "1.58825",
                )

    def test_rejects_non_positive_dimension(self):
        with self.assertRaisesRegex(ValueError, "positive"):
            set_camera_image_profile(SOURCE, "/tmp/unused.sdf", 0, 360)


if __name__ == "__main__":
    unittest.main()
