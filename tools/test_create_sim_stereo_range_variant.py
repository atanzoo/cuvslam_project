import tempfile
import unittest
import xml.etree.ElementTree as ET

from create_sim_stereo_range_variant import (
    CAMERA_SENSORS,
    set_stereo_far_clip,
)


SOURCE = (
    "deployment/isaac_ros/cuvslam_sim_observability/"
    "worlds/indoor_gz_sim.sdf"
)


class StereoRangeVariantTest(unittest.TestCase):
    def test_changes_only_both_stereo_far_clips(self):
        with tempfile.NamedTemporaryFile(suffix=".sdf") as output:
            changed = set_stereo_far_clip(SOURCE, output.name, 2.5)
            self.assertEqual(changed, 2)
            tree = ET.parse(output.name)
            sensors = {
                sensor.get("name"): sensor
                for sensor in tree.getroot().findall(".//sensor")
            }
            for name in CAMERA_SENSORS:
                self.assertEqual(
                    sensors[name].findtext("./camera/clip/near"),
                    "0.10",
                )
                self.assertEqual(
                    sensors[name].findtext("./camera/clip/far"),
                    "2.5",
                )
                self.assertEqual(
                    sensors[name].findtext("./camera/image/width"),
                    "424",
                )
                self.assertEqual(
                    sensors[name].findtext("update_rate"),
                    "30",
                )

    def test_rejects_clip_not_above_near(self):
        with self.assertRaisesRegex(ValueError, "exceed near"):
            set_stereo_far_clip(SOURCE, "/tmp/unused.sdf", 0.1)


if __name__ == "__main__":
    unittest.main()
