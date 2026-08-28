import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from create_stereo_texture_variant import (
    PATCHES,
    add_near_field_texture,
)
from create_observable_world import create_world


SOURCE = (
    "deployment/isaac_ros/cuvslam_sim_observability/"
    "worlds/indoor_gz_sim.sdf"
)


class StereoTextureVariantTest(unittest.TestCase):
    def test_adds_unique_visual_only_patches(self):
        with tempfile.NamedTemporaryFile(suffix=".sdf") as observable:
            create_world(Path(SOURCE), Path(observable.name))
            with tempfile.NamedTemporaryFile(suffix=".sdf") as output:
                count = add_near_field_texture(observable.name, output.name)
                tree = ET.parse(output.name)
            self.assertEqual(count, len(PATCHES))
            patches = tree.getroot().findall(
                ".//visual[@name='stereo_texture_00']/.."
            )
            self.assertEqual(len(patches), 1)
            self.assertEqual(
                len(
                    tree.getroot().findall(
                        ".//model[@name='near_field_observability_boards']"
                        "/link/visual"
                    )
                ),
                66,
            )
            self.assertEqual(
                tree.getroot().findtext(
                    ".//sensor[@name='d435i_infra1']/camera/clip/far"
                ),
                "20.0",
            )

    def test_rejects_duplicate_application(self):
        with tempfile.NamedTemporaryFile(suffix=".sdf") as observable:
            create_world(Path(SOURCE), Path(observable.name))
            with tempfile.NamedTemporaryFile(suffix=".sdf") as first:
                with tempfile.NamedTemporaryFile(suffix=".sdf") as second:
                    add_near_field_texture(observable.name, first.name)
                    with self.assertRaisesRegex(ValueError, "already exists"):
                        add_near_field_texture(first.name, second.name)


if __name__ == "__main__":
    unittest.main()
