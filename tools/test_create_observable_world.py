import tempfile
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET

from create_observable_world import create_world


ROOT = Path(__file__).resolve().parents[1]
BASELINE = (
    ROOT
    / "deployment"
    / "slam_gazebo"
    / "worlds"
    / "cuvslam_mapping_simple.sdf"
)


class ObservableWorldTest(unittest.TestCase):
    def test_generation_preserves_baseline_and_adds_one_model(self):
        baseline_before = BASELINE.read_bytes()
        baseline_tree = ET.parse(BASELINE)
        baseline_models = baseline_tree.findall("./world/model")

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "observable.sdf"
            create_world(BASELINE, output)
            generated = ET.parse(output)
            model_names = {
                model.attrib["name"]
                for model in generated.findall("./world/model")
            }

        self.assertEqual(BASELINE.read_bytes(), baseline_before)
        self.assertEqual(len(model_names), len(baseline_models) + 1)
        self.assertIn("near_field_observability_boards", model_names)
        self.assertIn("slam_bot", model_names)

    def test_all_square_route_legs_have_feature_boards(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "observable.sdf"
            create_world(BASELINE, output)
            generated = ET.parse(output)
            visual_names = {
                visual.attrib["name"]
                for visual in generated.findall(
                    ".//model[@name='near_field_observability_boards']"
                    "/link/visual"
                )
            }

        for leg in ("left", "right", "north", "west", "south"):
            self.assertTrue(
                any(name.startswith(leg) for name in visual_names),
                f"missing features for {leg} leg",
            )
        self.assertEqual(
            len([name for name in visual_names if name.startswith("floor_marker")]),
            4,
        )


if __name__ == "__main__":
    unittest.main()
