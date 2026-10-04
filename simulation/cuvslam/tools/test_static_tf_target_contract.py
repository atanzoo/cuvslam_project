import unittest
from pathlib import Path

from frame_contract_math import (
    compose,
    inverse,
    rotate_vector,
    translation_norm,
)
from static_tf_target_contract import (
    SENSOR_FRAMES,
    candidate_edges,
    load_sdf_truth,
)


SDF = (
    Path(__file__).resolve().parents[3]
    / "simulation"
    / "cuvslam"
    / "deployment"
    / "isaac_ros"
    / "cuvslam_sim_observability"
    / "worlds"
    / "indoor_gz_sim.sdf"
)


class StaticTfTargetContractTest(unittest.TestCase):
    def test_expected_frames_and_targets_exist(self):
        truth = load_sdf_truth(SDF)
        for frame in SENSOR_FRAMES:
            self.assertIn(frame, truth)
        targets = [
            frame
            for frame in truth
            if frame.startswith("validation_target_")
        ]
        self.assertEqual(len(targets), 3)

    def test_candidate_edges_reproduce_sdf_sensor_origins(self):
        truth = load_sdf_truth(SDF)
        actual = {"simulation_validation_world": truth["simulation_validation_world"]}
        unresolved = list(candidate_edges(truth))
        while unresolved:
            progress = False
            for edge in unresolved[:]:
                if edge.parent not in actual:
                    continue
                actual[edge.child] = compose(actual[edge.parent], edge.transform)
                unresolved.remove(edge)
                progress = True
            self.assertTrue(progress)
        for frame in SENSOR_FRAMES:
            residual = compose(inverse(truth[frame]), actual[frame])
            self.assertAlmostEqual(translation_norm(residual), 0.0, places=10)
            self.assertAlmostEqual(abs(residual.rotation[3]), 1.0, places=10)

    def test_optical_axis_contract(self):
        truth = load_sdf_truth(SDF)
        physical_optical = compose(
            inverse(truth["camera_infra1_frame"]),
            truth["camera_infra1_optical_frame"],
        )
        expected = {
            (1.0, 0.0, 0.0): (0.0, 0.0, 1.0),
            (0.0, 1.0, 0.0): (-1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0): (0.0, -1.0, 0.0),
        }
        optical_physical = inverse(physical_optical)
        for body_vector, optical_vector in expected.items():
            actual = rotate_vector(optical_physical.rotation, body_vector)
            for value, wanted in zip(actual, optical_vector):
                self.assertAlmostEqual(value, wanted, places=10)


if __name__ == "__main__":
    unittest.main()
