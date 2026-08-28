import unittest

from gz_pose_log import parse_pose_v_text


POSE_LOG = """header {
  stamp {
    sec: 12
    nsec: 340000000
  }
}
pose {
  name: "other"
  position {
    x: 9
  }
}
pose {
  name: "slam_bot"
  position {
    x: -1.7
    z: 0.06
  }
  orientation {
    y: 0.12
    w: 0.99
  }
}
header {
  stamp {
    sec: 12
    nsec: 360000000
  }
}
pose {
  name: "slam_bot"
  position {
    x: -1.6
    y: 0.1
  }
  orientation {
    w: 1
  }
}
"""


class GzPoseLogTest(unittest.TestCase):
    def test_extracts_named_entity_and_defaults_omitted_fields(self) -> None:
        samples = parse_pose_v_text(POSE_LOG, "slam_bot")

        self.assertEqual(len(samples), 2)
        self.assertEqual(samples[0].timestamp_ns, 12_340_000_000)
        self.assertEqual(samples[0].translation, (-1.7, 0.0, 0.06))
        self.assertEqual(samples[0].rotation, (0.0, 0.12, 0.0, 0.99))
        self.assertEqual(samples[1].translation, (-1.6, 0.1, 0.0))
        self.assertEqual(samples[1].rotation, (0.0, 0.0, 0.0, 1.0))

    def test_ignores_missing_entity(self) -> None:
        self.assertEqual(parse_pose_v_text(POSE_LOG, "missing"), [])


if __name__ == "__main__":
    unittest.main()
