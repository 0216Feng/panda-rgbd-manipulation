import unittest

from panda_manipulation.aruco_perception_validator import summarize_errors
from panda_manipulation.aruco_pose_estimator import (
    filtered_position,
    marker_index,
)
from panda_manipulation.geometry import Vector3


class ArucoPerceptionTests(unittest.TestCase):
    def test_marker_selection_requires_configured_id(self):
        self.assertEqual(marker_index([7, 0, 12], 0), 1)
        self.assertIsNone(marker_index([7, 12], 0))

    def test_position_filter_smooths_small_motion(self):
        filtered = filtered_position(
            Vector3(0.50, 0.0, 0.04),
            Vector3(0.52, 0.02, 0.04),
            smoothing_alpha=0.5,
            max_jump_m=0.08,
        )
        self.assertIsNotNone(filtered)
        self.assertAlmostEqual(filtered.x, 0.51)
        self.assertAlmostEqual(filtered.y, 0.01)

    def test_position_filter_rejects_implausible_jump(self):
        filtered = filtered_position(
            Vector3(0.50, 0.0, 0.04),
            Vector3(0.70, 0.0, 0.04),
            smoothing_alpha=0.5,
            max_jump_m=0.08,
        )
        self.assertIsNone(filtered)

    def test_perception_error_summary(self):
        summary = summarize_errors([0.005, 0.010, 0.015])
        self.assertEqual(summary["samples"], 3)
        self.assertAlmostEqual(summary["mean_error_m"], 0.010)
        self.assertAlmostEqual(summary["max_error_m"], 0.015)
        self.assertGreater(summary["std_error_m"], 0.0)


if __name__ == "__main__":
    unittest.main()
