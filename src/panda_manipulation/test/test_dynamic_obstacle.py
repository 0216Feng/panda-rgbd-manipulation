import math
import unittest

from panda_manipulation.depth_obstacle_tracker import (
    estimate_velocity,
    obstacle_cluster_center,
    predicted_swept_box,
    quaternion_rotation_matrix,
    transform_point,
)
from panda_manipulation.dynamic_obstacle_controller import linear_position
from panda_manipulation.dynamic_obstacle_perception_validator import summarize_position_errors
from panda_manipulation.trajectory_safety_monitor import (
    nearest_trajectory_index,
    sample_remaining_indices,
    trajectory_collision_lead_s,
)


class DynamicObstacleTests(unittest.TestCase):
    def test_quaternion_transform_rotates_and_translates_point(self):
        half = math.sqrt(0.5)
        rotation = quaternion_rotation_matrix((0.0, 0.0, half, half))
        transformed = transform_point((1.0, 0.0, 0.0), (0.5, -0.5, 0.2), rotation)
        self.assertAlmostEqual(transformed[0], 0.5, places=6)
        self.assertAlmostEqual(transformed[1], 0.5, places=6)
        self.assertAlmostEqual(transformed[2], 0.2, places=6)

    def test_obstacle_cluster_ignores_outside_roi_and_selects_box_footprint(self):
        obstacle = [
            (0.31 + 0.02 * x, -0.05 + 0.02 * y, 0.30)
            for x in range(6)
            for y in range(6)
        ]
        distractor = [(0.60 + 0.01 * index, 0.0, 0.30) for index in range(30)]
        detection = obstacle_cluster_center(
            obstacle + distractor,
            (0.25, -0.50, 0.21),
            (0.49, 0.50, 0.38),
            expected_width_m=0.12,
            grid_size_m=0.02,
            minimum_points=10,
        )
        self.assertIsNotNone(detection)
        center, count = detection
        self.assertEqual(count, 36)
        self.assertAlmostEqual(center[0], 0.36, places=6)
        self.assertAlmostEqual(center[1], 0.0, places=6)

    def test_linear_position_clamps_at_endpoint(self):
        self.assertAlmostEqual(linear_position(-0.5, 0.5, 0.2, 1.0), -0.3)
        self.assertAlmostEqual(linear_position(-0.5, 0.5, 0.2, 20.0), 0.5)
        self.assertAlmostEqual(linear_position(0.5, -0.5, 0.2, 20.0), -0.5)

    def test_position_error_summary(self):
        summary = summarize_position_errors([0.01, 0.02, 0.03])
        self.assertAlmostEqual(summary["mean_error_m"], 0.02)
        self.assertAlmostEqual(summary["max_error_m"], 0.03)

    def test_nearest_trajectory_index_starts_from_measured_state(self):
        rows = [[0.0, 0.0], [0.5, 0.5], [1.0, 1.0]]
        self.assertEqual(nearest_trajectory_index(rows, [0.45, 0.55]), 1)

    def test_remaining_samples_include_current_and_final_state(self):
        indices = sample_remaining_indices(10, 100, 8)
        self.assertEqual(indices[0], 10)
        self.assertEqual(indices[-1], 99)
        self.assertLessEqual(len(indices), 8)

    def test_velocity_estimate_is_smoothed_and_bounded(self):
        velocity = estimate_velocity(
            (0.0, 0.0, 0.0),
            (0.0, 0.2, 0.0),
            0.1,
            smoothing_alpha=1.0,
            maximum_speed_mps=0.5,
        )
        self.assertAlmostEqual(velocity[0], 0.0)
        self.assertAlmostEqual(velocity[1], 0.5)
        self.assertAlmostEqual(velocity[2], 0.0)

    def test_prediction_creates_forward_swept_volume(self):
        center, size, displacement = predicted_swept_box(
            (0.4, -0.2, 0.15),
            (0.0, 0.2, 0.0),
            (0.12, 0.12, 0.30),
            horizon_s=0.5,
            collision_padding_m=0.02,
        )
        self.assertAlmostEqual(displacement[1], 0.1)
        self.assertAlmostEqual(center[1], -0.15)
        self.assertAlmostEqual(size[1], 0.26)

    def test_prediction_ignores_stationary_noise(self):
        center, size, displacement = predicted_swept_box(
            (0.4, 0.0, 0.15),
            (0.0, 0.01, 0.0),
            (0.12, 0.12, 0.30),
            horizon_s=1.0,
            collision_padding_m=0.02,
            minimum_speed_mps=0.03,
        )
        self.assertEqual(displacement, (0.0, 0.0, 0.0))
        self.assertEqual(center, (0.4, 0.0, 0.15))
        self.assertEqual(size, (0.12, 0.16, 0.30))

    def test_collision_lead_uses_current_trajectory_progress(self):
        self.assertAlmostEqual(
            trajectory_collision_lead_s([0.0, 0.5, 1.2, 2.0], 1, 3),
            1.5,
        )


if __name__ == "__main__":
    unittest.main()
