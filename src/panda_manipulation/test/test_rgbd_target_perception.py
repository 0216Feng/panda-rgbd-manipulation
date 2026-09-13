import unittest

import numpy as np

from panda_manipulation.rgbd_target_pose_estimator import (
    fuse_rgb_xy_with_depth_height,
    largest_red_region,
    ray_plane_intersection,
    refine_target_from_depth,
)


class RgbdTargetPerceptionTests(unittest.TestCase):
    def test_red_segmentation_returns_largest_target(self):
        image = np.zeros((120, 160, 3), dtype=np.uint8)
        image[30:70, 50:90] = (0, 0, 240)
        image[5:10, 5:10] = (0, 0, 240)
        region, mask = largest_red_region(image, minimum_area_px=100.0)
        self.assertIsNotNone(region)
        self.assertAlmostEqual(region.centroid_u, 69.5, delta=1.0)
        self.assertAlmostEqual(region.centroid_v, 49.5, delta=1.0)
        self.assertGreater(region.area_px, 1400.0)
        self.assertGreater(int(mask.sum()), 0)

    def test_camera_ray_intersects_workspace_plane(self):
        camera_matrix = ((100.0, 0.0, 50.0), (0.0, 100.0, 50.0), (0.0, 0.0, 1.0))
        rotation = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))
        point = ray_plane_intersection(
            (50.0, 50.0),
            camera_matrix,
            (0.5, 0.0, 1.0),
            rotation,
            0.08,
        )
        self.assertIsNotNone(point)
        self.assertAlmostEqual(point[0], 0.5)
        self.assertAlmostEqual(point[1], 0.0)
        self.assertAlmostEqual(point[2], 0.08)

    def test_depth_refinement_rejects_table_and_uses_target_cluster(self):
        points = [
            (0.50, 0.00, 0.0),
            (0.51, 0.01, 0.08),
            (0.52, 0.00, 0.081),
            (0.51, -0.01, 0.079),
            (0.80, 0.00, 0.08),
        ]
        result = refine_target_from_depth(
            points,
            seed_xy=(0.51, 0.0),
            search_radius_m=0.06,
            minimum_z_m=0.02,
            maximum_z_m=0.12,
            minimum_points=3,
        )
        self.assertIsNotNone(result)
        center, count = result
        self.assertEqual(count, 3)
        self.assertAlmostEqual(center[0], 0.51)
        self.assertAlmostEqual(center[2], 0.08)

    def test_fusion_keeps_rgb_xy_and_uses_depth_height(self):
        fused = fuse_rgb_xy_with_depth_height(
            rgb_seed=(0.520, 0.001, 0.08),
            depth_surface_center=(0.527, -0.004, 0.081),
            object_height_m=0.08,
        )
        self.assertEqual(fused[:2], (0.520, 0.001))
        self.assertAlmostEqual(fused[2], 0.041)


if __name__ == "__main__":
    unittest.main()
