"""Tests for MoveIt Servo readiness result evaluation."""

import unittest

from panda_manipulation.servo_readiness import servo_motion_quality
from panda_manipulation.pick_plan_pipeline import (
    bounded_servo_velocity,
    servo_collision_limited_endpoint_acceptable,
    servo_fault_injection_due,
    servo_halt_allows_cartesian_fallback,
)


class ServoReadinessTests(unittest.TestCase):
    def test_bounded_lift_and_return_passes(self):
        result = servo_motion_quality(
            (0.4, 0.0, 0.5),
            (0.401, -0.001, 0.53),
            (0.401, 0.001, 0.502),
            minimum_lift_m=0.015,
            return_tolerance_m=0.015,
            horizontal_tolerance_m=0.015,
        )
        self.assertTrue(result["success"])
        self.assertAlmostEqual(result["lift_m"], 0.03)

    def test_short_lift_and_lateral_drift_fail(self):
        result = servo_motion_quality(
            (0.4, 0.0, 0.5),
            (0.42, 0.0, 0.51),
            (0.4, 0.0, 0.5),
            minimum_lift_m=0.015,
            return_tolerance_m=0.015,
            horizontal_tolerance_m=0.015,
        )
        self.assertFalse(result["success"])
        self.assertIn("below required", result["reason"])
        self.assertIn("horizontal_drift", result["reason"])

    def test_bounded_servo_velocity_stops_and_avoids_overshoot(self):
        self.assertEqual(bounded_servo_velocity(-0.001, 0.01, 0.02, 0.002), 0.0)
        self.assertAlmostEqual(
            bounded_servo_velocity(-0.0005, 0.01, 0.02, 0.0001),
            -0.01,
        )
        self.assertAlmostEqual(
            bounded_servo_velocity(0.0001, 0.01, 0.02, 0.0),
            0.005,
        )

    def test_collision_limited_endpoint_only_accepts_final_downward_stage(self):
        self.assertTrue(
            servo_collision_limited_endpoint_acceptable(
                -0.009,
                stage_index=5,
                stage_count=6,
                collision_deceleration_seen=True,
                acceptance_m=0.012,
            )
        )
        self.assertFalse(
            servo_collision_limited_endpoint_acceptable(
                -0.009,
                stage_index=4,
                stage_count=6,
                collision_deceleration_seen=True,
                acceptance_m=0.012,
            )
        )

    def test_only_singularity_halt_allows_cartesian_fallback(self):
        self.assertTrue(servo_halt_allows_cartesian_fallback(2, True))
        self.assertFalse(servo_halt_allows_cartesian_fallback(5, True))
        self.assertFalse(servo_halt_allows_cartesian_fallback(2, False))
        self.assertFalse(
            servo_collision_limited_endpoint_acceptable(
                -0.013,
                stage_index=5,
                stage_count=6,
                collision_deceleration_seen=True,
                acceptance_m=0.012,
            )
        )

    def test_servo_fault_injection_is_stage_and_delay_bounded(self):
        self.assertTrue(
            servo_fault_injection_due("contact_loss", 2, 1, 0.51, 0.50)
        )
        self.assertTrue(
            servo_fault_injection_due("payload_drift", 3, 2, 1.0, 0.25)
        )
        self.assertFalse(
            servo_fault_injection_due("none", 2, 1, 1.0, 0.50)
        )
        self.assertFalse(
            servo_fault_injection_due("contact_loss", 2, 0, 1.0, 0.50)
        )
        self.assertFalse(
            servo_fault_injection_due("contact_loss", 2, 1, 0.49, 0.50)
        )


if __name__ == "__main__":
    unittest.main()
