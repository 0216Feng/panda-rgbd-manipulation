"""Tests for synchronized transfer diagnostics helpers."""

import unittest
from types import SimpleNamespace

from panda_manipulation.transfer_diagnostics import (
    ARM_JOINTS,
    FINGER_JOINTS,
    contact_sides,
    controller_point_values,
    csv_fields,
    first_stage_arm_positions,
    local_point_drift,
    payload_phase_event,
    percentile,
    point_in_frame,
    relative_motion_drift,
    render_backend_comparison_markdown,
    render_comparison_markdown,
    render_physical_disturbance_safety_markdown,
    render_servo_watchdog_safety_markdown,
    root_mean_square,
    selected_grasp_yaw_offset,
    summarize_backend_comparison,
    summarize_comparison,
    summarize_physical_disturbance_safety,
    summarize_servo_watchdog_safety,
    summarize_rows,
)


class TransferDiagnosticsTests(unittest.TestCase):
    """Verify pure diagnostics calculations without a ROS graph."""

    def test_relative_motion_removes_constant_frame_translation(self):
        drift = relative_motion_drift(
            baseline_hand=(0.4, 0.0, 0.2),
            baseline_object=(0.5, 0.0, 0.8),
            current_hand=(0.5, -0.1, 0.3),
            current_object=(0.6, -0.1, 0.9),
        )
        self.assertAlmostEqual(drift, 0.0)

    def test_relative_motion_detects_payload_slip(self):
        drift = relative_motion_drift(
            baseline_hand=(0.4, 0.0, 0.2),
            baseline_object=(0.5, 0.0, 0.8),
            current_hand=(0.5, 0.0, 0.3),
            current_object=(0.61, 0.0, 0.9),
        )
        self.assertAlmostEqual(drift, 0.01)

    def test_hand_frame_payload_position_is_invariant_under_rigid_rotation(self):
        half_sqrt_two = 2.0**-0.5
        baseline = point_in_frame(
            frame_position=(0.0, 0.0, 0.0),
            frame_orientation=(0.0, 0.0, 0.0, 1.0),
            point_position=(0.1, 0.0, 0.0),
        )
        rotated = point_in_frame(
            frame_position=(1.0, 2.0, 3.0),
            frame_orientation=(0.0, 0.0, half_sqrt_two, half_sqrt_two),
            point_position=(1.0, 2.1, 3.0),
        )

        self.assertAlmostEqual(local_point_drift(baseline, rotated), 0.0)

    def test_hand_frame_payload_drift_detects_local_slip(self):
        self.assertAlmostEqual(
            local_point_drift((0.1, 0.0, 0.0), (0.11, 0.0, 0.0)),
            0.01,
        )

    def test_controller_state_is_mapped_by_joint_name(self):
        state = SimpleNamespace(
            joint_names=["panda_joint2", "panda_joint1"],
            error=SimpleNamespace(positions=[0.2, -0.1]),
        )
        self.assertEqual(
            controller_point_values(state, "error"),
            {"panda_joint2": 0.2, "panda_joint1": -0.1},
        )

    def test_arm_output_columns_and_position_commands(self):
        fields = csv_fields()
        for joint in ARM_JOINTS:
            self.assertIn(f"arm_output_{joint}", fields)
        state = SimpleNamespace(
            joint_names=["panda_joint2", "panda_joint1"],
            output=SimpleNamespace(positions=[0.45, -0.83], effort=[]),
        )
        self.assertEqual(controller_point_values(state, "output"),
                         {"panda_joint2": 0.45, "panda_joint1": -0.83})

    def test_missing_output_is_not_reported_as_zero(self):
        state = SimpleNamespace(joint_names=ARM_JOINTS)
        self.assertEqual(controller_point_values(state, "output"), {})

    def test_effort_controller_output_is_mapped_when_positions_are_empty(self):
        state = SimpleNamespace(
            joint_names=["panda_finger_joint1", "panda_finger_joint2"],
            output=SimpleNamespace(positions=[], effort=[-4.5, -4.7]),
        )
        self.assertEqual(
            controller_point_values(state, "output"),
            {"panda_finger_joint1": -4.5, "panda_finger_joint2": -4.7},
        )

    def test_contact_sides_requires_both_finger_names(self):
        contacts = SimpleNamespace(
            contacts=[
                SimpleNamespace(
                    collision1=SimpleNamespace(name="target_cube::collision"),
                    collision2=SimpleNamespace(name="panda::panda_leftfinger::collision"),
                ),
                SimpleNamespace(
                    collision1=SimpleNamespace(name="target_cube::collision"),
                    collision2=SimpleNamespace(name="panda::panda_rightfinger::collision"),
                ),
            ]
        )
        left, right, names = contact_sides(contacts)
        self.assertTrue(left)
        self.assertTrue(right)
        self.assertIn("target_cube::collision", names)

    def test_payload_baseline_resets_across_grasp_recovery(self):
        self.assertEqual(payload_phase_event("gripper_closed", "executing"), "reset")
        self.assertEqual(
            payload_phase_event("gripper_closed", "executed"), "probe_start"
        )
        self.assertEqual(
            payload_phase_event("gripper_recovery_open", "executed"), "reset"
        )
        self.assertEqual(
            payload_phase_event("object_attached", "visualized"),
            "transfer_start",
        )
        self.assertEqual(payload_phase_event("cartesian_lift", "executing"), "keep")

    def test_summary_reports_first_payload_drift_crossing_and_stage_metrics(self):
        rows = []
        for index, drift in enumerate((0.0, 0.008, 0.021)):
            row = {
                "elapsed_s": float(index),
                "stage": "cartesian_to_place",
                "mode": "executing",
                "payload_phase": True,
                "payload_epoch": "transfer",
                "bilateral_contact": index < 2,
                "payload_relative_drift_m": drift,
            }
            for joint in ARM_JOINTS:
                row[f"arm_error_{joint}"] = 0.01 * index
            for joint in FINGER_JOINTS:
                row[f"gripper_error_{joint}"] = 0.001 * index
            rows.append(row)

        summary = summarize_rows(rows, payload_drift_threshold_m=0.015)

        self.assertEqual(summary["samples"], 3)
        self.assertAlmostEqual(summary["max_arm_position_error_rad"], 0.02)
        self.assertAlmostEqual(summary["max_gripper_position_error_m"], 0.002)
        self.assertAlmostEqual(summary["max_payload_relative_drift_m"], 0.021)
        self.assertAlmostEqual(summary["max_transfer_relative_drift_m"], 0.021)
        self.assertAlmostEqual(summary["max_transfer_arm_position_error_rad"], 0.02)
        crossing = summary["first_payload_drift_threshold_crossing"]
        self.assertEqual(crossing["elapsed_s"], 2.0)
        self.assertEqual(crossing["stage"], "cartesian_to_place")
        self.assertEqual(
            summary["first_transfer_drift_threshold_crossing"]["stage"],
            "cartesian_to_place",
        )
        self.assertAlmostEqual(summary["payload_bilateral_contact_ratio"], 2.0 / 3.0)

    def test_first_stage_arm_positions_uses_measured_feedback(self):
        rows = [{"stage": "ompl_to_place"}]
        descent = {"stage": "cartesian_place_descent_stage_1_of_6"}
        for index, joint in enumerate(ARM_JOINTS):
            descent[f"arm_feedback_{joint}"] = 0.1 * index
        rows.append(descent)

        self.assertEqual(
            first_stage_arm_positions(rows),
            [0.0, 0.1, 0.2, 0.30000000000000004, 0.4, 0.5, 0.6000000000000001],
        )

    def test_first_stage_arm_positions_accepts_servo_descent(self):
        row = {"stage": "servo_place_descent_stage_1_of_6"}
        for index, joint in enumerate(ARM_JOINTS):
            row[f"arm_feedback_{joint}"] = -0.1 * index

        self.assertEqual(
            first_stage_arm_positions([row]),
            [0.0, -0.1, -0.2, -0.30000000000000004, -0.4, -0.5, -0.6000000000000001],
        )

    def test_grasp_yaw_requires_a_successful_selection_step(self):
        failed = {
            "active_grasp_yaw_offset_deg": 0.0,
            "steps": [],
        }
        selected = {
            "active_grasp_yaw_offset_deg": 90.0,
            "steps": [
                {
                    "step": "grasp_candidate_prevalidation_selected",
                    "success": True,
                }
            ],
        }

        self.assertIsNone(selected_grasp_yaw_offset(failed))
        self.assertEqual(selected_grasp_yaw_offset(selected), 90.0)

    def test_rms_ignores_missing_and_non_finite_samples(self):
        self.assertAlmostEqual(root_mean_square([3.0, 4.0]), 5.0 / (2.0**0.5))
        self.assertEqual(root_mean_square([None, float("nan")]), None)

    def test_percentile_interpolates_finite_values(self):
        self.assertAlmostEqual(percentile([1.0, 2.0, 3.0, 4.0, 5.0], 0.95), 4.8)
        self.assertIsNone(percentile([None, float("nan")], 0.95))
        with self.assertRaises(ValueError):
            percentile([1.0], 1.01)

    def test_paired_comparison_preserves_failures_and_tracking_ratio(self):
        rows = [
            {
                "mode": "unloaded",
                "final_state": "SUCCESS",
                "max_arm_position_error_rad": 0.02,
                "rms_arm_position_error_rad": 0.01,
                "max_transfer_arm_position_error_rad": 0.01,
                "rms_transfer_arm_position_error_rad": 0.005,
            },
            {
                "mode": "loaded",
                "final_state": "FAILED",
                "failure_reason": "payload drift",
                "max_arm_position_error_rad": 0.04,
                "rms_arm_position_error_rad": 0.03,
                "max_transfer_arm_position_error_rad": 0.03,
                "rms_transfer_arm_position_error_rad": 0.015,
                "max_payload_relative_drift_m": 0.025,
            },
        ]

        summary = summarize_comparison(rows)
        report = render_comparison_markdown(summary)

        self.assertEqual(summary["total_trials"], 2)
        self.assertEqual(summary["modes"]["loaded"]["successes"], 0)
        self.assertEqual(
            summary["modes"]["loaded"]["failure_reasons"], ["payload drift"]
        )
        self.assertAlmostEqual(
            summary["loaded_to_unloaded_rms_tracking_error_ratio"], 3.0
        )
        self.assertEqual(len(summary["modes"]["loaded"]["success_rate_ci"]), 2)
        self.assertAlmostEqual(
            summary["loaded_minus_unloaded_mean_rms_tracking_error_rad"], 0.01
        )
        self.assertIn("3.00x", report)

    def test_paired_comparison_gates_payload_inference_by_start_state(self):
        rows = []
        for pair, delta in ((1, 0.01), (2, 0.08)):
            rows.extend(
                [
                    {
                        "pair": pair,
                        "mode": "unloaded",
                        "final_state": "SUCCESS",
                        "grasp_yaw_offset_deg": 0.0,
                        "pre_descent_arm_positions_rad": [0.0] * 7,
                        "rms_transfer_arm_position_error_rad": 0.01,
                    },
                    {
                        "pair": pair,
                        "mode": "loaded",
                        "final_state": "SUCCESS",
                        "grasp_yaw_offset_deg": 0.0,
                        "pre_descent_arm_positions_rad": [delta] * 7,
                        "rms_transfer_arm_position_error_rad": 0.02,
                    },
                ]
            )

        summary = summarize_comparison(rows, matched_state_tolerance_rad=0.05)

        self.assertEqual(summary["complete_pair_count"], 2)
        self.assertEqual(summary["matched_state_pair_count"], 1)
        self.assertAlmostEqual(summary["matched_pair_rms_tracking_error_ratio"], 2.0)
        self.assertAlmostEqual(summary["mean_pre_descent_joint_delta_rad"], 0.045)
        self.assertAlmostEqual(summary["max_pre_descent_joint_delta_rad"], 0.08)
        self.assertIn("1/2", render_comparison_markdown(summary))

    def test_comparison_warns_when_ratio_uses_near_zero_unloaded_error(self):
        rows = [
            {
                "pair": 1,
                "mode": "unloaded",
                "final_state": "SUCCESS",
                "pre_descent_arm_positions_rad": [0.0] * 7,
                "rms_transfer_arm_position_error_rad": 0.00001,
                "max_transfer_arm_position_error_rad": 0.00002,
            },
            {
                "pair": 1,
                "mode": "loaded",
                "final_state": "SUCCESS",
                "pre_descent_arm_positions_rad": [0.0] * 7,
                "rms_transfer_arm_position_error_rad": 0.001,
                "max_transfer_arm_position_error_rad": 0.002,
                "max_payload_relative_drift_m": 0.003,
            },
        ]

        summary = summarize_comparison(rows)
        report = render_comparison_markdown(summary)

        self.assertTrue(summary["ratio_denominator_near_zero"])
        self.assertTrue(summary["matched_ratio_denominator_near_zero"])
        self.assertIn("Ratio caution", report)

    def test_backend_comparison_reports_matched_physical_trials(self):
        rows = [
            {
                "pair": 1,
                "mode": "loaded",
                "place_descent_backend": "cartesian",
                "final_state": "SUCCESS",
                "grasp_yaw_offset_deg": 0.0,
                "pre_descent_arm_positions_rad": [0.0] * 7,
                "place_descent_execution_time_s": 20.0,
                "rms_transfer_arm_position_error_rad": 0.002,
                "max_payload_relative_drift_m": 0.010,
                "physical_place_error_m": 0.012,
                "physical_final_tilt_deg": 0.5,
            },
            {
                "pair": 1,
                "mode": "loaded",
                "place_descent_backend": "servo",
                "final_state": "SUCCESS",
                "grasp_yaw_offset_deg": 0.0,
                "pre_descent_arm_positions_rad": [0.004] * 7,
                "place_descent_execution_time_s": 13.0,
                "rms_transfer_arm_position_error_rad": 0.001,
                "max_payload_relative_drift_m": 0.008,
                "physical_place_error_m": 0.006,
                "physical_final_tilt_deg": 0.1,
                "servo_cartesian_fallback_used": True,
                "servo_cartesian_fallback_count": 1,
            },
        ]

        summary = summarize_backend_comparison(rows)
        report = render_backend_comparison_markdown(summary)

        self.assertEqual(summary["matched_state_pair_count"], 1)
        self.assertEqual(summary["matched_both_successful_pair_count"], 1)
        self.assertAlmostEqual(
            summary["backends"]["servo"][
                "mean_place_descent_execution_time_s"
            ],
            13.0,
        )
        self.assertEqual(
            summary["backends"]["servo"]["servo_cartesian_fallback_count"],
            1,
        )
        self.assertAlmostEqual(
            summary["mean_servo_minus_cartesian_descent_time_s"], -7.0
        )
        self.assertAlmostEqual(
            summary["mean_cartesian_to_servo_descent_speedup_ratio"],
            20.0 / 13.0,
        )
        self.assertAlmostEqual(
            summary["mean_servo_minus_cartesian_place_error_m"], -0.006
        )
        self.assertIn("1/1", report)
        self.assertIn("0.0060", report)
        self.assertIn("7.00 s faster", report)

    def test_servo_watchdog_safety_treats_expected_failure_as_pass(self):
        rows = [
            {
                "servo_fault_injection_type": "contact_loss",
                "servo_fault_injection_stage": 2,
                "servo_watchdog_safe_stop_passed": True,
                "servo_watchdog_stop_latency_s": 0.08,
                "servo_watchdog_stop_latency_limit_s": 0.15,
                "final_state": "FAILED",
            },
            {
                "servo_fault_injection_type": "payload_drift",
                "servo_fault_injection_stage": 3,
                "servo_watchdog_safe_stop_passed": False,
                "servo_watchdog_stop_latency_s": 0.18,
                "servo_watchdog_stop_latency_limit_s": 0.15,
                "final_state": "FAILED",
                "failure_reason": "latency exceeded",
            },
            {
                "place_descent_backend": "servo",
                "servo_fault_injection_type": None,
                "final_state": "SUCCESS",
                "failure_reason": None,
            },
            {
                "place_descent_backend": "servo",
                "servo_fault_injection_type": "none",
                "final_state": "FAILED",
                "failure_reason": (
                    "servo_place_descent watchdog: bilateral contact lost"
                ),
            },
        ]

        summary = summarize_servo_watchdog_safety(rows)
        report = render_servo_watchdog_safety_markdown(summary)

        self.assertEqual(summary["total_trials"], 2)
        self.assertEqual(summary["safe_stops"], 1)
        self.assertAlmostEqual(
            summary["faults"]["contact_loss"]["mean_stop_latency_s"],
            0.08,
        )
        self.assertEqual(
            summary["faults"]["payload_drift"]["failed_verdict_reasons"],
            ["latency exceeded"],
        )
        self.assertEqual(len(summary["cases"]), 2)
        self.assertEqual(summary["cases"][0]["stage"], 2)
        self.assertEqual(summary["nominal"]["trials"], 2)
        self.assertEqual(summary["nominal"]["successful_completions"], 1)
        self.assertEqual(summary["nominal"]["watchdog_aborts"], 1)
        self.assertIn("Safe stops: 1/2", report)
        self.assertIn("Successful nominal completions: 1/2", report)
        self.assertIn("Nominal watchdog aborts: 1/2", report)
        self.assertIn("Stage Matrix", report)
        self.assertIn("latency exceeded", report)

    def test_physical_disturbance_report_requires_complete_safe_stop_verdict(self):
        rows = [
            {
                "physical_disturbance_enabled": True,
                "physical_disturbance_stage": 3,
                "physical_disturbance_applied": True,
                "physical_disturbance_cleared": True,
                "physical_disturbance_safe_stop_passed": True,
                "physical_disturbance_stop_latency_s": 0.082,
                "physical_disturbance_stop_latency_limit_s": 0.5,
                "physical_disturbance_force_x_n": 0.0,
                "physical_disturbance_force_y_n": 16.0,
                "physical_disturbance_force_z_n": 0.0,
                "physical_disturbance_duration_s": 0.1,
                "physical_disturbance_delay_s": 0.5,
                "failure_reason": "expected watchdog stop",
            },
            {
                "physical_disturbance_enabled": True,
                "physical_disturbance_stage": 4,
                "physical_disturbance_applied": True,
                "physical_disturbance_cleared": False,
                "physical_disturbance_safe_stop_passed": False,
                "physical_disturbance_stop_latency_s": 0.7,
                "physical_disturbance_stop_latency_limit_s": 0.5,
                "physical_disturbance_force_x_n": 0.0,
                "physical_disturbance_force_y_n": -16.0,
                "physical_disturbance_force_z_n": 0.0,
                "physical_disturbance_duration_s": 0.1,
                "physical_disturbance_delay_s": 0.5,
                "failure_reason": "disturbance was not cleared",
            },
        ]

        summary = summarize_physical_disturbance_safety(rows)
        report = render_physical_disturbance_safety_markdown(summary)

        self.assertEqual(summary["total_trials"], 2)
        self.assertEqual(summary["safe_stops"], 1)
        self.assertEqual(summary["disturbances_applied"], 2)
        self.assertEqual(summary["disturbances_cleared"], 1)
        self.assertEqual(len(summary["configurations"]), 2)
        self.assertFalse(summary["all_meet_strictest_stop_latency_limit"])
        self.assertAlmostEqual(summary["max_stop_latency_s"], 0.7)
        self.assertIn("Safe stops: 1/2", report)
        self.assertIn("[0.0, -16.0, 0.0] N", report)
        self.assertIn(
            "All observed stops meet the strictest 0.500 s limit: no",
            report,
        )
        self.assertIn("disturbance was not cleared", report)


if __name__ == "__main__":
    unittest.main()
