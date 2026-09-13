import json
import os
import sys
import unittest
from math import pi
from types import SimpleNamespace

PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PACKAGE_ROOT)

from panda_manipulation.benchmark_metrics import PlannerRun, aggregate_by_planner
from panda_manipulation.geometry import (
    PoseSpec,
    Vector3,
    compose_pose,
    pose_distance,
    pose_from_dict,
    quaternion_from_euler,
    relative_pose,
)
from panda_manipulation.grasp import generate_grasp_sequences, select_best_reachable_candidate
from panda_manipulation.gazebo_arm_tracking_test import ordered_positions
from panda_manipulation.pick_plan_pipeline import (
    anchored_place_descent_poses,
    bilateral_contact_is_recent,
    calibrated_object_in_hand_pose,
    clamp_near_joint_limits,
    contact_limited_gripper_outcome,
    equivalent_grasp_candidate,
    finger_asymmetry_correction,
    grasp_candidate_prevalidation_score,
    grasp_candidate_prevalidation_feasible,
    grasp_candidate_report,
    gripper_close_stalled,
    joint_path_length,
    joint_trajectory_quality_metrics,
    interpolated_place_descent_poses,
    measured_arm_endpoint_outcome,
    measured_gripper_endpoint_outcome,
    place_descent_stage_number,
    place_descent_trajectory_quality_error,
    place_descent_stage_path_is_safe,
    place_descent_stage_motion_diagnostic,
    normalized_joint_limit_margin,
    normalize_planner_ids,
    max_joint_position_delta,
    ordered_joint_positions,
    parse_joint_positions_csv,
    physical_grasp_verified,
    payload_compensated_place_pose,
    quaternion_angular_distance,
    release_object_motion,
    rigid_payload_translation_error,
    staged_place_feedback_poses,
    summarize_cpp_trajectory_metrics,
    target_position_in_measurement_frame,
    matching_contact_collision_names,
    translation_distance,
    translated_pose_dict,
)
from panda_manipulation.scene_manager import configured_scene
from panda_manipulation.task_state_machine import (
    ManipulationTaskStateMachine,
    StepResult,
    TaskState,
)


class GeometryTests(unittest.TestCase):
    def test_cpp_trajectory_metric_summary_preserves_quality_extrema(self):
        summary = summarize_cpp_trajectory_metrics(
            [
                {
                    "schema_version": 1,
                    "aggregate": {
                        "joint_path_length_rad": 1.25,
                        "max_joint_step_rad": 0.30,
                        "integrated_squared_acceleration": 4.0,
                    },
                    "metrics": [
                        {"min_normalized_joint_limit_margin": 0.20},
                        {"min_normalized_joint_limit_margin": 0.15},
                    ],
                },
                {
                    "schema_version": 1,
                    "aggregate": {
                        "joint_path_length_rad": 0.75,
                        "max_joint_step_rad": 0.45,
                        "integrated_squared_acceleration": 2.5,
                    },
                    "metrics": [
                        {"min_normalized_joint_limit_margin": 0.10},
                    ],
                },
                {
                    "schema_version": 2,
                    "aggregate": {"joint_path_length_rad": 999.0},
                    "metrics": [],
                },
                {
                    "schema_version": 1,
                    "aggregate": {"joint_path_length_rad": "invalid"},
                    "metrics": [],
                },
            ]
        )
        self.assertEqual(summary["message_count"], 3)
        self.assertEqual(summary["trajectory_segment_count"], 3)
        self.assertAlmostEqual(summary["joint_path_length_rad"], 2.0)
        self.assertAlmostEqual(summary["max_joint_step_rad"], 0.45)
        self.assertAlmostEqual(
            summary["integrated_squared_acceleration"], 6.5
        )
        self.assertAlmostEqual(
            summary["min_normalized_joint_limit_margin"], 0.10
        )

    def test_joint_replay_csv_requires_one_complete_finite_arm_state(self):
        self.assertEqual(
            parse_joint_positions_csv("0, 1, 2, 3, 4, 5, 6"),
            [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
        )
        self.assertEqual(parse_joint_positions_csv(""), [])
        with self.assertRaises(ValueError):
            parse_joint_positions_csv("0,1")
        with self.assertRaises(ValueError):
            parse_joint_positions_csv("0,1,2,3,4,5,nan")

    def test_joint_replay_endpoint_is_ordered_and_compared_by_name(self):
        ordered = ordered_joint_positions(
            ["panda_joint2", "panda_joint1", *[f"panda_joint{i}" for i in range(3, 8)]],
            [0.2, 0.1, 0.3, 0.4, 0.5, 0.6, 0.7],
        )
        self.assertEqual(ordered, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
        self.assertAlmostEqual(
            max_joint_position_delta(ordered, [0.1, 0.21, 0.3, 0.4, 0.5, 0.6, 0.7]),
            0.01,
        )

    def test_joint_trajectory_quality_reports_continuity_and_terminal_state(self):
        def point(time_s, positions, velocities, accelerations):
            sec = int(time_s)
            return SimpleNamespace(
                positions=positions,
                velocities=velocities,
                accelerations=accelerations,
                time_from_start=SimpleNamespace(
                    sec=sec,
                    nanosec=int(round((time_s - sec) * 1_000_000_000)),
                ),
            )

        metrics = joint_trajectory_quality_metrics(
            ["panda_joint1", "panda_joint2"],
            [
                point(0.1, [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]),
                point(0.3, [0.02, -0.04], [0.1, -0.2], [0.3, -0.4]),
                point(0.8, [0.03, -0.05], [0.0, 0.01], [0.0, -0.02]),
            ],
        )

        self.assertEqual(metrics["point_count"], 3)
        self.assertAlmostEqual(metrics["duration_s"], 0.8)
        self.assertAlmostEqual(metrics["min_segment_duration_s"], 0.2)
        self.assertAlmostEqual(metrics["max_joint_step_rad"], 0.04)
        self.assertEqual(metrics["max_joint_step_joint"], "panda_joint2")
        self.assertAlmostEqual(metrics["max_implied_velocity_rad_s"], 0.2)
        self.assertAlmostEqual(metrics["max_reported_velocity_rad_s"], 0.2)
        self.assertAlmostEqual(
            metrics["joint_path_length_rad"],
            (0.02 ** 2 + 0.04 ** 2) ** 0.5
            + (0.01 ** 2 + 0.01 ** 2) ** 0.5,
        )
        self.assertAlmostEqual(
            metrics["endpoint_joint_displacement_rad"],
            (0.03 ** 2 + 0.05 ** 2) ** 0.5,
        )
        self.assertGreaterEqual(metrics["joint_path_tortuosity"], 1.0)
        self.assertAlmostEqual(metrics["terminal_max_abs_velocity_rad_s"], 0.01)
        self.assertAlmostEqual(metrics["terminal_max_abs_acceleration_rad_s2"], 0.02)

    def test_place_descent_quality_gate_rejects_long_or_winding_paths(self):
        healthy = {"duration_s": 4.0, "joint_path_length_rad": 0.2}
        self.assertIsNone(
            place_descent_trajectory_quality_error(
                "cartesian_place_descent_stage_2_of_6",
                healthy,
                15.0,
                0.5,
            )
        )
        self.assertIn(
            "duration=88.200s",
            place_descent_trajectory_quality_error(
                "cartesian_place_descent_stage_5_of_6",
                {"duration_s": 88.2, "joint_path_length_rad": 0.3},
                15.0,
                0.5,
            ),
        )
        self.assertIn(
            "joint_path_length=0.800rad",
            place_descent_trajectory_quality_error(
                "cartesian_place_descent",
                {"duration_s": 5.0, "joint_path_length_rad": 0.8},
                15.0,
                0.5,
            ),
        )
        self.assertIsNone(
            place_descent_trajectory_quality_error(
                "cartesian_retreat",
                {"duration_s": 30.0, "joint_path_length_rad": 2.0},
                15.0,
                0.5,
            )
        )

    def test_arm_tracking_positions_are_reordered_and_require_all_joints(self):
        names = ["panda_joint2", "panda_joint1"] + [
            f"panda_joint{i}" for i in range(3, 8)
        ]
        self.assertEqual(
            ordered_positions(names, [0.2, 0.1, 0.3, 0.4, 0.5, 0.6, 0.7]),
            [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
        )
        self.assertIsNone(ordered_positions(names[:-1], [0.0] * 6))

    def test_rejected_candidate_report_excludes_ros_objects_without_mutation(self):
        trajectory = object()
        candidate = {"internal_pose": object()}
        result = {
            "feasible": False,
            "reason": "joint limit margin rejected",
            "joint_limit_margin": 0.0,
            "round": 2,
            "candidate": candidate,
            "pre_grasp_trajectory": trajectory,
        }
        report = grasp_candidate_report(result)
        decoded = json.loads(json.dumps(report))
        self.assertFalse(decoded["feasible"])
        self.assertEqual(decoded["reason"], result["reason"])
        self.assertEqual(decoded["round"], 2)
        self.assertNotIn("candidate", decoded)
        self.assertNotIn("pre_grasp_trajectory", decoded)
        self.assertIs(result["pre_grasp_trajectory"], trajectory)
        self.assertIs(result["candidate"], candidate)

    def test_candidate_report_keeps_early_planning_failure_diagnostics(self):
        result = {"feasible": False, "reason": "OMPL failed", "yaw_offset_deg": 90.0}
        self.assertEqual(grasp_candidate_report(result), result)
        self.assertEqual(grasp_candidate_report({}), {})

    def test_quaternion_distance_handles_sign_and_normalization(self):
        self.assertAlmostEqual(
            quaternion_angular_distance((0.0, 0.0, 0.0, 2.0), (0.0, 0.0, 0.0, -1.0)),
            0.0,
        )
        self.assertAlmostEqual(
            quaternion_angular_distance((0.0, 0.0, 0.0, 1.0), (1.0, 0.0, 0.0, 0.0)),
            pi,
        )

    def test_quaternion_distance_rejects_invalid_measurements(self):
        for invalid in [(0.0, 0.0, 0.0, 0.0), (float("nan"), 0.0, 0.0, 1.0),
                        (float("inf"), 0.0, 0.0, 1.0), (0.0, 1.0)]:
            with self.subTest(invalid=invalid):
                self.assertEqual(
                    quaternion_angular_distance(invalid, (0.0, 0.0, 0.0, 1.0)),
                    float("inf"),
                )

    def test_tactile_contact_extracts_matching_finger_collision(self):
        message = SimpleNamespace(
            contacts=[
                SimpleNamespace(
                    collision1=SimpleNamespace(
                        name="panda::panda_leftfinger::panda_leftfinger_collision"
                    ),
                    collision2=SimpleNamespace(
                        name="target_cube::link::target_collision"
                    ),
                ),
                SimpleNamespace(
                    collision1=SimpleNamespace(name="work_table::link::collision"),
                    collision2=SimpleNamespace(name="panda::finger::collision"),
                ),
            ]
        )
        self.assertEqual(
            matching_contact_collision_names(message, "panda_leftfinger"),
            ["panda::panda_leftfinger::panda_leftfinger_collision"],
        )

    def test_tactile_grasp_requires_recent_bilateral_contact(self):
        now_ns = 2_000_000_000
        self.assertTrue(
            bilateral_contact_is_recent(
                {"left": 1_800_000_000, "right": 1_900_000_000},
                now_ns,
                0.75,
            )
        )
        self.assertFalse(
            bilateral_contact_is_recent(
                {"left": 1_000_000_000, "right": 1_900_000_000},
                now_ns,
                0.75,
            )
        )
        self.assertFalse(
            bilateral_contact_is_recent(
                {"left": 1_900_000_000, "right": None},
                now_ns,
                0.75,
            )
        )

    def test_release_object_motion_separates_horizontal_and_vertical(self):
        horizontal, vertical, total = release_object_motion(
            (0.45, -0.16, 0.768),
            (0.48, -0.12, 0.760),
        )
        self.assertAlmostEqual(horizontal, 0.05)
        self.assertAlmostEqual(vertical, 0.008)
        self.assertAlmostEqual(total, (0.05**2 + 0.008**2) ** 0.5)

    def test_joint_limit_clamp_only_corrects_small_simulator_drift(self):
        corrected = clamp_near_joint_limits(
            ["panda_joint2", "panda_joint4", "unknown_joint"],
            [-1.7628004, -3.08, 4.2],
        )
        self.assertGreater(corrected[0], -1.7628)
        self.assertEqual(corrected[1], -3.08)
        self.assertEqual(corrected[2], 4.2)

    def test_quaternion_is_normalized(self):
        q = quaternion_from_euler(pi, 0.0, 0.0)
        norm = (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) ** 0.5
        self.assertAlmostEqual(norm, 1.0)

    def test_pose_distance(self):
        a = PoseSpec("base", Vector3(0.0, 0.0, 0.0), quaternion_from_euler(0.0, 0.0, 0.0))
        b = PoseSpec("base", Vector3(0.3, 0.4, 0.0), quaternion_from_euler(0.0, 0.0, 0.0))
        self.assertAlmostEqual(pose_distance(a, b), 0.5)

    def test_equivalent_grasp_yaw_preserves_object_transform(self):
        target = PoseSpec(
            "panda_link0",
            Vector3(0.52, 0.0, 0.08),
            quaternion_from_euler(pi, 0.0, 0.0),
        )
        place = PoseSpec(
            "panda_link0",
            Vector3(0.45, -0.16, 0.08),
            quaternion_from_euler(pi, 0.0, -0.785),
        )
        original = generate_grasp_sequences(target, place)[0].as_dict()
        original_grasp_orientation = dict(
            original["grasp_pose"]["orientation"]
        )
        rotated = equivalent_grasp_candidate(original, 90.0)

        self.assertEqual(rotated["object_pose"], original["object_pose"])
        self.assertEqual(rotated["place_pose"], original["place_pose"])
        self.assertNotEqual(
            rotated["grasp_pose"]["orientation"],
            original["grasp_pose"]["orientation"],
        )
        self.assertEqual(
            original["grasp_pose"]["orientation"],
            original_grasp_orientation,
        )
        reconstructed = compose_pose(
            pose_from_dict(rotated["grasp_pose"]),
            pose_from_dict(rotated["object_in_hand_pose"]),
            "panda_link0",
        )
        expected = pose_from_dict(original["object_pose"])
        self.assertAlmostEqual(pose_distance(reconstructed, expected), 0.0)
        dot = sum(
            first * second
            for first, second in zip(
                (
                    reconstructed.orientation.x,
                    reconstructed.orientation.y,
                    reconstructed.orientation.z,
                    reconstructed.orientation.w,
                ),
                (
                    expected.orientation.x,
                    expected.orientation.y,
                    expected.orientation.z,
                    expected.orientation.w,
                ),
            )
        )
        self.assertAlmostEqual(abs(dot), 1.0)

    def test_relative_pose_accounts_for_gripper_rotation(self):
        hand = PoseSpec("base", Vector3(0.52, 0.0, 0.22), quaternion_from_euler(pi, 0.0, 0.0))
        target = PoseSpec("base", Vector3(0.52, 0.0, 0.08), quaternion_from_euler(pi, 0.0, 0.0))
        local = relative_pose(hand, target, "panda_hand")
        self.assertAlmostEqual(local.position.z, 0.14)
        self.assertAlmostEqual(local.orientation.w, 1.0)

    def test_compose_pose_applies_reference_rotation_and_translation(self):
        reference = PoseSpec(
            "base",
            Vector3(1.0, 2.0, 3.0),
            quaternion_from_euler(0.0, 0.0, pi / 2.0),
        )
        local = PoseSpec(
            "camera",
            Vector3(1.0, 0.0, 0.0),
            quaternion_from_euler(0.0, 0.0, 0.0),
        )
        result = compose_pose(reference, local, "base")
        self.assertAlmostEqual(result.position.x, 1.0)
        self.assertAlmostEqual(result.position.y, 3.0)
        self.assertAlmostEqual(result.position.z, 3.0)

    def test_payload_error_uses_full_hand_rotation(self):
        hand = PoseSpec(
            "base",
            Vector3(0.4, -0.2, 0.8),
            quaternion_from_euler(0.0, 0.0, pi / 2.0),
        )
        local_object = PoseSpec(
            "panda_hand",
            Vector3(0.1, 0.0, -0.05),
            quaternion_from_euler(0.0, 0.0, 0.0),
        )
        expected = compose_pose(hand, local_object, "base")
        error = rigid_payload_translation_error(
            (
                expected.position.x,
                expected.position.y,
                expected.position.z,
            ),
            hand.as_dict(),
            local_object.as_dict(),
        )
        self.assertAlmostEqual(error, 0.0)

    def test_payload_place_compensation_targets_object_xyz(self):
        desired_object = PoseSpec(
            "base",
            Vector3(0.45, -0.16, 0.153),
            quaternion_from_euler(0.0, 0.0, pi / 2.0),
        )
        local_object = PoseSpec(
            "panda_hand",
            Vector3(0.06, -0.02, -0.10),
            quaternion_from_euler(0.0, 0.0, 0.0),
        )
        compensated = payload_compensated_place_pose(
            desired_object.as_dict(),
            local_object.as_dict(),
        )
        hand = pose_from_dict(compensated)
        placed_object = compose_pose(hand, local_object, "base")

        self.assertAlmostEqual(placed_object.position.x, 0.45)
        self.assertAlmostEqual(placed_object.position.y, -0.16)
        self.assertAlmostEqual(placed_object.position.z, 0.153)

    def test_place_feedback_recovery_lifts_before_horizontal_correction(self):
        hand = PoseSpec(
            "panda_link0",
            Vector3(0.49, -0.10, 0.15),
            quaternion_from_euler(pi, 0.0, -pi / 4.0),
        )
        correction = (-0.04, -0.06, -0.01)
        raised, translated, corrected = staged_place_feedback_poses(
            hand.as_dict(),
            correction,
            0.05,
        )

        self.assertAlmostEqual(raised["position"]["x"], 0.49)
        self.assertAlmostEqual(raised["position"]["z"], 0.20)
        self.assertAlmostEqual(translated["position"]["x"], 0.45)
        self.assertAlmostEqual(translated["position"]["y"], -0.16)
        self.assertAlmostEqual(translated["position"]["z"], 0.19)
        self.assertAlmostEqual(corrected["position"]["z"], 0.14)

    def test_place_descent_is_split_into_equal_stages(self):
        start = PoseSpec(
            "panda_link0",
            Vector3(0.45, -0.16, 0.27),
            quaternion_from_euler(pi, 0.0, -pi / 4.0),
        )
        goal = PoseSpec(
            "panda_link0",
            Vector3(0.45, -0.16, 0.15),
            quaternion_from_euler(pi, 0.0, -pi / 4.0),
        )
        stages = interpolated_place_descent_poses(
            start.as_dict(),
            goal.as_dict(),
            3,
        )

        self.assertEqual(len(stages), 3)
        self.assertAlmostEqual(stages[0]["position"]["z"], 0.23)
        self.assertAlmostEqual(stages[1]["position"]["z"], 0.19)
        self.assertAlmostEqual(stages[2]["position"]["z"], 0.15)
        self.assertEqual(stages[2]["orientation"], goal.as_dict()["orientation"])

    def test_place_descent_diagnostic_separates_hand_motion_from_payload_slip(self):
        diagnostic = place_descent_stage_motion_diagnostic(
            (0.45, -0.16, 0.27),
            (0.47, -0.15, 0.23),
            (0.45, -0.16, 0.30),
            (0.46, -0.16, 0.26),
        )

        self.assertAlmostEqual(diagnostic["horizontal_motion_m"], 0.02236068)
        self.assertAlmostEqual(diagnostic["hand_horizontal_motion_m"], 0.01)
        self.assertAlmostEqual(
            diagnostic["payload_relative_horizontal_m"],
            0.01414214,
        )
        self.assertAlmostEqual(diagnostic["payload_relative_dz_m"], 0.0)

    def test_place_descent_path_gate_uses_hand_motion_not_rotation_affected_payload_motion(self):
        diagnostic = {
            "horizontal_motion_m": 0.0136,
            "hand_horizontal_motion_m": 0.00001,
            "payload_relative_horizontal_m": 0.0136,
        }

        self.assertTrue(place_descent_stage_path_is_safe(diagnostic, 0.01))
        diagnostic["hand_horizontal_motion_m"] = 0.011
        self.assertFalse(place_descent_stage_path_is_safe(diagnostic, 0.01))

    def test_place_descent_stage_number_parses_only_staged_descent_steps(self):
        self.assertEqual(
            place_descent_stage_number("cartesian_place_descent_stage_2_of_3"),
            2,
        )
        self.assertIsNone(place_descent_stage_number("cartesian_place_descent"))
        self.assertIsNone(
            place_descent_stage_number("cartesian_place_descent_stage_invalid")
        )

    def test_six_stage_place_descent_keeps_short_measured_state_segments(self):
        start = PoseSpec(
            "panda_link0",
            Vector3(0.50, -0.30, 0.32),
            quaternion_from_euler(0.0, 0.0, 0.0),
        )
        goal = PoseSpec(
            "panda_link0",
            Vector3(0.50, -0.30, 0.20),
            quaternion_from_euler(0.0, 0.0, 0.0),
        )

        stages = interpolated_place_descent_poses(
            start.as_dict(),
            goal.as_dict(),
            6,
        )

        self.assertEqual(len(stages), 6)
        self.assertAlmostEqual(stages[0]["position"]["z"], 0.30)
        self.assertAlmostEqual(stages[-1]["position"]["z"], 0.20)

    def test_place_descent_line_is_anchored_to_measured_pre_place_position(self):
        start = PoseSpec(
            "panda_link0",
            Vector3(0.45, -0.16, 0.90),
            quaternion_from_euler(pi, 0.0, 0.0),
        )
        goal = PoseSpec(
            "panda_link0",
            Vector3(0.45, -0.16, 0.78),
            quaternion_from_euler(pi, 0.0, 0.0),
        )

        stages, offset = anchored_place_descent_poses(
            start.as_dict(),
            goal.as_dict(),
            (0.456, -0.164, 0.898),
            6,
        )

        self.assertEqual(len(stages), 6)
        for actual, expected in zip(offset, (0.006, -0.004, -0.002)):
            self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(stages[0]["position"]["x"], 0.456)
        self.assertAlmostEqual(stages[0]["position"]["y"], -0.164)
        self.assertAlmostEqual(stages[-1]["position"]["x"], 0.456)
        self.assertAlmostEqual(stages[-1]["position"]["y"], -0.164)
        self.assertAlmostEqual(stages[-1]["position"]["z"], 0.778)

    def test_payload_grasp_calibration_uses_world_to_robot_translation(self):
        hand = PoseSpec(
            "panda_link0",
            Vector3(0.52, 0.0, 0.20),
            quaternion_from_euler(0.0, 0.0, 0.0),
        )
        nominal_object = PoseSpec(
            "panda_link0",
            Vector3(0.52, 0.0, 0.04),
            quaternion_from_euler(0.0, 0.0, 0.0),
        )
        calibrated = calibrated_object_in_hand_pose(
            hand.as_dict(),
            nominal_object.as_dict(),
            (0.52, 0.0, 0.80),
            (0.52, 0.0, 0.76),
        )
        reconstructed = compose_pose(
            hand,
            pose_from_dict(calibrated),
            "panda_link0",
        )

        self.assertAlmostEqual(reconstructed.position.x, 0.52)
        self.assertAlmostEqual(reconstructed.position.y, 0.0)
        self.assertAlmostEqual(reconstructed.position.z, 0.08)

    def test_target_world_position_maps_to_candidate_frame(self):
        nominal_object = PoseSpec(
            "panda_link0",
            Vector3(0.52, 0.0, 0.04),
            quaternion_from_euler(0.0, 0.0, 0.0),
        )
        measured = target_position_in_measurement_frame(
            (0.45, -0.16, 0.768),
            (0.52, 0.0, 0.76),
            nominal_object.as_dict(),
        )
        self.assertAlmostEqual(measured[0], 0.45)
        self.assertAlmostEqual(measured[1], -0.16)
        self.assertAlmostEqual(measured[2], 0.048)

    def test_payload_error_accounts_for_gazebo_world_offset(self):
        hand = PoseSpec(
            "panda_link0",
            Vector3(0.45, -0.16, 0.28),
            quaternion_from_euler(pi, 0.0, -0.785),
        )
        local_object = PoseSpec(
            "panda_hand",
            Vector3(0.0, 0.0, 0.12),
            quaternion_from_euler(0.0, 0.0, 0.0),
        )
        expected_base = compose_pose(hand, local_object, "panda_link0")
        world_offset = (0.0, 0.0, 0.72)
        error = rigid_payload_translation_error(
            (
                expected_base.position.x,
                expected_base.position.y,
                expected_base.position.z + world_offset[2],
            ),
            hand.as_dict(),
            local_object.as_dict(),
            world_offset,
        )
        self.assertAlmostEqual(error, 0.0)


class GraspTests(unittest.TestCase):
    def test_visual_gripper_closure_preserves_target_width(self):
        target_width = 0.05
        closed_finger_position = 0.026
        self.assertGreaterEqual(2.0 * closed_finger_position, target_width)

    def test_generates_reachable_candidates(self):
        target = PoseSpec("panda_link0", Vector3(0.52, 0.0, 0.08), quaternion_from_euler(pi, 0.0, 0.0))
        place = PoseSpec("panda_link0", Vector3(0.35, -0.35, 0.16), quaternion_from_euler(pi, 0.0, -0.785))
        candidates = generate_grasp_sequences(target, place)
        self.assertGreaterEqual(len(candidates), 4)
        selected = select_best_reachable_candidate(candidates)
        self.assertIsNotNone(selected)
        self.assertGreater(selected.score, 0.0)
        self.assertGreater(selected.grasp_pose.position.z, target.position.z)
        self.assertEqual(selected.object_pose.position.z, target.position.z)
        self.assertIn("object_pose", selected.as_dict())
        self.assertIn("object_in_hand_pose", selected.as_dict())

    def test_contact_limited_close_can_continue_to_physical_validation(self):
        outcome = contact_limited_gripper_outcome(
            "gripper_closed",
            "grasp_pose",
            [0.025, 0.025],
        )
        self.assertIn("contact_limited_grasp", outcome)

    def test_contact_limited_close_rejects_unsynchronized_mimic_fingers(self):
        outcome = contact_limited_gripper_outcome(
            "gripper_closed",
            "grasp_pose",
            [0.04, 0.0],
        )
        self.assertIsNone(outcome)

    def test_contact_limited_release_requires_partial_opening_at_place(self):
        accepted = contact_limited_gripper_outcome(
            "gripper_open",
            "place_pose",
            [0.020, 0.020],
        )
        rejected = contact_limited_gripper_outcome(
            "gripper_open",
            None,
            [0.020, 0.020],
        )
        self.assertIn("contact_limited_release", accepted)
        self.assertIsNone(rejected)

    def test_recovery_open_accepts_synchronized_partial_opening(self):
        outcome = contact_limited_gripper_outcome(
            "gripper_recovery_open",
            None,
            [0.020, 0.020],
        )
        self.assertIn("contact_limited_release", outcome)

    def test_controller_timeout_accepts_measured_gripper_endpoint(self):
        outcome = measured_gripper_endpoint_outcome(
            0.028,
            [0.0280, 0.0281],
            0.001,
        )
        self.assertIn("measured_endpoint_after_controller_timeout", outcome)
        self.assertIn("max_error=0.0001m", outcome)

    def test_controller_timeout_rejects_unreached_gripper_endpoint(self):
        self.assertIsNone(
            measured_gripper_endpoint_outcome(
                0.040,
                [0.0245, 0.0245],
                0.001,
            )
        )

    def test_controller_timeout_rejects_one_finger_outside_tolerance(self):
        self.assertIsNone(
            measured_gripper_endpoint_outcome(
                0.028,
                [0.0280, 0.0300],
                0.001,
            )
        )

    def test_control_failed_accepts_safe_measured_arm_endpoint(self):
        outcome = measured_arm_endpoint_outcome(
            ["panda_joint1", "panda_joint2", "panda_joint3"],
            [0.2, -0.8, 1.85],
            [0.2, -0.8, 1.834],
            0.020,
            0.01,
        )
        self.assertIn("measured_endpoint_after_control_failed", outcome)
        self.assertIn("max_error=0.0160rad", outcome)

    def test_control_failed_rejects_arm_endpoint_at_joint_limit(self):
        self.assertIsNone(
            measured_arm_endpoint_outcome(
                ["panda_joint1", "panda_joint2"],
                [0.2, -1.7628],
                [0.2, -1.7628],
                0.020,
                0.01,
            )
        )

    def test_control_failed_rejects_large_arm_endpoint_error(self):
        self.assertIsNone(
            measured_arm_endpoint_outcome(
                ["panda_joint1", "panda_joint2"],
                [0.2, -0.8],
                [0.2, -0.821],
                0.020,
                0.01,
            )
        )

    def test_close_timeout_without_finger_motion_uses_local_retry(self):
        self.assertTrue(
            gripper_close_stalled(
                "gripper_closed",
                0.022,
                [0.040, 0.040],
                [0.040, 0.040],
                0.002,
            )
        )

    def test_close_timeout_after_contact_does_not_use_local_retry(self):
        self.assertFalse(
            gripper_close_stalled(
                "gripper_closed",
                0.022,
                [0.040, 0.040],
                [0.028, 0.028],
                0.002,
            )
        )

    def test_finger_asymmetry_correction_uses_grasp_orientation(self):
        correction = finger_asymmetry_correction(
            [0.028, 0.023],
            {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0},
            correction_gain=1.0,
            minimum_asymmetry_m=0.001,
            maximum_correction_m=0.004,
        )
        self.assertAlmostEqual(correction[0], 0.0)
        self.assertAlmostEqual(correction[1], -0.0025)
        self.assertAlmostEqual(correction[2], 0.0)

    def test_finger_asymmetry_correction_ignores_symmetric_contact(self):
        self.assertEqual(
            finger_asymmetry_correction(
                [0.0252, 0.0248],
                {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0},
                correction_gain=1.0,
                minimum_asymmetry_m=0.001,
                maximum_correction_m=0.004,
            ),
            (0.0, 0.0, 0.0),
        )


    def test_custom_pre_grasp_height_supports_horizontal_approach(self):
        target = PoseSpec(
            "panda_link0",
            Vector3(0.52, 0.0, 0.04),
            quaternion_from_euler(pi, 0.0, 0.0),
        )
        place = PoseSpec(
            "panda_link0",
            Vector3(0.45, -0.16, 0.18),
            quaternion_from_euler(pi, 0.0, -0.785),
        )
        candidate = generate_grasp_sequences(
            target,
            place,
            pre_grasp_height_offset=0.0,
        )[0]
        self.assertAlmostEqual(
            candidate.pre_grasp_pose.position.z,
            candidate.grasp_pose.position.z,
        )

    def test_top_down_pre_grasp_changes_only_height(self):
        target = PoseSpec(
            "panda_link0",
            Vector3(0.52, 0.0, 0.04),
            quaternion_from_euler(pi, 0.0, 0.0),
        )
        place = PoseSpec(
            "panda_link0",
            Vector3(0.45, -0.16, 0.14),
            quaternion_from_euler(pi, 0.0, -0.785),
        )
        candidate = generate_grasp_sequences(
            target,
            place,
            approach_distance=0.0,
            pre_grasp_height_offset=0.08,
        )[0]
        self.assertAlmostEqual(
            candidate.pre_grasp_pose.position.x,
            candidate.grasp_pose.position.x,
        )
        self.assertAlmostEqual(
            candidate.pre_grasp_pose.position.y,
            candidate.grasp_pose.position.y,
        )
        self.assertAlmostEqual(
            candidate.pre_grasp_pose.position.z
            - candidate.grasp_pose.position.z,
            0.08,
        )

    def test_top_down_retreat_preserves_place_xy(self):
        target = PoseSpec(
            "panda_link0",
            Vector3(0.52, 0.0, 0.04),
            quaternion_from_euler(pi, 0.0, 0.0),
        )
        place = PoseSpec(
            "panda_link0",
            Vector3(0.45, -0.16, 0.14),
            quaternion_from_euler(pi, 0.0, -0.785),
        )
        candidate = generate_grasp_sequences(
            target,
            place,
            retreat_height_offset=0.08,
        )[0]
        self.assertAlmostEqual(candidate.retreat_pose.position.x, place.position.x)
        self.assertAlmostEqual(candidate.retreat_pose.position.y, place.position.y)
        self.assertAlmostEqual(candidate.retreat_pose.position.z, place.position.z + 0.08)

    def test_physical_grasp_requires_measured_probe_lift(self):
        self.assertTrue(physical_grasp_verified(0.76, 0.781, 0.015))
        self.assertFalse(physical_grasp_verified(0.76, 0.768, 0.015))
        self.assertFalse(physical_grasp_verified(None, 0.781, 0.015))

    def test_recovery_pose_tracks_displaced_object(self):
        pose = {
            "frame_id": "panda_link0",
            "position": {"x": 0.52, "y": 0.0, "z": 0.18},
            "orientation": {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0},
        }
        translated = translated_pose_dict(pose, (0.02, -0.015, -0.01))
        self.assertAlmostEqual(translated["position"]["x"], 0.54)
        self.assertAlmostEqual(translated["position"]["y"], -0.015)
        self.assertAlmostEqual(translated["position"]["z"], 0.17)
        self.assertEqual(pose["position"]["x"], 0.52)

    def test_recovery_translation_distance_uses_xyz(self):
        self.assertAlmostEqual(translation_distance((0.03, 0.04, 0.0)), 0.05)

    def test_csv_planner_ids_override_ros_array_without_concatenation(self):
        planners = normalize_planner_ids(
            ["array-default"],
            "RRTConnectkConfigDefault,PRMkConfigDefault,RRTstarkConfigDefault",
            "fallback",
        )
        self.assertEqual(
            planners,
            [
                "RRTConnectkConfigDefault",
                "PRMkConfigDefault",
                "RRTstarkConfigDefault",
            ],
        )

    def test_single_csv_planner_is_not_duplicated(self):
        planners = normalize_planner_ids(
            [],
            "PRMkConfigDefault,PRMkConfigDefault",
            "fallback",
        )
        self.assertEqual(planners, ["PRMkConfigDefault"])

    def test_joint_path_length_sums_multijoint_segments(self):
        self.assertAlmostEqual(
            joint_path_length([[0.0, 0.0], [3.0, 4.0], [3.0, 6.0]]),
            7.0,
        )

    def test_joint_limit_margin_uses_nearest_arm_limit(self):
        margin = normalized_joint_limit_margin(
            ["panda_joint1", "panda_joint2", "panda_finger_joint1"],
            [0.0, 1.70, 0.04],
        )
        self.assertGreater(margin, 0.0)
        self.assertLess(margin, 0.1)

    def test_prevalidation_score_rewards_clearance_and_shorter_paths(self):
        safer = grasp_candidate_prevalidation_score(1.0, 0.6, 1.5, 0.5, 90.0)
        near_limit = grasp_candidate_prevalidation_score(1.0, 0.1, 1.5, 0.5, 90.0)
        longer = grasp_candidate_prevalidation_score(1.0, 0.6, 4.0, 0.5, 90.0)
        self.assertGreater(safer, near_limit)
        self.assertGreater(safer, longer)

    def test_prevalidation_rejects_contact_path_at_joint_limit(self):
        self.assertFalse(
            grasp_candidate_prevalidation_feasible(
                1,
                1.0,
                0.98,
                0.0,
                0.01,
            )
        )
        self.assertTrue(
            grasp_candidate_prevalidation_feasible(
                1,
                1.0,
                0.98,
                0.05,
                0.01,
            )
        )

    def test_configured_scene_updates_only_obstacle_geometry(self):
        scene = configured_scene((0.4, -0.1, 0.15), (0.12, 0.20, 0.30))
        objects = {item["id"]: item for item in scene["objects"]}
        self.assertEqual(objects["obstacle_block"]["pose_xyz"], [0.4, -0.1, 0.15])
        self.assertEqual(objects["obstacle_block"]["size"], [0.12, 0.2, 0.3])
        self.assertEqual(objects["table"]["pose_xyz"], [0.65, 0.0, -0.025])


class StateMachineTests(unittest.TestCase):
    def test_successful_run(self):
        machine = ManipulationTaskStateMachine(max_retries=1)
        report = machine.run({state: lambda: StepResult(True) for state in TaskState})
        self.assertEqual(report.final_state, TaskState.SUCCESS)
        self.assertIn(TaskState.RETURN_HOME.value, report.history)

    def test_failure_after_retries(self):
        machine = ManipulationTaskStateMachine(max_retries=1)
        report = machine.run({TaskState.APPROACH: lambda: StepResult(False, "approach blocked")})
        self.assertEqual(report.final_state, TaskState.FAILED)
        self.assertEqual(report.failures[TaskState.APPROACH.value], 2)
        self.assertEqual(report.failure_reason, "approach blocked")


class BenchmarkTests(unittest.TestCase):
    def test_aggregate_by_planner(self):
        runs = [
            PlannerRun("RRTConnect", "s1", True, 1.0, 0.5, 4.0),
            PlannerRun("RRTConnect", "s2", False, 2.0, 0.7, 5.0, "timeout"),
            PlannerRun("PRM", "s1", True, 1.5, 0.6, 4.5),
        ]
        summary = aggregate_by_planner(runs)
        self.assertAlmostEqual(summary["RRTConnect"]["success_rate"], 0.5)
        self.assertAlmostEqual(summary["RRTConnect"]["avg_planning_time_s"], 1.0)
        self.assertAlmostEqual(summary["PRM"]["success_rate"], 1.0)


if __name__ == "__main__":
    unittest.main()
