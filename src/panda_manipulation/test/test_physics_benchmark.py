import os
import tempfile
import unittest
import xml.etree.ElementTree as ET

from panda_manipulation.gazebo_pick_validator import upright_tilt_degrees
from panda_manipulation.physics_benchmark import (
    CHALLENGE_SCENARIOS,
    failure_category,
    pipeline_process_exit_reason,
    read_report_rows,
    reclassify_infrastructure_failures,
    render_markdown,
    render_perception_comparison_svg,
    render_planner_comparison_svg,
    summarize,
    validation_to_row,
    wilson_interval,
    write_reports,
    write_svg_report,
    write_trial_world,
)


class PhysicsBenchmarkTests(unittest.TestCase):
    def test_side_challenges_leave_clearance_around_the_target(self):
        self.assertEqual(CHALLENGE_SCENARIOS[1].target_y, -0.10)
        self.assertEqual(CHALLENGE_SCENARIOS[2].target_y, 0.10)

    def test_pipeline_crash_is_detected_before_trial_timeout(self):
        line = (
            "[ERROR] [pick_plan_pipeline-11]: process has died "
            "[pid 123, exit code 1]"
        )
        self.assertEqual(
            pipeline_process_exit_reason(line),
            "pick pipeline process exited before physical validation",
        )
        self.assertIsNone(pipeline_process_exit_reason("unrelated process has died"))

    def test_aruco_estimator_crash_is_a_perception_failure(self):
        line = (
            "[ERROR] [aruco_pose_estimator-9]: process has died "
            "[pid 123, exit code -11]"
        )
        reason = pipeline_process_exit_reason(line)
        self.assertEqual(
            reason,
            "perception failed: ArUco pose estimator process exited",
        )
        self.assertEqual(failure_category(reason), "perception")

    def test_rgbd_estimator_crash_is_a_perception_failure(self):
        line = (
            "[ERROR] [rgbd_target_pose_estimator-9]: process has died "
            "[pid 123, exit code 1]"
        )
        reason = pipeline_process_exit_reason(line)
        self.assertEqual(
            reason,
            "perception failed: RGB-D target pose estimator process exited",
        )
        self.assertEqual(failure_category(reason), "perception")

    def test_controller_spawner_crash_is_an_infrastructure_failure(self):
        line = (
            "[ERROR] [spawner-14]: process has died [exit code 1, cmd "
            "'spawner panda_arm_controller']"
        )
        self.assertEqual(
            pipeline_process_exit_reason(line),
            "infrastructure startup failed: controller spawner exited before activation",
        )
        self.assertEqual(
            failure_category(pipeline_process_exit_reason(line)),
            "infrastructure_startup",
        )

    def test_summary_uses_only_physically_successful_measurements(self):
        success = validation_to_row(
            1,
            {
                "final_state": "SUCCESS",
                "pipeline_state": "SUCCESS",
                "initial_pose": [0.52, 0.0, 0.76],
                "max_z": 0.86,
                "final_pose": [0.45, -0.16, 0.76],
                "lift_delta_m": 0.10,
                "place_error_m": 0.001,
                "final_tilt_deg": 2.5,
                "reason": "physical lift and placement verified",
                "planner_id": "RRTConnectkConfigDefault",
                "direct_path_fraction": 0.42,
                "direct_path_blocked": True,
                "ompl_planning_time_s": 0.25,
                "ompl_joint_path_length_rad": 2.5,
            },
            20.0,
        )
        failure = validation_to_row(
            2,
            {"final_state": "FAILED", "reason": "object was not lifted"},
            30.0,
        )
        summary = summarize([success, failure])
        self.assertEqual(summary["successes"], 1)
        self.assertEqual(summary["failures"], 1)
        self.assertEqual(summary["success_rate"], 0.5)
        self.assertEqual(summary["valid_start_success_rate"], 0.5)
        self.assertIsNotNone(summary["success_rate_ci"])
        self.assertAlmostEqual(summary["mean_lift_delta_m"], 0.10)
        self.assertAlmostEqual(summary["mean_place_error_m"], 0.001)
        self.assertAlmostEqual(summary["mean_final_tilt_deg"], 2.5)
        self.assertEqual(summary["initial_x_range"], [0.52, 0.52])
        self.assertEqual(summary["initial_y_range"], [0.0, 0.0])
        self.assertEqual(summary["failure_reasons"]["object was not lifted"], 1)
        self.assertEqual(
            summary["planner_results"]["RRTConnectkConfigDefault"]["success_rate"],
            1.0,
        )
        self.assertEqual(summary["blocked_direct_paths"], 1)
        self.assertAlmostEqual(summary["mean_ompl_planning_time_s"], 0.25)
        self.assertAlmostEqual(summary["mean_ompl_joint_path_length_rad"], 2.5)

    def test_reports_include_physical_metrics(self):
        row = validation_to_row(
            1,
            {
                "final_state": "SUCCESS",
                "pipeline_state": "SUCCESS",
                "initial_pose": [0.52, 0.0, 0.76],
                "max_z": 0.86,
                "final_pose": [0.45, -0.16, 0.76],
                "lift_delta_m": 0.10,
                "place_error_m": 0.002,
                "final_tilt_deg": 3.0,
                "reason": "verified",
                "planner_id": "RRTConnectkConfigDefault",
                "direct_path_fraction": 0.35,
                "direct_path_blocked": True,
                "ompl_planning_time_s": 0.2,
                "ompl_joint_path_length_rad": 2.2,
            },
            18.0,
        )
        with tempfile.TemporaryDirectory() as directory:
            csv_path = os.path.join(directory, "results.csv")
            markdown_path = os.path.join(directory, "report.md")
            summary = write_reports([row], csv_path, markdown_path)
            with open(markdown_path, encoding="utf-8") as report:
                report_text = report.read()
            self.assertEqual(summary["success_rate"], 1.0)
            self.assertIn("Raw physical success rate: 100.0%", report_text)
            self.assertIn("Valid-start physical success rate: 100.0%", report_text)
            self.assertIn("Mean placement error: 0.0020 m", report_text)
            self.assertIn("Mean final tilt: 3.00 deg", report_text)
            self.assertIn("Measured target X range: [0.520, 0.520] m", report_text)
            self.assertIn("## Planner Comparison", report_text)
            self.assertIn("Collision-aware direct paths blocked: 1/1", report_text)
            self.assertIn("Mean OMPL planning time: 0.2000 s", report_text)
            self.assertTrue(os.path.exists(csv_path))
            loaded_rows = read_report_rows(csv_path)
            self.assertEqual(len(loaded_rows), 1)
            self.assertIs(loaded_rows[0]["direct_path_blocked"], True)

    def test_saved_spawner_failures_are_reclassified_without_hiding_raw_trials(self):
        row = validation_to_row(
            77,
            {
                "final_state": "FAILED",
                "reason": "trial timed out after 180s",
            },
            181.0,
        )
        with tempfile.TemporaryDirectory() as directory:
            log_path = os.path.join(directory, "trial_077.log")
            with open(log_path, "w", encoding="utf-8") as log_file:
                for controller in (
                    "joint_state_broadcaster",
                    "panda_arm_controller",
                    "panda_hand_controller",
                ):
                    log_file.write(
                        f"[ERROR] [spawner-1]: process has died "
                        f"[cmd 'spawner {controller}']\n"
                    )
            classified = reclassify_infrastructure_failures([row], directory)
            summary = summarize(classified)
            self.assertEqual(
                classified[0]["failure_category"],
                "infrastructure_startup",
            )
            self.assertIn(
                "controller spawners exited before activation",
                classified[0]["reason"],
            )
            self.assertEqual(summary["total"], 1)
            self.assertEqual(summary["success_rate"], 0.0)
            self.assertEqual(summary["infrastructure_failures"], 1)
            self.assertEqual(summary["valid_start_trials"], 0)

    def test_wilson_interval_is_bounded_and_non_degenerate(self):
        lower, upper = wilson_interval(81, 90)
        self.assertGreater(lower, 0.8)
        self.assertLess(upper, 1.0)
        self.assertLess(lower, 0.9)
        self.assertGreater(upper, 0.9)
        self.assertIsNone(wilson_interval(0, 0))

    def test_planner_comparison_svg_contains_all_metrics(self):
        rows = []
        for trial, planner, planning_time, path_length, elapsed in [
            (1, "RRTConnectkConfigDefault", 0.03, 8.4, 70.0),
            (2, "PRMkConfigDefault", 0.04, 7.3, 64.0),
            (3, "RRTstarkConfigDefault", 16.0, 8.0, 99.0),
        ]:
            rows.append(
                validation_to_row(
                    trial,
                    {
                        "final_state": "SUCCESS",
                        "pipeline_state": "SUCCESS",
                        "initial_pose": [0.52, 0.0, 0.76],
                        "max_z": 0.90,
                        "final_pose": [0.50, -0.30, 0.76],
                        "lift_delta_m": 0.14,
                        "place_error_m": 0.003,
                        "final_tilt_deg": 0.1,
                        "reason": "verified",
                        "planner_id": planner,
                        "direct_path_fraction": 0.35,
                        "direct_path_blocked": True,
                        "ompl_planning_time_s": planning_time,
                        "ompl_joint_path_length_rad": path_length,
                    },
                    elapsed,
                )
            )
        summary = summarize(rows)
        svg = render_planner_comparison_svg(summary)
        self.assertIn("<svg", svg)
        self.assertIn("RRTConnect", svg)
        self.assertIn("PRM", svg)
        self.assertIn("RRTstar", svg)
        self.assertIn("Mean OMPL planning time", svg)
        self.assertIn("Mean joint-space path length", svg)
        self.assertIn("16.0000 s", svg)

        with tempfile.TemporaryDirectory() as directory:
            svg_path = os.path.join(directory, "comparison.svg")
            write_svg_report(summary, svg_path)
            self.assertTrue(os.path.exists(svg_path))

    def test_perception_suite_reports_accuracy_and_generates_comparison_svg(self):
        rows = []
        for trial, mode, perception_error, detection_rate, elapsed in [
            (1, "synthetic", None, None, 42.0),
            (2, "aruco", 0.0079, 0.98, 47.0),
        ]:
            rows.append(
                validation_to_row(
                    trial,
                    {
                        "final_state": "SUCCESS",
                        "pipeline_state": "SUCCESS",
                        "perception_mode": mode,
                        "perception_validation_state": (
                            "SUCCESS" if mode == "aruco" else None
                        ),
                        "perception_samples": 10 if mode == "aruco" else None,
                        "perception_mean_error_m": perception_error,
                        "perception_max_error_m": 0.009 if mode == "aruco" else None,
                        "perception_std_error_m": 0.001 if mode == "aruco" else None,
                        "perception_first_detection_latency_s": (
                            2.1 if mode == "aruco" else None
                        ),
                        "perception_validation_duration_s": (
                            0.5 if mode == "aruco" else None
                        ),
                        "perception_detection_rate": detection_rate,
                        "initial_pose": [0.52, 0.0, 0.76],
                        "max_z": 0.86,
                        "final_pose": [0.45, -0.16, 0.76],
                        "lift_delta_m": 0.10,
                        "place_error_m": 0.004,
                        "final_tilt_deg": 1.0,
                        "reason": "verified",
                        "planner_id": "RRTConnectkConfigDefault",
                    },
                    elapsed,
                )
            )
        summary = summarize(rows)
        report = render_markdown(summary)
        svg = render_perception_comparison_svg(summary)
        self.assertEqual(
            summary["perception_results"]["aruco"]["mean_perception_error_m"],
            0.0079,
        )
        self.assertEqual(
            summary["perception_results"]["aruco"][
                "perception_validation_success_rate"
            ],
            1.0,
        )
        self.assertIn("## Perception Input Comparison", report)
        self.assertIn("| aruco |", report)
        self.assertIn("1/1 (100.0%)", report)
        self.assertIn("Synthetic pose", svg)
        self.assertIn("ArUco camera", svg)
        self.assertIn("Mean perception pose error", svg)
        self.assertIn("7.9 mm", svg)

        with tempfile.TemporaryDirectory() as directory:
            svg_path = os.path.join(directory, "perception.svg")
            write_svg_report(summary, svg_path)
            with open(svg_path, encoding="utf-8") as report_file:
                self.assertIn("Perception-Driven Pick Benchmark", report_file.read())

    def test_upright_tilt_ignores_yaw_but_rejects_a_fallen_cube(self):
        self.assertAlmostEqual(upright_tilt_degrees((0.0, 0.0, 0.7071068, 0.7071068)), 0.0)
        self.assertAlmostEqual(upright_tilt_degrees((0.7071068, 0.0, 0.0, 0.7071068)), 90.0)

    def test_empty_summary_renders_without_division_error(self):
        report = render_markdown(summarize([]))
        self.assertIn("Trials: 0", report)
        self.assertIn("Mean lift delta: N/A", report)

    def test_randomized_world_updates_only_target_xy(self):
        source = os.path.join(
            os.path.dirname(__file__),
            "..",
            "worlds",
            "panda_table.sdf",
        )
        with tempfile.TemporaryDirectory() as directory:
            output = os.path.join(directory, "trial.sdf")
            write_trial_world(source, output, 0.487, -0.043)
            root = ET.parse(output).getroot()
            pose = root.find(".//model[@name='target_cube']/pose")
            self.assertIsNotNone(pose)
            values = [float(value) for value in pose.text.split()]
            self.assertEqual(values[:2], [0.487, -0.043])
            self.assertEqual(values[2:], [0.76, 0.0, 0.0, 0.0])

    def test_challenge_world_updates_matching_obstacle_pose_and_size(self):
        source = os.path.join(
            os.path.dirname(__file__),
            "..",
            "worlds",
            "panda_table.sdf",
        )
        with tempfile.TemporaryDirectory() as directory:
            output = os.path.join(directory, "challenge.sdf")
            write_trial_world(
                source,
                output,
                0.55,
                0.0,
                (0.40, 0.0, 0.15),
                (0.12, 0.18, 0.30),
            )
            obstacle = ET.parse(output).getroot().find(
                ".//model[@name='obstacle_block']"
            )
            self.assertIsNotNone(obstacle)
            pose = [float(value) for value in obstacle.find("pose").text.split()]
            self.assertEqual(pose[:3], [0.40, 0.0, 0.87])
            sizes = {
                tuple(float(value) for value in size.text.split())
                for size in obstacle.findall(".//box/size")
            }
            self.assertEqual(sizes, {(0.12, 0.18, 0.30)})


if __name__ == "__main__":
    unittest.main()
