import os
import sys
import unittest

PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PACKAGE_ROOT)

from panda_manipulation.pick_plan_benchmark_recorder import (
    final_failed_step,
    is_terminal_result,
    render_markdown,
    summarize_results,
)


class PickPlanBenchmarkRecorderTests(unittest.TestCase):
    def test_summarize_results(self):
        summary = summarize_results(
            [
                {
                    "final_state": "SUCCESS",
                    "steps": [{"step": "a", "success": True}],
                },
                {
                    "final_state": "FAILED",
                    "steps": [
                        {"step": "a", "success": True},
                        {"step": "b", "success": False},
                    ],
                },
            ]
        )
        self.assertEqual(summary["total"], 2)
        self.assertEqual(summary["successes"], 1)
        self.assertAlmostEqual(summary["success_rate"], 0.5)
        self.assertEqual(summary["failed_steps"]["b"], 1)

    def test_final_failed_step_prefers_failure_reason(self):
        result = {
            "final_state": "FAILED",
            "failure_reason": "ompl_to_place: action_status=6",
            "steps": [
                {"step": "ompl_to_pre_grasp_orientation_fallback", "success": False},
                {"step": "ompl_to_pre_grasp", "success": True},
                {"step": "ompl_to_place", "success": False},
            ],
        }
        self.assertEqual(final_failed_step(result), "ompl_to_place")

    def test_render_markdown(self):
        markdown = render_markdown(
            {
                "total": 1,
                "successes": 1,
                "failures": 0,
                "success_rate": 1.0,
                "avg_completed_steps": 4.0,
                "failed_steps": {},
                "failure_reasons": [],
            }
        )
        self.assertIn("Success rate: 100.0%", markdown)
        self.assertIn("Resume Metric", markdown)

    def test_execution_summary(self):
        summary = summarize_results(
            [
                {
                    "final_state": "SUCCESS",
                    "execute_trajectories": True,
                    "ompl_execution_backend": "move_group",
                    "steps": [{"step": "a", "success": True}],
                }
            ]
        )
        self.assertEqual(summary["execution_trials"], 1)
        self.assertEqual(summary["execution_successes"], 1)
        self.assertAlmostEqual(summary["execution_success_rate"], 1.0)
        markdown = render_markdown(summary)
        self.assertIn("Execution success rate: 100.0%", markdown)
        self.assertIn("执行成功率", markdown)

    def test_waiting_result_is_not_a_trial(self):
        self.assertFalse(is_terminal_result({"final_state": "WAITING"}))
        summary = summarize_results(
            [
                {"final_state": "WAITING", "failure_reason": "waiting for /joint_states"},
                {"final_state": "SUCCESS", "steps": [{"step": "a", "success": True}]},
            ]
        )
        self.assertEqual(summary["total"], 1)
        self.assertEqual(summary["successes"], 1)
        self.assertEqual(summary["failures"], 0)


if __name__ == "__main__":
    unittest.main()
