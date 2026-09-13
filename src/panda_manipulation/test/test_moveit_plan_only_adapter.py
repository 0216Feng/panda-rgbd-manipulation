import os
import sys
import unittest

PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PACKAGE_ROOT)

from panda_manipulation.moveit_plan_only_adapter import selected_candidate


class MoveItPlanOnlyAdapterTests(unittest.TestCase):
    def test_selected_candidate_returns_matching_candidate(self):
        payload = {
            "selected_candidate_id": "grasp_01",
            "candidates": [
                {"candidate_id": "grasp_00"},
                {"candidate_id": "grasp_01", "score": 0.9},
            ],
        }
        self.assertEqual(selected_candidate(payload)["score"], 0.9)

    def test_selected_candidate_returns_none_for_missing_candidate(self):
        payload = {
            "selected_candidate_id": "grasp_99",
            "candidates": [{"candidate_id": "grasp_00"}],
        }
        self.assertIsNone(selected_candidate(payload))


if __name__ == "__main__":
    unittest.main()
