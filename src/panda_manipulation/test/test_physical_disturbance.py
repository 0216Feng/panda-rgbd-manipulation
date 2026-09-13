import unittest

from panda_manipulation.physical_disturbance_injector import (
    servo_stage_from_demo_state,
)


class PhysicalDisturbanceTests(unittest.TestCase):
    def test_extracts_executing_servo_stage(self):
        self.assertEqual(
            servo_stage_from_demo_state(
                {
                    "step": "servo_place_descent_stage_3_of_6",
                    "mode": "executing",
                }
            ),
            3,
        )

    def test_rejects_nonexecuting_or_invalid_stage_event(self):
        self.assertIsNone(
            servo_stage_from_demo_state(
                {
                    "step": "servo_place_descent_stage_3_of_6",
                    "mode": "executed",
                }
            )
        )
        self.assertIsNone(
            servo_stage_from_demo_state(
                {
                    "step": "servo_place_descent_stage_7_of_6",
                    "mode": "executing",
                }
            )
        )
        self.assertIsNone(
            servo_stage_from_demo_state(
                {"step": "cartesian_approach", "mode": "executing"}
            )
        )


if __name__ == "__main__":
    unittest.main()
