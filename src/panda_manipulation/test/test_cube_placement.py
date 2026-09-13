from copy import deepcopy

import pytest

from panda_manipulation.pick_plan_pipeline import equivalent_grasp_candidate


def test_cube_yaw_policy_is_opt_in_and_preserves_positions():
    pose = {"frame_id": "panda_link0", "position": {"x": 0.5, "y": -0.3, "z": 0.15},
            "orientation": {"x": 1.0, "y": 0.0, "z": 0.0, "w": 0.0}}
    original = {key: deepcopy(pose) for key in
                ("grasp_pose", "lift_pose", "place_pose", "pre_place_pose", "retreat_pose")}
    assert equivalent_grasp_candidate(original, 90)["place_pose"] == pose
    original["cube_symmetric_placement"] = True
    rotated = equivalent_grasp_candidate(original, 90)
    for key in ("place_pose", "pre_place_pose", "retreat_pose"):
        assert rotated[key]["position"] == pose["position"]
        assert rotated[key]["orientation"] == pytest.approx(rotated["grasp_pose"]["orientation"])
    assert original["place_pose"] == pose
