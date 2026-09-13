from math import pi

import pytest

from panda_manipulation.geometry import (
    PoseSpec, Vector3, Quaternion, compose_pose, pose_from_dict,
    quaternion_from_euler, quaternion_multiply,
)
from panda_manipulation.pick_plan_pipeline import cube_symmetric_hand_goal


@pytest.mark.parametrize("yaw", [0, 90, 180, 270])
def test_symmetry_preserves_object_center_with_off_axis_grip(yaw):
    hand = PoseSpec("panda_link0", Vector3(0.5, -0.3, 0.15),
                    quaternion_from_euler(pi, 0.0, -0.785))
    local = PoseSpec("panda_hand", Vector3(0.013, -0.009, 0.1), Quaternion(1, 0, 0, 0))
    original = compose_pose(hand, local, hand.frame_id)
    goal = cube_symmetric_hand_goal(hand.as_dict(), local.as_dict(), yaw)
    placed = compose_pose(pose_from_dict(goal), local, hand.frame_id)
    assert placed.position.as_dict() == pytest.approx(original.position.as_dict())
    expected = quaternion_multiply(quaternion_from_euler(0, 0, yaw*pi/180), original.orientation)
    assert placed.orientation.as_dict() == pytest.approx(expected.as_dict())


@pytest.mark.parametrize("yaw", [45, float("nan"), float("inf")])
def test_non_cube_rotations_are_rejected(yaw):
    with pytest.raises(ValueError):
        cube_symmetric_hand_goal({}, {}, yaw)
