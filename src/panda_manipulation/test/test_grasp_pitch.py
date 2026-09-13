from copy import deepcopy

import pytest

from panda_manipulation.geometry import (
    Vector3, compose_pose, pose_from_dict, quaternion_from_euler, rotate_vector,
)
from panda_manipulation.grasp import tilted_grasp_candidate
from panda_manipulation.pick_plan_pipeline import equivalent_grasp_candidate, payload_compensated_place_pose


def candidate():
    def pose(x, y, z, frame="panda_link0", pitch_hand=True):
        q = quaternion_from_euler(3.141592653589793, 0.0, -0.785) if pitch_hand else quaternion_from_euler(0, 0, 0)
        return {"frame_id": frame, "position": dict(x=x, y=y, z=z), "orientation": q.as_dict()}
    from panda_manipulation.geometry import relative_pose
    hand, obj = pose(0.55, 0.0, 0.155), pose(0.55, 0.0, 0.04, pitch_hand=False)
    return {"candidate_id": "nominal", "grasp_pose": hand, "object_pose": obj,
            "object_in_hand_pose": relative_pose(pose_from_dict(hand), pose_from_dict(obj), "panda_hand").as_dict(),
            "pre_grasp_pose": pose(0.55, 0.0, 0.255), "lift_pose": pose(0.55, 0.0, 0.305),
            "place_pose": pose(0.5, -0.3, 0.04), "retreat_pose": pose(0.5, -0.3, 0.16)}


def assert_pose_equal(a, b):
    assert a.frame_id == b.frame_id
    assert a.position.as_dict() == pytest.approx(b.position.as_dict(), abs=1e-10)
    dot = sum(getattr(a.orientation, k) * getattr(b.orientation, k) for k in ("x", "y", "z", "w"))
    assert abs(dot) == pytest.approx(1.0, abs=1e-10)


def placed_object(data):
    hand = payload_compensated_place_pose(data["place_pose"], data["object_in_hand_pose"])
    return compose_pose(pose_from_dict(hand), pose_from_dict(data["object_in_hand_pose"]), "panda_link0")


@pytest.mark.parametrize("pitch", [-20.0, -10.0, 10.0, 20.0])
def test_pitch_keeps_object_goal_and_closing_direction(pitch):
    original = candidate()
    before = deepcopy(original)
    tilted = tilted_grasp_candidate(original, pitch)
    hand = pose_from_dict(tilted["grasp_pose"])
    local = pose_from_dict(tilted["object_in_hand_pose"])
    assert_pose_equal(compose_pose(hand, local, "panda_link0"), pose_from_dict(original["object_pose"]))
    assert_pose_equal(placed_object(tilted), placed_object(original))
    assert local.position.as_dict() == pytest.approx(pose_from_dict(original["object_in_hand_pose"]).position.as_dict())
    closing = rotate_vector(Vector3(0, 1, 0), hand.orientation)
    old_closing = rotate_vector(Vector3(0, 1, 0), pose_from_dict(original["grasp_pose"]).orientation)
    assert closing.as_dict() == pytest.approx(old_closing.as_dict())
    assert original == before
    assert tilted["place_pose"]["position"] == original["place_pose"]["position"]


@pytest.mark.parametrize("yaw", [-90.0, 90.0, 180.0])
@pytest.mark.parametrize("symmetric", [False, True])
def test_yaw_recovery_rotates_pitched_hand_offset_consistently(yaw, symmetric):
    base = candidate()
    base["cube_symmetric_placement"] = symmetric
    first = equivalent_grasp_candidate(tilted_grasp_candidate(base, 10.0), yaw)
    second = tilted_grasp_candidate(equivalent_grasp_candidate(base, yaw), 10.0)
    for key in ("grasp_pose", "pre_grasp_pose", "lift_pose", "place_pose", "object_in_hand_pose"):
        assert_pose_equal(pose_from_dict(first[key]), pose_from_dict(second[key]))
    assert_pose_equal(placed_object(first), placed_object(equivalent_grasp_candidate(base, yaw)))


@pytest.mark.parametrize("pitch", [-21, 21, float("nan"), float("inf")])
def test_invalid_pitch_rejected(pitch):
    with pytest.raises(ValueError):
        tilted_grasp_candidate(candidate(), pitch)


def test_zero_pitch_preserves_legacy_candidate():
    base = candidate()
    assert tilted_grasp_candidate(base, 0.0) == base
