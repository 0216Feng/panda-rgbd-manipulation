from types import SimpleNamespace as NS

import pytest

pytest.importorskip("moveit_msgs.msg")
from moveit_msgs.msg import RobotState
from panda_manipulation import reverse_approach_probe as probe


@pytest.mark.parametrize("ik_success", [False, True])
def test_probe_is_bounded_and_never_executes(monkeypatch, ik_success):
    seed = RobotState()
    seed.joint_state.name = [f"panda_joint{i}" for i in range(1, 8)]
    seed.joint_state.position = [0.0] * 7
    calls, completed = [], []
    monkeypatch.setattr(probe, "placement_precheck_request", lambda *args: NS(
        ik_request=NS(robot_state=seed)))
    def call(client, request, success, failure):
        calls.append(client)
        if client == "ik":
            success(NS(error_code=NS(val=1 if ik_success else -31), solution=seed))
        elif client == "validity":
            assert list(request.robot_state.joint_state.position[-2:]) == [0.024, 0.024]
            success(NS(valid=True))
        else:
            assert client == "cartesian" and request.avoid_collisions
            success(NS(fraction=1.0, error_code=NS(val=1), solution=NS(
                joint_trajectory=NS(joint_names=seed.joint_state.name, points=[NS(positions=[0.1]*7)]))))
    node = NS(fixed_home_state=lambda: seed, precheck_target_dimensions=[.05,.05,.08],
              group_name="panda_arm", end_effector_link="panda_hand",
              gripper_closed_position=.024, place_object_clearance_m=.008,
              to_pose_stamped=lambda p:p, to_pose=lambda p:p,
              diagnostic_ik_client="ik", diagnostic_validity_client="validity",
              cartesian_client="cartesian", cartesian_retry_max_step=.005,
              grasp_candidate_prevalidation_min_joint_limit_margin=.01,
              make_cartesian_request=lambda *args, **kwargs:NS(),
              call_place_precheck_service=call)
    probe.ReverseApproachProbe(node, {"grasp_pose":{}, "pre_grasp_pose":{}},
                               seed, lambda *args:.2, completed.append).start()
    assert len(completed) == 1
    assert all(not r["executed"] for r in completed[0])
    assert completed[0][-1]["reverse_valid"] == ik_success
    assert calls == (["ik", "validity", "cartesian"] if ik_success else ["ik"]*3)
    assert len(seed.joint_state.name) == 7


def test_collect_all_returns_each_bounded_valid_branch(monkeypatch):
    seed = RobotState()
    seed.joint_state.name = [f"panda_joint{i}" for i in range(1, 8)]
    seed.joint_state.position = [0.0] * 7
    completed = []
    monkeypatch.setattr(probe, "placement_precheck_request", lambda *args: NS(
        ik_request=NS(robot_state=seed)))
    node = NS(fixed_home_state=lambda: seed, precheck_target_dimensions=[.05,.05,.08],
              group_name="panda_arm", end_effector_link="panda_hand",
              gripper_closed_position=.024, place_object_clearance_m=.008,
              to_pose_stamped=lambda p:p, to_pose=lambda p:p,
              diagnostic_ik_client="ik", diagnostic_validity_client="validity",
              cartesian_client="cartesian", cartesian_retry_max_step=.005,
              grasp_candidate_prevalidation_min_joint_limit_margin=.01,
              make_cartesian_request=lambda *args, **kwargs:NS())
    def call(client, request, success, failure):
        if client == "ik": success(NS(error_code=NS(val=1), solution=seed))
        elif client == "validity": success(NS(valid=True))
        else: success(NS(fraction=1.0, error_code=NS(val=1), solution=NS(
            joint_trajectory=NS(joint_names=seed.joint_state.name,
                                points=[NS(positions=[.1]*7)]))))
    node.call_place_precheck_service = call
    probe.ReverseApproachProbe(node, {"grasp_pose":{}, "pre_grasp_pose":{}},
                               seed, lambda *args:.2, completed.append,
                               collect_all=True).start()
    assert len(completed) == 1
    assert len(completed[0]) == 3
    assert all(report["reverse_valid"] for report in completed[0])
