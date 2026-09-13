from copy import deepcopy
from types import SimpleNamespace as NS
import json

import pytest
from test_placement_precheck import method
from panda_manipulation.pick_plan_pipeline import rank_reverse_joint_branches
pytest.importorskip("moveit_msgs.msg")
from moveit_msgs.msg import RobotState


def test_reverse_branches_rank_by_loaded_state_distance():
    state = RobotState()
    state.joint_state.name = [f"panda_joint{i}" for i in range(1,8)]
    state.joint_state.position = [0.0] * 7
    reports = [
        dict(seed_index=0, reverse_valid=True, joint_limit_margin=.5,
             pre_grasp_joint_names=state.joint_state.name,
             pre_grasp_joint_positions=[1.0]*7),
        dict(seed_index=1, reverse_valid=True, joint_limit_margin=.2,
             pre_grasp_joint_names=list(reversed(state.joint_state.name)),
             pre_grasp_joint_positions=[.2]*7),
        dict(seed_index=2, reverse_valid=False),
    ]
    ranked = rank_reverse_joint_branches(state, reports)
    assert [item["seed_index"] for item in ranked] == [1, 0]
    assert ranked[0]["loaded_state_max_joint_delta_rad"] == pytest.approx(.2)


@pytest.mark.parametrize("valid", [False, True])
def test_runtime_reverse_uses_compensated_goal_and_current_payload(valid):
    candidate = {"place_pose":{"position":{"x":.528,"y":-.328,"z":.157}},
                 "object_in_hand_pose":{"position":{"x":.028,"y":.01,"z":.11}}}
    original = deepcopy(candidate)
    pre = {"position":{"x":.528,"y":-.328,"z":.277}}
    state = RobotState()
    state.joint_state.name = [f"panda_joint{i}" for i in range(1,8)]
    state.joint_state.position = [0.0] * 7
    planned, failures = [], []
    node = NS(candidate=candidate, measured_robot_state=lambda:state,
              pre_place_candidate_attempt_count=1,
              get_logger=lambda:NS(info=lambda _:None),
              record_step=lambda *args:None,
              submit_pre_place_candidate=lambda *args:planned.append(args),
              retry_or_fail_pre_place_candidate=failures.append)
    class Probe:
        def __init__(self, n, c, seed, margin, done, collect_all=False):
            assert seed is state
            assert c["grasp_pose"] == candidate["place_pose"]
            assert c["pre_grasp_pose"] == pre
            assert c["object_in_hand_pose"] == candidate["object_in_hand_pose"]
            self.done = done
            assert collect_all is True
        def start(self):
            self.done([dict(reverse_valid=valid, executed=False,
                            pre_grasp_joint_names=[f"panda_joint{i}" for i in range(1,8)],
                            pre_grasp_joint_positions=[.1]*7)])
    invoke = method("probe_measured_pre_place_branch", deepcopy=deepcopy, json=json,
                    ReverseApproachProbe=Probe, normalized_joint_limit_margin=lambda *args:1,
                    ordered_joint_positions=lambda names,values:values,
                    rank_reverse_joint_branches=rank_reverse_joint_branches)
    invoke(node, pre)
    assert candidate == original
    if valid:
        assert planned == [(pre,[.1]*7)]
        assert not failures
    else:
        assert not planned and len(failures) == 1


def test_runtime_reverse_refuses_missing_measured_state():
    failures=[]
    node=NS(measured_robot_state=lambda:None,
            retry_or_fail_pre_place_candidate=failures.append)
    method("probe_measured_pre_place_branch")(node,{})
    assert failures == ["measured state unavailable for reverse placement"]
