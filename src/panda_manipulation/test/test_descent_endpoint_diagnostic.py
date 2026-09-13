import ast
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest


class Future:
    def __init__(self, result):
        self.value = result

    def result(self):
        return self.value

    def add_done_callback(self, callback):
        callback(self)


@pytest.mark.parametrize("ik_code, expected_calls", [(1, 1), (-31, 0)])
@pytest.mark.parametrize("seed", ["pre_place", "home", "elbow_positive", "elbow_negative"])
def test_endpoint_diagnostic_is_read_only(ik_code, expected_calls, seed):
    path = Path(__file__).resolve().parents[1] / "panda_manipulation/pick_plan_pipeline.py"
    tree = ast.parse(path.read_text())
    method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
                  and n.name == "diagnose_descent_endpoint")
    module = ast.parse("from __future__ import annotations")
    module.body.append(method)
    logs, ik_requests, validity_requests = [], [], []
    ik_reply = NS(error_code=NS(val=ik_code), solution=NS(is_diff=False,
                  joint_state=NS(name=["panda_joint1"], position=[0.1])))
    validity_reply = NS(valid=False, contacts=[NS(contact_body_1="target_cube",
                                                 contact_body_2="obstacle_block")])

    def ik_call(request):
        ik_requests.append(request)
        return Future(ik_reply)

    def validity_call(request):
        validity_requests.append(request)
        return Future(validity_reply)

    namespace = {"deepcopy": deepcopy, "json": json,
                 "GetPositionIK": NS(Request=lambda: NS(ik_request=NS(timeout=NS(sec=0)))),
                 "GetStateValidity": NS(Request=lambda: NS())}
    exec(compile(module, str(path), "exec"), namespace)
    observer = NS(pre_place_candidate_attempt_count=2, group_name="panda_arm",
                  end_effector_link="panda_hand", candidate={"place_pose": {"goal": 1}},
                  get_logger=lambda: NS(info=logs.append),
                  end_state_from_trajectory=lambda _: NS(is_diff=True),
                  fixed_home_state=lambda: NS(is_diff=True, joint_state=NS(
                      position=[0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])),
                  to_pose_stamped=lambda pose: pose,
                  diagnostic_ik_client=NS(service_is_ready=lambda: True, call_async=ik_call),
                  diagnostic_validity_client=NS(service_is_ready=lambda: True, call_async=validity_call))
    namespace["diagnose_descent_endpoint"](observer, object(), seed)
    assert ik_requests[0].ik_request.avoid_collisions is False
    assert ik_requests[0].ik_request.timeout.sec == 1
    assert len(validity_requests) == expected_calls
    report = json.loads(logs[-1])
    assert report["executed"] is False
    assert report["seed_variant"] == seed
    if expected_calls:
        assert validity_requests[0].robot_state.is_diff is True
        assert report["contacts"] == [["target_cube", "obstacle_block"]]
