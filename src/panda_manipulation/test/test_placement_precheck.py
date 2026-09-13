import ast
import math
from typing import Dict
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from panda_manipulation.placement_precheck import target_box_dimensions
from panda_manipulation import pick_plan_pipeline


def method(name, **symbols):
    tree = ast.parse(Path(pick_plan_pipeline.__file__).read_text())
    function = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
    namespace = dict(symbols)
    exec(compile(ast.Module(body=[function], type_ignores=[]), "pipeline_method", "exec"), namespace)
    return namespace[name]


@pytest.mark.parametrize("fraction,points", [(float("nan"), [1]),
                                             (1.1, [1]), (-0.1, [1]), (1.0, [])])
def test_invalid_cartesian_response_cannot_remain_feasible(fraction, points):
    completed = []
    response = NS(fraction=fraction, error_code=NS(val=1),
                  solution=NS(joint_trajectory=NS(points=points)))
    node = NS(complete_grasp_cartesian_candidate=completed.append)
    provisional = {"feasible": True}
    method("on_grasp_prevalidation_cartesian_result", Dict=Dict,
           isfinite=math.isfinite)(node, NS(result=lambda: response),
                                 0.0, {"id": "candidate"}, provisional)
    assert completed == [provisional]
    assert provisional["feasible"] is False
    assert provisional["reason"].startswith("Cartesian result error:")


def test_approach_prevalidation_uses_bounded_service_and_fails_closed():
    completed = []
    candidate = {"grasp_pose": "grasp", "pre_grasp_pose": "pre"}
    provisional = {"feasible": True}
    request = NS()
    client = object()
    def bounded(actual_client, actual_request, success, failure):
        assert actual_client is client
        assert actual_request.waypoints == ["pre", "grasp"]
        assert actual_request.avoid_collisions is True
        failure("service response timed out")
    node = NS(grasp_candidate_cartesian_prevalidation_index=0,
              grasp_candidate_cartesian_prevalidation=[(0, candidate, provisional, "end")],
              cartesian_retry_max_step=0.005, cartesian_client=client,
              make_cartesian_request=lambda *args, **kwargs: request,
              to_pose=lambda value: value, call_place_precheck_service=bounded,
              complete_grasp_cartesian_candidate=completed.append)
    method("prevalidate_next_grasp_cartesian_candidate")(node)
    assert completed == [provisional]
    assert provisional["feasible"] is False
    assert "timed out" in provisional["reason"]


@pytest.mark.parametrize("valid", [None, False])
def test_endpoint_rejection_cannot_be_selected_by_approach_fallback(valid):
    failures = []
    candidate = {"score": 100, "candidate": {}, "feasible": True,
                 "place_endpoint_valid": valid, "joint_limit_margin": 0.4}
    node = NS(get_parameter=lambda _: NS(value=True),
              grasp_candidate_prevalidation_results=[candidate],
              grasp_candidate_prevalidation_min_joint_limit_margin=0.01,
              finish_failed=failures.append)
    method("select_prevalidated_grasp_candidate")(node)
    assert len(failures) == 1
    assert "no eligible reachable candidate" in failures[0]


def test_timeout_discards_late_service_response():
    success, failures, destroyed = [], [], []
    timer = NS(cancel=lambda: None)
    future = NS(add_done_callback=lambda callback: setattr(timer, "receive", callback))
    client = NS(service_is_ready=lambda: True, call_async=lambda _: future)
    def create_timer(period, callback, **kwargs):
        assert period == 10.0
        timer.fire = callback
        return timer
    node = NS(create_timer=create_timer, destroy_timer=destroyed.append)
    invoke = method("call_place_precheck_service", Clock=lambda **_: None,
                    ClockType=NS(STEADY_TIME=1))
    invoke(node, client, object(), success.append, failures.append)
    timer.fire()
    timer.receive(NS(result=lambda: "late"))
    assert failures == ["service response timed out"]
    assert not success
    assert destroyed == [timer]


@pytest.mark.parametrize("dimensions", [[0.05, 0.05, 0.08], [0.0, 0.05, 0.08], [float("nan"), 1, 1], [1, 1]])
def test_scene_dimensions_are_read_and_validated(dimensions):
    shape = NS(type=1, BOX=1, dimensions=dimensions)
    obj = NS(id="target_cube", primitives=[shape], meshes=[], planes=[])
    scene = NS(world=NS(collision_objects=[obj]))
    if dimensions == [0.05, 0.05, 0.08]:
        assert target_box_dimensions(scene) == dimensions
    else:
        with pytest.raises(ValueError):
            target_box_dimensions(scene)


def test_request_carries_nominal_payload_without_mutating_live_state():
    messages = pytest.importorskip("moveit_msgs.msg")
    from panda_manipulation.placement_precheck import placement_precheck_request
    def pose(x, y, z, qx=0.0, qw=1.0, frame="panda_link0"):
        return {"frame_id": frame, "position": dict(x=x, y=y, z=z),
                "orientation": dict(x=qx, y=0.0, z=0.0, w=qw)}
    candidate = {"place_pose": pose(0.5, -0.3, 0.16, 1.0, 0.0),
                 "object_pose": pose(0.55, 0.0, 0.04),
                 "object_in_hand_pose": pose(0.01, 0.0, 0.10, 1.0, 0.0, "panda_hand")}
    seed = messages.RobotState()
    seed.joint_state.name = [f"panda_joint{i}" for i in range(1, 8)]
    seed.joint_state.position = [0.0] * 7
    original = deepcopy(candidate)
    request = placement_precheck_request(candidate, seed, [0.05, 0.05, 0.08],
                                        "panda_arm", "panda_hand", 0.024, 0.008)
    ik = request.ik_request
    assert ik.avoid_collisions and ik.robot_state.is_diff
    assert ik.pose_stamped.pose.position.x == pytest.approx(0.49)
    assert ik.pose_stamped.pose.position.z == pytest.approx(0.148)
    assert list(ik.robot_state.joint_state.position[-2:]) == [0.024, 0.024]
    attached, = ik.robot_state.attached_collision_objects
    assert attached.object.id == "target_cube"
    assert list(attached.object.primitives[0].dimensions) == [0.05, 0.05, 0.08]
    assert attached.object.primitive_poses[0].position.x == 0.01
    assert attached.link_name == "panda_hand"
    assert not seed.attached_collision_objects and len(seed.joint_state.name) == 7
    assert candidate == original
