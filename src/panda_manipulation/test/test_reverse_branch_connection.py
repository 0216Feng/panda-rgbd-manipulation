from types import SimpleNamespace as NS

import pytest

pytest.importorskip("moveit_msgs.msg")
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState, MotionPlanRequest, PlanningOptions
from panda_manipulation.reverse_branch_connection import connection_goal


def test_connection_restores_target_locally_and_uses_measured_empty_state():
    measured = RobotState()
    measured.joint_state.name = [f"panda_joint{i}" for i in range(1, 8)]
    measured.joint_state.position = [0.1] * 7
    calls = []
    def request(values, start, planner, tolerance):
        calls.append((values, start, tolerance))
        value = MotionPlanRequest()
        value.start_state = start
        return value
    node = NS(grasp_candidate_prevalidation_start_state=measured,
              make_joint_motion_plan_request=request,
              grasp_prevalidation_planner_id=lambda:"RRTConnectkConfigDefault",
              make_planning_options=PlanningOptions,
              precheck_target_dimensions=[.05,.05,.08], to_pose=lambda _:Pose())
    branch = {"pre_grasp_joint_names":list(reversed(measured.joint_state.name)),
              "pre_grasp_joint_positions":list(range(7))}
    goal = connection_goal(node, {"object_pose":{"frame_id":"panda_link0"}}, branch)
    assert goal.planning_options.plan_only
    assert calls[0][0] == list(reversed(range(7)))
    assert calls[0][2] == .002
    assert list(goal.request.start_state.joint_state.position) == [0.1]*7
    assert not goal.request.start_state.attached_collision_objects
    scene = goal.planning_options.planning_scene_diff
    assert scene.is_diff and scene.robot_state.is_diff
    target, = scene.world.collision_objects
    assert target.id == "target_cube" and target.operation == target.ADD
    assert list(target.primitives[0].dimensions) == [.05,.05,.08]
    assert not measured.is_diff


def test_invalid_reverse_joint_set_rejected_before_planning():
    with pytest.raises(ValueError, match="exactly seven"):
        connection_goal(NS(), {}, {"pre_grasp_joint_names":["panda_joint1"]*7,
                                   "pre_grasp_joint_positions":[0.0]*7})
