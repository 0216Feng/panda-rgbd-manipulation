"""Plan-only connection from the measured empty hand to a reverse IK branch."""

from copy import deepcopy
from math import isfinite
from types import SimpleNamespace

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive


def connection_goal(node, candidate, branch):
    names = branch["pre_grasp_joint_names"]
    values = branch["pre_grasp_joint_positions"]
    expected = [f"panda_joint{i}" for i in range(1, 8)]
    if len(names) != 7 or set(names) != set(expected) or len(values) != 7:
        raise ValueError("reverse branch must contain exactly seven arm joints")
    if not all(isfinite(v) for v in values):
        raise ValueError("reverse branch contains nonfinite joints")
    positions = dict(zip(names, values))
    start = deepcopy(node.grasp_candidate_prevalidation_start_state)
    if start is None:
        raise ValueError("measured prevalidation start missing")
    start.is_diff = True
    goal = MoveGroup.Goal()
    goal.request = node.make_joint_motion_plan_request(
        [positions[name] for name in expected], start,
        node.grasp_prevalidation_planner_id(), 0.002)
    goal.planning_options = node.make_planning_options()
    goal.planning_options.plan_only = True
    scene = goal.planning_options.planning_scene_diff
    scene.is_diff = True
    scene.robot_state = deepcopy(start)
    target = CollisionObject()
    target.id = "target_cube"
    target.header.frame_id = candidate["object_pose"]["frame_id"]
    target.operation = CollisionObject.ADD
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = list(node.precheck_target_dimensions)
    target.primitives = [box]
    target.primitive_poses = [node.to_pose(candidate["object_pose"])]
    scene.world.collision_objects = [target]
    return goal


def connect_reverse_branch(node, candidate, branch, success, failure):
    try:
        goal = connection_goal(node, candidate, branch)
    except (ValueError, KeyError, TypeError) as exc:
        failure(str(exc))
        return

    def received(wrapper):
        if wrapper.status != 4 or wrapper.result.error_code.val != 1:
            failure(f"reverse branch connection: status={wrapper.status}, code={wrapper.result.error_code.val}")
            return
        success(wrapper.result)

    def accepted(handle):
        if not handle.accepted:
            failure("reverse branch connection rejected")
            return
        adapter = SimpleNamespace(service_is_ready=lambda: True,
                                  call_async=lambda _: handle.get_result_async())
        def timeout(reason):
            handle.cancel_goal_async()
            failure(reason)
        node.call_place_precheck_service(adapter, None, received, timeout,
                                        timeout_s=max(10.0, node.allowed_planning_time + 5.0))

    adapter = SimpleNamespace(service_is_ready=node.move_client.server_is_ready,
                              call_async=node.move_client.send_goal_async)
    node.call_place_precheck_service(adapter, goal, accepted, failure)
