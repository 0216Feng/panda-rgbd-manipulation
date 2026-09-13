from copy import deepcopy
from types import SimpleNamespace as NS

import pytest

pytest.importorskip("moveit_msgs.msg")
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject, MotionPlanRequest, PlanningOptions, RobotState
from moveit_msgs.srv import GetCartesianPath

from panda_manipulation.loaded_path_precheck import LoadedPathPrecheck, advance_predicted_state
from panda_manipulation.geometry import compose_pose, pose_from_dict
from test_placement_precheck import method


def pose(x, y, z, frame="panda_link0", downward=False):
    return {"frame_id": frame, "position": dict(x=x, y=y, z=z),
            "orientation": dict(x=1.0 if downward else 0.0, y=0.0, z=0.0,
                                w=0.0 if downward else 1.0)}


def ready(value):
    return NS(result=lambda: value)


@pytest.mark.parametrize("duration,passes", [(15, True), (16, False), (61, False)])
def test_nominal_descent_uses_execution_duration_limit(duration, passes):
    fixture = Fixture()
    original = fixture.trajectory
    def timed(state):
        path = original(state)
        path.joint_trajectory.points[-1].time_from_start.sec = duration
        return path
    fixture.trajectory = timed
    fixture.run()
    passed, checks, reason = fixture.outcomes[0]
    assert passed is passes
    descent = next(c for c in checks if c["stage"] == "descent_1")
    assert descent["trajectory_quality"]["duration_s"] == duration
    if not passes:
        assert "contact-zone limit 15.000s" in reason
        assert checks[-1]["stage"] == "descent_1"
        assert len(fixture.goals) == 1  # Transfer only; release and home never planned.


@pytest.mark.parametrize("valid", [False, True])
def test_reverse_placement_requires_branch_and_rechecks_forward_descent(monkeypatch, valid):
    from panda_manipulation import reverse_approach_probe
    fixture = Fixture()
    state = RobotState()
    check = LoadedPathPrecheck(fixture, {"grasp_pitch_offset_deg":-20}, state,
                              lambda *args: fixture.outcomes.append(args))
    check.state = state
    check.place, check.pre_place = pose(.5,-.3,.15), pose(.5,-.3,.27)
    planned = []
    check.plan = lambda *args, **kwargs: planned.append((args, kwargs))
    class Probe:
        def __init__(self, node, candidate, seed, margin, complete, collect_all=False):
            assert candidate["grasp_pose"] == check.place
            assert candidate["pre_grasp_pose"] == check.pre_place
            assert seed is not state
            self.complete = complete
            assert collect_all is True
        def start(self):
            self.complete([dict(reverse_valid=valid, executed=False,
                                pre_grasp_joint_names=[f"panda_joint{i}" for i in range(1,8)],
                                pre_grasp_joint_positions=[.1]*7)])
    monkeypatch.setattr(reverse_approach_probe, "ReverseApproachProbe", Probe)
    check.reverse_place_branch()
    if valid:
        args, kwargs = planned[0]
        assert args[0] == "transfer" and args[2] == check.descent
        assert kwargs["joint_target"] == [.1]*7
        assert not fixture.outcomes
    else:
        assert not planned
        assert fixture.outcomes[0][0] is False


class Fixture:
    def __init__(self, fraction=1.0, plan_error=1):
        self.fraction, self.plan_error = fraction, plan_error
        self.cartesian_client, self.precheck_fk_client = object(), object()
        self.goals, self.cartesian_requests, self.outcomes = [], [], []
        self.precheck_target_dimensions = [0.05, 0.05, 0.08]
        self.group_name, self.end_effector_link = "panda_arm", "panda_hand"
        self.gripper_closed_position, self.gripper_open_position = 0.024, 0.04
        self.place_object_clearance_m, self.pre_place_height_offset = 0.008, 0.12
        self.cartesian_retry_max_step, self.cartesian_min_fraction = 0.005, 0.9
        self.place_transfer_max_joint_path_length = 4.5
        self.place_descent_max_joint_path_length_rad = 1.5
        self.place_descent_max_trajectory_duration_s = 15.0
        self.place_descent_stage_count = 3
        self.release_retreat_max_joint_path_length = 1.5
        self.release_clearance_height = 0.12
        self.use_cartesian_pre_place_transfer = False
        self.allowed_planning_time = 5.0
        self.move_client = NS(server_is_ready=lambda: True, send_goal_async=self.send_goal)
        self.grasp_prevalidation_planner_id = lambda: "RRTConnectkConfigDefault"
        self.make_home_motion_plan_request = MotionPlanRequest
        self.make_planning_options = PlanningOptions
        self.actual_release_hand = pose(0.462, -0.081, 0.15, downward=True)

    def trajectory(self, state):
        names = [f"panda_joint{i}" for i in range(1, 8)]
        q = dict(zip(state.joint_state.name, state.joint_state.position))
        initial = [q[name] for name in names]
        final = list(initial)
        final[0] += 0.02
        return NS(joint_trajectory=NS(joint_names=names,
                                      points=[NS(positions=initial, time_from_start=NS(sec=0, nanosec=0)),
                                              NS(positions=final, time_from_start=NS(sec=1, nanosec=0))]))

    def send_goal(self, goal):
        self.goals.append(deepcopy(goal))
        response = NS(status=4, result=NS(error_code=NS(val=self.plan_error),
                                         planned_trajectory=self.trajectory(goal.request.start_state)))
        return ready(NS(accepted=True, get_result_async=lambda: ready(response),
                        cancel_goal_async=lambda: None))

    def make_motion_plan_request(self, target, state, *args, **kwargs):
        request = MotionPlanRequest()
        request.start_state = state
        return request

    def make_cartesian_request(self, target, step, start_state_override, **kwargs):
        request = GetCartesianPath.Request()
        request.start_state = start_state_override
        return request

    def call_place_precheck_service(self, client, request, success, failure, **kwargs):
        if client is self.cartesian_client:
            self.cartesian_requests.append(deepcopy(request))
            success(NS(fraction=self.fraction, error_code=NS(val=1),
                       solution=self.trajectory(request.start_state)))
        elif client is self.precheck_fk_client:
            success(NS(error_code=NS(val=1),
                       pose_stamped=[self.to_pose_stamped(self.actual_release_hand)]))
        else:
            success(client.call_async(request).result())

    def to_pose_stamped(self, data):
        stamped = PoseStamped()
        stamped.header.frame_id = data["frame_id"]
        for key in ("x", "y", "z"):
            setattr(stamped.pose.position, key, float(data["position"][key]))
        for key in ("x", "y", "z", "w"):
            setattr(stamped.pose.orientation, key, float(data["orientation"][key]))
        return stamped

    def to_pose(self, data):
        return self.to_pose_stamped(data).pose

    def run(self):
        state = RobotState()
        state.joint_state.name = [f"panda_joint{i}" for i in range(1, 8)]
        state.joint_state.position = [0.0] * 7
        candidate = {"place_pose": pose(0.45, -0.08, 0.16, downward=True),
                     "object_pose": pose(0.52, 0.0, 0.04),
                     "object_in_hand_pose": pose(0.01, 0.0, 0.1, "panda_hand", True),
                     "lift_pose": pose(0.52, 0.0, 0.29, downward=True)}
        check = LoadedPathPrecheck(self, candidate, state,
                                  lambda *args: self.outcomes.append(args))
        check.start()
        assert not state.attached_collision_objects
        return candidate


def test_full_nominal_chain_preserves_payload_and_uses_actual_release_fk():
    fixture = Fixture()
    candidate = fixture.run()
    passed, checks, _ = fixture.outcomes[0]
    assert passed
    assert [item["stage"] for item in checks] == ["lift", "transfer", "descent_1", "descent_2", "descent_3", "release_state", "retreat", "home"]
    assert all(item["success"] and not item["executed"] for item in checks)
    assert all(goal.planning_options.plan_only for goal in fixture.goals)
    assert all(request.avoid_collisions for request in fixture.cartesian_requests)
    lift, descent = fixture.cartesian_requests[:2]
    assert lift.start_state.joint_state.position[0] == 0
    assert descent.start_state.joint_state.position[0] == pytest.approx(0.04)
    assert descent.start_state.attached_collision_objects[0].object.operation == CollisionObject.ADD
    assert list(descent.start_state.joint_state.position[-2:]) == [0.024, 0.024]
    expected = compose_pose(pose_from_dict(fixture.actual_release_hand),
                            pose_from_dict(candidate["object_in_hand_pose"]), "panda_link0")
    for goal in fixture.goals[1:]:
        scene = goal.planning_options.planning_scene_diff
        assert scene.is_diff and scene.robot_state.is_diff
        released, = scene.world.collision_objects
        assert released.primitive_poses[0].position.x == pytest.approx(expected.position.x)
        assert released.primitive_poses[0].position.x != candidate["place_pose"]["position"]["x"]
        assert goal.request.start_state.attached_collision_objects[0].object.operation == CollisionObject.REMOVE
        assert list(goal.request.start_state.joint_state.position[-2:]) == [0.04, 0.04]


@pytest.mark.parametrize("fraction", [0.5, float("nan")])
def test_incomplete_lift_never_starts_transfer(fraction):
    fixture = Fixture(fraction=fraction)
    fixture.run()
    assert fixture.outcomes[0][0] is False
    assert not fixture.goals


def test_failed_transfer_never_starts_descent_or_release():
    fixture = Fixture(plan_error=-1)
    fixture.run()
    assert fixture.outcomes[0][0] is False
    assert len(fixture.cartesian_requests) == 1
    assert len(fixture.goals) == 1


def test_cartesian_transfer_profile_is_not_silently_replaced_by_ompl():
    fixture = Fixture()
    fixture.use_cartesian_pre_place_transfer = True
    fixture.run()
    assert fixture.outcomes[0][0] is True
    assert len(fixture.cartesian_requests) == 5
    assert len(fixture.goals) == 2


def test_descent_uses_existing_per_stage_limit_not_aggregate_limit():
    fixture = Fixture()
    fixture.place_descent_max_joint_path_length_rad = 0.03
    fixture.run()
    passed, checks, _ = fixture.outcomes[0]
    descent = [c for c in checks if c["stage"].startswith("descent_")]
    assert passed and len(descent) == 3
    assert all(c["joint_path_length_rad"] < 0.03 for c in descent)
    assert sum(c["joint_path_length_rad"] for c in descent) > 0.03


def test_loaded_path_gate_cannot_fall_back_to_endpoint_only_candidate():
    failures = []
    node = NS(get_parameter=lambda _: NS(value=True),
              grasp_candidate_prevalidation_results=[{
                  "score": 100, "candidate": {}, "feasible": True,
                  "place_endpoint_valid": True, "loaded_path_valid": False,
              }], grasp_candidate_prevalidation_min_joint_limit_margin=0.01,
              finish_failed=failures.append)
    method("select_prevalidated_grasp_candidate")(node)
    assert len(failures) == 1


def test_empty_path_cannot_advance_predicted_state():
    with pytest.raises(ValueError):
        advance_predicted_state(RobotState(), NS(joint_trajectory=NS(points=[], joint_names=[])))
