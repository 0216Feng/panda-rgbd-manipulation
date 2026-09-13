"""Read-only, request-local nominal pick/place path lookahead."""

from copy import deepcopy
from math import isfinite
from types import SimpleNamespace

from .geometry import compose_pose, pose_from_dict
from .placement_precheck import placement_precheck_request


def advance_predicted_state(state, trajectory):
    """Carry fingers and attached geometry across arm-only trajectory endpoints."""
    path = trajectory.joint_trajectory
    if not path.points or not path.joint_names:
        raise ValueError("planned path has no joint endpoint")
    values = list(path.points[-1].positions)
    if len(path.joint_names) != len(values) or not all(isfinite(v) for v in values):
        raise ValueError("invalid planned joint endpoint")
    advanced = deepcopy(state)
    positions = dict(zip(advanced.joint_state.name, advanced.joint_state.position))
    positions.update(zip(path.joint_names, values))
    advanced.joint_state.name = list(positions)
    advanced.joint_state.position = list(positions.values())
    advanced.joint_state.velocity = []
    advanced.joint_state.effort = []
    advanced.is_diff = True
    return advanced


class LoadedPathPrecheck:
    """Drive existing MoveIt APIs; no execution or global-scene publisher exists here."""

    def __init__(self, node, candidate, contact_state, complete):
        self.node = node
        self.candidate = deepcopy(candidate)
        self.complete = complete
        self.checks = []
        self.finished = False
        self.released_object = None
        self.state = None
        self.contact_state = contact_state

    def finish(self, passed, reason):
        if self.finished:
            return
        self.finished = True
        self.complete(passed, self.checks, reason)

    def start(self):
        n = self.node
        try:
            request = placement_precheck_request(
                self.candidate, self.contact_state, n.precheck_target_dimensions,
                n.group_name, n.end_effector_link, n.gripper_closed_position,
                n.place_object_clearance_m,
            )
            self.state = request.ik_request.robot_state
            goal = request.ik_request.pose_stamped
            self.place = {
                "frame_id": goal.header.frame_id,
                "position": {k: getattr(goal.pose.position, k) for k in ("x", "y", "z")},
                "orientation": {k: getattr(goal.pose.orientation, k) for k in ("x", "y", "z", "w")},
            }
            self.pre_place = deepcopy(self.place)
            self.pre_place["position"]["z"] += max(0.05, n.pre_place_height_offset)
            self.cartesian("lift", self.candidate["lift_pose"], self.transfer)
        except (ValueError, KeyError, TypeError) as exc:
            self.finish(False, f"lookahead input invalid: {exc}")

    def record(self, stage):
        report = {"stage": stage, "executed": False, "success": False,
                  "nominal_payload_attached": self.released_object is None}
        self.checks.append(report)
        return report

    def reject(self, report, reason):
        report["reason"] = reason
        self.finish(False, f"{report['stage']}: {reason}")

    def accept(self, report, trajectory, next_stage, limit=None):
        from .pick_plan_pipeline import joint_path_length
        try:
            rows = [list(point.positions) for point in trajectory.joint_trajectory.points]
            length = joint_path_length(rows)
            if not isfinite(length) or (limit is not None and length > limit):
                self.reject(report, f"path length {length:.3f}rad exceeds limit {limit}")
                return
            self.state = advance_predicted_state(self.state, trajectory)
        except (ValueError, TypeError) as exc:
            self.reject(report, str(exc))
            return
        report.update(success=True, joint_path_length_rad=length)
        next_stage()

    def cartesian(self, stage, target, next_stage):
        n = self.node
        report = self.record(stage)
        is_descent = stage.startswith("descent_")
        request = n.make_cartesian_request(
            target, n.cartesian_retry_max_step, start_state_override=deepcopy(self.state),
            step_name=(f"cartesian_place_descent_stage_{stage.split('_')[-1]}" if is_descent
                       else "cartesian_to_place" if stage == "transfer" else "cartesian_lift"),
        )
        request.avoid_collisions = True

        def result(response):
            fraction = float(response.fraction)
            required = max(0.98, n.cartesian_min_fraction)
            report.update(fraction=fraction, required_fraction=required,
                          moveit_error_code=int(response.error_code.val))
            if response.error_code.val != 1 or not isfinite(fraction) or fraction < required:
                self.reject(report, "collision-checked Cartesian path incomplete")
                return
            if is_descent:
                from .pick_plan_pipeline import (
                    joint_trajectory_quality_metrics, place_descent_trajectory_quality_error,
                )
                trajectory = response.solution.joint_trajectory
                metrics = joint_trajectory_quality_metrics(
                    list(trajectory.joint_names), list(trajectory.points))
                report["trajectory_quality"] = metrics
                quality_error = place_descent_trajectory_quality_error(
                    f"cartesian_place_descent_stage_{stage.split('_')[-1]}", metrics,
                    n.place_descent_max_trajectory_duration_s,
                    n.place_descent_max_joint_path_length_rad)
                if quality_error is not None:
                    self.reject(report, quality_error)
                    return
            self.accept(report, response.solution, next_stage,
                        n.place_descent_max_joint_path_length_rad if is_descent
                        else n.place_transfer_max_joint_path_length if stage == "transfer" else None)

        n.call_place_precheck_service(n.cartesian_client, request, result,
                                     lambda reason: self.reject(report, reason))

    def transfer(self):
        if self.node.use_cartesian_pre_place_transfer:
            self.cartesian("transfer", self.pre_place,
                           self.descent)
            return
        if abs(float(self.candidate.get("grasp_pitch_offset_deg", 0.0))) > 0:
            self.reverse_place_branch()
            return
        self.plan("transfer", self.pre_place, self.descent,
                  self.node.place_transfer_max_joint_path_length)

    def reverse_place_branch(self):
        from .reverse_approach_probe import ReverseApproachProbe
        from .pick_plan_pipeline import normalized_joint_limit_margin
        candidate = deepcopy(self.candidate)
        candidate["grasp_pose"] = deepcopy(self.place)
        candidate["pre_grasp_pose"] = deepcopy(self.pre_place)
        report = self.record("reverse_place_branch")

        def completed(checks):
            report["seed_checks"] = checks
            valid = [check for check in checks if check["reverse_valid"]]
            branch = min(valid, key=self.branch_displacement, default=None)
            if branch is None:
                self.reject(report, "no valid loaded reverse descent branch")
                return
            report["success"] = True
            positions = dict(zip(branch["pre_grasp_joint_names"], branch["pre_grasp_joint_positions"]))
            self.plan("transfer", self.pre_place, self.descent,
                      self.node.place_transfer_max_joint_path_length,
                      joint_target=[positions[f"panda_joint{i}"] for i in range(1, 8)])

        ReverseApproachProbe(self.node, candidate, deepcopy(self.state),
                             normalized_joint_limit_margin, completed,
                             collect_all=True).start()

    def branch_displacement(self, branch):
        from .pick_plan_pipeline import ordered_joint_positions, max_joint_position_delta
        names = list(self.state.joint_state.name)
        current = ordered_joint_positions(names, list(self.state.joint_state.position))
        target = ordered_joint_positions(branch["pre_grasp_joint_names"],
                                         branch["pre_grasp_joint_positions"])
        if current is None or target is None:
            return float("inf")
        delta = max_joint_position_delta(current, target)
        return float("inf") if delta is None else delta

    def descent(self):
        from .pick_plan_pipeline import interpolated_place_descent_poses
        self.descent_stages = interpolated_place_descent_poses(
            self.pre_place, self.place, self.node.place_descent_stage_count
        )
        self.descend_stage(0)

    def descend_stage(self, index):
        if index == len(self.descent_stages):
            self.release()
            return
        self.cartesian(f"descent_{index + 1}", self.descent_stages[index],
                       lambda: self.descend_stage(index + 1))

    def plan(self, stage, target, next_stage, limit=None, joint_target=None):
        from moveit_msgs.action import MoveGroup
        n = self.node
        report = self.record(stage)
        goal = MoveGroup.Goal()
        if joint_target is not None:
            goal.request = n.make_joint_motion_plan_request(
                joint_target, deepcopy(self.state), n.grasp_prevalidation_planner_id(), 0.002)
        elif stage == "home":
            goal.request = n.make_home_motion_plan_request()
            goal.request.start_state = deepcopy(self.state)
        else:
            goal.request = n.make_motion_plan_request(
                n.to_pose_stamped(target), deepcopy(self.state), True,
                n.grasp_prevalidation_planner_id(), orientation_tolerance=0.04,
            )
        goal.planning_options = n.make_planning_options()
        goal.planning_options.plan_only = True
        scene = goal.planning_options.planning_scene_diff
        scene.is_diff = True
        scene.robot_state = deepcopy(self.state)
        scene.robot_state.is_diff = True
        if self.released_object is not None:
            scene.world.collision_objects = [deepcopy(self.released_object)]

        def response(wrapper):
            report["moveit_error_code"] = int(wrapper.result.error_code.val)
            if wrapper.status != 4 or wrapper.result.error_code.val != 1:
                self.reject(report, f"plan-only action failed: status={wrapper.status}")
                return
            self.accept(report, wrapper.result.planned_trajectory, next_stage, limit)

        def accepted(handle):
            if not handle.accepted:
                self.reject(report, "plan-only goal rejected")
                return
            adapter = SimpleNamespace(service_is_ready=lambda: True,
                                      call_async=lambda _: handle.get_result_async())
            def failed(reason):
                handle.cancel_goal_async()
                self.reject(report, reason)
            n.call_place_precheck_service(adapter, None, response, failed,
                                         timeout_s=max(10.0, n.allowed_planning_time + 5.0))

        adapter = SimpleNamespace(service_is_ready=n.move_client.server_is_ready,
                                  call_async=n.move_client.send_goal_async)
        n.call_place_precheck_service(adapter, goal, accepted,
                                     lambda reason: self.reject(report, reason))

    def release(self):
        from moveit_msgs.srv import GetPositionFK
        n = self.node
        report = self.record("release_state")
        request = GetPositionFK.Request()
        request.header.frame_id = self.place["frame_id"]
        request.fk_link_names = [n.end_effector_link]
        request.robot_state = deepcopy(self.state)

        def response(value):
            if value.error_code.val != 1 or len(value.pose_stamped) != 1:
                self.reject(report, "planned release endpoint FK failed")
                return
            stamped = value.pose_stamped[0]
            if stamped.header.frame_id != self.place["frame_id"]:
                self.reject(report, "unexpected release FK frame")
                return
            actual_hand = {
                "frame_id": stamped.header.frame_id,
                "position": {k: getattr(stamped.pose.position, k) for k in ("x", "y", "z")},
                "orientation": {k: getattr(stamped.pose.orientation, k) for k in ("x", "y", "z", "w")},
            }
            report["success"] = True
            self.apply_release(actual_hand)

        n.call_place_precheck_service(n.precheck_fk_client, request, response,
                                     lambda reason: self.reject(report, reason))

    def apply_release(self, actual_hand):
        from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
        n = self.node
        placed = compose_pose(pose_from_dict(actual_hand),
                              pose_from_dict(self.candidate["object_in_hand_pose"]),
                              self.place["frame_id"])
        payload = next(body for body in self.state.attached_collision_objects
                       if body.object.id == "target_cube")
        self.released_object = deepcopy(payload.object)
        self.released_object.header.frame_id = placed.frame_id
        self.released_object.primitive_poses = [n.to_pose(placed.as_dict())]
        self.released_object.operation = CollisionObject.ADD
        remove = AttachedCollisionObject()
        remove.link_name = n.end_effector_link
        remove.object.id = "target_cube"
        remove.object.operation = CollisionObject.REMOVE
        self.state.attached_collision_objects = [
            body for body in self.state.attached_collision_objects if body.object.id != "target_cube"
        ] + [remove]
        for index, name in enumerate(self.state.joint_state.name):
            if name in ("panda_finger_joint1", "panda_finger_joint2"):
                self.state.joint_state.position[index] = n.gripper_open_position
        retreat = deepcopy(self.pre_place)
        retreat["position"]["z"] = max(retreat["position"]["z"],
                                        self.place["position"]["z"] + n.release_clearance_height)
        self.plan("retreat", retreat, self.home, n.release_retreat_max_joint_path_length)

    def home(self):
        self.plan("home", None, lambda: self.finish(True, "nominal loaded path and release recovery checked"))
