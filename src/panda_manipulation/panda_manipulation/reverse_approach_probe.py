"""Read-only search for a loaded grasp-to-pregrasp IK branch."""

from copy import deepcopy
from math import isfinite

from moveit_msgs.srv import GetStateValidity

from .placement_precheck import placement_precheck_request


class ReverseApproachProbe:
    def __init__(self, node, candidate, seed, margin, complete, collect_all=False):
        self.node, self.candidate = node, candidate
        self.seed, self.margin, self.complete = seed, margin, complete
        self.collect_all = bool(collect_all)
        self.reports = []

    def start(self, index=0):
        n = self.node
        if index >= 3:
            self.complete(self.reports)
            return
        state = deepcopy(self.seed if index == 0 else n.fixed_home_state())
        if index:
            sign = 1 if index == 1 else -1
            state.joint_state.position[0] = 0.5 * sign
            state.joint_state.position[2] = 1.5 * sign
            state.joint_state.position[4] = -1.0 * sign
        report = {"seed_index": index, "executed": False, "reverse_valid": False}
        self.reports.append(report)

        def reject(reason):
            report["reason"] = reason
            self.start(index + 1)

        try:
            request = placement_precheck_request(
                self.candidate, state, n.precheck_target_dimensions,
                n.group_name, n.end_effector_link, n.gripper_closed_position,
                n.place_object_clearance_m)
            request.ik_request.pose_stamped = n.to_pose_stamped(self.candidate["grasp_pose"])
        except (ValueError, KeyError, TypeError) as exc:
            reject(str(exc))
            return

        def cartesian(response):
            fraction = float(response.fraction)
            trajectory = response.solution.joint_trajectory
            report["fraction"] = fraction if isfinite(fraction) else None
            report["cartesian_error_code"] = int(response.error_code.val)
            if response.error_code.val != 1 or not isfinite(fraction) or not 0.98 <= fraction <= 1:
                reject("reverse Cartesian path incomplete")
                return
            if not trajectory.points:
                reject("reverse Cartesian path empty")
                return
            margin = min(self.margin(list(trajectory.joint_names), list(p.positions))
                         for p in trajectory.points)
            report["joint_limit_margin"] = margin
            if margin < n.grasp_candidate_prevalidation_min_joint_limit_margin:
                reject("reverse path joint margin insufficient")
                return
            report.update(reverse_valid=True,
                          pre_grasp_joint_names=list(trajectory.joint_names),
                          pre_grasp_joint_positions=list(trajectory.points[-1].positions),
                          reason="reverse branch found; forward global connection remains unverified")
            if self.collect_all:
                self.start(index + 1)
            else:
                self.complete(self.reports)

        def ik(response):
            report["ik_error_code"] = int(response.error_code.val)
            if response.error_code.val != 1:
                reject("loaded grasp IK failed")
                return
            loaded = deepcopy(response.solution)
            loaded.is_diff = True
            loaded.attached_collision_objects = deepcopy(request.ik_request.robot_state.attached_collision_objects)
            # Keep commanded fingers explicit; IK responses may omit passive joints.
            values = dict(zip(loaded.joint_state.name, loaded.joint_state.position))
            values.update(panda_finger_joint1=n.gripper_closed_position,
                          panda_finger_joint2=n.gripper_closed_position)
            loaded.joint_state.name = list(values)
            loaded.joint_state.position = list(values.values())
            loaded.joint_state.velocity = []
            loaded.joint_state.effort = []
            validity = GetStateValidity.Request()
            validity.robot_state, validity.group_name = loaded, n.group_name

            def valid(response):
                if not response.valid:
                    reject("loaded grasp state invalid")
                    return
                path = n.make_cartesian_request(
                    self.candidate["pre_grasp_pose"], n.cartesian_retry_max_step,
                    start_state_override=loaded, step_name="reverse_approach_probe")
                path.waypoints = [n.to_pose(self.candidate["pre_grasp_pose"])]
                path.avoid_collisions = True
                n.call_place_precheck_service(n.cartesian_client, path, cartesian, reject)

            n.call_place_precheck_service(n.diagnostic_validity_client, validity, valid, reject)

        n.call_place_precheck_service(n.diagnostic_ik_client, request, ik, reject)
