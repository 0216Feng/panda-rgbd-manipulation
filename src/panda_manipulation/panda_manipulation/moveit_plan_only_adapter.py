"""Plan-only adapter that sends grasp candidate poses to MoveIt2."""

from __future__ import annotations

import json
from typing import Dict, List, Optional

from .ros_helpers import require_ros2


def selected_candidate(payload: Dict[str, object]) -> Optional[Dict[str, object]]:
    selected_id = payload.get("selected_candidate_id")
    for candidate in payload.get("candidates", []):
        if candidate.get("candidate_id") == selected_id:
            return candidate
    return None


def main(args: List[str] | None = None) -> None:
    rclpy = require_ros2()
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import Constraints, MotionPlanRequest, OrientationConstraint, PlanningOptions
    from moveit_msgs.msg import PositionConstraint
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from shape_msgs.msg import SolidPrimitive
    from std_msgs.msg import String

    class MoveItPlanOnlyAdapter(Node):
        def __init__(self) -> None:
            super().__init__("moveit_plan_only_adapter")
            self.declare_parameter("action_name", "/move_action")
            self.declare_parameter("planning_group", "panda_arm")
            self.declare_parameter("end_effector_link", "panda_hand")
            self.declare_parameter("planner_id", "RRTConnectkConfigDefault")
            self.declare_parameter("allowed_planning_time", 5.0)
            self.declare_parameter("planning_attempts", 10)
            self.declare_parameter("position_tolerance", 0.04)
            self.declare_parameter("use_orientation_constraint", False)
            self.declare_parameter("orientation_tolerance", 1.57)
            self.declare_parameter(
                "plan_sequence",
                ["pre_grasp_pose", "lift_pose", "place_pose"],
            )
            self.declare_parameter("plan_once", True)

            self.action_name = str(self.get_parameter("action_name").value)
            self.group_name = str(self.get_parameter("planning_group").value)
            self.end_effector_link = str(self.get_parameter("end_effector_link").value)
            self.planner_id = str(self.get_parameter("planner_id").value)
            self.allowed_planning_time = float(self.get_parameter("allowed_planning_time").value)
            self.planning_attempts = int(self.get_parameter("planning_attempts").value)
            self.position_tolerance = float(self.get_parameter("position_tolerance").value)
            self.use_orientation_constraint = bool(self.get_parameter("use_orientation_constraint").value)
            self.orientation_tolerance = float(self.get_parameter("orientation_tolerance").value)
            self.plan_sequence = list(self.get_parameter("plan_sequence").value)
            self.plan_once = bool(self.get_parameter("plan_once").value)

            self.client = ActionClient(self, MoveGroup, self.action_name)
            result_qos = QoSProfile(depth=10)
            result_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.result_publisher = self.create_publisher(String, "/plan_only_result", result_qos)
            self.create_subscription(String, "/grasp_candidates", self.on_grasp_candidates, 10)
            self.watchdog_timer = self.create_timer(1.0, self.check_watchdog)
            self.has_started = False
            self.active = False
            self.active_pose_key: Optional[str] = None
            self.active_started_ns: Optional[int] = None
            self.current_candidate: Optional[Dict[str, object]] = None
            self.current_step_index = 0
            self.results: List[Dict[str, object]] = []

            self.get_logger().info(
                "MoveIt plan-only adapter waiting for /grasp_candidates "
                f"and action server {self.action_name}."
            )

        def on_grasp_candidates(self, message: String) -> None:
            if self.plan_once and self.has_started:
                return
            if self.active:
                return

            payload = json.loads(message.data)
            candidate = selected_candidate(payload)
            if candidate is None:
                self.publish_summary("FAILED", "no selected candidate")
                return

            if not self.client.wait_for_server(timeout_sec=1.0):
                reason = f"MoveGroup action server {self.action_name} is not available yet"
                self.get_logger().warn(reason)
                self.publish_summary("WAITING", reason)
                return

            self.has_started = True
            self.active = True
            self.current_candidate = candidate
            self.current_step_index = 0
            self.results = []
            self.send_current_step()

        def send_current_step(self) -> None:
            if self.current_candidate is None:
                self.publish_summary("FAILED", "missing candidate")
                return
            if self.current_step_index >= len(self.plan_sequence):
                self.publish_summary("SUCCESS", "")
                return

            pose_key = self.plan_sequence[self.current_step_index]
            pose_data = self.current_candidate.get(pose_key)
            if pose_data is None:
                self.record_step(pose_key, False, "missing pose")
                self.publish_summary("FAILED", f"{pose_key} missing")
                return

            goal = MoveGroup.Goal()
            goal.request = self.make_motion_plan_request(self.to_pose_stamped(pose_data))
            goal.planning_options = self.make_planning_options()

            self.get_logger().info(f"Planning {pose_key} with {self.planner_id}.")
            self.active_pose_key = pose_key
            self.active_started_ns = self.get_clock().now().nanoseconds
            future = self.client.send_goal_async(goal)
            future.add_done_callback(lambda done_future, key=pose_key: self.on_goal_response(done_future, key))

        def check_watchdog(self) -> None:
            if not self.active or self.active_started_ns is None or self.active_pose_key is None:
                return
            elapsed_s = (self.get_clock().now().nanoseconds - self.active_started_ns) / 1_000_000_000.0
            timeout_s = self.allowed_planning_time + 8.0
            if elapsed_s <= timeout_s:
                return
            reason = f"{self.active_pose_key} timed out after {elapsed_s:.1f}s"
            self.record_step(self.active_pose_key, False, reason)
            self.publish_summary("FAILED", reason)

        def on_goal_response(self, future, pose_key: str) -> None:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.record_step(pose_key, False, "goal rejected")
                self.publish_summary("FAILED", f"{pose_key} goal rejected")
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda done_future, key=pose_key: self.on_plan_result(done_future, key))

        def on_plan_result(self, future, pose_key: str) -> None:
            result_wrapper = future.result()
            action_status_ok = result_wrapper.status == GoalStatus.STATUS_SUCCEEDED
            error_code = result_wrapper.result.error_code.val
            success = action_status_ok and error_code == 1
            reason = "" if success else f"action_status={result_wrapper.status}, moveit_error_code={error_code}"
            self.record_step(pose_key, success, reason)
            self.active_pose_key = None
            self.active_started_ns = None

            if not success:
                self.publish_summary("FAILED", f"{pose_key}: {reason}")
                return

            self.current_step_index += 1
            self.send_current_step()

        def make_motion_plan_request(self, target_pose: PoseStamped) -> MotionPlanRequest:
            request = MotionPlanRequest()
            request.group_name = self.group_name
            request.start_state.is_diff = True
            request.num_planning_attempts = self.planning_attempts
            request.allowed_planning_time = self.allowed_planning_time
            request.max_velocity_scaling_factor = 0.1
            request.max_acceleration_scaling_factor = 0.1
            request.planner_id = self.planner_id
            request.goal_constraints = [self.make_pose_constraint(target_pose)]
            return request

        def make_pose_constraint(self, target_pose: PoseStamped) -> Constraints:
            constraints = Constraints()
            constraints.name = f"{self.end_effector_link}_pose_goal"

            sphere = SolidPrimitive()
            sphere.type = SolidPrimitive.SPHERE
            sphere.dimensions = [self.position_tolerance]

            position_constraint = PositionConstraint()
            position_constraint.header = target_pose.header
            position_constraint.link_name = self.end_effector_link
            position_constraint.constraint_region.primitives.append(sphere)
            position_constraint.constraint_region.primitive_poses.append(target_pose.pose)
            position_constraint.weight = 1.0

            orientation_constraint = OrientationConstraint()
            orientation_constraint.header = target_pose.header
            orientation_constraint.link_name = self.end_effector_link
            orientation_constraint.orientation = target_pose.pose.orientation
            orientation_constraint.absolute_x_axis_tolerance = self.orientation_tolerance
            orientation_constraint.absolute_y_axis_tolerance = self.orientation_tolerance
            orientation_constraint.absolute_z_axis_tolerance = self.orientation_tolerance
            orientation_constraint.weight = 1.0

            constraints.position_constraints.append(position_constraint)
            if self.use_orientation_constraint:
                constraints.orientation_constraints.append(orientation_constraint)
            return constraints

        def make_planning_options(self) -> PlanningOptions:
            options = PlanningOptions()
            options.plan_only = True
            options.look_around = False
            options.replan = False
            return options

        def to_pose_stamped(self, pose_data: Dict[str, object]) -> PoseStamped:
            message = PoseStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = str(pose_data["frame_id"])
            position = pose_data["position"]
            orientation = pose_data["orientation"]
            message.pose.position.x = float(position["x"])
            message.pose.position.y = float(position["y"])
            message.pose.position.z = float(position["z"])
            message.pose.orientation.x = float(orientation["x"])
            message.pose.orientation.y = float(orientation["y"])
            message.pose.orientation.z = float(orientation["z"])
            message.pose.orientation.w = float(orientation["w"])
            return message

        def record_step(self, pose_key: str, success: bool, reason: str) -> None:
            self.results.append({"pose_key": pose_key, "success": success, "reason": reason})

        def publish_summary(self, final_state: str, reason: str) -> None:
            self.active = False
            output = String()
            output.data = json.dumps(
                {
                    "final_state": final_state,
                    "planner_id": self.planner_id,
                    "planning_group": self.group_name,
                    "end_effector_link": self.end_effector_link,
                    "position_tolerance": self.position_tolerance,
                    "use_orientation_constraint": self.use_orientation_constraint,
                    "steps": self.results,
                    "failure_reason": reason or None,
                }
            )
            self.result_publisher.publish(output)
            self.get_logger().info(output.data)

    rclpy.init(args=args)
    node = MoveItPlanOnlyAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
