"""Plan and execute one joint goal through MoveGroup into Gazebo ros2_control."""

from __future__ import annotations

import json
import time
from typing import List

from .ros_helpers import require_ros2


JOINT_NAMES = [f"panda_joint{index}" for index in range(1, 8)]
DEFAULT_TARGET = [0.0, -0.45, 0.0, -2.0, 0.0, 1.65, 0.785]


def main(args: List[str] | None = None) -> None:
    rclpy = require_ros2()
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from std_msgs.msg import String

    class MoveItGazeboSmokeTest(Node):
        def __init__(self) -> None:
            super().__init__("moveit_gazebo_smoke_test")
            self.declare_parameter("action_name", "/move_action")
            self.declare_parameter("planning_group", "panda_arm")
            self.declare_parameter("planner_id", "RRTConnectkConfigDefault")
            self.declare_parameter("server_timeout_s", 40.0)
            self.declare_parameter("target_positions", DEFAULT_TARGET)

            self.done = False
            self.goal_sent = False
            self.started_at = time.monotonic()
            action_name = str(self.get_parameter("action_name").value)
            self.client = ActionClient(self, MoveGroup, action_name)
            self.publisher = self.create_publisher(String, "/moveit_gazebo_result", 10)
            self.timer = self.create_timer(0.5, self.try_send_goal)
            self.get_logger().info(f"Waiting for MoveGroup action {action_name}.")

        def try_send_goal(self) -> None:
            if self.goal_sent or self.done:
                return
            timeout_s = float(self.get_parameter("server_timeout_s").value)
            if time.monotonic() - self.started_at > timeout_s:
                self.finish(False, "timed out waiting for MoveGroup")
                return
            if not self.client.server_is_ready():
                return

            target = [float(value) for value in self.get_parameter("target_positions").value]
            if len(target) != len(JOINT_NAMES):
                self.finish(False, "target_positions must contain exactly seven values")
                return

            constraints = Constraints()
            constraints.name = "gazebo_smoke_test_goal"
            for joint_name, position in zip(JOINT_NAMES, target):
                joint_constraint = JointConstraint()
                joint_constraint.joint_name = joint_name
                joint_constraint.position = position
                joint_constraint.tolerance_above = 0.01
                joint_constraint.tolerance_below = 0.01
                joint_constraint.weight = 1.0
                constraints.joint_constraints.append(joint_constraint)

            goal = MoveGroup.Goal()
            goal.request.group_name = str(self.get_parameter("planning_group").value)
            goal.request.planner_id = str(self.get_parameter("planner_id").value)
            goal.request.num_planning_attempts = 5
            goal.request.allowed_planning_time = 5.0
            goal.request.max_velocity_scaling_factor = 0.2
            goal.request.max_acceleration_scaling_factor = 0.2
            goal.request.start_state.is_diff = True
            goal.request.goal_constraints = [constraints]
            goal.planning_options.plan_only = False
            goal.planning_options.replan = True
            goal.planning_options.replan_attempts = 2

            self.goal_sent = True
            self.timer.cancel()
            self.get_logger().info("Requesting MoveIt plan+execute into Gazebo.")
            self.client.send_goal_async(goal).add_done_callback(self.on_goal_response)

        def on_goal_response(self, future) -> None:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.finish(False, "MoveGroup rejected the goal")
                return
            goal_handle.get_result_async().add_done_callback(self.on_result)

        def on_result(self, future) -> None:
            wrapped_result = future.result()
            moveit_result = wrapped_result.result
            success = moveit_result.error_code.val == MoveItErrorCodes.SUCCESS
            self.finish(
                success,
                "MoveIt planned and executed through panda_arm_controller"
                if success
                else f"MoveIt error_code={moveit_result.error_code.val}",
            )

        def finish(self, success: bool, reason: str) -> None:
            payload = {
                "final_state": "SUCCESS" if success else "FAILED",
                "backend": "gazebo_ros2_control",
                "controller": "panda_arm_controller",
                "reason": reason,
            }
            message = String()
            message.data = json.dumps(payload)
            self.publisher.publish(message)
            log = self.get_logger().info if success else self.get_logger().error
            log(message.data)
            self.done = True

    rclpy.init(args=args)
    node = MoveItGazeboSmokeTest()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
