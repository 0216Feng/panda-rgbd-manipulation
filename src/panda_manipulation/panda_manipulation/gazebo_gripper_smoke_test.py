"""Verify synchronized dual-finger control in Gazebo."""

from __future__ import annotations

import json
import time
from typing import List

from .ros_helpers import require_ros2


FINGER_JOINTS = ["panda_finger_joint1", "panda_finger_joint2"]


def main(args: List[str] | None = None) -> None:
    rclpy = require_ros2()
    from builtin_interfaces.msg import Duration
    from control_msgs.action import FollowJointTrajectory
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from std_msgs.msg import String
    from trajectory_msgs.msg import JointTrajectoryPoint

    class GazeboGripperSmokeTest(Node):
        def __init__(self) -> None:
            super().__init__("gazebo_gripper_smoke_test")
            self.declare_parameter(
                "action_name",
                "/panda_hand_controller/follow_joint_trajectory",
            )
            self.declare_parameter("closed_position", 0.015)
            self.declare_parameter("open_position", 0.04)
            self.declare_parameter("motion_duration_s", 1.5)
            self.declare_parameter("server_timeout_s", 30.0)
            self.declare_parameter("finger_sync_tolerance", 0.003)

            self.done = False
            self.goal_active = False
            self.current_positions = {}
            self.started_at = time.monotonic()
            action_name = str(self.get_parameter("action_name").value)
            self.client = ActionClient(self, FollowJointTrajectory, action_name)
            self.publisher = self.create_publisher(String, "/gazebo_gripper_result", 10)
            from sensor_msgs.msg import JointState

            self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
            self.timer = self.create_timer(0.5, self.try_start)
            self.pause_timer = None
            self.get_logger().info(f"Waiting for hand controller action {action_name}.")

        def on_joint_state(self, message) -> None:
            self.current_positions.update(zip(message.name, message.position))

        def try_start(self) -> None:
            if self.goal_active or self.done:
                return
            timeout_s = float(self.get_parameter("server_timeout_s").value)
            if time.monotonic() - self.started_at > timeout_s:
                self.finish(False, "timed out waiting for hand controller")
                return
            if not self.client.server_is_ready():
                return
            if not all(joint_name in self.current_positions for joint_name in FINGER_JOINTS):
                return
            self.timer.cancel()
            self.send_position(float(self.get_parameter("closed_position").value), "close")

        def send_position(self, position: float, phase: str) -> None:
            duration_s = float(self.get_parameter("motion_duration_s").value)
            seconds = int(duration_s)
            start_point = JointTrajectoryPoint()
            start_point.positions = [
                float(self.current_positions[joint_name])
                for joint_name in FINGER_JOINTS
            ]
            start_point.time_from_start = Duration(sec=0, nanosec=100_000_000)

            target_point = JointTrajectoryPoint()
            target_point.positions = [position, position]
            target_point.time_from_start = Duration(
                sec=seconds,
                nanosec=int((duration_s - seconds) * 1_000_000_000),
            )
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(FINGER_JOINTS)
            goal.trajectory.points = [start_point, target_point]
            self.goal_active = True
            self.get_logger().info(f"Sending synchronized gripper {phase} command: {position:.3f} m.")
            self.client.send_goal_async(goal).add_done_callback(
                lambda future: self.on_goal_response(future, phase)
            )

        def on_goal_response(self, future, phase: str) -> None:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.finish(False, f"hand controller rejected {phase} goal")
                return
            goal_handle.get_result_async().add_done_callback(
                lambda future: self.on_result(future, phase)
            )

        def on_result(self, future, phase: str) -> None:
            result = future.result().result
            self.goal_active = False
            if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                self.finish(
                    False,
                    f"{phase} failed: code={result.error_code}, message={result.error_string}",
                )
                return
            first = float(self.current_positions[FINGER_JOINTS[0]])
            second = float(self.current_positions[FINGER_JOINTS[1]])
            sync_error = abs(first - second)
            tolerance = float(self.get_parameter("finger_sync_tolerance").value)
            if sync_error > tolerance:
                self.finish(
                    False,
                    f"{phase} dual-finger synchronization failed: "
                    f"finger1={first:.4f}, finger2={second:.4f}, "
                    f"delta={sync_error:.4f}",
                )
                return
            if phase == "close":
                self.pause_timer = self.create_timer(1.0, self.send_open_once)
                return
            self.finish(True, "both physical fingers closed and opened synchronously")

        def send_open_once(self) -> None:
            if self.pause_timer is not None:
                self.pause_timer.cancel()
                self.pause_timer = None
            self.send_position(float(self.get_parameter("open_position").value), "open")

        def finish(self, success: bool, reason: str) -> None:
            payload = {
                "final_state": "SUCCESS" if success else "FAILED",
                "controller": "panda_hand_controller",
                "reason": reason,
            }
            message = String()
            message.data = json.dumps(payload)
            self.publisher.publish(message)
            log = self.get_logger().info if success else self.get_logger().error
            log(message.data)
            self.done = True

    rclpy.init(args=args)
    node = GazeboGripperSmokeTest()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
