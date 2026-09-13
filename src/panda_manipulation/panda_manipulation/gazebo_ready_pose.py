"""Send one ready-pose trajectory to the Gazebo Panda arm controller."""

from __future__ import annotations

import time
from typing import List

from .ros_helpers import require_ros2


JOINT_NAMES = [f"panda_joint{index}" for index in range(1, 8)]
DEFAULT_READY_POSITIONS = [0.0, -0.45, 0.0, -2.0, 0.0, 1.65, 0.785]


def main(args: List[str] | None = None) -> None:
    rclpy = require_ros2()
    from control_msgs.action import FollowJointTrajectory
    from builtin_interfaces.msg import Duration
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from trajectory_msgs.msg import JointTrajectoryPoint

    class GazeboReadyPose(Node):
        def __init__(self) -> None:
            super().__init__("gazebo_ready_pose")
            self.declare_parameter(
                "action_name",
                "/panda_arm_controller/follow_joint_trajectory",
            )
            self.declare_parameter("trajectory_duration_s", 4.0)
            self.declare_parameter("server_timeout_s", 30.0)
            self.declare_parameter("ready_positions", DEFAULT_READY_POSITIONS)

            self.done = False
            self.goal_sent = False
            self.started_at = time.monotonic()
            action_name = str(self.get_parameter("action_name").value)
            self.client = ActionClient(self, FollowJointTrajectory, action_name)
            self.timer = self.create_timer(0.5, self.try_send_goal)
            self.get_logger().info(f"Waiting for trajectory controller action {action_name}.")

        def try_send_goal(self) -> None:
            if self.goal_sent or self.done:
                return
            timeout_s = float(self.get_parameter("server_timeout_s").value)
            if time.monotonic() - self.started_at > timeout_s:
                self.get_logger().error("Timed out waiting for the Panda trajectory controller.")
                self.done = True
                return
            if not self.client.server_is_ready():
                return

            positions = [float(value) for value in self.get_parameter("ready_positions").value]
            if len(positions) != len(JOINT_NAMES):
                self.get_logger().error("ready_positions must contain exactly seven values.")
                self.done = True
                return

            duration_s = float(self.get_parameter("trajectory_duration_s").value)
            point = JointTrajectoryPoint()
            point.positions = positions
            seconds = int(duration_s)
            point.time_from_start = Duration(
                sec=seconds,
                nanosec=int((duration_s - seconds) * 1_000_000_000),
            )

            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(JOINT_NAMES)
            goal.trajectory.points = [point]
            self.goal_sent = True
            self.timer.cancel()
            self.get_logger().info("Sending Panda home-to-ready trajectory.")
            self.client.send_goal_async(goal).add_done_callback(self.on_goal_response)

        def on_goal_response(self, future) -> None:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Ready-pose trajectory was rejected.")
                self.done = True
                return
            goal_handle.get_result_async().add_done_callback(self.on_result)

        def on_result(self, future) -> None:
            result = future.result().result
            if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                self.get_logger().info("Panda reached the Gazebo ready pose.")
            else:
                self.get_logger().error(
                    f"Ready-pose execution failed: code={result.error_code}, "
                    f"message={result.error_string}"
                )
            self.done = True

    rclpy.init(args=args)
    node = GazeboReadyPose()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
