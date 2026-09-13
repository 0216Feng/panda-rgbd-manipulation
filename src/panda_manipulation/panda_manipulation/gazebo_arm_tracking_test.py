"""Measure low-speed Panda arm tracking through Gazebo ros2_control."""

from __future__ import annotations

import json
import time
from typing import Dict, List, Optional

from .ros_helpers import require_ros2
from .transfer_diagnostics import controller_point_values


JOINT_NAMES = [f"panda_joint{index}" for index in range(1, 8)]
DEFAULT_TARGET_DELTAS = [0.06, -0.04, 0.05, 0.03, -0.08, 0.06, 0.04]


def ordered_positions(names: List[str], positions: List[float]) -> Optional[List[float]]:
    """Return Panda arm positions in controller order when all joints exist."""
    by_name = {str(name): float(value) for name, value in zip(names, positions)}
    if any(name not in by_name for name in JOINT_NAMES):
        return None
    return [by_name[name] for name in JOINT_NAMES]


def main(args: List[str] | None = None) -> None:
    rclpy = require_ros2()
    from builtin_interfaces.msg import Duration
    from control_msgs.action import FollowJointTrajectory
    from control_msgs.msg import JointTrajectoryControllerState
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from trajectory_msgs.msg import JointTrajectoryPoint

    class GazeboArmTrackingTest(Node):
        def __init__(self) -> None:
            super().__init__("gazebo_arm_tracking_test")
            self.declare_parameter(
                "action_name",
                "/panda_arm_controller/follow_joint_trajectory",
            )
            self.declare_parameter("trajectory_duration_s", 10.0)
            self.declare_parameter("server_timeout_s", 30.0)
            self.declare_parameter("terminal_tolerance_rad", 0.01)
            self.declare_parameter("target_deltas", DEFAULT_TARGET_DELTAS)

            self.done = False
            self.goal_sent = False
            self.started_at = time.monotonic()
            self.goal_started_at: Optional[float] = None
            self.latest_positions: Optional[List[float]] = None
            self.start_positions: Optional[List[float]] = None
            self.target_positions: Optional[List[float]] = None
            self.max_tracking_errors: Dict[str, float] = {
                name: 0.0 for name in JOINT_NAMES
            }

            action_name = str(self.get_parameter("action_name").value)
            self.client = ActionClient(self, FollowJointTrajectory, action_name)
            self.create_subscription(
                JointState,
                "/joint_states",
                self.on_joint_state,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                JointTrajectoryControllerState,
                "/panda_arm_controller/controller_state",
                self.on_controller_state,
                qos_profile_sensor_data,
            )
            durable_qos = QoSProfile(depth=1)
            durable_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.publisher = self.create_publisher(
                String,
                "/gazebo_arm_tracking_result",
                durable_qos,
            )
            self.timer = self.create_timer(0.25, self.try_send_goal)
            self.get_logger().info(
                f"Waiting for trajectory controller action {action_name}."
            )

        def on_joint_state(self, message: JointState) -> None:
            self.latest_positions = ordered_positions(
                list(message.name),
                list(message.position),
            )

        def on_controller_state(self, message) -> None:
            if not self.goal_sent or self.done:
                return
            errors = controller_point_values(message, "error")
            for joint_name in JOINT_NAMES:
                value = errors.get(joint_name)
                if value is not None:
                    self.max_tracking_errors[joint_name] = max(
                        self.max_tracking_errors[joint_name],
                        abs(float(value)),
                    )

        def try_send_goal(self) -> None:
            if self.goal_sent or self.done:
                return
            timeout_s = float(self.get_parameter("server_timeout_s").value)
            if time.monotonic() - self.started_at > timeout_s:
                self.finish(False, None, "timed out waiting for controller/joints")
                return
            if not self.client.server_is_ready() or self.latest_positions is None:
                return

            deltas = [
                float(value)
                for value in self.get_parameter("target_deltas").value
            ]
            if len(deltas) != len(JOINT_NAMES):
                self.finish(False, None, "target_deltas must contain seven values")
                return
            duration_s = float(self.get_parameter("trajectory_duration_s").value)
            if duration_s <= 0.0:
                self.finish(False, None, "trajectory_duration_s must be positive")
                return

            self.start_positions = list(self.latest_positions)
            self.target_positions = [
                start + delta for start, delta in zip(self.start_positions, deltas)
            ]
            point = JointTrajectoryPoint()
            point.positions = list(self.target_positions)
            seconds = int(duration_s)
            point.time_from_start = Duration(
                sec=seconds,
                nanosec=int((duration_s - seconds) * 1_000_000_000),
            )
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(JOINT_NAMES)
            goal.trajectory.points = [point]

            self.goal_sent = True
            self.goal_started_at = time.monotonic()
            self.timer.cancel()
            self.get_logger().info(
                f"Sending {duration_s:.1f}s low-speed tracking trajectory."
            )
            self.client.send_goal_async(goal).add_done_callback(
                self.on_goal_response
            )

        def on_goal_response(self, future) -> None:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.finish(False, None, "trajectory goal rejected")
                return
            goal_handle.get_result_async().add_done_callback(self.on_result)

        def on_result(self, future) -> None:
            wrapped = future.result()
            result = wrapped.result
            success = (
                result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
            )
            reason = (
                "controller reached the low-speed joint target"
                if success
                else (
                    f"action_status={wrapped.status}, error_code="
                    f"{result.error_code}, message={result.error_string}"
                )
            )
            self.finish(success, int(wrapped.status), reason)

        def finish(
            self,
            action_success: bool,
            action_status: Optional[int],
            reason: str,
        ) -> None:
            final_positions = self.latest_positions
            final_errors = None
            max_final_error = None
            if final_positions is not None and self.target_positions is not None:
                final_errors = [
                    target - actual
                    for target, actual in zip(self.target_positions, final_positions)
                ]
                max_final_error = max(abs(value) for value in final_errors)
            tolerance = float(
                self.get_parameter("terminal_tolerance_rad").value
            )
            success = (
                action_success
                and max_final_error is not None
                and max_final_error <= tolerance
            )
            payload = {
                "final_state": "SUCCESS" if success else "FAILED",
                "controller": "panda_arm_controller",
                "trajectory_duration_s": float(
                    self.get_parameter("trajectory_duration_s").value
                ),
                "action_status": action_status,
                "reason": reason,
                "start_positions_rad": self.start_positions,
                "target_positions_rad": self.target_positions,
                "final_positions_rad": final_positions,
                "final_errors_rad": final_errors,
                "max_final_error_rad": max_final_error,
                "terminal_tolerance_rad": tolerance,
                "max_tracking_errors_rad": self.max_tracking_errors,
                "elapsed_s": (
                    time.monotonic() - self.goal_started_at
                    if self.goal_started_at is not None
                    else time.monotonic() - self.started_at
                ),
            }
            message = String()
            message.data = json.dumps(payload)
            self.publisher.publish(message)
            log = self.get_logger().info if success else self.get_logger().error
            log(message.data)
            self.done = True

    rclpy.init(args=args)
    node = GazeboArmTrackingTest()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.25)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
