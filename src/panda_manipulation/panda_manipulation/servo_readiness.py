"""Exercise MoveIt Servo against Gazebo with a bounded lift-and-return motion."""

from __future__ import annotations

import json
import time
from math import hypot
from typing import Dict, List, Optional, Sequence

from .ros_helpers import require_ros2


SERVO_HALT_CODES = {-1, 2, 5, 6}


def servo_motion_quality(
    initial_position: Sequence[float],
    peak_position: Sequence[float],
    final_position: Sequence[float],
    minimum_lift_m: float,
    return_tolerance_m: float,
    horizontal_tolerance_m: float,
) -> Dict[str, object]:
    """Evaluate the measured lift, return error, and lateral drift."""
    lift_m = float(peak_position[2]) - float(initial_position[2])
    return_error_m = sum(
        (float(final) - float(initial)) ** 2
        for initial, final in zip(initial_position, final_position)
    ) ** 0.5
    horizontal_drift_m = hypot(
        float(peak_position[0]) - float(initial_position[0]),
        float(peak_position[1]) - float(initial_position[1]),
    )
    failures = []
    if lift_m < minimum_lift_m:
        failures.append(
            f"lift={lift_m:.4f}m below required {minimum_lift_m:.4f}m"
        )
    if return_error_m > return_tolerance_m:
        failures.append(
            f"return_error={return_error_m:.4f}m exceeds {return_tolerance_m:.4f}m"
        )
    if horizontal_drift_m > horizontal_tolerance_m:
        failures.append(
            f"horizontal_drift={horizontal_drift_m:.4f}m exceeds "
            f"{horizontal_tolerance_m:.4f}m"
        )
    return {
        "success": not failures,
        "lift_m": lift_m,
        "return_error_m": return_error_m,
        "horizontal_drift_m": horizontal_drift_m,
        "reason": "; ".join(failures) if failures else "bounded lift and return verified",
    }


def main(args: List[str] | None = None) -> None:
    rclpy = require_ros2()
    from geometry_msgs.msg import TwistStamped
    from moveit_msgs.msg import ServoStatus
    from moveit_msgs.srv import ServoCommandType
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    from tf2_ros import Buffer, TransformException, TransformListener

    class ServoReadinessTest(Node):
        def __init__(self) -> None:
            super().__init__("servo_readiness_test")
            self.declare_parameter("command_topic", "/servo_node/delta_twist_cmds")
            self.declare_parameter("status_topic", "/servo_node/status")
            self.declare_parameter(
                "switch_service", "/servo_node/switch_command_type"
            )
            self.declare_parameter("pause_service", "/servo_node/pause_servo")
            self.declare_parameter("planning_frame", "panda_link0")
            self.declare_parameter("end_effector_frame", "panda_hand")
            self.declare_parameter("linear_speed_mps", 0.02)
            self.declare_parameter("motion_duration_s", 1.5)
            self.declare_parameter("settle_duration_s", 0.5)
            self.declare_parameter("startup_timeout_s", 45.0)
            self.declare_parameter("minimum_lift_m", 0.015)
            self.declare_parameter("return_tolerance_m", 0.015)
            self.declare_parameter("horizontal_tolerance_m", 0.015)

            self.done = False
            self.state = "WAITING"
            self.started_at = time.monotonic()
            self.phase_started_at = self.started_at
            self.initial_position: Optional[List[float]] = None
            self.peak_position: Optional[List[float]] = None
            self.final_position: Optional[List[float]] = None
            self.status_history: List[Dict[str, object]] = []
            self.halt_status: Optional[Dict[str, object]] = None

            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.twist_publisher = self.create_publisher(
                TwistStamped,
                str(self.get_parameter("command_topic").value),
                10,
            )
            self.switch_client = self.create_client(
                ServoCommandType,
                str(self.get_parameter("switch_service").value),
            )
            self.pause_client = self.create_client(
                SetBool,
                str(self.get_parameter("pause_service").value),
            )
            self.create_subscription(
                ServoStatus,
                str(self.get_parameter("status_topic").value),
                self.on_status,
                10,
            )
            durable_qos = QoSProfile(depth=1)
            durable_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.result_publisher = self.create_publisher(
                String,
                "/servo_readiness_result",
                durable_qos,
            )
            self.timer = self.create_timer(0.02, self.tick)
            self.get_logger().info("Waiting for MoveIt Servo services and Panda TF.")

        def measured_position(self) -> Optional[List[float]]:
            try:
                transform = self.tf_buffer.lookup_transform(
                    str(self.get_parameter("planning_frame").value),
                    str(self.get_parameter("end_effector_frame").value),
                    rclpy.time.Time(),
                )
            except TransformException:
                return None
            translation = transform.transform.translation
            return [float(translation.x), float(translation.y), float(translation.z)]

        def on_status(self, message: ServoStatus) -> None:
            item = {"code": int(message.code), "message": str(message.message)}
            if not self.status_history or item != self.status_history[-1]:
                self.status_history.append(item)
            if int(message.code) in SERVO_HALT_CODES:
                self.halt_status = item

        def call_switch(self) -> None:
            request = ServoCommandType.Request()
            request.command_type = ServoCommandType.Request.TWIST
            self.state = "SWITCHING"
            self.switch_client.call_async(request).add_done_callback(
                self.on_switched
            )

        def on_switched(self, future) -> None:
            try:
                response = future.result()
            except Exception as exc:  # pragma: no cover - ROS transport failure
                self.finish(False, f"switch command service failed: {exc}")
                return
            if not response.success:
                self.finish(False, "Servo rejected TWIST command mode")
                return
            request = SetBool.Request()
            request.data = False
            self.state = "UNPAUSING"
            self.pause_client.call_async(request).add_done_callback(
                self.on_unpaused
            )

        def on_unpaused(self, future) -> None:
            try:
                response = future.result()
            except Exception as exc:  # pragma: no cover - ROS transport failure
                self.finish(False, f"unpause service failed: {exc}")
                return
            if not response.success:
                self.finish(False, f"Servo unpause rejected: {response.message}")
                return
            self.state = "LIFT"
            self.phase_started_at = time.monotonic()

        def publish_velocity(self, z_velocity: float) -> None:
            message = TwistStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = str(
                self.get_parameter("planning_frame").value
            )
            message.twist.linear.z = float(z_velocity)
            self.twist_publisher.publish(message)

        def tick(self) -> None:
            if self.done:
                return
            now = time.monotonic()
            position = self.measured_position()
            if position is not None:
                if self.initial_position is None:
                    self.initial_position = list(position)
                    self.peak_position = list(position)
                elif self.peak_position is not None and position[2] > self.peak_position[2]:
                    self.peak_position = list(position)

            if now - self.started_at > float(
                self.get_parameter("startup_timeout_s").value
            ) and self.state in {"WAITING", "SWITCHING", "UNPAUSING"}:
                self.finish(False, f"timed out in state {self.state}")
                return

            if self.state == "WAITING":
                if (
                    self.initial_position is not None
                    and self.switch_client.service_is_ready()
                    and self.pause_client.service_is_ready()
                ):
                    self.call_switch()
                return

            speed = float(self.get_parameter("linear_speed_mps").value)
            motion_duration = float(
                self.get_parameter("motion_duration_s").value
            )
            settle_duration = float(
                self.get_parameter("settle_duration_s").value
            )
            elapsed = now - self.phase_started_at
            if self.state == "LIFT":
                if elapsed < motion_duration:
                    self.publish_velocity(speed)
                else:
                    self.publish_velocity(0.0)
                    self.state = "TOP_SETTLE"
                    self.phase_started_at = now
            elif self.state == "TOP_SETTLE":
                self.publish_velocity(0.0)
                if elapsed >= settle_duration:
                    self.state = "RETURN"
                    self.phase_started_at = now
            elif self.state == "RETURN":
                if elapsed < motion_duration:
                    self.publish_velocity(-speed)
                else:
                    self.publish_velocity(0.0)
                    self.state = "FINAL_SETTLE"
                    self.phase_started_at = now
            elif self.state == "FINAL_SETTLE":
                self.publish_velocity(0.0)
                if elapsed >= settle_duration:
                    self.final_position = position
                    request = SetBool.Request()
                    request.data = True
                    self.state = "PAUSING"
                    self.pause_client.call_async(request).add_done_callback(
                        self.on_paused
                    )

        def on_paused(self, future) -> None:
            try:
                response = future.result()
                paused = bool(response.success)
                message = str(response.message)
            except Exception as exc:  # pragma: no cover - ROS transport failure
                paused = False
                message = str(exc)
            self.finish(paused, "Servo paused after test" if paused else message)

        def finish(self, service_success: bool, reason: str) -> None:
            quality = None
            if (
                self.initial_position is not None
                and self.peak_position is not None
                and self.final_position is not None
            ):
                quality = servo_motion_quality(
                    self.initial_position,
                    self.peak_position,
                    self.final_position,
                    float(self.get_parameter("minimum_lift_m").value),
                    float(self.get_parameter("return_tolerance_m").value),
                    float(self.get_parameter("horizontal_tolerance_m").value),
                )
            success = (
                service_success
                and quality is not None
                and bool(quality["success"])
                and self.halt_status is None
            )
            failure_reason = reason
            if quality is not None and not quality["success"]:
                failure_reason = str(quality["reason"])
            if self.halt_status is not None:
                failure_reason = (
                    f"Servo halt code={self.halt_status['code']}: "
                    f"{self.halt_status['message']}"
                )
            payload = {
                "final_state": "SUCCESS" if success else "FAILED",
                "backend": "moveit_servo",
                "initial_position": self.initial_position,
                "peak_position": self.peak_position,
                "final_position": self.final_position,
                "linear_speed_mps": float(
                    self.get_parameter("linear_speed_mps").value
                ),
                "motion_duration_s": float(
                    self.get_parameter("motion_duration_s").value
                ),
                "servo_status_history": self.status_history,
                "halt_status": self.halt_status,
                "quality": quality,
                "reason": quality["reason"] if success else failure_reason,
                "elapsed_s": time.monotonic() - self.started_at,
            }
            message = String()
            message.data = json.dumps(payload)
            self.result_publisher.publish(message)
            log = self.get_logger().info if success else self.get_logger().error
            log(message.data)
            self.done = True

    rclpy.init(args=args)
    node = ServoReadinessTest()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
