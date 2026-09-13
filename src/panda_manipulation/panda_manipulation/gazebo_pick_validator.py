"""Validate Gazebo pick success from the target model's measured pose."""

from __future__ import annotations

import json
from math import acos, degrees, sqrt
from typing import List, Optional, Tuple

from .ros_helpers import require_ros2


def upright_tilt_degrees(quaternion: Tuple[float, float, float, float]) -> float:
    """Return the angle between the object's local and world Z axes."""
    x, y, z, w = quaternion
    norm = sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-9:
        return 180.0
    x /= norm
    y /= norm
    z_alignment = 1.0 - 2.0 * (x * x + y * y)
    return degrees(acos(max(-1.0, min(1.0, z_alignment))))


def main(args: List[str] | None = None) -> None:
    rclpy = require_ros2()
    from geometry_msgs.msg import Pose
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from std_msgs.msg import String

    class GazeboPickValidator(Node):
        def __init__(self) -> None:
            super().__init__("gazebo_pick_validator")
            self.declare_parameter("target_pose_topic", "/gazebo/target_pose")
            self.declare_parameter("minimum_lift_m", 0.05)
            self.declare_parameter("place_x", 0.45)
            self.declare_parameter("place_y", -0.16)
            self.declare_parameter("place_z_world", 0.76)
            self.declare_parameter("place_tolerance_m", 0.08)
            self.declare_parameter("maximum_tilt_deg", 20.0)
            self.declare_parameter("settle_time_s", 2.0)

            self.initial_pose: Optional[Tuple[float, float, float]] = None
            self.latest_pose: Optional[Tuple[float, float, float]] = None
            self.latest_orientation: Optional[Tuple[float, float, float, float]] = None
            self.max_z: Optional[float] = None
            self.pipeline_result = None
            self.validation_published = False
            self.settle_timer = None

            result_qos = QoSProfile(depth=1)
            result_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.publisher = self.create_publisher(
                String,
                "/gazebo_pick_validation",
                result_qos,
            )
            self.create_subscription(
                Pose,
                str(self.get_parameter("target_pose_topic").value),
                self.on_target_pose,
                qos_profile_sensor_data,
            )
            self.create_subscription(String, "/pick_plan_result", self.on_pick_result, 10)
            self.get_logger().info("Waiting for Gazebo target pose and terminal pick result.")

        def on_target_pose(self, message: Pose) -> None:
            pose = (
                float(message.position.x),
                float(message.position.y),
                float(message.position.z),
            )
            if self.initial_pose is None:
                self.initial_pose = pose
                self.max_z = pose[2]
                self.get_logger().info(
                    f"Captured initial target pose: x={pose[0]:.3f}, "
                    f"y={pose[1]:.3f}, z={pose[2]:.3f}."
                )
            self.latest_pose = pose
            self.latest_orientation = (
                float(message.orientation.x),
                float(message.orientation.y),
                float(message.orientation.z),
                float(message.orientation.w),
            )
            self.max_z = pose[2] if self.max_z is None else max(self.max_z, pose[2])

        def on_pick_result(self, message: String) -> None:
            try:
                result = json.loads(message.data)
            except json.JSONDecodeError:
                return
            if result.get("final_state") not in {"SUCCESS", "FAILED"}:
                return
            self.pipeline_result = result
            if self.settle_timer is not None:
                self.settle_timer.cancel()
            self.settle_timer = self.create_timer(
                float(self.get_parameter("settle_time_s").value),
                self.evaluate_once,
            )

        def evaluate_once(self) -> None:
            if self.settle_timer is not None:
                self.settle_timer.cancel()
                self.settle_timer = None
            if self.validation_published:
                return
            self.validation_published = True

            if (
                self.initial_pose is None
                or self.latest_pose is None
                or self.latest_orientation is None
                or self.max_z is None
            ):
                self.publish_validation(
                    False, "Gazebo target pose was unavailable", None, None, None
                )
                return
            if self.pipeline_result is None or self.pipeline_result.get("final_state") != "SUCCESS":
                failure_reason = (
                    self.pipeline_result.get("failure_reason")
                    if self.pipeline_result is not None
                    else None
                )
                reason = "pick pipeline did not succeed"
                if failure_reason:
                    reason = f"pick pipeline failed: {failure_reason}"
                self.publish_validation(False, reason, None, None, None)
                return

            lift_delta = self.max_z - self.initial_pose[2]
            expected_place = (
                float(self.get_parameter("place_x").value),
                float(self.get_parameter("place_y").value),
                float(self.get_parameter("place_z_world").value),
            )
            place_error = sqrt(
                sum(
                    (actual - expected) ** 2
                    for actual, expected in zip(self.latest_pose, expected_place)
                )
            )
            lift_ok = lift_delta >= float(self.get_parameter("minimum_lift_m").value)
            place_ok = place_error <= float(self.get_parameter("place_tolerance_m").value)
            final_tilt_deg = upright_tilt_degrees(self.latest_orientation)
            upright_ok = final_tilt_deg <= float(
                self.get_parameter("maximum_tilt_deg").value
            )
            reasons = []
            if not lift_ok:
                reasons.append(f"lift_delta={lift_delta:.3f}m below threshold")
            if not place_ok:
                reasons.append(f"place_error={place_error:.3f}m above tolerance")
            if not upright_ok:
                reasons.append(f"final_tilt={final_tilt_deg:.1f}deg above tolerance")
            self.publish_validation(
                lift_ok and place_ok and upright_ok,
                (
                    "physical lift and upright placement verified"
                    if not reasons
                    else "; ".join(reasons)
                ),
                lift_delta,
                place_error,
                final_tilt_deg,
            )

        def publish_validation(
            self,
            success: bool,
            reason: str,
            lift_delta: Optional[float],
            place_error: Optional[float],
            final_tilt_deg: Optional[float],
        ) -> None:
            payload = {
                "final_state": "SUCCESS" if success else "FAILED",
                "pipeline_state": (
                    self.pipeline_result.get("final_state") if self.pipeline_result else None
                ),
                "initial_pose": self.initial_pose,
                "max_z": self.max_z,
                "final_pose": self.latest_pose,
                "lift_delta_m": lift_delta,
                "place_error_m": place_error,
                "final_tilt_deg": final_tilt_deg,
                "reason": reason,
            }
            if self.pipeline_result is not None:
                for key in (
                    "direct_path_fraction",
                    "direct_path_unchecked_fraction",
                    "direct_path_blocked",
                    "ompl_planning_time_s",
                    "ompl_joint_path_length_rad",
                    "ompl_metrics",
                ):
                    payload[key] = self.pipeline_result.get(key)
            message = String()
            message.data = json.dumps(payload)
            self.publisher.publish(message)
            log = self.get_logger().info if success else self.get_logger().error
            log(message.data)

    rclpy.init(args=args)
    node = GazeboPickValidator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
