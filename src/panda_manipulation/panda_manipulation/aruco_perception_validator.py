"""Validate visual target estimates against Gazebo ground truth."""

from __future__ import annotations

import json
from math import sqrt
from statistics import fmean, pstdev
from time import monotonic
from typing import Dict, List

from .ros_helpers import require_ros2


def summarize_errors(errors_m: List[float]) -> Dict[str, float | int | None]:
    return {
        "samples": len(errors_m),
        "mean_error_m": fmean(errors_m) if errors_m else None,
        "max_error_m": max(errors_m) if errors_m else None,
        "std_error_m": pstdev(errors_m) if errors_m else None,
    }


def main(args: List[str] | None = None) -> None:
    rclpy = require_ros2()
    from geometry_msgs.msg import Pose, PoseStamped
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        QoSProfile,
        qos_profile_sensor_data,
    )
    from std_msgs.msg import String

    class ArucoPerceptionValidator(Node):
        def __init__(self) -> None:
            super().__init__("aruco_perception_validator")
            self.declare_parameter("required_samples", 30)
            self.declare_parameter("max_mean_error_m", 0.02)
            self.declare_parameter("max_single_error_m", 0.04)
            self.declare_parameter("ground_truth_base_z_offset_m", -0.72)
            self.declare_parameter("status_topic", "/aruco_detection_status")
            self.declare_parameter(
                "result_topic", "/aruco_perception_validation"
            )
            self.declare_parameter("source_name", "ArUco")
            self.required_samples = int(
                self.get_parameter("required_samples").value
            )
            self.max_mean_error_m = float(
                self.get_parameter("max_mean_error_m").value
            )
            self.max_single_error_m = float(
                self.get_parameter("max_single_error_m").value
            )
            self.ground_truth_base_z_offset_m = float(
                self.get_parameter("ground_truth_base_z_offset_m").value
            )
            self.source_name = str(self.get_parameter("source_name").value)
            self.latest_ground_truth: tuple[float, float, float] | None = None
            self.errors_m: List[float] = []
            self.x_errors_m: List[float] = []
            self.y_errors_m: List[float] = []
            self.z_errors_m: List[float] = []
            self.started_at = monotonic()
            self.first_detection_at: float | None = None
            self.status_counts: Dict[str, int] = {}
            self.finished = False
            result_qos = QoSProfile(depth=1)
            result_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.publisher = self.create_publisher(
                String,
                str(self.get_parameter("result_topic").value),
                result_qos,
            )
            self.create_subscription(
                Pose,
                "/gazebo/target_pose",
                self.on_ground_truth,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                PoseStamped,
                "/detected_object_pose",
                self.on_detection,
                10,
            )
            self.create_subscription(
                String,
                str(self.get_parameter("status_topic").value),
                self.on_detection_status,
                10,
            )

        def on_detection_status(self, message: String) -> None:
            try:
                state = str(json.loads(message.data).get("state") or "UNKNOWN")
            except (json.JSONDecodeError, AttributeError):
                state = "INVALID_STATUS"
            self.status_counts[state] = self.status_counts.get(state, 0) + 1

        def on_ground_truth(self, message: Pose) -> None:
            self.latest_ground_truth = (
                float(message.position.x),
                float(message.position.y),
                float(message.position.z)
                + self.ground_truth_base_z_offset_m,
            )

        def on_detection(self, message: PoseStamped) -> None:
            if self.finished or self.latest_ground_truth is None:
                return
            if self.first_detection_at is None:
                self.first_detection_at = monotonic()
            truth_x, truth_y, truth_z = self.latest_ground_truth
            x_error = float(message.pose.position.x) - truth_x
            y_error = float(message.pose.position.y) - truth_y
            z_error = float(message.pose.position.z) - truth_z
            error = sqrt(
                x_error**2
                + y_error**2
                + z_error**2
            )
            self.errors_m.append(error)
            self.x_errors_m.append(x_error)
            self.y_errors_m.append(y_error)
            self.z_errors_m.append(z_error)
            if len(self.errors_m) < self.required_samples:
                return
            metrics = summarize_errors(self.errors_m)
            completed_at = monotonic()
            observed_statuses = sum(self.status_counts.values())
            detected_statuses = self.status_counts.get("DETECTED", 0)
            mean_error = float(metrics["mean_error_m"] or 0.0)
            max_error = float(metrics["max_error_m"] or 0.0)
            success = (
                mean_error <= self.max_mean_error_m
                and max_error <= self.max_single_error_m
            )
            result = {
                "final_state": "SUCCESS" if success else "FAILED",
                **metrics,
                "first_detection_latency_s": (
                    self.first_detection_at - self.started_at
                    if self.first_detection_at is not None
                    else None
                ),
                "validation_duration_s": (
                    completed_at - self.first_detection_at
                    if self.first_detection_at is not None
                    else None
                ),
                "status_messages": observed_statuses,
                "detected_statuses": detected_statuses,
                "detection_rate": (
                    detected_statuses / observed_statuses
                    if observed_statuses
                    else None
                ),
                "status_counts": self.status_counts,
                "mean_x_error_m": fmean(self.x_errors_m),
                "mean_y_error_m": fmean(self.y_errors_m),
                "mean_z_error_m": fmean(self.z_errors_m),
                "max_mean_error_m": self.max_mean_error_m,
                "max_single_error_m": self.max_single_error_m,
                "reason": (
                    f"{self.source_name} pose accuracy verified"
                    if success
                    else f"{self.source_name} pose error exceeded tolerance"
                ),
            }
            output = String()
            output.data = json.dumps(result, sort_keys=True)
            self.publisher.publish(output)
            self.get_logger().info(output.data)
            self.finished = True

    rclpy.init(args=args)
    node = ArucoPerceptionValidator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
