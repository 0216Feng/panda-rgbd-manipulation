"""Validate RGB-D obstacle tracking against Gazebo model ground truth."""

from __future__ import annotations

import json
from math import sqrt
from statistics import fmean, pstdev
from typing import Dict, List, Optional, Tuple

from .ros_helpers import require_ros2


def summarize_position_errors(errors: List[float]) -> Dict[str, float]:
    if not errors:
        return {"mean_error_m": 0.0, "max_error_m": 0.0, "std_error_m": 0.0}
    return {
        "mean_error_m": fmean(errors),
        "max_error_m": max(errors),
        "std_error_m": pstdev(errors),
    }


def main(args: List[str] | None = None) -> None:
    rclpy = require_ros2()
    from geometry_msgs.msg import Pose
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from std_msgs.msg import String

    class DynamicObstaclePerceptionValidator(Node):
        def __init__(self) -> None:
            super().__init__("dynamic_obstacle_perception_validator")
            self.declare_parameter("required_samples", 20)
            self.declare_parameter("mean_error_threshold_m", 0.03)
            self.declare_parameter("max_error_threshold_m", 0.06)
            self.declare_parameter("base_world_z_m", 0.72)
            self.declare_parameter("minimum_tracking_span_m", 0.0)
            self.declare_parameter("skip_initial_tracking_samples", 3)
            self.required_samples = max(1, int(self.get_parameter("required_samples").value))
            self.mean_threshold = float(self.get_parameter("mean_error_threshold_m").value)
            self.max_threshold = float(self.get_parameter("max_error_threshold_m").value)
            self.base_world_z = float(self.get_parameter("base_world_z_m").value)
            self.minimum_tracking_span = max(
                0.0,
                float(self.get_parameter("minimum_tracking_span_m").value),
            )
            self.skip_initial_tracking_samples = max(
                0,
                int(self.get_parameter("skip_initial_tracking_samples").value),
            )
            result_qos = QoSProfile(depth=1)
            result_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.publisher = self.create_publisher(
                String,
                "/dynamic_obstacle_perception_validation",
                result_qos,
            )
            self.create_subscription(
                String,
                "/dynamic_obstacle_detection",
                self.on_detection,
                10,
            )
            self.create_subscription(
                Pose,
                "/gazebo/dynamic_obstacle_pose",
                self.on_ground_truth,
                qos_profile_sensor_data,
            )
            self.latest_ground_truth: Optional[Tuple[float, float, float]] = None
            self.errors: List[float] = []
            self.x_errors: List[float] = []
            self.y_errors: List[float] = []
            self.z_errors: List[float] = []
            self.ground_truth_y: List[float] = []
            self.first_detection_ns: Optional[int] = None
            self.skipped_samples = 0
            self.started_ns = self.get_clock().now().nanoseconds
            self.finished = False

        def on_ground_truth(self, message: Pose) -> None:
            self.latest_ground_truth = (
                float(message.position.x),
                float(message.position.y),
                float(message.position.z) - self.base_world_z,
            )

        def on_detection(self, message: String) -> None:
            if self.finished or self.latest_ground_truth is None:
                return
            try:
                payload = json.loads(message.data)
            except json.JSONDecodeError:
                return
            if payload.get("state") != "TRACKING":
                return
            center = payload.get("center")
            if not isinstance(center, list) or len(center) != 3:
                return
            if self.first_detection_ns is None:
                self.first_detection_ns = self.get_clock().now().nanoseconds
            if self.skipped_samples < self.skip_initial_tracking_samples:
                self.skipped_samples += 1
                return
            deltas = [float(center[index]) - self.latest_ground_truth[index] for index in range(3)]
            self.x_errors.append(deltas[0])
            self.y_errors.append(deltas[1])
            self.z_errors.append(deltas[2])
            self.errors.append(sqrt(sum(delta * delta for delta in deltas)))
            self.ground_truth_y.append(self.latest_ground_truth[1])
            tracking_span = max(self.ground_truth_y) - min(self.ground_truth_y)
            if (
                len(self.errors) >= self.required_samples
                and tracking_span >= self.minimum_tracking_span
            ):
                self.finish()

        def finish(self) -> None:
            self.finished = True
            summary = summarize_position_errors(self.errors)
            tracking_span = max(self.ground_truth_y) - min(self.ground_truth_y)
            success = (
                summary["mean_error_m"] <= self.mean_threshold
                and summary["max_error_m"] <= self.max_threshold
                and tracking_span >= self.minimum_tracking_span
            )
            now_ns = self.get_clock().now().nanoseconds
            payload = {
                "final_state": "SUCCESS" if success else "FAILED",
                "samples": len(self.errors),
                "skipped_initial_samples": self.skipped_samples,
                **summary,
                "mean_x_error_m": fmean(self.x_errors),
                "mean_y_error_m": fmean(self.y_errors),
                "mean_z_error_m": fmean(self.z_errors),
                "tracking_span_m": tracking_span,
                "first_detection_latency_s": (
                    ((self.first_detection_ns or now_ns) - self.started_ns) / 1e9
                ),
                "validation_duration_s": (now_ns - self.started_ns) / 1e9,
                "reason": (
                    "RGB-D dynamic obstacle pose verified"
                    if success
                    else "RGB-D dynamic obstacle pose or tracking span failed validation"
                ),
            }
            message = String()
            message.data = json.dumps(payload)
            self.publisher.publish(message)
            self.get_logger().info(message.data)

    rclpy.init(args=args)
    node = DynamicObstaclePerceptionValidator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
