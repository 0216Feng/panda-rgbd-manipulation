"""Publish a repeatable sequence of synthetic target poses for benchmarking."""

from __future__ import annotations

import json
from math import pi
from typing import List

from .geometry import PoseSpec, Vector3, quaternion_from_euler
from .ros_helpers import require_ros2


DEFAULT_TARGETS = [
    (0.50, -0.04, 0.08),
    (0.52, 0.00, 0.08),
    (0.50, 0.04, 0.08),
    (0.53, -0.04, 0.08),
    (0.54, 0.00, 0.08),
    (0.53, 0.04, 0.08),
    (0.48, -0.04, 0.08),
    (0.49, 0.00, 0.08),
    (0.48, 0.04, 0.08),
]


def main(args: List[str] | None = None) -> None:
    rclpy = require_ros2()
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from std_msgs.msg import String

    class SyntheticTargetSequencePublisher(Node):
        def __init__(self) -> None:
            super().__init__("synthetic_target_sequence_publisher")
            self.declare_parameter("base_frame", "panda_link0")
            self.declare_parameter("publish_period_s", 12.0)
            self.declare_parameter("initial_delay_s", 3.0)
            self.declare_parameter("publish_after_result", True)
            self.declare_parameter("repeat", False)

            self.base_frame = str(self.get_parameter("base_frame").value)
            self.publish_period_s = float(self.get_parameter("publish_period_s").value)
            self.initial_delay_s = float(self.get_parameter("initial_delay_s").value)
            self.publish_after_result = bool(self.get_parameter("publish_after_result").value)
            self.repeat = bool(self.get_parameter("repeat").value)
            self.index = 0
            self.waiting_for_result = False
            self.pending_timer = None
            self.publisher = self.create_publisher(PoseStamped, "/detected_object_pose", 10)
            if self.publish_after_result:
                self.create_subscription(String, "/pick_plan_result", self.on_pick_plan_result, 10)
                self.schedule_next(self.initial_delay_s)
            else:
                self.timer = self.create_timer(self.publish_period_s, self.publish_next)
            self.get_logger().info(
                f"Publishing {len(DEFAULT_TARGETS)} synthetic benchmark target poses "
                f"with publish_after_result={self.publish_after_result}."
            )

        def schedule_next(self, delay_s: float) -> None:
            if self.pending_timer is not None:
                self.pending_timer.cancel()
            self.pending_timer = self.create_timer(delay_s, self.on_scheduled_publish)

        def on_scheduled_publish(self) -> None:
            if self.pending_timer is not None:
                self.pending_timer.cancel()
                self.pending_timer = None
            self.publish_next()

        def on_pick_plan_result(self, message: String) -> None:
            if not self.publish_after_result or not self.waiting_for_result:
                return
            try:
                result = json.loads(message.data)
            except json.JSONDecodeError:
                self.get_logger().warn("Ignoring malformed /pick_plan_result while waiting for target sequence.")
                return
            if result.get("final_state") not in {"SUCCESS", "FAILED"}:
                self.get_logger().info(
                    f"Ignoring non-terminal pick result while sequencing: {result.get('final_state')}"
                )
                return
            self.waiting_for_result = False
            self.schedule_next(self.publish_period_s)

        def publish_next(self) -> None:
            if self.index >= len(DEFAULT_TARGETS):
                if not self.repeat:
                    self.get_logger().info("Synthetic benchmark target sequence completed.")
                    return
                self.index = 0

            x, y, z = DEFAULT_TARGETS[self.index]
            pose = PoseSpec(
                frame_id=self.base_frame,
                position=Vector3(x, y, z),
                orientation=quaternion_from_euler(pi, 0.0, 0.0),
            )
            message = PoseStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = pose.frame_id
            message.pose.position.x = pose.position.x
            message.pose.position.y = pose.position.y
            message.pose.position.z = pose.position.z
            message.pose.orientation.x = pose.orientation.x
            message.pose.orientation.y = pose.orientation.y
            message.pose.orientation.z = pose.orientation.z
            message.pose.orientation.w = pose.orientation.w
            self.publisher.publish(message)
            self.waiting_for_result = self.publish_after_result
            self.get_logger().info(f"Published benchmark target {self.index + 1}/{len(DEFAULT_TARGETS)}.")
            self.index += 1

    rclpy.init(args=args)
    node = SyntheticTargetSequencePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
