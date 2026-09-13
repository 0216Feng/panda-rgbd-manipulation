"""Apply a bounded Gazebo wrench during one Servo placement stage."""

from __future__ import annotations

import json
import re
import time
from typing import Dict, List, Optional

from .ros_helpers import require_ros2


SERVO_STAGE_PATTERN = re.compile(
    r"^servo_place_descent_stage_(?P<stage>\d+)_of_(?P<total>\d+)$"
)


def servo_stage_from_demo_state(payload: Dict[str, object]) -> Optional[int]:
    """Return the active Servo stage only for an executing stage event."""
    if payload.get("mode") != "executing":
        return None
    match = SERVO_STAGE_PATTERN.fullmatch(str(payload.get("step") or ""))
    if match is None:
        return None
    stage = int(match.group("stage"))
    total = int(match.group("total"))
    if stage < 1 or total < 1 or stage > total:
        return None
    return stage


def main(args: List[str] | None = None) -> None:
    rclpy = require_ros2()
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from ros_gz_interfaces.msg import Entity, EntityWrench
    from std_msgs.msg import String

    class PhysicalDisturbanceInjector(Node):
        def __init__(self) -> None:
            super().__init__("physical_disturbance_injector")
            self.declare_parameter("trigger_stage", 3)
            self.declare_parameter("trigger_delay_s", 0.5)
            self.declare_parameter("duration_s", 0.10)
            self.declare_parameter("force_x_n", 0.0)
            self.declare_parameter("force_y_n", 2.0)
            self.declare_parameter("force_z_n", 0.0)
            self.declare_parameter("target_model_name", "target_cube")
            self.declare_parameter(
                "wrench_topic",
                "/gazebo/target_wrench/persistent",
            )
            self.declare_parameter(
                "clear_topic",
                "/gazebo/target_wrench/clear",
            )

            self.trigger_stage = max(
                1,
                int(self.get_parameter("trigger_stage").value),
            )
            self.trigger_delay_s = max(
                0.0,
                float(self.get_parameter("trigger_delay_s").value),
            )
            self.duration_s = max(
                0.001,
                float(self.get_parameter("duration_s").value),
            )
            self.force = (
                float(self.get_parameter("force_x_n").value),
                float(self.get_parameter("force_y_n").value),
                float(self.get_parameter("force_z_n").value),
            )
            self.target_model_name = str(
                self.get_parameter("target_model_name").value
            )

            self.wrench_publisher = self.create_publisher(
                EntityWrench,
                str(self.get_parameter("wrench_topic").value),
                10,
            )
            self.clear_publisher = self.create_publisher(
                Entity,
                str(self.get_parameter("clear_topic").value),
                10,
            )
            self.event_publisher = self.create_publisher(
                String,
                "/physical_disturbance_event",
                10,
            )
            demo_qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                String,
                "/pick_demo_state",
                self.on_demo_state,
                demo_qos,
            )
            self.armed_time = None
            self.applied_time = None
            self.applied = False
            self.cleared = False
            self.timer = self.create_timer(0.01, self.update)

        def on_demo_state(self, message: String) -> None:
            if self.armed_time is not None:
                return
            try:
                payload = json.loads(message.data)
            except json.JSONDecodeError:
                return
            if servo_stage_from_demo_state(payload) != self.trigger_stage:
                return
            self.armed_time = self.get_clock().now()
            self.publish_event("ARMED")
            self.get_logger().warn(
                "Physical target disturbance armed for Servo stage "
                f"{self.trigger_stage}: force={self.force}N, delay="
                f"{self.trigger_delay_s:.3f}s, duration={self.duration_s:.3f}s."
            )

        def update(self) -> None:
            if self.armed_time is None or self.cleared:
                return
            now = self.get_clock().now()
            if not self.applied:
                if now - self.armed_time < Duration(seconds=self.trigger_delay_s):
                    return
                command = EntityWrench()
                command.header.stamp = now.to_msg()
                command.entity = Entity(
                    name=self.target_model_name,
                    type=Entity.MODEL,
                )
                command.wrench.force.x = self.force[0]
                command.wrench.force.y = self.force[1]
                command.wrench.force.z = self.force[2]
                self.wrench_publisher.publish(command)
                self.applied_time = now
                self.applied = True
                self.publish_event("APPLIED")
                return
            if self.applied_time is None:
                return
            if now - self.applied_time < Duration(seconds=self.duration_s):
                return
            self.clear_wrench(now)

        def clear_wrench(self, now) -> None:
            if self.cleared:
                return
            self.clear_publisher.publish(
                Entity(name=self.target_model_name, type=Entity.MODEL)
            )
            self.cleared = True
            self.publish_event("CLEARED", now=now)
            self.get_logger().warn(
                f"Cleared physical disturbance after {self.duration_s:.3f}s."
            )

        def publish_event(self, state: str, now=None) -> None:
            event_time = now or self.get_clock().now()
            message = String()
            message.data = json.dumps(
                {
                    "state": state,
                    "stage": self.trigger_stage,
                    "force_n": list(self.force),
                    "delay_s": self.trigger_delay_s,
                    "duration_s": self.duration_s,
                    "target": self.target_model_name,
                    "stamp_ns": event_time.nanoseconds,
                    "monotonic_ns": time.monotonic_ns(),
                }
            )
            self.event_publisher.publish(message)
            self.get_logger().info(message.data)

    rclpy.init(args=args)
    node = PhysicalDisturbanceInjector()
    try:
        rclpy.spin(node)
    finally:
        if node.applied and not node.cleared:
            node.clear_wrench(node.get_clock().now())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
