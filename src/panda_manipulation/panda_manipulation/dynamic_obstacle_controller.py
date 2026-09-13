"""Move a Gazebo obstacle across the workspace after the pick begins."""

from __future__ import annotations

import json
from typing import List, Optional

from .ros_helpers import require_ros2


def linear_position(start: float, end: float, speed: float, elapsed_s: float) -> float:
    direction = 1.0 if end >= start else -1.0
    travelled = max(0.0, speed) * max(0.0, elapsed_s)
    value = start + direction * travelled
    return min(end, value) if direction > 0.0 else max(end, value)


def main(args: List[str] | None = None) -> None:
    rclpy = require_ros2()
    from geometry_msgs.msg import Pose
    from rclpy.node import Node
    from ros_gz_interfaces.msg import Entity
    from ros_gz_interfaces.srv import SetEntityPose
    from std_msgs.msg import String

    class DynamicObstacleController(Node):
        def __init__(self) -> None:
            super().__init__("dynamic_obstacle_controller")
            self.declare_parameter("service_name", "/world/panda_table/set_pose")
            self.declare_parameter("model_name", "dynamic_obstacle")
            self.declare_parameter("x", 0.37)
            self.declare_parameter("z", 0.87)
            self.declare_parameter("start_y", -0.24)
            self.declare_parameter("end_y", 0.24)
            self.declare_parameter("speed_mps", 0.04)
            self.declare_parameter("trigger_step", "gripper_open")
            self.declare_parameter("trigger_mode", "executed")
            self.declare_parameter("trigger_delay_s", 0.8)
            self.declare_parameter("auto_start", False)
            self.model_name = str(self.get_parameter("model_name").value)
            self.x = float(self.get_parameter("x").value)
            self.z = float(self.get_parameter("z").value)
            self.start_y = float(self.get_parameter("start_y").value)
            self.end_y = float(self.get_parameter("end_y").value)
            self.speed = max(0.01, float(self.get_parameter("speed_mps").value))
            self.trigger_step = str(self.get_parameter("trigger_step").value)
            self.trigger_mode = str(self.get_parameter("trigger_mode").value)
            self.trigger_delay_s = max(0.0, float(self.get_parameter("trigger_delay_s").value))
            self.client = self.create_client(
                SetEntityPose,
                str(self.get_parameter("service_name").value),
            )
            self.publisher = self.create_publisher(String, "/dynamic_obstacle_motion", 10)
            self.create_subscription(String, "/pick_demo_state", self.on_demo_state, 10)
            self.create_subscription(
                String,
                "/trajectory_safety_event",
                self.on_safety_event,
                10,
            )
            self.trigger_time_ns: Optional[int] = (
                self.get_clock().now().nanoseconds
                if bool(self.get_parameter("auto_start").value)
                else None
            )
            self.request_in_flight = False
            self.initialized = False
            self.finished = False
            self.last_commanded_y = self.start_y
            self.timer = self.create_timer(0.1, self.update)

        def on_demo_state(self, message: String) -> None:
            if self.trigger_time_ns is not None:
                return
            try:
                payload = json.loads(message.data)
            except json.JSONDecodeError:
                return
            if (
                payload.get("step") == self.trigger_step
                and payload.get("mode") == self.trigger_mode
            ):
                self.trigger_time_ns = self.get_clock().now().nanoseconds
                self.get_logger().info(
                    f"Dynamic obstacle armed by {self.trigger_step}/{self.trigger_mode}; "
                    "moving after "
                    f"{self.trigger_delay_s:.1f}s."
                )

        def on_safety_event(self, message: String) -> None:
            if self.finished or self.trigger_time_ns is None:
                return
            try:
                payload = json.loads(message.data)
            except json.JSONDecodeError:
                return
            if payload.get("event") != "trajectory_invalidated":
                return
            self.finished = True
            status = String()
            status.data = json.dumps(
                {
                    "state": "HALTED_FOR_REPLAN",
                    "x": self.x,
                    "y": self.last_commanded_y,
                    "z": self.z,
                    "success": True,
                }
            )
            self.publisher.publish(status)
            self.get_logger().info(
                f"Dynamic obstacle halted at y={self.last_commanded_y:.3f} for replanning."
            )

        def update(self) -> None:
            if self.finished or self.request_in_flight:
                return
            if not self.client.service_is_ready():
                return
            if not self.initialized:
                self.send_pose(self.start_y, initial=True)
                return
            if self.trigger_time_ns is None:
                return
            elapsed_s = (self.get_clock().now().nanoseconds - self.trigger_time_ns) / 1e9
            motion_elapsed_s = elapsed_s - self.trigger_delay_s
            if motion_elapsed_s < 0.0:
                return
            y = linear_position(self.start_y, self.end_y, self.speed, motion_elapsed_s)
            self.send_pose(y, initial=False)

        def send_pose(self, y: float, initial: bool) -> None:
            request = SetEntityPose.Request()
            request.entity = Entity(name=self.model_name, type=Entity.MODEL)
            request.pose = Pose()
            request.pose.position.x = self.x
            request.pose.position.y = y
            request.pose.position.z = self.z
            request.pose.orientation.w = 1.0
            self.last_commanded_y = y
            self.request_in_flight = True
            future = self.client.call_async(request)
            future.add_done_callback(lambda done: self.on_pose_set(done, y, initial))

        def on_pose_set(self, future, y: float, initial: bool) -> None:
            self.request_in_flight = False
            try:
                success = bool(future.result().success)
            except Exception as exc:
                self.get_logger().warn(f"Dynamic obstacle pose update failed: {exc}")
                return
            if initial:
                self.initialized = success
                return
            status = String()
            state = "MOVING"
            if success and abs(y - self.end_y) <= 1e-6:
                state = "COMPLETE"
                self.finished = True
            status.data = json.dumps(
                {"state": state, "x": self.x, "y": y, "z": self.z, "success": success}
            )
            self.publisher.publish(status)

    rclpy.init(args=args)
    node = DynamicObstacleController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
