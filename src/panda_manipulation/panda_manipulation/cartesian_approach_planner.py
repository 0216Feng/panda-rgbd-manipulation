"""Plan a straight Cartesian approach from pre-grasp to grasp pose."""

from __future__ import annotations

import json
from typing import Dict, List, Optional

from .moveit_plan_only_adapter import selected_candidate
from .ros_helpers import require_ros2


def main(args: List[str] | None = None) -> None:
    rclpy = require_ros2()
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import DisplayTrajectory
    from moveit_msgs.srv import GetCartesianPath
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String

    class CartesianApproachPlanner(Node):
        def __init__(self) -> None:
            super().__init__("cartesian_approach_planner")
            self.declare_parameter("service_name", "/compute_cartesian_path")
            self.declare_parameter("planning_group", "panda_arm")
            self.declare_parameter("end_effector_link", "panda_hand")
            self.declare_parameter("max_step", 0.01)
            self.declare_parameter("jump_threshold", 0.0)
            self.declare_parameter("min_fraction", 0.9)
            self.declare_parameter("plan_once", True)

            self.service_name = str(self.get_parameter("service_name").value)
            self.group_name = str(self.get_parameter("planning_group").value)
            self.end_effector_link = str(self.get_parameter("end_effector_link").value)
            self.max_step = float(self.get_parameter("max_step").value)
            self.jump_threshold = float(self.get_parameter("jump_threshold").value)
            self.min_fraction = float(self.get_parameter("min_fraction").value)
            self.plan_once = bool(self.get_parameter("plan_once").value)

            result_qos = QoSProfile(depth=10)
            result_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.result_publisher = self.create_publisher(String, "/cartesian_approach_result", result_qos)
            self.display_publisher = self.create_publisher(DisplayTrajectory, "/display_planned_path", 10)
            self.client = self.create_client(GetCartesianPath, self.service_name)
            self.create_subscription(String, "/grasp_candidates", self.on_grasp_candidates, 10)
            self.has_started = False
            self.active = False
            self.active_started_ns: Optional[int] = None
            self.watchdog_timer = self.create_timer(1.0, self.check_watchdog)

            self.get_logger().info(
                "Cartesian approach planner waiting for /grasp_candidates "
                f"and service {self.service_name}."
            )

        def on_grasp_candidates(self, message: String) -> None:
            if self.plan_once and self.has_started:
                return
            if self.active:
                return

            payload = json.loads(message.data)
            candidate = selected_candidate(payload)
            if candidate is None:
                self.publish_result("FAILED", 0.0, "no selected candidate")
                return

            pre_grasp_pose = candidate.get("pre_grasp_pose")
            grasp_pose = candidate.get("grasp_pose")
            if pre_grasp_pose is None or grasp_pose is None:
                self.publish_result("FAILED", 0.0, "candidate missing pre_grasp_pose or grasp_pose")
                return

            if not self.client.wait_for_service(timeout_sec=1.0):
                reason = f"Cartesian path service {self.service_name} is not available yet"
                self.get_logger().warn(reason)
                self.publish_result("WAITING", 0.0, reason)
                return

            self.has_started = True
            self.active = True
            self.active_started_ns = self.get_clock().now().nanoseconds
            request = self.make_request(pre_grasp_pose, grasp_pose)
            self.get_logger().info(
                f"Planning Cartesian approach with max_step={self.max_step}, "
                f"jump_threshold={self.jump_threshold}."
            )
            future = self.client.call_async(request)
            future.add_done_callback(self.on_cartesian_result)

        def make_request(self, pre_grasp_pose: Dict[str, object], grasp_pose: Dict[str, object]):
            request = GetCartesianPath.Request()
            request.header.stamp = self.get_clock().now().to_msg()
            request.header.frame_id = str(pre_grasp_pose["frame_id"])
            request.start_state.is_diff = True
            request.group_name = self.group_name
            request.link_name = self.end_effector_link
            request.waypoints = [
                self.to_pose(pre_grasp_pose),
                self.to_pose(grasp_pose),
            ]
            request.max_step = self.max_step
            request.jump_threshold = self.jump_threshold
            request.avoid_collisions = True
            return request

        def to_pose(self, pose_data: Dict[str, object]) -> Pose:
            pose = Pose()
            position = pose_data["position"]
            orientation = pose_data["orientation"]
            pose.position.x = float(position["x"])
            pose.position.y = float(position["y"])
            pose.position.z = float(position["z"])
            pose.orientation.x = float(orientation["x"])
            pose.orientation.y = float(orientation["y"])
            pose.orientation.z = float(orientation["z"])
            pose.orientation.w = float(orientation["w"])
            return pose

        def on_cartesian_result(self, future) -> None:
            self.active = False
            self.active_started_ns = None
            try:
                response = future.result()
            except Exception as exc:
                self.publish_result("FAILED", 0.0, f"service call failed: {exc}")
                return

            fraction = float(response.fraction)
            error_code = int(response.error_code.val)
            success = fraction >= self.min_fraction and error_code == 1
            if success:
                self.publish_display_trajectory(response)
            reason = "" if success else f"fraction={fraction:.3f}, moveit_error_code={error_code}"
            self.publish_result("SUCCESS" if success else "FAILED", fraction, reason)

        def publish_display_trajectory(self, response) -> None:
            message = DisplayTrajectory()
            message.trajectory_start = response.start_state
            message.trajectory.append(response.solution)
            self.display_publisher.publish(message)

        def check_watchdog(self) -> None:
            if not self.active or self.active_started_ns is None:
                return
            elapsed_s = (self.get_clock().now().nanoseconds - self.active_started_ns) / 1_000_000_000.0
            if elapsed_s <= 8.0:
                return
            self.active = False
            self.publish_result("FAILED", 0.0, f"Cartesian path service timed out after {elapsed_s:.1f}s")

        def publish_result(self, final_state: str, fraction: float, reason: str) -> None:
            output = String()
            output.data = json.dumps(
                {
                    "final_state": final_state,
                    "planning_group": self.group_name,
                    "end_effector_link": self.end_effector_link,
                    "max_step": self.max_step,
                    "jump_threshold": self.jump_threshold,
                    "min_fraction": self.min_fraction,
                    "fraction": fraction,
                    "failure_reason": reason or None,
                }
            )
            self.result_publisher.publish(output)
            self.get_logger().info(output.data)

    rclpy.init(args=args)
    node = CartesianApproachPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
