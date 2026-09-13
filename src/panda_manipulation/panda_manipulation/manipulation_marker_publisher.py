"""Publish RViz markers for target objects and grasp waypoints."""

from __future__ import annotations

import json
from typing import Dict, List, Optional, Tuple

from .ros_helpers import require_ros2


def main(args: List[str] | None = None) -> None:
    rclpy = require_ros2()
    from geometry_msgs.msg import Pose, PoseStamped
    from rclpy.node import Node
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker, MarkerArray

    class ManipulationMarkerPublisher(Node):
        def __init__(self) -> None:
            super().__init__("manipulation_marker_publisher")
            self.declare_parameter("publish_compat_topic", True)
            self.publish_compat_topic = bool(self.get_parameter("publish_compat_topic").value)

            self.marker_publisher = self.create_publisher(MarkerArray, "/manipulation_markers", 10)
            self.compat_publisher = self.create_publisher(MarkerArray, "/rviz_visual_tools", 10)
            self.latest_target: Optional[PoseStamped] = None
            self.latest_candidate: Optional[Dict[str, object]] = None
            self.active_pose_key: Optional[str] = None
            self.object_attached = False
            self.gripper_closed = False

            self.create_subscription(PoseStamped, "/detected_object_pose", self.on_target_pose, 10)
            self.create_subscription(String, "/grasp_candidates", self.on_grasp_candidates, 10)
            self.create_subscription(String, "/pick_demo_state", self.on_pick_demo_state, 10)
            self.timer = self.create_timer(0.5, self.publish_markers)
            self.get_logger().info("Publishing manipulation visualization markers.")

        def on_target_pose(self, message: PoseStamped) -> None:
            self.latest_target = message

        def on_grasp_candidates(self, message: String) -> None:
            payload = json.loads(message.data)
            selected_id = payload.get("selected_candidate_id")
            self.latest_candidate = None
            for candidate in payload.get("candidates", []):
                if candidate.get("candidate_id") == selected_id:
                    self.latest_candidate = candidate
                    break

        def on_pick_demo_state(self, message: String) -> None:
            payload = json.loads(message.data)
            step = str(payload.get("step", ""))
            pose_key = payload.get("pose_key")
            self.active_pose_key = str(pose_key) if pose_key else None
            if step == "object_attached":
                self.object_attached = True
            elif step == "object_released":
                self.object_attached = False
            elif step == "gripper_closed":
                self.gripper_closed = True
            elif step == "gripper_open":
                self.gripper_closed = False

        def publish_markers(self) -> None:
            marker_array = MarkerArray()
            marker_array.markers.extend(self.delete_markers())
            if self.latest_target is not None:
                marker_array.markers.append(self.target_marker(self.latest_target))
                marker_array.markers.append(self.target_label_marker(self.latest_target))
            if self.latest_candidate is not None:
                marker_array.markers.extend(self.waypoint_markers(self.latest_candidate))

            self.marker_publisher.publish(marker_array)
            if self.publish_compat_topic:
                self.compat_publisher.publish(marker_array)

        def delete_markers(self) -> List[Marker]:
            markers = []
            for marker_id in range(30):
                marker = Marker()
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.header.frame_id = "panda_link0"
                marker.ns = "panda_manipulation"
                marker.id = marker_id
                marker.action = Marker.DELETE
                markers.append(marker)
            return markers

        def target_marker(self, target: PoseStamped) -> Marker:
            marker = Marker()
            frame_id, pose, frame_locked = self.active_target_marker_pose(target)
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = frame_id
            marker.ns = "panda_manipulation"
            marker.id = 1
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = pose
            marker.frame_locked = frame_locked
            marker.pose.position.z = max(marker.pose.position.z, 0.04)
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.08
            marker.color.r = 0.95
            marker.color.g = 0.18
            marker.color.b = 0.12
            marker.color.a = 0.9
            return marker

        def target_label_marker(self, target: PoseStamped) -> Marker:
            marker = Marker()
            frame_id, pose, frame_locked = self.active_target_marker_pose(target)
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = frame_id
            marker.ns = "panda_manipulation"
            marker.id = 8
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose = pose
            marker.frame_locked = frame_locked
            if frame_locked:
                marker.pose.position.z -= 0.08
            else:
                marker.pose.position.z = max(marker.pose.position.z + 0.08, 0.12)
            marker.scale.z = 0.035
            marker.color.r = 1.0
            marker.color.g = 0.95
            marker.color.b = 0.85
            marker.color.a = 0.95
            marker.text = "target"
            return marker

        def active_target_marker_pose(self, target: PoseStamped) -> Tuple[str, Pose, bool]:
            if self.object_attached and self.latest_candidate is not None:
                attached_pose = self.latest_candidate.get("object_in_hand_pose")
                if isinstance(attached_pose, dict):
                    return (
                        str(attached_pose.get("frame_id", "panda_hand")),
                        self.pose_from_data(attached_pose),
                        True,
                    )
            if (
                self.latest_candidate is not None
                and self.active_pose_key == "place_pose"
                and self.active_pose_key in {"grasp_pose", "lift_pose", "place_pose", "retreat_pose"}
                and self.active_pose_key in self.latest_candidate
            ):
                return (
                    target.header.frame_id,
                    self.object_pose_for_hand_waypoint(self.latest_candidate[self.active_pose_key]),
                    False,
                )
            return target.header.frame_id, self.copy_pose(target.pose), False

        def copy_pose(self, source: Pose) -> Pose:
            pose = Pose()
            pose.position.x = source.position.x
            pose.position.y = source.position.y
            pose.position.z = source.position.z
            pose.orientation.x = source.orientation.x
            pose.orientation.y = source.orientation.y
            pose.orientation.z = source.orientation.z
            pose.orientation.w = source.orientation.w
            return pose

        def object_pose_for_hand_waypoint(self, hand_pose_data: Dict[str, object]) -> Pose:
            if (
                self.latest_candidate is None
                or "object_pose" not in self.latest_candidate
                or "grasp_pose" not in self.latest_candidate
            ):
                return self.pose_from_data(hand_pose_data)

            object_data = self.latest_candidate["object_pose"]
            grasp_data = self.latest_candidate["grasp_pose"]
            hand_position = hand_pose_data["position"]
            object_position = object_data["position"]
            grasp_position = grasp_data["position"]

            pose = Pose()
            pose.position.x = float(hand_position["x"]) - (
                float(grasp_position["x"]) - float(object_position["x"])
            )
            pose.position.y = float(hand_position["y"]) - (
                float(grasp_position["y"]) - float(object_position["y"])
            )
            pose.position.z = float(hand_position["z"]) - (
                float(grasp_position["z"]) - float(object_position["z"])
            )
            orientation = object_data["orientation"]
            pose.orientation.x = float(orientation["x"])
            pose.orientation.y = float(orientation["y"])
            pose.orientation.z = float(orientation["z"])
            pose.orientation.w = float(orientation["w"])
            return pose

        def waypoint_markers(self, candidate: Dict[str, object]) -> List[Marker]:
            specs = [
                ("pre_grasp_pose", 2, (0.1, 0.45, 1.0, 0.85), 0.035),
                ("grasp_pose", 3, (1.0, 0.85, 0.1, 0.9), 0.04),
                ("lift_pose", 4, (0.1, 0.85, 0.25, 0.85), 0.035),
                ("place_pose", 5, (0.85, 0.2, 1.0, 0.85), 0.04),
                ("retreat_pose", 6, (0.45, 0.45, 0.45, 0.65), 0.03),
            ]
            markers = []
            for pose_key, marker_id, color, scale in specs:
                pose_data = candidate.get(pose_key)
                if pose_data is None:
                    continue
                markers.append(self.sphere_marker(pose_key, marker_id, pose_data, color, scale))
                markers.append(self.label_marker(pose_key, marker_id + 10, pose_data, color))
            return markers

        def sphere_marker(
            self,
            pose_key: str,
            marker_id: int,
            pose_data: Dict[str, object],
            color: Tuple[float, float, float, float],
            scale: float,
        ) -> Marker:
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = str(pose_data["frame_id"])
            marker.ns = "panda_manipulation"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = self.pose_from_data(pose_data)
            marker.scale.x = scale
            marker.scale.y = scale
            marker.scale.z = scale
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = color[3]
            marker.text = pose_key
            return marker

        def label_marker(
            self,
            pose_key: str,
            marker_id: int,
            pose_data: Dict[str, object],
            color: Tuple[float, float, float, float],
        ) -> Marker:
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = str(pose_data["frame_id"])
            marker.ns = "panda_manipulation"
            marker.id = marker_id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose = self.pose_from_data(pose_data)
            marker.pose.position.z += 0.055
            marker.scale.z = 0.028
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 0.95
            marker.text = pose_key.replace("_pose", "").replace("_", "-")
            return marker

        def pose_from_data(self, pose_data: Dict[str, object]) -> Pose:
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

    rclpy.init(args=args)
    node = ManipulationMarkerPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
