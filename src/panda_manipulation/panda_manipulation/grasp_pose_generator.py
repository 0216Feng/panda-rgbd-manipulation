"""Generate grasp candidates from the detected target pose."""

from __future__ import annotations

import json
from math import pi
from typing import List

from .geometry import PoseSpec, Quaternion, Vector3, quaternion_from_euler
from .grasp import generate_grasp_sequences, select_best_reachable_candidate
from .ros_helpers import require_ros2


def main(args: List[str] | None = None) -> None:
    rclpy = require_ros2()
    from geometry_msgs.msg import PoseArray, PoseStamped
    from rclpy.node import Node
    from std_msgs.msg import String

    class GraspPoseGenerator(Node):
        def __init__(self) -> None:
            super().__init__("grasp_pose_generator")
            self.declare_parameter("place_x", 0.35)
            self.declare_parameter("place_y", -0.35)
            self.declare_parameter("place_z", 0.16)
            self.declare_parameter("approach_distance", 0.12)
            self.declare_parameter("grasp_height_offset", 0.12)
            self.declare_parameter("lift_distance", 0.15)
            self.declare_parameter("retreat_distance", 0.10)
            # A negative value preserves the legacy lift_distance / 2 approach height.
            self.declare_parameter("pre_grasp_height_offset", -1.0)
            # A negative value preserves the legacy horizontal retreat.
            self.declare_parameter("retreat_height_offset", -1.0)

            self.publisher = self.create_publisher(String, "/grasp_candidates", 10)
            self.pose_array_publisher = self.create_publisher(PoseArray, "/grasp_candidate_poses", 10)
            self.create_subscription(PoseStamped, "/detected_object_pose", self.on_detected_pose, 10)
            self.get_logger().info("Grasp pose generator waiting for detected object poses.")

        def on_detected_pose(self, message: PoseStamped) -> None:
            target_pose = PoseSpec(
                frame_id=message.header.frame_id,
                position=Vector3(
                    message.pose.position.x,
                    message.pose.position.y,
                    message.pose.position.z,
                ),
                orientation=Quaternion(
                    message.pose.orientation.x,
                    message.pose.orientation.y,
                    message.pose.orientation.z,
                    message.pose.orientation.w,
                ),
            )
            place_pose = PoseSpec(
                frame_id=message.header.frame_id,
                position=Vector3(
                    float(self.get_parameter("place_x").value),
                    float(self.get_parameter("place_y").value),
                    float(self.get_parameter("place_z").value),
                ),
                orientation=quaternion_from_euler(pi, 0.0, -0.785),
            )
            pre_grasp_height_offset = float(
                self.get_parameter("pre_grasp_height_offset").value
            )
            retreat_height_offset = float(
                self.get_parameter("retreat_height_offset").value
            )
            candidates = generate_grasp_sequences(
                target_pose=target_pose,
                place_pose=place_pose,
                approach_distance=float(self.get_parameter("approach_distance").value),
                grasp_height_offset=float(self.get_parameter("grasp_height_offset").value),
                lift_distance=float(self.get_parameter("lift_distance").value),
                retreat_distance=float(self.get_parameter("retreat_distance").value),
                pre_grasp_height_offset=(
                    None if pre_grasp_height_offset < 0.0 else pre_grasp_height_offset
                ),
                retreat_height_offset=(
                    None if retreat_height_offset < 0.0 else retreat_height_offset
                ),
            )
            selected = select_best_reachable_candidate(candidates)
            payload = {
                "selected_candidate_id": selected.candidate_id if selected else None,
                "candidates": [candidate.as_dict() for candidate in candidates],
            }
            output = String()
            output.data = json.dumps(payload)
            self.publisher.publish(output)
            self.pose_array_publisher.publish(self._to_pose_array(message.header.frame_id, candidates))

        def _to_pose_array(self, frame_id: str, candidates) -> PoseArray:
            array = PoseArray()
            array.header.stamp = self.get_clock().now().to_msg()
            array.header.frame_id = frame_id
            for candidate in candidates:
                pose = array.poses.add() if hasattr(array.poses, "add") else None
                if pose is None:
                    from geometry_msgs.msg import Pose

                    pose = Pose()
                    array.poses.append(pose)
                pose.position.x = candidate.grasp_pose.position.x
                pose.position.y = candidate.grasp_pose.position.y
                pose.position.z = candidate.grasp_pose.position.z
                pose.orientation.x = candidate.grasp_pose.orientation.x
                pose.orientation.y = candidate.grasp_pose.orientation.y
                pose.orientation.z = candidate.grasp_pose.orientation.z
                pose.orientation.w = candidate.grasp_pose.orientation.w
            return array

    rclpy.init(args=args)
    node = GraspPoseGenerator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
