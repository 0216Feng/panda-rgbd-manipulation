"""Manage target object world/attached PlanningScene state."""

from __future__ import annotations

import json
from typing import Dict, List, Optional

from .ros_helpers import require_ros2


TOUCH_LINKS = [
    "panda_hand",
    "panda_leftfinger",
    "panda_rightfinger",
]


def main(args: List[str] | None = None) -> None:
    rclpy = require_ros2()
    from geometry_msgs.msg import Pose, PoseStamped
    from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
    from rclpy.node import Node
    from shape_msgs.msg import SolidPrimitive
    from std_msgs.msg import String

    class TargetObjectManager(Node):
        def __init__(self) -> None:
            super().__init__("target_object_manager")
            self.declare_parameter("object_id", "target_cube")
            self.declare_parameter("base_frame", "panda_link0")
            self.declare_parameter("attach_link", "panda_hand")
            self.declare_parameter("size_x", 0.05)
            self.declare_parameter("size_y", 0.05)
            self.declare_parameter("size_z", 0.08)
            self.declare_parameter("publish_period_s", 1.0)
            self.declare_parameter("manage_collision_object", False)
            self.declare_parameter("use_live_release_pose", False)

            self.object_id = str(self.get_parameter("object_id").value)
            self.base_frame = str(self.get_parameter("base_frame").value)
            self.attach_link = str(self.get_parameter("attach_link").value)
            self.size = [
                float(self.get_parameter("size_x").value),
                float(self.get_parameter("size_y").value),
                float(self.get_parameter("size_z").value),
            ]
            self.manage_collision_object = bool(self.get_parameter("manage_collision_object").value)
            self.use_live_release_pose = bool(
                self.get_parameter("use_live_release_pose").value
            )
            self.latest_target_pose: Optional[PoseStamped] = None
            self.latest_candidate: Optional[Dict[str, object]] = None
            self.attached = False
            self.released = False
            self.approaching = False
            self.release_pending = False

            self.publisher = self.create_publisher(PlanningScene, "/planning_scene", 10)
            self.create_subscription(PoseStamped, "/detected_object_pose", self.on_target_pose, 10)
            self.create_subscription(String, "/grasp_candidates", self.on_grasp_candidates, 10)
            self.create_subscription(
                String,
                "/active_grasp_candidate",
                self.on_active_grasp_candidate,
                10,
            )
            self.create_subscription(String, "/pick_demo_state", self.on_pick_demo_state, 10)
            self.timer = self.create_timer(float(self.get_parameter("publish_period_s").value), self.publish_state)
            self.get_logger().info(
                "Target object manager started. "
                f"manage_collision_object={self.manage_collision_object}"
            )

        def on_target_pose(self, message: PoseStamped) -> None:
            self.latest_target_pose = message

        def on_grasp_candidates(self, message: String) -> None:
            payload = json.loads(message.data)
            selected_id = payload.get("selected_candidate_id")
            self.latest_candidate = None
            for candidate in payload.get("candidates", []):
                if candidate.get("candidate_id") == selected_id:
                    self.latest_candidate = candidate
                    break

        def on_active_grasp_candidate(self, message: String) -> None:
            candidate = json.loads(message.data)
            if isinstance(candidate, dict):
                self.latest_candidate = candidate

        def on_pick_demo_state(self, message: String) -> None:
            payload = json.loads(message.data)
            step = str(payload.get("step", ""))
            if step == "grasp_candidate_prevalidation_contact":
                self.attached = False
                self.released = False
                self.approaching = True
                self.release_pending = False
                self.publish_removed_from_world()
            elif step == "grasp_candidate_prevalidation_restore":
                self.attached = False
                self.released = False
                self.approaching = False
                self.release_pending = False
                if self.latest_target_pose is not None:
                    self.publish_world(
                        self.latest_target_pose.pose,
                        self.latest_target_pose.header.frame_id,
                    )
            elif step == "cartesian_pre_grasp_alignment":
                self.attached = False
                self.released = False
                self.approaching = True
                self.release_pending = False
                self.publish_removed_from_world()
            elif step == "object_attached":
                self.attached = True
                self.released = False
                self.approaching = False
                self.release_pending = False
                self.publish_attached()
            elif step == "object_released":
                self.attached = False
                self.released = False
                self.approaching = False
                self.release_pending = True
                self.publish_detached()
            elif step in {"cartesian_retreat", "ompl_retreat_fallback"} and self.release_pending:
                self.released = True
                self.release_pending = False
                self.publish_detached_at_place()
            elif step == "object_collision_cleared":
                self.attached = False
                self.released = False
                self.approaching = True
                self.release_pending = False
                if self.use_live_release_pose and self.latest_target_pose is not None:
                    self.publish_world(
                        self.latest_target_pose.pose,
                        self.latest_target_pose.header.frame_id,
                    )
                else:
                    self.publish_removed_from_world()

        def publish_state(self) -> None:
            if not self.manage_collision_object:
                return
            if self.attached:
                self.publish_attached()
            elif self.released:
                self.publish_detached_at_place()
            elif self.approaching:
                return
            elif self.release_pending:
                return
            elif self.latest_target_pose is not None:
                self.publish_world(self.latest_target_pose.pose, self.latest_target_pose.header.frame_id)

        def publish_world(self, pose: Pose, frame_id: str) -> None:
            scene = PlanningScene()
            scene.is_diff = True
            scene.world.collision_objects.append(self.collision_object(frame_id, pose, CollisionObject.ADD))
            self.publisher.publish(scene)

        def publish_removed_from_world(self) -> None:
            if not self.manage_collision_object:
                return
            scene = PlanningScene()
            scene.is_diff = True
            remove_object = CollisionObject()
            remove_object.header.frame_id = self.base_frame
            remove_object.id = self.object_id
            remove_object.operation = CollisionObject.REMOVE
            scene.world.collision_objects.append(remove_object)
            self.publisher.publish(scene)

        def publish_attached(self) -> None:
            if not self.manage_collision_object:
                return
            scene = PlanningScene()
            scene.is_diff = True

            remove_object = CollisionObject()
            remove_object.header.frame_id = self.base_frame
            remove_object.id = self.object_id
            remove_object.operation = CollisionObject.REMOVE
            scene.world.collision_objects.append(remove_object)

            attached = AttachedCollisionObject()
            attached.link_name = self.attach_link
            attached.object = self.collision_object(self.attach_link, self.attached_pose(), CollisionObject.ADD)
            attached.touch_links = list(TOUCH_LINKS)
            scene.robot_state.attached_collision_objects.append(attached)
            scene.robot_state.is_diff = True
            self.publisher.publish(scene)

        def publish_detached(self) -> None:
            if not self.manage_collision_object:
                return
            detach_scene = PlanningScene()
            detach_scene.is_diff = True
            detach = AttachedCollisionObject()
            detach.link_name = self.attach_link
            detach.object.id = self.object_id
            detach.object.operation = CollisionObject.REMOVE
            detach_scene.robot_state.attached_collision_objects.append(detach)
            detach_scene.robot_state.is_diff = True
            self.publisher.publish(detach_scene)

            # MoveIt may return a detached object to the world at the current
            # hand pose. Remove that representation in a second, ordered diff
            # until the fingers clear it, then restore it at place_pose.
            remove_scene = PlanningScene()
            remove_scene.is_diff = True
            remove_world = CollisionObject()
            remove_world.header.frame_id = self.base_frame
            remove_world.id = self.object_id
            remove_world.operation = CollisionObject.REMOVE
            remove_scene.world.collision_objects.append(remove_world)
            self.publisher.publish(remove_scene)

        def publish_detached_at_place(self) -> None:
            if not self.manage_collision_object:
                return
            scene = PlanningScene()
            scene.is_diff = True

            if self.use_live_release_pose and self.latest_target_pose is not None:
                frame_id = self.latest_target_pose.header.frame_id
                place_pose = self.latest_target_pose.pose
            else:
                frame_id = self.base_frame
                place_pose = self.object_pose_from_hand_waypoint("place_pose")
            scene.world.collision_objects.append(
                self.collision_object(frame_id, place_pose, CollisionObject.ADD)
            )
            self.publisher.publish(scene)

        def collision_object(self, frame_id: str, pose: Pose, operation: int) -> CollisionObject:
            collision_object = CollisionObject()
            collision_object.header.frame_id = frame_id
            collision_object.id = self.object_id
            collision_object.operation = operation
            if operation != CollisionObject.ADD:
                return collision_object

            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = list(self.size)
            collision_object.primitives.append(primitive)
            collision_object.primitive_poses.append(pose)
            return collision_object

        def attached_pose(self) -> Pose:
            if self.latest_candidate is not None:
                attached_data = self.latest_candidate.get("object_in_hand_pose")
                if isinstance(attached_data, dict):
                    return self.pose_from_data(attached_data)
                object_data = self.latest_candidate.get("object_pose")
                grasp_data = self.latest_candidate.get("grasp_pose")
                if isinstance(object_data, dict) and isinstance(grasp_data, dict):
                    object_position = object_data["position"]
                    grasp_position = grasp_data["position"]
                    pose = Pose()
                    pose.position.x = float(object_position["x"]) - float(grasp_position["x"])
                    pose.position.y = float(object_position["y"]) - float(grasp_position["y"])
                    pose.position.z = float(object_position["z"]) - float(grasp_position["z"])
                    pose.orientation.w = 1.0
                    return pose
            pose = Pose()
            pose.orientation.w = 1.0
            pose.position.z = 0.06
            return pose

        def pose_from_data(self, data: Dict[str, object]) -> Pose:
            pose = Pose()
            position = data["position"]
            orientation = data["orientation"]
            pose.position.x = float(position["x"])
            pose.position.y = float(position["y"])
            pose.position.z = float(position["z"])
            pose.orientation.x = float(orientation["x"])
            pose.orientation.y = float(orientation["y"])
            pose.orientation.z = float(orientation["z"])
            pose.orientation.w = float(orientation["w"])
            return pose

        def object_pose_from_hand_waypoint(self, pose_key: str) -> Pose:
            if (
                self.latest_candidate is None
                or pose_key not in self.latest_candidate
                or "object_pose" not in self.latest_candidate
                or "grasp_pose" not in self.latest_candidate
            ):
                return self.pose_from_candidate(pose_key)

            hand_data = self.latest_candidate[pose_key]
            object_data = self.latest_candidate["object_pose"]
            grasp_data = self.latest_candidate["grasp_pose"]
            hand_position = hand_data["position"]
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

        def pose_from_candidate(self, pose_key: str) -> Pose:
            if self.latest_candidate is not None and pose_key in self.latest_candidate:
                data = self.latest_candidate[pose_key]
                pose = Pose()
                position = data["position"]
                orientation = data["orientation"]
                pose.position.x = float(position["x"])
                pose.position.y = float(position["y"])
                pose.position.z = float(position["z"])
                pose.orientation.x = float(orientation["x"])
                pose.orientation.y = float(orientation["y"])
                pose.orientation.z = float(orientation["z"])
                pose.orientation.w = float(orientation["w"])
                return pose
            if self.latest_target_pose is not None:
                return self.latest_target_pose.pose
            pose = Pose()
            pose.orientation.w = 1.0
            return pose

    rclpy.init(args=args)
    node = TargetObjectManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
