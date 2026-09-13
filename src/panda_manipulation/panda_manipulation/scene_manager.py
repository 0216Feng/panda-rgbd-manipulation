"""Publish a tabletop planning scene description for the manipulation demo."""

from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from copy import deepcopy
from typing import Dict, List, Sequence

from .ros_helpers import require_ros2


DEFAULT_SCENE: Dict[str, object] = {
    "frame_id": "panda_link0",
    "objects": [
        {
            "id": "table",
            "type": "box",
            "size": [0.9, 1.2, 0.05],
            "pose_xyz": [0.65, 0.0, -0.025],
        },
        {
            "id": "target_cube",
            "type": "box",
            "size": [0.05, 0.05, 0.08],
            "pose_xyz": [0.52, 0.0, 0.04],
        },
        {
            "id": "obstacle_block",
            "type": "box",
            "size": [0.12, 0.12, 0.18],
            "pose_xyz": [0.42, 0.30, 0.09],
        },
    ],
}


def configured_scene(
    obstacle_pose_xyz: Sequence[float],
    obstacle_size_xyz: Sequence[float],
) -> Dict[str, object]:
    scene = deepcopy(DEFAULT_SCENE)
    obstacle = next(
        scene_object
        for scene_object in scene["objects"]
        if scene_object["id"] == "obstacle_block"
    )
    obstacle["pose_xyz"] = [float(value) for value in obstacle_pose_xyz]
    obstacle["size"] = [float(value) for value in obstacle_size_xyz]
    return scene


def static_auxiliary_obstacle(world_path: str) -> List[Dict[str, object]]:
    """Import the parked auxiliary box for static benchmarks, in panda_link0."""
    model = ET.parse(world_path).getroot().find("./world/model[@name='dynamic_obstacle']")
    if model is None:
        return []
    if model.findtext("static", "false").strip() not in ("true", "1"):
        raise ValueError("auxiliary obstacle must be static for static scene import")
    links = model.findall("link")
    if len(links) != 1 or len(links[0].findall("collision")) != 1:
        raise ValueError("auxiliary obstacle requires one link and one box collision")
    collision = links[0].find("collision")
    size = [float(v) for v in collision.findtext("geometry/box/size", "").split()]
    if len(size) != 3 or not all(math.isfinite(v) and v > 0 for v in size):
        raise ValueError("auxiliary obstacle requires a finite positive box size")
    xyz = [0.0, 0.0, -0.72]
    for element in (model, links[0], collision):
        pose = element.find("pose")
        values = [float(v) for v in (pose.text if pose is not None else "0 0 0 0 0 0").split()]
        if (len(values) != 6 or not all(math.isfinite(v) for v in values)
                or any(abs(v) > 1e-9 for v in values[3:])
                or (pose is not None and pose.attrib)):
            raise ValueError("auxiliary obstacle requires axis-aligned, parent-relative poses")
        xyz = [a + b for a, b in zip(xyz, values[:3])]
    return [{"id": "static_auxiliary_obstacle", "type": "box", "size": size,
             "pose_xyz": xyz}]


def main(args: List[str] | None = None) -> None:
    rclpy = require_ros2()
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject, PlanningScene
    from rclpy.node import Node
    from shape_msgs.msg import SolidPrimitive
    from std_msgs.msg import String

    class SceneManager(Node):
        def __init__(self) -> None:
            super().__init__("scene_manager")
            self.declare_parameter("publish_moveit_scene", True)
            self.declare_parameter("include_target_collision", False)
            self.declare_parameter("static_environment_world", "")
            self.declare_parameter("obstacle_x", 0.42)
            self.declare_parameter("obstacle_y", 0.30)
            self.declare_parameter("obstacle_z", 0.09)
            self.declare_parameter("obstacle_size_x", 0.12)
            self.declare_parameter("obstacle_size_y", 0.12)
            self.declare_parameter("obstacle_size_z", 0.18)
            self.publish_moveit_scene = bool(self.get_parameter("publish_moveit_scene").value)
            self.include_target_collision = bool(self.get_parameter("include_target_collision").value)
            self.scene = configured_scene(
                [
                    self.get_parameter("obstacle_x").value,
                    self.get_parameter("obstacle_y").value,
                    self.get_parameter("obstacle_z").value,
                ],
                [
                    self.get_parameter("obstacle_size_x").value,
                    self.get_parameter("obstacle_size_y").value,
                    self.get_parameter("obstacle_size_z").value,
                ],
            )

            static_world = str(self.get_parameter("static_environment_world").value)
            if static_world:
                self.scene["objects"].extend(static_auxiliary_obstacle(static_world))
            self.description_publisher = self.create_publisher(String, "/planning_scene_description", 10)
            self.planning_scene_publisher = self.create_publisher(PlanningScene, "/planning_scene", 10)
            self.timer = self.create_timer(1.0, self.publish_scene)
            self.get_logger().info(
                "Scene manager publishing tabletop scene. "
                f"publish_moveit_scene={self.publish_moveit_scene}, "
                f"include_target_collision={self.include_target_collision}"
            )

        def publish_scene(self) -> None:
            message = String()
            message.data = json.dumps(self.scene)
            self.description_publisher.publish(message)
            if self.publish_moveit_scene:
                self.planning_scene_publisher.publish(self._to_planning_scene(self.scene))

        def _to_planning_scene(self, scene: Dict[str, object]) -> PlanningScene:
            planning_scene = PlanningScene()
            planning_scene.is_diff = True
            frame_id = str(scene["frame_id"])
            planning_scene.world.collision_objects = [
                self._to_collision_object(frame_id, scene_object)
                for scene_object in scene["objects"]
                if self.include_target_collision or scene_object["id"] != "target_cube"
            ]
            return planning_scene

        def _to_collision_object(self, frame_id: str, scene_object: Dict[str, object]) -> CollisionObject:
            collision_object = CollisionObject()
            collision_object.header.frame_id = frame_id
            collision_object.id = str(scene_object["id"])
            collision_object.operation = CollisionObject.ADD

            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [float(value) for value in scene_object["size"]]

            pose = self._pose_from_xyz(scene_object["pose_xyz"])
            collision_object.primitives.append(primitive)
            collision_object.primitive_poses.append(pose)
            return collision_object

        def _pose_from_xyz(self, xyz: Sequence[float]) -> Pose:
            pose = Pose()
            pose.position.x = float(xyz[0])
            pose.position.y = float(xyz[1])
            pose.position.z = float(xyz[2])
            pose.orientation.w = 1.0
            return pose

    rclpy.init(args=args)
    node = SceneManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
