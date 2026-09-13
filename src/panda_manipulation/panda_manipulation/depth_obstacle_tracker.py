"""Extract a dynamic obstacle from RGB-D points and update MoveIt's scene."""

from __future__ import annotations

import json
from collections import deque
from math import floor, sqrt
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from .ros_helpers import require_ros2


Point3 = Tuple[float, float, float]


def quaternion_rotation_matrix(
    quaternion: Sequence[float],
) -> Tuple[Tuple[float, float, float], ...]:
    """Return a normalized 3x3 rotation matrix for x, y, z, w."""
    x, y, z, w = (float(value) for value in quaternion)
    norm = sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise ValueError("quaternion norm is zero")
    x, y, z, w = (value / norm for value in (x, y, z, w))
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def transform_point(
    point: Point3,
    translation: Sequence[float],
    rotation: Sequence[Sequence[float]],
) -> Point3:
    return tuple(
        float(translation[row])
        + sum(float(rotation[row][column]) * point[column] for column in range(3))
        for row in range(3)
    )


def obstacle_cluster_center(
    points: Iterable[Point3],
    roi_min: Point3,
    roi_max: Point3,
    expected_width_m: float,
    grid_size_m: float = 0.02,
    minimum_points: int = 12,
) -> Optional[Tuple[Point3, int]]:
    """Find the box-like connected XY cluster closest to the expected footprint."""
    cells: Dict[Tuple[int, int], List[Point3]] = {}
    for point in points:
        if not all(roi_min[index] <= point[index] <= roi_max[index] for index in range(3)):
            continue
        key = (
            int(floor(point[0] / grid_size_m)),
            int(floor(point[1] / grid_size_m)),
        )
        cells.setdefault(key, []).append(point)
    if not cells:
        return None

    remaining = set(cells)
    candidates: List[Tuple[float, Point3, int]] = []
    while remaining:
        start = remaining.pop()
        queue = deque([start])
        cluster_cells = [start]
        while queue:
            current = queue.popleft()
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    neighbor = (current[0] + dx, current[1] + dy)
                    if neighbor in remaining:
                        remaining.remove(neighbor)
                        queue.append(neighbor)
                        cluster_cells.append(neighbor)
        cluster = [point for key in cluster_cells for point in cells[key]]
        if len(cluster) < minimum_points:
            continue
        minimum = tuple(min(point[index] for point in cluster) for index in range(3))
        maximum = tuple(max(point[index] for point in cluster) for index in range(3))
        span_x = maximum[0] - minimum[0]
        span_y = maximum[1] - minimum[1]
        if span_x > expected_width_m * 1.8 or span_y > expected_width_m * 1.8:
            continue
        center = (
            0.5 * (minimum[0] + maximum[0]),
            0.5 * (minimum[1] + maximum[1]),
            sum(point[2] for point in cluster) / len(cluster),
        )
        footprint_error = abs(span_x - expected_width_m) + abs(span_y - expected_width_m)
        score = footprint_error - min(len(cluster), 500) * 1e-4
        candidates.append((score, center, len(cluster)))
    if not candidates:
        return None
    _, center, count = min(candidates, key=lambda item: item[0])
    return center, count


def estimate_velocity(
    previous_center: Point3,
    current_center: Point3,
    elapsed_s: float,
    previous_velocity: Point3 = (0.0, 0.0, 0.0),
    smoothing_alpha: float = 0.45,
    maximum_speed_mps: float = 0.50,
) -> Point3:
    """Estimate a bounded, low-pass-filtered Cartesian velocity."""
    if elapsed_s <= 1e-6:
        return previous_velocity
    alpha = min(1.0, max(0.0, smoothing_alpha))
    raw = tuple(
        (float(current) - float(previous)) / elapsed_s
        for previous, current in zip(previous_center, current_center)
    )
    filtered = tuple(
        alpha * value + (1.0 - alpha) * previous
        for value, previous in zip(raw, previous_velocity)
    )
    speed = sqrt(sum(value * value for value in filtered))
    limit = max(0.0, maximum_speed_mps)
    if limit > 0.0 and speed > limit:
        scale = limit / speed
        filtered = tuple(value * scale for value in filtered)
    return filtered


def predicted_swept_box(
    center: Point3,
    velocity: Point3,
    obstacle_size: Point3,
    horizon_s: float,
    collision_padding_m: float = 0.0,
    minimum_speed_mps: float = 0.03,
    maximum_displacement_m: float = 0.20,
) -> Tuple[Point3, Point3, Point3]:
    """Return center, dimensions and displacement of a bounded swept AABB."""
    horizon_s = max(0.0, horizon_s)
    speed = sqrt(sum(float(value) ** 2 for value in velocity))
    if speed < max(0.0, minimum_speed_mps):
        displacement = (0.0, 0.0, 0.0)
    else:
        displacement = tuple(float(value) * horizon_s for value in velocity)
        distance = sqrt(sum(value * value for value in displacement))
        limit = max(0.0, maximum_displacement_m)
        if limit > 0.0 and distance > limit:
            scale = limit / distance
            displacement = tuple(value * scale for value in displacement)
    swept_center = tuple(
        float(value) + 0.5 * delta for value, delta in zip(center, displacement)
    )
    padding = max(0.0, collision_padding_m)
    swept_size = (
        float(obstacle_size[0]) + abs(displacement[0]),
        float(obstacle_size[1]) + 2.0 * padding + abs(displacement[1]),
        float(obstacle_size[2]) + abs(displacement[2]),
    )
    return swept_center, swept_size, displacement


def main(args: List[str] | None = None) -> None:
    rclpy = require_ros2()
    from geometry_msgs.msg import Pose, PoseStamped
    from moveit_msgs.msg import CollisionObject, PlanningScene
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from shape_msgs.msg import SolidPrimitive
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformException, TransformListener

    class DepthObstacleTracker(Node):
        def __init__(self) -> None:
            super().__init__("depth_obstacle_tracker")
            self.declare_parameter("points_topic", "/camera/depth/points")
            self.declare_parameter("base_frame", "panda_link0")
            self.declare_parameter("collision_object_id", "dynamic_obstacle_depth")
            self.declare_parameter("obstacle_size", [0.12, 0.12, 0.30])
            self.declare_parameter("collision_padding_m", 0.0)
            self.declare_parameter("roi_min", [0.25, -0.55, 0.21])
            self.declare_parameter("roi_max", [0.49, 0.55, 0.38])
            self.declare_parameter("point_stride", 6)
            self.declare_parameter("grid_size_m", 0.02)
            self.declare_parameter("minimum_points", 10)
            self.declare_parameter("smoothing_alpha", 0.80)
            self.declare_parameter("enable_motion_prediction", False)
            self.declare_parameter("prediction_horizon_s", 0.65)
            self.declare_parameter("velocity_smoothing_alpha", 0.45)
            self.declare_parameter("minimum_prediction_speed_mps", 0.03)
            self.declare_parameter("maximum_obstacle_speed_mps", 0.50)
            self.declare_parameter("maximum_prediction_displacement_m", 0.20)
            self.declare_parameter("publish_period_s", 0.15)
            self.declare_parameter("missing_detection_limit", 3)
            self.base_frame = str(self.get_parameter("base_frame").value)
            self.object_id = str(self.get_parameter("collision_object_id").value)
            self.obstacle_size = tuple(float(value) for value in self.get_parameter("obstacle_size").value)
            self.collision_padding_m = max(
                0.0,
                float(self.get_parameter("collision_padding_m").value),
            )
            self.roi_min = tuple(float(value) for value in self.get_parameter("roi_min").value)
            self.roi_max = tuple(float(value) for value in self.get_parameter("roi_max").value)
            self.point_stride = max(1, int(self.get_parameter("point_stride").value))
            self.grid_size_m = max(0.005, float(self.get_parameter("grid_size_m").value))
            self.minimum_points = max(3, int(self.get_parameter("minimum_points").value))
            self.alpha = min(1.0, max(0.01, float(self.get_parameter("smoothing_alpha").value)))
            self.enable_motion_prediction = bool(
                self.get_parameter("enable_motion_prediction").value
            )
            self.prediction_horizon_s = max(
                0.0,
                float(self.get_parameter("prediction_horizon_s").value),
            )
            self.velocity_alpha = min(
                1.0,
                max(0.0, float(self.get_parameter("velocity_smoothing_alpha").value)),
            )
            self.minimum_prediction_speed_mps = max(
                0.0,
                float(self.get_parameter("minimum_prediction_speed_mps").value),
            )
            self.maximum_obstacle_speed_mps = max(
                0.0,
                float(self.get_parameter("maximum_obstacle_speed_mps").value),
            )
            self.maximum_prediction_displacement_m = max(
                0.0,
                float(self.get_parameter("maximum_prediction_displacement_m").value),
            )
            self.missing_detection_limit = max(
                1,
                int(self.get_parameter("missing_detection_limit").value),
            )
            self.filtered_center: Optional[Point3] = None
            self.filtered_velocity: Point3 = (0.0, 0.0, 0.0)
            self.previous_velocity_center: Optional[Point3] = None
            self.previous_detection_ns: Optional[int] = None
            self.pending_detection: Optional[Tuple[Point3, Point3, int, str]] = None
            self.pending_status: Optional[Dict[str, object]] = None
            self.pending_remove = False
            self.object_present = False
            self.missing_detection_count = 0
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.scene_publisher = self.create_publisher(PlanningScene, "/planning_scene", 10)
            self.pose_publisher = self.create_publisher(PoseStamped, "/dynamic_obstacle_pose", 10)
            self.status_publisher = self.create_publisher(String, "/dynamic_obstacle_detection", 10)
            self.create_subscription(
                PointCloud2,
                str(self.get_parameter("points_topic").value),
                self.on_points,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                String,
                "/trajectory_safety_event",
                self.on_safety_event,
                10,
            )
            self.hazard_latched = False
            self.timer = self.create_timer(
                max(0.05, float(self.get_parameter("publish_period_s").value)),
                self.publish_detection,
            )

        def on_points(self, message: PointCloud2) -> None:
            if self.hazard_latched:
                return
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    message.header.frame_id,
                    Time.from_msg(message.header.stamp),
                )
            except TransformException:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.base_frame,
                        message.header.frame_id,
                        Time(),
                    )
                except TransformException as exc:
                    self.get_logger().warn(f"Point-cloud TF unavailable: {exc}", throttle_duration_sec=2.0)
                    return
            translation = (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            )
            quaternion = (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            )
            rotation = quaternion_rotation_matrix(quaternion)
            raw = point_cloud2.read_points_numpy(
                message,
                field_names=["x", "y", "z"],
                skip_nans=True,
            )
            transformed: List[Point3] = []
            for index in range(0, len(raw), self.point_stride):
                row = raw[index]
                transformed.append(
                    transform_point(
                        (float(row[0]), float(row[1]), float(row[2])),
                        translation,
                        rotation,
                    )
                )
            detection = obstacle_cluster_center(
                transformed,
                self.roi_min,
                self.roi_max,
                self.obstacle_size[0],
                self.grid_size_m,
                self.minimum_points,
            )
            if detection is None:
                self.missing_detection_count += 1
                if (
                    self.object_present
                    and self.missing_detection_count >= self.missing_detection_limit
                ):
                    self.pending_remove = True
                roi_count = sum(
                    1
                    for point in transformed
                    if all(
                        self.roi_min[index] <= point[index] <= self.roi_max[index]
                        for index in range(3)
                    )
                )
                bounds = None
                if transformed:
                    bounds = {
                        "min": [min(point[index] for point in transformed) for index in range(3)],
                        "max": [max(point[index] for point in transformed) for index in range(3)],
                    }
                self.pending_status = {
                    "state": "NO_CLUSTER",
                    "frame_id": self.base_frame,
                    "source_frame": message.header.frame_id,
                    "sampled_points": len(transformed),
                    "roi_points": roi_count,
                    "bounds": bounds,
                    "roi_min": list(self.roi_min),
                    "roi_max": list(self.roi_max),
                }
                return
            observed_center, count = detection
            self.missing_detection_count = 0
            self.pending_remove = False
            # The top-down camera measures the visible top surface. Planning uses
            # the known box height and only estimates its horizontal center.
            observed_center = (
                observed_center[0],
                observed_center[1],
                0.5 * self.obstacle_size[2],
            )
            if self.filtered_center is None:
                self.filtered_center = observed_center
            else:
                self.filtered_center = tuple(
                    self.alpha * observed + (1.0 - self.alpha) * previous
                    for observed, previous in zip(observed_center, self.filtered_center)
                )
            stamp_ns = (
                int(message.header.stamp.sec) * 1_000_000_000
                + int(message.header.stamp.nanosec)
            )
            if stamp_ns <= 0:
                stamp_ns = self.get_clock().now().nanoseconds
            if (
                self.previous_velocity_center is not None
                and self.previous_detection_ns is not None
            ):
                elapsed_s = (stamp_ns - self.previous_detection_ns) / 1e9
                if 1e-3 <= elapsed_s <= 1.0:
                    self.filtered_velocity = estimate_velocity(
                        self.previous_velocity_center,
                        self.filtered_center,
                        elapsed_s,
                        self.filtered_velocity,
                        self.velocity_alpha,
                        self.maximum_obstacle_speed_mps,
                    )
            self.previous_velocity_center = self.filtered_center
            self.previous_detection_ns = stamp_ns
            self.pending_detection = (
                self.filtered_center,
                self.filtered_velocity,
                count,
                message.header.frame_id,
            )
            self.pending_status = None

        def on_safety_event(self, message: String) -> None:
            if self.hazard_latched:
                return
            try:
                payload = json.loads(message.data)
            except json.JSONDecodeError:
                return
            obstacle = payload.get("obstacle")
            center = obstacle.get("center") if isinstance(obstacle, dict) else None
            if (
                payload.get("event") != "trajectory_invalidated"
                or not isinstance(center, list)
                or len(center) != 3
            ):
                return
            latched_center = tuple(float(value) for value in center)
            self.hazard_latched = True
            self.filtered_center = latched_center
            self.filtered_velocity = (0.0, 0.0, 0.0)
            self.previous_velocity_center = latched_center
            self.previous_detection_ns = None
            self.missing_detection_count = 0
            self.pending_remove = False
            self.pending_status = None
            self.pending_detection = (
                latched_center,
                self.filtered_velocity,
                0,
                "hazard_latch",
            )
            self.get_logger().info(
                f"Latched dynamic obstacle at {latched_center} after safety event."
            )

        def publish_detection(self) -> None:
            if self.pending_detection is None:
                if self.pending_remove:
                    self.scene_publisher.publish(self.make_remove_scene())
                    self.pending_remove = False
                    self.object_present = False
                    self.filtered_center = None
                    self.get_logger().info(
                        f"Removed stale PlanningScene object {self.object_id}."
                    )
                if self.pending_status is not None:
                    status = String()
                    status.data = json.dumps(self.pending_status)
                    self.status_publisher.publish(status)
                    self.get_logger().info(status.data, throttle_duration_sec=2.0)
                    self.pending_status = None
                return
            center, velocity, count, source_frame = self.pending_detection
            self.pending_detection = None
            scene_center, scene_size, displacement = self.scene_geometry(center, velocity)
            self.scene_publisher.publish(self.make_scene(scene_center, scene_size))
            self.object_present = True
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = self.base_frame
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = center
            pose.pose.orientation.w = 1.0
            self.pose_publisher.publish(pose)
            status = String()
            status.data = json.dumps(
                {
                    "state": "TRACKING",
                    "frame_id": self.base_frame,
                    "source_frame": source_frame,
                    "center": list(center),
                    "size": list(self.obstacle_size),
                    "velocity": list(velocity),
                    "speed_mps": sqrt(sum(value * value for value in velocity)),
                    "prediction_enabled": self.enable_motion_prediction,
                    "prediction_horizon_s": self.prediction_horizon_s,
                    "prediction_displacement": list(displacement),
                    "predicted_center": list(scene_center),
                    "predicted_size": list(scene_size),
                    "cluster_points": count,
                }
            )
            self.status_publisher.publish(status)
            self.get_logger().info(status.data, throttle_duration_sec=2.0)

        def scene_geometry(
            self,
            center: Point3,
            velocity: Point3,
        ) -> Tuple[Point3, Point3, Point3]:
            if not self.enable_motion_prediction:
                return predicted_swept_box(
                    center,
                    (0.0, 0.0, 0.0),
                    self.obstacle_size,
                    0.0,
                    self.collision_padding_m,
                )
            return predicted_swept_box(
                center,
                velocity,
                self.obstacle_size,
                self.prediction_horizon_s,
                self.collision_padding_m,
                self.minimum_prediction_speed_mps,
                self.maximum_prediction_displacement_m,
            )

        def make_scene(self, center: Point3, size: Point3) -> PlanningScene:
            scene = PlanningScene()
            scene.is_diff = True
            collision = CollisionObject()
            collision.header.frame_id = self.base_frame
            collision.id = self.object_id
            collision.operation = CollisionObject.ADD
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = list(size)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = center
            pose.orientation.w = 1.0
            collision.primitives.append(primitive)
            collision.primitive_poses.append(pose)
            scene.world.collision_objects.append(collision)
            return scene

        def make_remove_scene(self) -> PlanningScene:
            scene = PlanningScene()
            scene.is_diff = True
            collision = CollisionObject()
            collision.header.frame_id = self.base_frame
            collision.id = self.object_id
            collision.operation = CollisionObject.REMOVE
            scene.world.collision_objects.append(collision)
            return scene

    rclpy.init(args=args)
    node = DepthObstacleTracker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
