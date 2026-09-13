"""Estimate a markerless target pose by fusing RGB segmentation and depth points."""

from __future__ import annotations

import json
from dataclasses import dataclass
from math import pi
from statistics import median
from typing import Iterable, List, Optional, Sequence, Tuple

from .aruco_pose_estimator import filtered_position
from .depth_obstacle_tracker import quaternion_rotation_matrix, transform_point
from .geometry import PoseSpec, Vector3, quaternion_from_euler
from .ros_helpers import require_ros2


Point3 = Tuple[float, float, float]


@dataclass(frozen=True)
class ColorRegion:
    centroid_u: float
    centroid_v: float
    area_px: float
    bounding_box: Tuple[int, int, int, int]
    fill_ratio: float


def largest_red_region(
    image_bgr: object,
    minimum_area_px: float = 80.0,
) -> Tuple[Optional[ColorRegion], object]:
    """Return the largest sufficiently saturated red image region and its mask."""
    import cv2
    import numpy as np

    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    lower_red = cv2.inRange(hsv, np.array((0, 85, 55)), np.array((14, 255, 255)))
    upper_red = cv2.inRange(hsv, np.array((166, 85, 55)), np.array((179, 255, 255)))
    mask = cv2.bitwise_or(lower_red, upper_red)
    kernel = np.ones((3, 3), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = [contour for contour in contours if cv2.contourArea(contour) >= minimum_area_px]
    if not candidates:
        return None, mask
    contour = max(candidates, key=cv2.contourArea)
    moments = cv2.moments(contour)
    if abs(moments["m00"]) < 1e-9:
        return None, mask
    x, y, width, height = cv2.boundingRect(contour)
    area = float(cv2.contourArea(contour))
    return (
        ColorRegion(
            centroid_u=float(moments["m10"] / moments["m00"]),
            centroid_v=float(moments["m01"] / moments["m00"]),
            area_px=area,
            bounding_box=(int(x), int(y), int(width), int(height)),
            fill_ratio=area / max(1.0, float(width * height)),
        ),
        mask,
    )


def ray_plane_intersection(
    pixel_uv: Sequence[float],
    camera_matrix: Sequence[Sequence[float]],
    translation: Sequence[float],
    rotation: Sequence[Sequence[float]],
    plane_z: float,
) -> Optional[Point3]:
    """Intersect an optical-camera pixel ray with a horizontal base-frame plane."""
    u, v = (float(value) for value in pixel_uv)
    fx = float(camera_matrix[0][0])
    fy = float(camera_matrix[1][1])
    cx = float(camera_matrix[0][2])
    cy = float(camera_matrix[1][2])
    if abs(fx) < 1e-9 or abs(fy) < 1e-9:
        return None
    ray = ((u - cx) / fx, (v - cy) / fy, 1.0)
    base_direction = tuple(
        sum(float(rotation[row][column]) * ray[column] for column in range(3))
        for row in range(3)
    )
    if abs(base_direction[2]) < 1e-9:
        return None
    distance = (float(plane_z) - float(translation[2])) / base_direction[2]
    if distance <= 0.0:
        return None
    return tuple(
        float(translation[index]) + distance * base_direction[index]
        for index in range(3)
    )


def refine_target_from_depth(
    points: Iterable[Point3],
    seed_xy: Sequence[float],
    search_radius_m: float,
    minimum_z_m: float,
    maximum_z_m: float,
    minimum_points: int,
) -> Optional[Tuple[Point3, int]]:
    """Refine a color-derived XY seed using nearby above-table depth returns."""
    selected = select_target_depth_points(
        points,
        seed_xy,
        search_radius_m,
        minimum_z_m,
        maximum_z_m,
    )
    if len(selected) < max(1, int(minimum_points)):
        return None
    center = tuple(float(median(point[index] for point in selected)) for index in range(3))
    return center, len(selected)


def select_target_depth_points(
    points: Iterable[Point3],
    seed_xy: Sequence[float],
    search_radius_m: float,
    minimum_z_m: float,
    maximum_z_m: float,
) -> List[Point3]:
    """Select above-table depth points near an RGB-derived target seed."""
    radius_squared = max(0.001, float(search_radius_m)) ** 2
    return [
        point
        for point in points
        if minimum_z_m <= point[2] <= maximum_z_m
        and (point[0] - float(seed_xy[0])) ** 2
        + (point[1] - float(seed_xy[1])) ** 2
        <= radius_squared
    ]


def fuse_rgb_xy_with_depth_height(
    rgb_seed: Point3,
    depth_surface_center: Point3,
    object_height_m: float,
) -> Point3:
    """Use high-resolution RGB for XY and depth returns for object height."""
    return (
        float(rgb_seed[0]),
        float(rgb_seed[1]),
        max(0.0, float(depth_surface_center[2]) - 0.5 * float(object_height_m)),
    )


def main(args: List[str] | None = None) -> None:
    rclpy = require_ros2()
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header, String
    from tf2_ros import Buffer, TransformException, TransformListener

    class RgbdTargetPoseEstimator(Node):
        def __init__(self) -> None:
            super().__init__("rgbd_target_pose_estimator")
            self.declare_parameter("image_topic", "/camera/image_raw")
            self.declare_parameter("camera_info_topic", "/camera/camera_info")
            self.declare_parameter("points_topic", "/camera/depth/points")
            self.declare_parameter("base_frame", "panda_link0")
            self.declare_parameter("object_height_m", 0.08)
            self.declare_parameter("minimum_red_area_px", 80.0)
            self.declare_parameter("depth_search_radius_m", 0.065)
            self.declare_parameter("depth_min_z_m", 0.02)
            self.declare_parameter("depth_max_z_m", 0.12)
            self.declare_parameter("minimum_depth_points", 3)
            self.declare_parameter("rgb_x_bias_m", 0.0)
            self.declare_parameter("rgb_y_bias_m", 0.0)
            self.declare_parameter("maximum_rgb_age_s", 0.75)
            self.declare_parameter("smoothing_alpha", 0.50)
            self.declare_parameter("max_pose_jump_m", 0.10)
            self.declare_parameter("publish_debug_image", True)

            self.base_frame = str(self.get_parameter("base_frame").value)
            self.object_height_m = float(self.get_parameter("object_height_m").value)
            self.minimum_red_area_px = float(self.get_parameter("minimum_red_area_px").value)
            self.search_radius_m = float(self.get_parameter("depth_search_radius_m").value)
            self.depth_min_z_m = float(self.get_parameter("depth_min_z_m").value)
            self.depth_max_z_m = float(self.get_parameter("depth_max_z_m").value)
            self.minimum_depth_points = int(self.get_parameter("minimum_depth_points").value)
            self.rgb_x_bias_m = float(self.get_parameter("rgb_x_bias_m").value)
            self.rgb_y_bias_m = float(self.get_parameter("rgb_y_bias_m").value)
            self.maximum_rgb_age_ns = int(
                float(self.get_parameter("maximum_rgb_age_s").value) * 1_000_000_000
            )
            self.smoothing_alpha = float(self.get_parameter("smoothing_alpha").value)
            self.max_pose_jump_m = float(self.get_parameter("max_pose_jump_m").value)
            self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)

            self.camera_matrix: Optional[object] = None
            self.latest_seed: Optional[Point3] = None
            self.latest_region: Optional[ColorRegion] = None
            self.latest_rgb_stamp_ns = 0
            self.filtered_target_position: Optional[Vector3] = None
            self.bridge = None
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.pose_publisher = self.create_publisher(PoseStamped, "/detected_object_pose", 10)
            self.status_publisher = self.create_publisher(String, "/rgbd_target_detection", 10)
            self.debug_publisher = self.create_publisher(Image, "/rgbd_target/debug_image", 2)
            self.cluster_publisher = self.create_publisher(
                PointCloud2, "/rgbd_target/points", 2
            )
            self._init_cv_bridge()
            self.create_subscription(
                CameraInfo,
                str(self.get_parameter("camera_info_topic").value),
                self.on_camera_info,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Image,
                str(self.get_parameter("image_topic").value),
                self.on_image,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                PointCloud2,
                str(self.get_parameter("points_topic").value),
                self.on_points,
                qos_profile_sensor_data,
            )
            self.get_logger().info("Waiting for RGB, CameraInfo, depth points and camera TF.")

        def _init_cv_bridge(self) -> None:
            try:
                from cv_bridge import CvBridge

                self.bridge = CvBridge()
            except ImportError:
                self.publish_status("ERROR", "cv_bridge unavailable")

        def on_camera_info(self, message: CameraInfo) -> None:
            import numpy as np

            self.camera_matrix = np.array(message.k, dtype=float).reshape((3, 3))

        def on_image(self, message: Image) -> None:
            if self.bridge is None or self.camera_matrix is None:
                return
            import cv2

            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            region, mask = largest_red_region(image, self.minimum_red_area_px)
            if region is None:
                self.latest_seed = None
                self.latest_region = None
                self.publish_status("NOT_FOUND", "no red target region")
                self.publish_debug(message, image, mask, None)
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
                        self.base_frame, message.header.frame_id, Time()
                    )
                except TransformException as error:
                    self.publish_status("TF_UNAVAILABLE", str(error))
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
            seed = ray_plane_intersection(
                (region.centroid_u, region.centroid_v),
                self.camera_matrix,
                translation,
                quaternion_rotation_matrix(quaternion),
                self.object_height_m,
            )
            if seed is None:
                self.publish_status("INVALID_RAY", "target ray does not intersect workspace plane")
                return
            self.latest_seed = (
                seed[0] + self.rgb_x_bias_m,
                seed[1] + self.rgb_y_bias_m,
                seed[2],
            )
            self.latest_region = region
            self.latest_rgb_stamp_ns = self._stamp_ns(message.header.stamp)
            self.publish_debug(message, image, mask, region)

        def on_points(self, message: PointCloud2) -> None:
            if self.latest_seed is None or self.latest_region is None:
                return
            points_stamp_ns = self._stamp_ns(message.header.stamp)
            if (
                self.latest_rgb_stamp_ns > 0
                and points_stamp_ns > 0
                and abs(points_stamp_ns - self.latest_rgb_stamp_ns) > self.maximum_rgb_age_ns
            ):
                self.publish_status("STALE_RGB", "RGB segmentation is older than depth data")
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
                        self.base_frame, message.header.frame_id, Time()
                    )
                except TransformException as error:
                    self.publish_status("TF_UNAVAILABLE", str(error))
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
                message, field_names=["x", "y", "z"], skip_nans=True
            )
            transformed = [
                transform_point(
                    (float(row[0]), float(row[1]), float(row[2])),
                    translation,
                    rotation,
                )
                for row in raw
            ]
            selected_points = select_target_depth_points(
                transformed,
                self.latest_seed[:2],
                self.search_radius_m,
                self.depth_min_z_m,
                self.depth_max_z_m,
            )
            refined = refine_target_from_depth(
                selected_points,
                self.latest_seed[:2],
                self.search_radius_m * 2.0,
                self.depth_min_z_m,
                self.depth_max_z_m,
                self.minimum_depth_points,
            )
            if refined is None:
                self.publish_status(
                    "NO_DEPTH_CONFIRMATION",
                    "red region found but no matching above-table depth cluster",
                )
                return
            cluster_header = Header()
            cluster_header.stamp = message.header.stamp
            cluster_header.frame_id = self.base_frame
            self.cluster_publisher.publish(
                point_cloud2.create_cloud_xyz32(cluster_header, selected_points)
            )
            depth_center, depth_count = refined
            fused_center = fuse_rgb_xy_with_depth_height(
                self.latest_seed,
                depth_center,
                self.object_height_m,
            )
            measured = Vector3(*fused_center)
            filtered = filtered_position(
                self.filtered_target_position,
                measured,
                self.smoothing_alpha,
                self.max_pose_jump_m,
            )
            if filtered is None:
                self.publish_status("REJECTED_JUMP", "fused pose exceeded max_pose_jump_m")
                return
            self.filtered_target_position = filtered
            pose = PoseSpec(
                frame_id=self.base_frame,
                position=filtered,
                orientation=quaternion_from_euler(pi, 0.0, 0.0),
            )
            self.pose_publisher.publish(self._to_pose_stamped(pose))
            self.publish_status(
                "DETECTED",
                "RGB segmentation confirmed and refined by depth points",
                position=filtered,
                depth_points=depth_count,
                depth_surface_z=depth_center[2],
            )

        def publish_debug(
            self,
            source: Image,
            image: object,
            mask: object,
            region: Optional[ColorRegion],
        ) -> None:
            if not self.publish_debug_image or self.bridge is None:
                return
            import cv2
            import numpy as np

            overlay = image.copy()
            overlay[mask > 0] = (
                0.35 * overlay[mask > 0]
                + 0.65 * np.array((0, 255, 255), dtype=np.float32)
            ).astype("uint8")
            if region is not None:
                x, y, width, height = region.bounding_box
                cv2.rectangle(overlay, (x, y), (x + width, y + height), (30, 255, 30), 2)
                cv2.drawMarker(
                    overlay,
                    (int(region.centroid_u), int(region.centroid_v)),
                    (255, 255, 0),
                    cv2.MARKER_CROSS,
                    16,
                    2,
                )
                cv2.putText(
                    overlay,
                    f"markerless target area={region.area_px:.0f}px",
                    (max(4, x), max(22, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (30, 255, 30),
                    2,
                )
            output = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            output.header = source.header
            self.debug_publisher.publish(output)

        def publish_status(
            self,
            state: str,
            reason: str,
            position: Optional[Vector3] = None,
            depth_points: Optional[int] = None,
            depth_surface_z: Optional[float] = None,
        ) -> None:
            payload = {"state": state, "reason": reason, "source": "rgbd_markerless"}
            if self.latest_region is not None:
                payload.update(
                    {
                        "pixel_centroid": [
                            self.latest_region.centroid_u,
                            self.latest_region.centroid_v,
                        ],
                        "segmentation_area_px": self.latest_region.area_px,
                        "segmentation_fill_ratio": self.latest_region.fill_ratio,
                    }
                )
            if self.latest_seed is not None:
                payload["rgb_seed_base"] = list(self.latest_seed)
            if position is not None:
                payload["position"] = position.as_dict()
            if depth_points is not None:
                payload["depth_points"] = depth_points
            if depth_surface_z is not None:
                payload["depth_surface_z_m"] = depth_surface_z
            message = String()
            message.data = json.dumps(payload, sort_keys=True)
            self.status_publisher.publish(message)

        def _to_pose_stamped(self, pose: PoseSpec) -> PoseStamped:
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
            return message

        @staticmethod
        def _stamp_ns(stamp: object) -> int:
            return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    rclpy.init(args=args)
    node = RgbdTargetPoseEstimator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
