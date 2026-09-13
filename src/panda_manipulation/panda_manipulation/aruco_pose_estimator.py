"""Estimate a filtered target pose from ArUco or publish a synthetic pose."""

from __future__ import annotations

import json
from math import pi, sqrt
from typing import List, Optional, Sequence

from .geometry import (
    PoseSpec,
    Quaternion,
    Vector3,
    compose_pose,
    quaternion_from_euler,
)
from .ros_helpers import require_ros2


def marker_index(marker_ids: Sequence[int], target_marker_id: int) -> int | None:
    """Return the requested marker index without silently accepting another ID."""
    for index, marker_id in enumerate(marker_ids):
        if int(marker_id) == target_marker_id:
            return index
    return None


def filtered_position(
    previous: Vector3 | None,
    measured: Vector3,
    smoothing_alpha: float,
    max_jump_m: float,
) -> Vector3 | None:
    """Reject implausible jumps and apply an exponential position filter."""
    if previous is None:
        return measured
    jump = sqrt(
        (measured.x - previous.x) ** 2
        + (measured.y - previous.y) ** 2
        + (measured.z - previous.z) ** 2
    )
    if jump > max_jump_m:
        return None
    alpha = max(0.0, min(1.0, smoothing_alpha))
    return Vector3(
        alpha * measured.x + (1.0 - alpha) * previous.x,
        alpha * measured.y + (1.0 - alpha) * previous.y,
        alpha * measured.z + (1.0 - alpha) * previous.z,
    )


def main(args: List[str] | None = None) -> None:
    rclpy = require_ros2()
    from geometry_msgs.msg import PoseStamped
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformException, TransformListener

    class ArucoPoseEstimator(Node):
        def __init__(self) -> None:
            super().__init__("aruco_pose_estimator")
            self.declare_parameter("use_synthetic_pose", True)
            self.declare_parameter("base_frame", "panda_link0")
            self.declare_parameter(
                "camera_frame",
                "overhead_camera_optical_frame",
            )
            self.declare_parameter("marker_dictionary", "DICT_4X4_50")
            self.declare_parameter("marker_id", 0)
            self.declare_parameter("marker_size_m", 0.045)
            self.declare_parameter("object_center_offset_z_m", -0.04)
            self.declare_parameter("smoothing_alpha", 0.35)
            self.declare_parameter("max_pose_jump_m", 0.08)
            self.declare_parameter("tf_timeout_s", 0.2)
            self.declare_parameter("target_x_m", 0.52)
            self.declare_parameter("target_y_m", 0.0)
            self.declare_parameter("target_z_m", 0.08)

            self.use_synthetic_pose = bool(
                self.get_parameter("use_synthetic_pose").value
            )
            self.base_frame = str(self.get_parameter("base_frame").value)
            self.camera_frame = str(self.get_parameter("camera_frame").value)
            self.marker_dictionary = str(
                self.get_parameter("marker_dictionary").value
            )
            self.target_marker_id = int(self.get_parameter("marker_id").value)
            self.marker_size_m = float(
                self.get_parameter("marker_size_m").value
            )
            self.object_center_offset_z_m = float(
                self.get_parameter("object_center_offset_z_m").value
            )
            self.smoothing_alpha = float(
                self.get_parameter("smoothing_alpha").value
            )
            self.max_pose_jump_m = float(
                self.get_parameter("max_pose_jump_m").value
            )
            self.tf_timeout_s = float(
                self.get_parameter("tf_timeout_s").value
            )
            self.target_x_m = float(self.get_parameter("target_x_m").value)
            self.target_y_m = float(self.get_parameter("target_y_m").value)
            self.target_z_m = float(self.get_parameter("target_z_m").value)

            self.publisher = self.create_publisher(
                PoseStamped,
                "/detected_object_pose",
                10,
            )
            self.status_publisher = self.create_publisher(
                String,
                "/aruco_detection_status",
                10,
            )
            self.camera_matrix: Optional[object] = None
            self.distortion_coefficients: Optional[object] = None
            self.filtered_target_position: Vector3 | None = None
            self.bridge = None
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

            if self.use_synthetic_pose:
                self.timer = self.create_timer(0.5, self.publish_synthetic_pose)
                self.get_logger().info(
                    "Using synthetic target pose for fast pipeline validation."
                )
            else:
                self._init_cv_bridge()
                self.create_subscription(
                    CameraInfo,
                    "/camera/camera_info",
                    self.on_camera_info,
                    qos_profile_sensor_data,
                )
                self.create_subscription(
                    Image,
                    "/camera/image_raw",
                    self.on_image,
                    qos_profile_sensor_data,
                )
                self.get_logger().info(
                    "Waiting for camera image, camera_info and camera TF."
                )

        def _init_cv_bridge(self) -> None:
            try:
                from cv_bridge import CvBridge

                self.bridge = CvBridge()
            except ImportError:
                self.publish_status("ERROR", "cv_bridge unavailable")
                self.get_logger().error(
                    "cv_bridge is unavailable. Install ros-jazzy-cv-bridge."
                )

        def publish_synthetic_pose(self) -> None:
            pose = PoseSpec(
                frame_id=self.base_frame,
                position=Vector3(
                    self.target_x_m,
                    self.target_y_m,
                    self.target_z_m,
                ),
                orientation=quaternion_from_euler(pi, 0.0, 0.0),
            )
            self.publisher.publish(self._to_pose_stamped(pose))
            self.publish_status("SYNTHETIC", "synthetic target pose published")

        def on_camera_info(self, message: CameraInfo) -> None:
            try:
                import numpy as np

                self.camera_matrix = np.array(
                    message.k,
                    dtype=float,
                ).reshape((3, 3))
                self.distortion_coefficients = np.array(
                    message.d,
                    dtype=float,
                )
            except ImportError:
                self.publish_status("ERROR", "numpy unavailable")
                self.get_logger().error(
                    "numpy is required for ArUco pose estimation."
                )

        def on_image(self, message: Image) -> None:
            if (
                self.bridge is None
                or self.camera_matrix is None
                or self.distortion_coefficients is None
            ):
                return
            try:
                import cv2
            except ImportError:
                self.publish_status("ERROR", "OpenCV unavailable")
                self.get_logger().error(
                    "OpenCV is required for ArUco pose estimation."
                )
                return

            image = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="bgr8",
            )
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            dictionary_id = getattr(
                cv2.aruco,
                self.marker_dictionary,
                None,
            )
            if dictionary_id is None:
                self.publish_status(
                    "ERROR",
                    f"unknown dictionary {self.marker_dictionary}",
                )
                return
            dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
            corners, detected_ids, _ = cv2.aruco.detectMarkers(
                gray,
                dictionary,
            )
            if detected_ids is None or len(detected_ids) == 0:
                self.publish_status("NOT_FOUND", "no marker detected")
                return

            ids = [int(value) for value in detected_ids.flatten()]
            selected_index = marker_index(ids, self.target_marker_id)
            if selected_index is None:
                self.publish_status(
                    "WRONG_ID",
                    f"expected {self.target_marker_id}, detected {ids}",
                )
                return

            _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                [corners[selected_index]],
                self.marker_size_m,
                self.camera_matrix,
                self.distortion_coefficients,
            )
            tvec = tvecs[0][0]
            camera_pose = PoseSpec(
                frame_id=message.header.frame_id or self.camera_frame,
                position=Vector3(
                    float(tvec[0]),
                    float(tvec[1]),
                    float(tvec[2]),
                ),
                orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
            )
            base_pose = self.transform_to_base(camera_pose)
            if base_pose is None:
                return
            object_position = Vector3(
                base_pose.position.x,
                base_pose.position.y,
                base_pose.position.z + self.object_center_offset_z_m,
            )
            filtered = filtered_position(
                self.filtered_target_position,
                object_position,
                self.smoothing_alpha,
                self.max_pose_jump_m,
            )
            if filtered is None:
                self.publish_status(
                    "REJECTED_JUMP",
                    "detected pose exceeded max_pose_jump_m",
                )
                return
            self.filtered_target_position = filtered
            target_pose = PoseSpec(
                frame_id=self.base_frame,
                position=filtered,
                # The marker orientation is not used as a grasp orientation.
                # The top-down Panda grasp remains stable under marker yaw.
                orientation=quaternion_from_euler(pi, 0.0, 0.0),
            )
            self.publisher.publish(self._to_pose_stamped(target_pose))
            self.publish_status(
                "DETECTED",
                "camera pose transformed and filtered",
                ids=ids,
                position=filtered,
            )

        def transform_to_base(self, pose: PoseSpec) -> PoseSpec | None:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    pose.frame_id,
                    Time(),
                    timeout=Duration(seconds=self.tf_timeout_s),
                )
            except TransformException as error:
                self.publish_status("TF_UNAVAILABLE", str(error))
                return None
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            reference = PoseSpec(
                frame_id=self.base_frame,
                position=Vector3(
                    float(translation.x),
                    float(translation.y),
                    float(translation.z),
                ),
                orientation=Quaternion(
                    float(rotation.x),
                    float(rotation.y),
                    float(rotation.z),
                    float(rotation.w),
                ),
            )
            return compose_pose(reference, pose, self.base_frame)

        def publish_status(
            self,
            state: str,
            reason: str,
            ids: Sequence[int] | None = None,
            position: Vector3 | None = None,
        ) -> None:
            payload = {
                "state": state,
                "reason": reason,
                "marker_id": self.target_marker_id,
            }
            if ids is not None:
                payload["detected_ids"] = list(ids)
            if position is not None:
                payload["position"] = position.as_dict()
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

    rclpy.init(args=args)
    node = ArucoPoseEstimator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
