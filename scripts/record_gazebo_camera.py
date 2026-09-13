#!/usr/bin/env python3
import argparse
import json
import time
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class CameraRecorder(Node):
    def __init__(
        self,
        output: Path,
        fps: float,
        timeout_s: float,
        post_result_s: float,
        image_topic: str,
        result_topic: str,
    ) -> None:
        super().__init__("gazebo_camera_recorder")
        self.output = output
        self.fps = fps
        self.timeout_s = timeout_s
        self.post_result_s = post_result_s
        self.started = time.monotonic()
        self.finished_at = None
        self.frames = 0
        self.bridge = CvBridge()
        self.writer = None
        self.create_subscription(
            Image,
            image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            result_topic,
            self._on_validation,
            10,
        )

    def _on_image(self, message: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        if self.writer is None:
            height, width = frame.shape[:2]
            self.output.parent.mkdir(parents=True, exist_ok=True)
            self.writer = cv2.VideoWriter(
                str(self.output),
                cv2.VideoWriter_fourcc(*"mp4v"),
                self.fps,
                (width, height),
            )
            if not self.writer.isOpened():
                raise RuntimeError(f"could not open video writer for {self.output}")
        self.writer.write(frame)
        self.frames += 1

    def _on_validation(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if payload.get("final_state") in {"SUCCESS", "FAILED"}:
            self.finished_at = self.finished_at or time.monotonic()

    def done(self) -> bool:
        now = time.monotonic()
        return (
            now - self.started >= self.timeout_s
            or self.finished_at is not None
            and now - self.finished_at >= self.post_result_s
        )

    def close(self) -> None:
        if self.writer is not None:
            self.writer.release()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Record the Gazebo RGB camera topic as an MP4 demo."
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--timeout-s", type=float, default=300.0)
    parser.add_argument("--post-result-s", type=float, default=3.0)
    parser.add_argument("--image-topic", default="/camera/image_raw")
    parser.add_argument("--result-topic", default="/gazebo_pick_validation")
    args = parser.parse_args()

    rclpy.init()
    recorder = CameraRecorder(
        args.output,
        args.fps,
        args.timeout_s,
        args.post_result_s,
        args.image_topic,
        args.result_topic,
    )
    try:
        while rclpy.ok() and not recorder.done():
            rclpy.spin_once(recorder, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        recorder.close()
        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print(f"Recorded {recorder.frames} frames to {args.output}", flush=True)
    return 0 if recorder.frames > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
