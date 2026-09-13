"""Small geometry helpers kept independent from ROS message classes."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, asin, cos, sin, sqrt
from typing import Dict, Iterable, Tuple


@dataclass(frozen=True)
class Vector3:
    x: float
    y: float
    z: float

    def as_dict(self) -> Dict[str, float]:
        return {"x": self.x, "y": self.y, "z": self.z}


@dataclass(frozen=True)
class Quaternion:
    x: float
    y: float
    z: float
    w: float

    def normalized(self) -> "Quaternion":
        norm = sqrt(self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w)
        if norm == 0:
            return Quaternion(0.0, 0.0, 0.0, 1.0)
        return Quaternion(self.x / norm, self.y / norm, self.z / norm, self.w / norm)

    def as_dict(self) -> Dict[str, float]:
        q = self.normalized()
        return {"x": q.x, "y": q.y, "z": q.z, "w": q.w}


@dataclass(frozen=True)
class PoseSpec:
    frame_id: str
    position: Vector3
    orientation: Quaternion

    def translated(self, dx: float, dy: float, dz: float) -> "PoseSpec":
        return PoseSpec(
            frame_id=self.frame_id,
            position=Vector3(
                self.position.x + dx,
                self.position.y + dy,
                self.position.z + dz,
            ),
            orientation=self.orientation,
        )

    def as_dict(self) -> Dict[str, object]:
        return {
            "frame_id": self.frame_id,
            "position": self.position.as_dict(),
            "orientation": self.orientation.as_dict(),
        }


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)

    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    ).normalized()


def euler_from_quaternion(q: Quaternion) -> Tuple[float, float, float]:
    q = q.normalized()
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1.0:
        pitch = 1.5707963267948966 if sinp > 0 else -1.5707963267948966
    else:
        pitch = asin(sinp)

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def pose_distance(a: PoseSpec, b: PoseSpec) -> float:
    return sqrt(
        (a.position.x - b.position.x) ** 2
        + (a.position.y - b.position.y) ** 2
        + (a.position.z - b.position.z) ** 2
    )


def quaternion_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    return Quaternion(
        x=a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y=a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z=a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        w=a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    )


def rotate_vector(vector: Vector3, rotation: Quaternion) -> Vector3:
    q = rotation.normalized()
    vector_quaternion = Quaternion(vector.x, vector.y, vector.z, 0.0)
    conjugate = Quaternion(-q.x, -q.y, -q.z, q.w)
    rotated = quaternion_multiply(quaternion_multiply(q, vector_quaternion), conjugate)
    return Vector3(rotated.x, rotated.y, rotated.z)


def relative_pose(reference: PoseSpec, target: PoseSpec, frame_id: str) -> PoseSpec:
    """Express target pose in the coordinate frame of reference."""
    reference_q = reference.orientation.normalized()
    inverse_reference_q = Quaternion(
        -reference_q.x,
        -reference_q.y,
        -reference_q.z,
        reference_q.w,
    )
    world_delta = Vector3(
        target.position.x - reference.position.x,
        target.position.y - reference.position.y,
        target.position.z - reference.position.z,
    )
    return PoseSpec(
        frame_id=frame_id,
        position=rotate_vector(world_delta, inverse_reference_q),
        orientation=quaternion_multiply(inverse_reference_q, target.orientation).normalized(),
    )


def compose_pose(reference: PoseSpec, local: PoseSpec, frame_id: str) -> PoseSpec:
    """Apply a reference-frame transform to a pose expressed in that frame."""
    rotated_position = rotate_vector(local.position, reference.orientation)
    return PoseSpec(
        frame_id=frame_id,
        position=Vector3(
            reference.position.x + rotated_position.x,
            reference.position.y + rotated_position.y,
            reference.position.z + rotated_position.z,
        ),
        orientation=quaternion_multiply(
            reference.orientation,
            local.orientation,
        ).normalized(),
    )


def pose_from_dict(data: Dict[str, object]) -> PoseSpec:
    position_data = data.get("position", {})
    orientation_data = data.get("orientation", {})
    return PoseSpec(
        frame_id=str(data.get("frame_id", "panda_link0")),
        position=Vector3(
            float(position_data.get("x", 0.0)),
            float(position_data.get("y", 0.0)),
            float(position_data.get("z", 0.0)),
        ),
        orientation=Quaternion(
            float(orientation_data.get("x", 0.0)),
            float(orientation_data.get("y", 0.0)),
            float(orientation_data.get("z", 0.0)),
            float(orientation_data.get("w", 1.0)),
        ).normalized(),
    )


def average_path_length(poses: Iterable[PoseSpec]) -> float:
    poses = list(poses)
    if len(poses) < 2:
        return 0.0
    return sum(pose_distance(a, b) for a, b in zip(poses[:-1], poses[1:]))
