"""Grasp candidate generation for tabletop pick-and-place tasks."""

from __future__ import annotations

from dataclasses import dataclass
from copy import deepcopy
from math import isfinite, radians
from typing import Dict, Iterable, List, Optional

from .geometry import (PoseSpec, pose_distance, relative_pose, pose_from_dict,
                       quaternion_from_euler, quaternion_multiply, rotate_vector)


def tilted_grasp_candidate(candidate, pitch_degrees):
    """Tilt about the hand's closing axis while retaining the placed object pose."""
    if not isfinite(pitch_degrees) or abs(pitch_degrees) > 20:
        raise ValueError("experimental grasp pitch must be finite and within +/-20 degrees")
    result = deepcopy(candidate)
    if abs(pitch_degrees) < 1e-9:
        return result
    original_hand = pose_from_dict(candidate["grasp_pose"])
    target = pose_from_dict(candidate["object_pose"])
    local = pose_from_dict(candidate["object_in_hand_pose"])
    if original_hand.frame_id != target.frame_id:
        raise ValueError("grasp and object frames differ")
    rotation = quaternion_from_euler(0.0, radians(pitch_degrees), 0.0)
    for key in ("pre_grasp_pose", "grasp_pose", "lift_pose", "place_pose",
                "pre_place_pose", "retreat_pose"):
        if key not in result:
            continue
        old = pose_from_dict(result[key])
        result[key]["orientation"] = quaternion_multiply(old.orientation, rotation).normalized().as_dict()
    offset = rotate_vector(local.position, pose_from_dict(result["grasp_pose"]).orientation)
    new_position = {axis: getattr(target.position, axis) - getattr(offset, axis)
                    for axis in ("x", "y", "z")}
    delta = {axis: new_position[axis] - getattr(original_hand.position, axis)
             for axis in ("x", "y", "z")}
    # Place positions denote object goals; only pick-side hand positions shift.
    for key in ("pre_grasp_pose", "grasp_pose", "lift_pose"):
        for axis in ("x", "y", "z"):
            result[key]["position"][axis] += delta[axis]
    result["object_in_hand_pose"] = relative_pose(
        pose_from_dict(result["grasp_pose"]), target, "panda_hand"
    ).as_dict()
    result["grasp_pitch_offset_deg"] = float(pitch_degrees)
    return result


@dataclass(frozen=True)
class GraspSequence:
    candidate_id: str
    object_pose: PoseSpec
    object_in_hand_pose: PoseSpec
    pre_grasp_pose: PoseSpec
    grasp_pose: PoseSpec
    lift_pose: PoseSpec
    place_pose: PoseSpec
    retreat_pose: PoseSpec
    score: float

    def as_dict(self) -> Dict[str, object]:
        return {
            "candidate_id": self.candidate_id,
            "object_pose": self.object_pose.as_dict(),
            "object_in_hand_pose": self.object_in_hand_pose.as_dict(),
            "pre_grasp_pose": self.pre_grasp_pose.as_dict(),
            "grasp_pose": self.grasp_pose.as_dict(),
            "lift_pose": self.lift_pose.as_dict(),
            "place_pose": self.place_pose.as_dict(),
            "retreat_pose": self.retreat_pose.as_dict(),
            "score": self.score,
        }


def generate_grasp_sequences(
    target_pose: PoseSpec,
    place_pose: PoseSpec,
    approach_distance: float = 0.12,
    grasp_height_offset: float = 0.12,
    lift_distance: float = 0.15,
    retreat_distance: float = 0.10,
    lateral_offsets: Iterable[float] = (-0.04, 0.0, 0.04, 0.08),
    pre_grasp_height_offset: Optional[float] = None,
    retreat_height_offset: Optional[float] = None,
) -> List[GraspSequence]:
    sequences: List[GraspSequence] = []
    approach_height = (
        lift_distance * 0.5
        if pre_grasp_height_offset is None
        else pre_grasp_height_offset
    )
    for index, lateral_offset in enumerate(lateral_offsets):
        grasp_pose = target_pose.translated(0.0, lateral_offset, grasp_height_offset)
        pre_grasp_pose = grasp_pose.translated(-approach_distance, 0.0, approach_height)
        lift_pose = grasp_pose.translated(0.0, 0.0, lift_distance)
        retreat_pose = (
            place_pose.translated(-retreat_distance, 0.0, 0.0)
            if retreat_height_offset is None
            else place_pose.translated(0.0, 0.0, retreat_height_offset)
        )
        score = _candidate_score(pre_grasp_pose, grasp_pose, lift_pose, place_pose, lateral_offset)
        sequences.append(
            GraspSequence(
                candidate_id=f"grasp_{index:02d}",
                object_pose=target_pose,
                object_in_hand_pose=relative_pose(grasp_pose, target_pose, "panda_hand"),
                pre_grasp_pose=pre_grasp_pose,
                grasp_pose=grasp_pose,
                lift_pose=lift_pose,
                place_pose=place_pose,
                retreat_pose=retreat_pose,
                score=score,
            )
        )
    return sorted(sequences, key=lambda candidate: candidate.score, reverse=True)


def select_best_reachable_candidate(
    candidates: Iterable[GraspSequence],
    workspace_radius: float = 0.85,
    min_z: float = 0.02,
) -> GraspSequence | None:
    for candidate in candidates:
        poses = [
            candidate.pre_grasp_pose,
            candidate.grasp_pose,
            candidate.lift_pose,
            candidate.place_pose,
            candidate.retreat_pose,
        ]
        if all(_pose_is_reachable(pose, workspace_radius, min_z) for pose in poses):
            return candidate
    return None


def _pose_is_reachable(pose: PoseSpec, workspace_radius: float, min_z: float) -> bool:
    radius = (pose.position.x**2 + pose.position.y**2 + pose.position.z**2) ** 0.5
    return radius <= workspace_radius and pose.position.z >= min_z


def _candidate_score(
    pre_grasp_pose: PoseSpec,
    grasp_pose: PoseSpec,
    lift_pose: PoseSpec,
    place_pose: PoseSpec,
    lateral_offset: float,
) -> float:
    travel = (
        pose_distance(pre_grasp_pose, grasp_pose)
        + pose_distance(grasp_pose, lift_pose)
        + pose_distance(lift_pose, place_pose)
    )
    offset_penalty = abs(lateral_offset) * 2.0
    return max(0.0, 1.0 / (1.0 + travel + offset_penalty))
