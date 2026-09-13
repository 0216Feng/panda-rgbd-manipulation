"""Hybrid pick plan pipeline: OMPL global plans plus Cartesian approach."""

from __future__ import annotations

from copy import deepcopy
import json
from math import acos, isfinite, radians, sqrt
import time
from typing import Dict, List, Optional, Tuple

from .geometry import (
    PoseSpec,
    Quaternion,
    Vector3,
    compose_pose,
    quaternion_from_euler,
    quaternion_multiply,
    pose_from_dict,
    relative_pose,
    rotate_vector,
)
from .grasp import tilted_grasp_candidate
from .moveit_plan_only_adapter import selected_candidate
from .ros_helpers import require_ros2


PANDA_JOINT_LIMITS = {
    "panda_joint1": (-2.8973, 2.8973),
    "panda_joint2": (-1.7628, 1.7628),
    "panda_joint3": (-2.8973, 2.8973),
    "panda_joint4": (-3.0718, -0.0698),
    "panda_joint5": (-2.8973, 2.8973),
    "panda_joint6": (-0.0175, 3.7525),
    "panda_joint7": (-2.8973, 2.8973),
    "panda_finger_joint1": (0.0, 0.04),
    "panda_finger_joint2": (0.0, 0.04),
}
PANDA_ARM_JOINTS = [f"panda_joint{index}" for index in range(1, 8)]
SERVO_HALT_STATUS_CODES = {-1, 2, 5, 6}


def bounded_servo_velocity(
    remaining_distance_m: float,
    speed_mps: float,
    command_period_s: float,
    position_tolerance_m: float,
) -> float:
    """Return a signed Servo speed that cannot overshoot in one command period."""
    remaining = float(remaining_distance_m)
    tolerance = max(0.0, float(position_tolerance_m))
    if abs(remaining) <= tolerance:
        return 0.0
    period = max(1e-4, float(command_period_s))
    speed = max(0.0, float(speed_mps))
    magnitude = min(speed, abs(remaining) / period)
    return magnitude if remaining > 0.0 else -magnitude


def servo_collision_limited_endpoint_acceptable(
    remaining_distance_m: float,
    stage_index: int,
    stage_count: int,
    collision_deceleration_seen: bool,
    acceptance_m: float,
) -> bool:
    """Accept only a final downward stage safely limited near scene collision."""
    return (
        bool(collision_deceleration_seen)
        and int(stage_count) > 0
        and int(stage_index) == int(stage_count) - 1
        and float(remaining_distance_m) < 0.0
        and abs(float(remaining_distance_m)) <= max(0.0, float(acceptance_m))
    )


def servo_halt_allows_cartesian_fallback(
    status_code: Optional[int],
    enabled: bool,
) -> bool:
    """Allow measured-state replanning only for a Servo singularity halt."""
    return bool(enabled) and status_code == 2


def servo_fault_injection_due(
    fault_type: str,
    target_stage_number: int,
    current_stage_index: int,
    elapsed_s: float,
    delay_s: float,
) -> bool:
    """Return whether an explicitly configured watchdog fault is now due."""
    return (
        str(fault_type).strip().lower() in {"contact_loss", "payload_drift"}
        and int(target_stage_number) == int(current_stage_index) + 1
        and float(elapsed_s) >= max(0.0, float(delay_s))
    )


def parse_joint_positions_csv(value: str) -> List[float]:
    """Parse one complete finite Panda arm joint vector from a launch value."""
    tokens = [token.strip() for token in str(value).split(",") if token.strip()]
    if not tokens:
        return []
    if len(tokens) != len(PANDA_ARM_JOINTS):
        raise ValueError(
            f"expected {len(PANDA_ARM_JOINTS)} arm joints, got {len(tokens)}"
        )
    positions = [float(token) for token in tokens]
    if not all(isfinite(position) for position in positions):
        raise ValueError("joint replay positions must all be finite")
    return positions


def ordered_joint_positions(
    joint_names: List[str],
    joint_positions: List[float],
    expected_names: List[str] = PANDA_ARM_JOINTS,
) -> Optional[List[float]]:
    """Order a trajectory endpoint by Panda joint name."""
    positions = {
        str(name): float(position)
        for name, position in zip(joint_names, joint_positions)
        if isfinite(float(position))
    }
    if any(name not in positions for name in expected_names):
        return None
    return [positions[name] for name in expected_names]


def max_joint_position_delta(
    first: List[float],
    second: List[float],
) -> Optional[float]:
    """Return the maximum paired joint delta for complete equal-size vectors."""
    if not first or len(first) != len(second):
        return None
    return max(abs(float(a) - float(b)) for a, b in zip(first, second))


def rank_reverse_joint_branches(
    start_state: RobotState,
    reports: List[Dict[str, object]],
) -> List[Dict[str, object]]:
    """Prefer valid reverse branches closest to the measured loaded state."""
    current = ordered_joint_positions(
        list(start_state.joint_state.name), list(start_state.joint_state.position))
    if current is None:
        return []
    ranked = []
    for report in reports:
        if not bool(report.get("reverse_valid", False)):
            continue
        target = ordered_joint_positions(
            list(report.get("pre_grasp_joint_names", [])),
            list(report.get("pre_grasp_joint_positions", [])))
        distance = max_joint_position_delta(current, target or [])
        if distance is None or not isfinite(distance):
            continue
        item = deepcopy(report)
        item["loaded_state_max_joint_delta_rad"] = distance
        ranked.append(item)
    return sorted(
        ranked,
        key=lambda item: (
            float(item["loaded_state_max_joint_delta_rad"]),
            -float(item.get("joint_limit_margin", 0.0)),
            int(item.get("seed_index", 0)),
        ),
    )


def joint_trajectory_quality_metrics(
    joint_names: List[str],
    points: List[object],
) -> Dict[str, object]:
    """Summarize timing and continuity of a planned joint trajectory."""
    metrics: Dict[str, object] = {
        "point_count": len(points),
        "joint_count": len(joint_names),
        "duration_s": 0.0,
        "min_segment_duration_s": None,
        "max_joint_step_rad": 0.0,
        "max_joint_step_joint": None,
        "max_implied_velocity_rad_s": 0.0,
        "max_implied_velocity_joint": None,
        "max_reported_velocity_rad_s": 0.0,
        "joint_path_length_rad": 0.0,
        "endpoint_joint_displacement_rad": 0.0,
        "joint_path_tortuosity": None,
        "terminal_max_abs_velocity_rad_s": None,
        "terminal_max_abs_acceleration_rad_s2": None,
    }
    if not points:
        return metrics

    def point_time_s(point: object) -> float:
        stamp = point.time_from_start
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    times = [point_time_s(point) for point in points]
    metrics["duration_s"] = times[-1]
    min_segment_duration_s: Optional[float] = None
    max_joint_step = 0.0
    max_joint_step_joint: Optional[str] = None
    max_implied_velocity = 0.0
    max_implied_velocity_joint: Optional[str] = None
    max_reported_velocity = 0.0

    for index, point in enumerate(points):
        velocities = list(getattr(point, "velocities", []))
        for velocity in velocities[: len(joint_names)]:
            magnitude = abs(float(velocity))
            if isfinite(magnitude):
                max_reported_velocity = max(max_reported_velocity, magnitude)
        if index == 0:
            continue
        segment_duration = times[index] - times[index - 1]
        if segment_duration > 0.0:
            min_segment_duration_s = (
                segment_duration
                if min_segment_duration_s is None
                else min(min_segment_duration_s, segment_duration)
            )
        previous_positions = list(getattr(points[index - 1], "positions", []))
        positions = list(getattr(point, "positions", []))
        count = min(len(joint_names), len(previous_positions), len(positions))
        for joint_index in range(count):
            step = abs(float(positions[joint_index]) - float(previous_positions[joint_index]))
            if not isfinite(step):
                continue
            if step > max_joint_step:
                max_joint_step = step
                max_joint_step_joint = joint_names[joint_index]
            if segment_duration > 0.0:
                implied_velocity = step / segment_duration
                if implied_velocity > max_implied_velocity:
                    max_implied_velocity = implied_velocity
                    max_implied_velocity_joint = joint_names[joint_index]

    terminal = points[-1]
    position_rows = [
        [float(value) for value in list(getattr(point, "positions", []))]
        for point in points
    ]
    complete_rows = [
        row[: len(joint_names)]
        for row in position_rows
        if len(row) >= len(joint_names)
    ]
    path_length = joint_path_length(complete_rows)
    endpoint_displacement = 0.0
    if len(complete_rows) >= 2:
        endpoint_displacement = sqrt(
            sum(
                (end - start) ** 2
                for start, end in zip(complete_rows[0], complete_rows[-1])
            )
        )
    terminal_velocities = [
        abs(float(value))
        for value in list(getattr(terminal, "velocities", []))
        if isfinite(float(value))
    ]
    terminal_accelerations = [
        abs(float(value))
        for value in list(getattr(terminal, "accelerations", []))
        if isfinite(float(value))
    ]
    metrics.update(
        {
            "min_segment_duration_s": min_segment_duration_s,
            "max_joint_step_rad": max_joint_step,
            "max_joint_step_joint": max_joint_step_joint,
            "max_implied_velocity_rad_s": max_implied_velocity,
            "max_implied_velocity_joint": max_implied_velocity_joint,
            "max_reported_velocity_rad_s": max_reported_velocity,
            "joint_path_length_rad": path_length,
            "endpoint_joint_displacement_rad": endpoint_displacement,
            "joint_path_tortuosity": (
                path_length / endpoint_displacement
                if endpoint_displacement > 1e-9
                else None
            ),
            "terminal_max_abs_velocity_rad_s": (
                max(terminal_velocities) if terminal_velocities else None
            ),
            "terminal_max_abs_acceleration_rad_s2": (
                max(terminal_accelerations) if terminal_accelerations else None
            ),
        }
    )
    return metrics


def summarize_cpp_trajectory_metrics(
    messages: List[Dict[str, object]],
) -> Dict[str, object]:
    """Aggregate C++ metrics without recomputing the trajectory in Python."""
    aggregates = [
        message.get("aggregate")
        for message in messages
        if isinstance(message, dict)
        and message.get("schema_version") == 1
        and isinstance(message.get("aggregate"), dict)
    ]
    trajectory_metrics = [
        metric
        for message in messages
        if isinstance(message, dict)
        and message.get("schema_version") == 1
        for metric in message.get("metrics", [])
        if isinstance(metric, dict)
    ]

    def finite_values(items: List[object], key: str) -> List[float]:
        values = []
        for item in items:
            if not isinstance(item, dict):
                continue
            value = item.get(key)
            if value is None:
                continue
            try:
                number = float(value)
            except (TypeError, ValueError):
                continue
            if isfinite(number):
                values.append(number)
        return values

    path_lengths = finite_values(aggregates, "joint_path_length_rad")
    smoothness = finite_values(
        aggregates,
        "integrated_squared_acceleration",
    )
    max_steps = finite_values(aggregates, "max_joint_step_rad")
    margins = finite_values(
        trajectory_metrics,
        "min_normalized_joint_limit_margin",
    )
    return {
        "message_count": len(aggregates),
        "trajectory_segment_count": len(trajectory_metrics),
        "joint_path_length_rad": sum(path_lengths),
        "max_joint_step_rad": max(max_steps) if max_steps else None,
        "integrated_squared_acceleration": sum(smoothness),
        "min_normalized_joint_limit_margin": min(margins) if margins else None,
    }


def place_descent_trajectory_quality_error(
    step_name: str,
    metrics: Dict[str, object],
    max_duration_s: float,
    max_joint_path_length_rad: float,
) -> Optional[str]:
    """Reject pathological contact-zone paths before physical execution."""
    is_descent = step_name in {
        "cartesian_place_descent",
        "cartesian_place_correction",
    } or step_name.startswith("cartesian_place_descent_stage_")
    if not is_descent:
        return None
    duration_s = float(metrics.get("duration_s", 0.0))
    path_length = float(metrics.get("joint_path_length_rad", 0.0))
    if duration_s > max_duration_s:
        return (
            f"duration={duration_s:.3f}s exceeds contact-zone limit "
            f"{max_duration_s:.3f}s"
        )
    if path_length > max_joint_path_length_rad:
        return (
            f"joint_path_length={path_length:.3f}rad exceeds contact-zone "
            f"limit {max_joint_path_length_rad:.3f}rad"
        )
    return None


def rigid_payload_translation_error(
    actual_target_position: Tuple[float, float, float],
    hand_pose_data: Dict[str, object],
    object_in_hand_pose_data: Dict[str, object],
    measurement_frame_translation: Tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> float:
    """Compare Gazebo target truth with the full hand-to-object transform."""
    hand_pose = pose_from_dict(hand_pose_data)
    expected = compose_pose(
        hand_pose,
        pose_from_dict(object_in_hand_pose_data),
        hand_pose.frame_id,
    )
    return translation_distance(
        (
            float(actual_target_position[0])
            - expected.position.x
            - measurement_frame_translation[0],
            float(actual_target_position[1])
            - expected.position.y
            - measurement_frame_translation[1],
            float(actual_target_position[2])
            - expected.position.z
            - measurement_frame_translation[2],
        )
    )


def payload_compensated_place_pose(
    desired_object_pose_data: Dict[str, object],
    object_in_hand_pose_data: Dict[str, object],
) -> Dict[str, object]:
    """Shift the hand goal so the rigidly held object reaches the desired pose."""
    desired = pose_from_dict(desired_object_pose_data)
    local_object = pose_from_dict(object_in_hand_pose_data)
    zero_origin_hand = PoseSpec(
        desired.frame_id,
        Vector3(0.0, 0.0, 0.0),
        desired.orientation,
    )
    rotated_offset = compose_pose(
        zero_origin_hand,
        local_object,
        desired.frame_id,
    ).position
    compensated = deepcopy(desired_object_pose_data)
    position = compensated["position"]
    position["x"] = desired.position.x - rotated_offset.x
    position["y"] = desired.position.y - rotated_offset.y
    position["z"] = desired.position.z - rotated_offset.z
    return compensated


def cube_symmetric_hand_goal(hand_data, object_in_hand_data, yaw_degrees):
    """Rotate about the placed object's center, retaining its world position."""
    if not isfinite(yaw_degrees) or abs(yaw_degrees / 90 - round(yaw_degrees / 90)) > 1e-9:
        raise ValueError("cube symmetry requires a finite multiple of 90 degrees")
    hand = pose_from_dict(hand_data)
    local = pose_from_dict(object_in_hand_data)
    placed_object = compose_pose(hand, local, hand.frame_id)
    rotated = deepcopy(hand_data)
    rotated["position"] = placed_object.position.as_dict()
    rotated["orientation"] = quaternion_multiply(
        quaternion_from_euler(0.0, 0.0, radians(yaw_degrees)), hand.orientation
    ).normalized().as_dict()
    return payload_compensated_place_pose(rotated, object_in_hand_data)


def staged_place_feedback_poses(
    hand_pose_data: Dict[str, object],
    correction: Tuple[float, float, float],
    recovery_height_m: float,
) -> Tuple[Dict[str, object], Dict[str, object], Dict[str, object]]:
    """Build lift, suspended translation and descent goals for place recovery."""
    raised = translated_pose_dict(
        hand_pose_data,
        (0.0, 0.0, max(0.0, recovery_height_m)),
    )
    translated = translated_pose_dict(
        hand_pose_data,
        (
            correction[0],
            correction[1],
            correction[2] + max(0.0, recovery_height_m),
        ),
    )
    corrected = translated_pose_dict(hand_pose_data, correction)
    return raised, translated, corrected


def interpolated_place_descent_poses(
    start_pose_data: Dict[str, object],
    goal_pose_data: Dict[str, object],
    stage_count: int,
) -> List[Dict[str, object]]:
    """Split a place descent into equal Cartesian translation stages."""
    start = pose_from_dict(start_pose_data)
    goal = pose_from_dict(goal_pose_data)
    if start.frame_id != goal.frame_id:
        raise ValueError("place descent poses must use the same frame")
    count = max(1, int(stage_count))
    stages: List[Dict[str, object]] = []
    for index in range(1, count + 1):
        ratio = index / count
        stage = deepcopy(goal_pose_data)
        stage["position"] = {
            "x": start.position.x + (goal.position.x - start.position.x) * ratio,
            "y": start.position.y + (goal.position.y - start.position.y) * ratio,
            "z": start.position.z + (goal.position.z - start.position.z) * ratio,
        }
        stages.append(stage)
    return stages


def anchored_place_descent_poses(
    start_pose_data: Dict[str, object],
    goal_pose_data: Dict[str, object],
    measured_start_position: Tuple[float, float, float],
    stage_count: int,
) -> Tuple[List[Dict[str, object]], Tuple[float, float, float]]:
    """Translate the full descent line to the measured pre-place position."""
    start = pose_from_dict(start_pose_data)
    offset = tuple(
        float(measured) - float(nominal)
        for measured, nominal in zip(
            measured_start_position,
            (start.position.x, start.position.y, start.position.z),
        )
    )
    anchored_start = translated_pose_dict(start_pose_data, offset)
    anchored_goal = translated_pose_dict(goal_pose_data, offset)
    return (
        interpolated_place_descent_poses(
            anchored_start,
            anchored_goal,
            stage_count,
        ),
        offset,
    )


def place_descent_stage_number(step_name: Optional[str]) -> Optional[int]:
    """Extract the one-based stage number from a staged descent step name."""
    prefix = "cartesian_place_descent_stage_"
    if not step_name or not step_name.startswith(prefix):
        return None
    token = step_name[len(prefix):].split("_of_", 1)[0]
    try:
        stage_number = int(token)
    except ValueError:
        return None
    return stage_number if stage_number > 0 else None


def place_descent_stage_motion_diagnostic(
    before_object: Tuple[float, float, float],
    after_object: Tuple[float, float, float],
    before_hand: Tuple[float, float, float],
    after_hand: Tuple[float, float, float],
) -> Dict[str, float]:
    """Separate end-effector tracking motion from payload slip during descent."""
    object_delta = tuple(after - before for before, after in zip(before_object, after_object))
    hand_delta = tuple(after - before for before, after in zip(before_hand, after_hand))
    relative_delta = tuple(
        object_value - hand_value
        for object_value, hand_value in zip(object_delta, hand_delta)
    )
    return {
        "dx_m": object_delta[0],
        "dy_m": object_delta[1],
        "dz_m": object_delta[2],
        "horizontal_motion_m": sqrt(
            object_delta[0] * object_delta[0]
            + object_delta[1] * object_delta[1]
        ),
        "total_motion_m": translation_distance(object_delta),
        "hand_dx_m": hand_delta[0],
        "hand_dy_m": hand_delta[1],
        "hand_dz_m": hand_delta[2],
        "hand_horizontal_motion_m": sqrt(
            hand_delta[0] * hand_delta[0] + hand_delta[1] * hand_delta[1]
        ),
        "payload_relative_dx_m": relative_delta[0],
        "payload_relative_dy_m": relative_delta[1],
        "payload_relative_dz_m": relative_delta[2],
        "payload_relative_horizontal_m": sqrt(
            relative_delta[0] * relative_delta[0]
            + relative_delta[1] * relative_delta[1]
        ),
        "payload_relative_motion_m": translation_distance(relative_delta),
    }


def place_descent_stage_path_is_safe(
    diagnostic: Dict[str, float],
    max_horizontal_motion_m: float,
) -> bool:
    """Check the hand path; rigid payload drift is verified in the hand frame."""
    return (
        float(diagnostic["hand_horizontal_motion_m"])
        <= max(0.0, float(max_horizontal_motion_m))
    )


def calibrated_object_in_hand_pose(
    hand_pose_data: Dict[str, object],
    nominal_object_pose_data: Dict[str, object],
    actual_target_world_position: Tuple[float, float, float],
    reference_target_world_position: Tuple[float, float, float],
) -> Dict[str, object]:
    """Estimate the measured hand-to-object transform after a verified grasp."""
    hand_pose = pose_from_dict(hand_pose_data)
    nominal_object = pose_from_dict(nominal_object_pose_data)
    measured_position = target_position_in_measurement_frame(
        actual_target_world_position,
        reference_target_world_position,
        nominal_object_pose_data,
    )
    measured_object = PoseSpec(
        nominal_object.frame_id,
        Vector3(*measured_position),
        nominal_object.orientation,
    )
    return relative_pose(
        hand_pose,
        measured_object,
        "panda_hand",
    ).as_dict()


def target_position_in_measurement_frame(
    actual_target_world_position: Tuple[float, float, float],
    reference_target_world_position: Tuple[float, float, float],
    nominal_object_pose_data: Dict[str, object],
) -> Tuple[float, float, float]:
    """Map simulator world truth into the candidate's robot measurement frame."""
    nominal_object = pose_from_dict(nominal_object_pose_data)
    nominal_position = (
        nominal_object.position.x,
        nominal_object.position.y,
        nominal_object.position.z,
    )
    world_from_measurement_frame = tuple(
        reference - nominal
        for reference, nominal in zip(
            reference_target_world_position,
            nominal_position,
        )
    )
    return tuple(
        actual - frame_offset
        for actual, frame_offset in zip(
            actual_target_world_position,
            world_from_measurement_frame,
        )
    )


def clamp_near_joint_limits(
    names: List[str],
    positions: List[float],
    tolerance: float = 0.002,
    margin: float = 1e-6,
) -> List[float]:
    """Clamp only tiny simulator drift beyond the Panda's declared limits."""
    clamped = list(positions)
    for index, (name, value) in enumerate(zip(names, positions)):
        limits = PANDA_JOINT_LIMITS.get(name)
        if limits is None:
            continue
        lower, upper = limits
        if value < lower and lower - value <= tolerance:
            clamped[index] = lower + margin
        elif value > upper and value - upper <= tolerance:
            clamped[index] = upper - margin
    return clamped


def contact_limited_gripper_outcome(
    step_name: str,
    pose_key: Optional[str],
    finger_positions: List[float],
) -> Optional[str]:
    """Recognize useful contact after a trajectory goal-tolerance violation."""
    if len(finger_positions) != 2:
        return None
    if any(position < -0.002 or position > 0.042 for position in finger_positions):
        return None
    if abs(finger_positions[0] - finger_positions[1]) > 0.004:
        return None
    opening = sum(finger_positions)
    position_text = ", ".join(f"{position:.4f}" for position in finger_positions)
    if step_name == "gripper_closed" and opening <= 0.065:
        return f"contact_limited_grasp, fingers=[{position_text}], opening={opening:.4f}m"
    release_or_recovery = (
        step_name == "gripper_recovery_open"
        or (step_name == "gripper_open" and pose_key == "place_pose")
    )
    if release_or_recovery and opening >= 0.035:
        return f"contact_limited_release, fingers=[{position_text}], opening={opening:.4f}m"
    return None


def measured_gripper_endpoint_outcome(
    target_position: float,
    finger_positions: List[float],
    tolerance_m: float,
) -> Optional[str]:
    """Accept a controller timeout only when both fingers reached the command."""
    if len(finger_positions) != 2:
        return None
    if any(position < -0.002 or position > 0.042 for position in finger_positions):
        return None
    tolerance_m = max(0.0, tolerance_m)
    max_error = max(
        abs(position - target_position) for position in finger_positions
    )
    if max_error > tolerance_m:
        return None
    position_text = ", ".join(f"{position:.4f}" for position in finger_positions)
    return (
        "measured_endpoint_after_controller_timeout, "
        f"target={target_position:.4f}m, max_error={max_error:.4f}m, "
        f"fingers=[{position_text}], opening={sum(finger_positions):.4f}m"
    )


def measured_arm_endpoint_outcome(
    joint_names: List[str],
    target_positions: List[float],
    current_positions: List[float],
    tolerance_rad: float,
    minimum_joint_limit_margin: float,
) -> Optional[str]:
    """Recognize a near endpoint without accepting a joint-limit solution."""
    if (
        len(joint_names) != len(target_positions)
        or len(joint_names) != len(current_positions)
        or not joint_names
    ):
        return None
    target_margin = normalized_joint_limit_margin(joint_names, target_positions)
    if target_margin < max(0.0, minimum_joint_limit_margin):
        return None
    max_error = max(
        abs(current - target)
        for current, target in zip(current_positions, target_positions)
    )
    if max_error > max(0.0, tolerance_rad):
        return None
    worst_index = max(
        range(len(joint_names)),
        key=lambda index: abs(current_positions[index] - target_positions[index]),
    )
    return (
        "measured_endpoint_after_control_failed, "
        f"max_error={max_error:.4f}rad at {joint_names[worst_index]}, "
        f"joint_limit_margin={target_margin:.4f}"
    )


def quaternion_angular_distance(
    first: Tuple[float, float, float, float],
    second: Tuple[float, float, float, float],
) -> float:
    """Return the shortest angular distance between two quaternions."""
    if len(first) != 4 or len(second) != 4:
        return float("inf")
    if not all(isfinite(value) for value in (*first, *second)):
        return float("inf")
    first_norm = sqrt(sum(value * value for value in first))
    second_norm = sqrt(sum(value * value for value in second))
    if first_norm <= 1e-12 or second_norm <= 1e-12:
        return float("inf")
    dot = abs(
        sum(left * right for left, right in zip(first, second))
        / (first_norm * second_norm)
    )
    return 2.0 * acos(max(-1.0, min(1.0, dot)))


def gripper_close_stalled(
    step_name: str,
    target_position: float,
    start_positions: List[float],
    current_positions: List[float],
    minimum_movement_m: float,
) -> bool:
    """Return whether a close command timed out without moving either finger."""
    if step_name != "gripper_closed":
        return False
    if len(start_positions) != 2 or len(current_positions) != 2:
        return False
    if target_position >= min(start_positions):
        return False
    movement = max(
        abs(current - start)
        for start, current in zip(start_positions, current_positions)
    )
    return movement < max(0.0, minimum_movement_m)


def finger_asymmetry_correction(
    finger_positions: List[float],
    orientation: Dict[str, float],
    correction_gain: float,
    minimum_asymmetry_m: float,
    maximum_correction_m: float,
) -> Tuple[float, float, float]:
    """Convert unequal finger contact into a bounded grasp-center correction."""
    if len(finger_positions) != 2:
        return (0.0, 0.0, 0.0)
    asymmetry = finger_positions[0] - finger_positions[1]
    if abs(asymmetry) < max(0.0, minimum_asymmetry_m):
        return (0.0, 0.0, 0.0)
    local_y = 0.5 * asymmetry * max(0.0, correction_gain)
    limit = max(0.0, maximum_correction_m)
    local_y = max(-limit, min(limit, local_y))

    x = float(orientation.get("x", 0.0))
    y = float(orientation.get("y", 0.0))
    z = float(orientation.get("z", 0.0))
    w = float(orientation.get("w", 1.0))
    norm = sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-9:
        return (0.0, 0.0, 0.0)
    x, y, z, w = (value / norm for value in (x, y, z, w))

    # Rotate the local vector (0, local_y, 0) by the grasp quaternion.
    return (
        2.0 * (x * y - z * w) * local_y,
        (1.0 - 2.0 * (x * x + z * z)) * local_y,
        2.0 * (y * z + x * w) * local_y,
    )


def physical_grasp_verified(
    initial_z: Optional[float],
    current_z: Optional[float],
    minimum_delta_m: float,
) -> bool:
    return (
        initial_z is not None
        and current_z is not None
        and current_z - initial_z >= minimum_delta_m
    )


def release_object_motion(
    reference_pose: Tuple[float, float, float],
    settled_pose: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    """Return horizontal, vertical, and 3D object motion during release."""
    dx = float(settled_pose[0]) - float(reference_pose[0])
    dy = float(settled_pose[1]) - float(reference_pose[1])
    dz = float(settled_pose[2]) - float(reference_pose[2])
    return (sqrt(dx * dx + dy * dy), abs(dz), sqrt(dx * dx + dy * dy + dz * dz))


def matching_contact_collision_names(
    contacts: object,
    collision_token: str,
) -> List[str]:
    """Return collision names matching a token in a Gazebo contact message."""
    token = str(collision_token).strip()
    if not token:
        return []
    names = set()
    for contact in getattr(contacts, "contacts", []):
        for field_name in ("collision1", "collision2"):
            collision = getattr(contact, field_name, None)
            name = str(getattr(collision, "name", ""))
            if token in name:
                names.add(name)
    return sorted(names)


def bilateral_contact_is_recent(
    last_contact_ns: Dict[str, Optional[int]],
    now_ns: int,
    maximum_age_s: float,
) -> bool:
    """Require recent target contact from both fingertip sensors."""
    maximum_age_ns = int(max(0.0, maximum_age_s) * 1_000_000_000)
    return all(
        timestamp is not None
        and 0 <= now_ns - timestamp <= maximum_age_ns
        for timestamp in (
            last_contact_ns.get("left"),
            last_contact_ns.get("right"),
        )
    )


def translated_pose_dict(
    pose: Dict[str, object],
    translation: Tuple[float, float, float],
) -> Dict[str, object]:
    translated = {
        "frame_id": pose["frame_id"],
        "position": dict(pose["position"]),
        "orientation": dict(pose["orientation"]),
    }
    for axis, delta in zip(("x", "y", "z"), translation):
        translated["position"][axis] = float(translated["position"][axis]) + delta
    return translated


def pre_place_height_candidate(nominal, place, attempt, step):
    """Diversify clearance without changing the final placement or orientation."""
    candidate = deepcopy(nominal)
    if not isfinite(step) or step < 0:
        raise ValueError("pre-place height step must be finite and nonnegative")
    offsets = (0.0, 1.0, -1.0, 2.0)
    candidate["position"]["z"] = max(
        float(place["position"]["z"]) + 0.05,
        float(nominal["position"]["z"]) + offsets[(attempt - 1) % 4] * step,
    ) if step else float(nominal["position"]["z"])
    return candidate


def equivalent_grasp_candidate(
    candidate: Dict[str, object],
    yaw_offset_degrees: float,
) -> Dict[str, object]:
    """Rotate cube-equivalent grasp waypoints around the world vertical axis."""
    rotated = deepcopy(candidate)
    yaw_rotation = quaternion_from_euler(
        0.0,
        0.0,
        radians(float(yaw_offset_degrees)),
    )
    pose_keys = ["pre_grasp_pose", "grasp_pose", "lift_pose"]
    if candidate.get("cube_symmetric_placement", False):
        pose_keys.extend(["place_pose", "pre_place_pose", "retreat_pose"])
    for pose_key in pose_keys:
        pose = rotated.get(pose_key)
        if not isinstance(pose, dict):
            continue
        orientation = pose.get("orientation")
        if not isinstance(orientation, dict):
            continue
        original = Quaternion(
            float(orientation.get("x", 0.0)),
            float(orientation.get("y", 0.0)),
            float(orientation.get("z", 0.0)),
            float(orientation.get("w", 1.0)),
        ).normalized()
        pose["orientation"] = quaternion_multiply(
            yaw_rotation,
            original,
        ).normalized().as_dict()

    object_pose = rotated.get("object_pose")
    grasp_pose = rotated.get("grasp_pose")
    if isinstance(object_pose, dict) and isinstance(grasp_pose, dict):
        if candidate.get("grasp_pitch_offset_deg", 0.0):
            # A pitched hand has an XY offset. Yaw must rotate that offset too.
            local = pose_from_dict(candidate["object_in_hand_pose"])
            offset = rotate_vector(local.position, pose_from_dict(grasp_pose).orientation)
            delta = {
                axis: float(object_pose["position"][axis]) - getattr(offset, axis)
                - float(grasp_pose["position"][axis]) for axis in ("x", "y", "z")
            }
            for key in ("pre_grasp_pose", "grasp_pose", "lift_pose"):
                for axis in ("x", "y", "z"):
                    rotated[key]["position"][axis] += delta[axis]
        rotated["object_in_hand_pose"] = relative_pose(
            pose_from_dict(grasp_pose),
            pose_from_dict(object_pose),
            "panda_hand",
        ).as_dict()
    rotated["orientation_yaw_offset_deg"] = float(yaw_offset_degrees)
    return rotated


def translation_distance(translation: Tuple[float, float, float]) -> float:
    return sqrt(sum(value * value for value in translation))


def joint_path_length(position_rows: List[List[float]]) -> float:
    """Return cumulative Euclidean joint-space path length in radians."""
    return sum(
        sqrt(sum((current - previous) ** 2 for previous, current in zip(first, second)))
        for first, second in zip(position_rows, position_rows[1:])
    )


def normalized_joint_limit_margin(
    names: List[str],
    positions: List[float],
) -> float:
    """Return the smallest normalized distance from an arm joint limit."""
    margins = []
    for name, value in zip(names, positions):
        if name.startswith("panda_finger_joint"):
            continue
        limits = PANDA_JOINT_LIMITS.get(name)
        if limits is None:
            continue
        lower, upper = limits
        half_range = 0.5 * (upper - lower)
        if half_range <= 0.0:
            continue
        margin = min(float(value) - lower, upper - float(value)) / half_range
        margins.append(max(0.0, min(1.0, margin)))
    return min(margins) if margins else 0.0


def grasp_candidate_prevalidation_score(
    cartesian_fraction: float,
    joint_limit_margin: float,
    joint_path_length_rad: float,
    planning_time_s: float,
    yaw_offset_deg: float,
) -> float:
    """Score feasible grasps while mildly preferring shorter, centered motions."""
    return (
        100.0 * max(0.0, min(1.0, float(cartesian_fraction)))
        + 5.0 * max(0.0, min(1.0, float(joint_limit_margin)))
        - 0.5 * max(0.0, float(joint_path_length_rad))
        - 0.1 * max(0.0, float(planning_time_s))
        - 0.001 * abs(float(yaw_offset_deg))
    )


def grasp_candidate_report(result: Dict[str, object]) -> Dict[str, object]:
    """Keep diagnostics separate from internal poses and ROS trajectories."""
    return {
        key: value
        for key, value in result.items()
        if key not in {"candidate", "pre_grasp_trajectory"}
    }


def grasp_candidate_prevalidation_feasible(
    error_code: int,
    cartesian_fraction: float,
    minimum_fraction: float,
    joint_limit_margin: float,
    minimum_joint_limit_margin: float,
) -> bool:
    """Require both a complete contact path and executable joint clearance."""
    return (
        error_code == 1
        and cartesian_fraction >= minimum_fraction
        and joint_limit_margin >= minimum_joint_limit_margin
    )


def normalize_planner_ids(
    planner_ids: List[str],
    planner_ids_csv: str,
    fallback_planner_id: str,
) -> List[str]:
    """Normalize ROS array or launch-friendly CSV planner configuration."""
    candidates = planner_ids_csv.split(",") if planner_ids_csv.strip() else planner_ids
    normalized = list(
        dict.fromkeys(str(planner).strip() for planner in candidates if str(planner).strip())
    )
    return normalized or [fallback_planner_id]


def main(args: List[str] | None = None) -> None:
    rclpy = require_ros2()
    from action_msgs.msg import GoalStatus
    from builtin_interfaces.msg import Duration
    from control_msgs.action import FollowJointTrajectory
    from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
    from moveit_msgs.action import ExecuteTrajectory, MoveGroup
    from moveit_msgs.msg import Constraints, DisplayTrajectory, JointConstraint, MotionPlanRequest
    from moveit_msgs.msg import OrientationConstraint, PlanningSceneComponents
    from moveit_msgs.msg import PlanningOptions
    from moveit_msgs.msg import PositionConstraint, RobotState, ServoStatus
    from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetPositionIK, GetPlanningScene, GetStateValidity, ServoCommandType
    from rclpy.clock import Clock, ClockType
    from .placement_precheck import placement_precheck_request, target_box_dimensions
    from .loaded_path_precheck import LoadedPathPrecheck
    from .reverse_approach_probe import ReverseApproachProbe
    from .reverse_branch_connection import connect_reverse_branch
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from rclpy.time import Time
    from ros_gz_interfaces.msg import Contacts
    from sensor_msgs.msg import JointState
    from shape_msgs.msg import SolidPrimitive
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    from tf2_ros import Buffer, TransformException, TransformListener
    from trajectory_msgs.msg import JointTrajectoryPoint

    class PickPlanPipeline(Node):
        def __init__(self) -> None:
            super().__init__("pick_plan_pipeline")
            self.declare_parameter("move_action_name", "/move_action")
            self.declare_parameter("execute_trajectory_action_name", "/execute_trajectory")
            self.declare_parameter(
                "arm_controller_action_name",
                "/panda_arm_controller/follow_joint_trajectory",
            )
            self.declare_parameter("cartesian_service_name", "/compute_cartesian_path")
            self.declare_parameter("planning_group", "panda_arm")
            self.declare_parameter("end_effector_link", "panda_hand")
            self.declare_parameter("planner_id", "RRTConnectkConfigDefault")
            self.declare_parameter(
                "planner_ids",
                ["RRTConnectkConfigDefault"],
            )
            self.declare_parameter("planner_ids_csv", "")
            self.declare_parameter("allowed_planning_time", 8.0)
            self.declare_parameter("planning_attempts", 20)
            self.declare_parameter("ompl_velocity_scaling_factor", 0.1)
            self.declare_parameter("ompl_invalid_plan_retries", 0)
            self.declare_parameter("recovery_ompl_retries", 1)
            self.declare_parameter(
                "recovery_planner_id",
                "RRTConnectkConfigDefault",
            )
            self.declare_parameter("position_tolerance", 0.04)
            self.declare_parameter("use_fixed_home_start", True)
            self.declare_parameter("pre_grasp_orientation_tolerance", 0.75)
            self.declare_parameter("recovery_orientation_tolerance", 0.04)
            self.declare_parameter("recovery_orientation_tolerance_step", 0.03)
            self.declare_parameter(
                "equivalent_grasp_yaw_offsets_deg",
                [90.0, -90.0, 180.0],
            )
            self.declare_parameter("restrict_grasp_yaw_to_nominal", False)
            self.declare_parameter("force_grasp_yaw_offset", False)
            self.declare_parameter("forced_grasp_yaw_offset_deg", 0.0)
            self.declare_parameter("enable_grasp_candidate_prevalidation", False)
            self.declare_parameter("enable_place_endpoint_precheck", False)
            self.declare_parameter("enable_loaded_path_precheck", False)
            self.declare_parameter("grasp_pitch_offset_deg", 0.0)
            self.declare_parameter(
                "grasp_candidate_prevalidation_min_fraction",
                0.98,
            )
            self.declare_parameter(
                "grasp_candidate_prevalidation_min_joint_limit_margin",
                0.01,
            )
            self.declare_parameter(
                "grasp_candidate_prevalidation_scene_settle_s",
                0.5,
            )
            self.declare_parameter(
                "grasp_candidate_prevalidation_max_rounds",
                2,
            )
            self.declare_parameter(
                "defer_grasp_candidate_prevalidation_until_dynamic_replan",
                False,
            )
            self.declare_parameter("cartesian_max_step", 0.01)
            self.declare_parameter("cartesian_retry_max_step", 0.005)
            self.declare_parameter(
                "cartesian_timing_retry_max_steps",
                [0.004, 0.003],
            )
            self.declare_parameter("cartesian_start_state_replan_max_retries", 1)
            self.declare_parameter("cartesian_execution_replan_max_retries", 1)
            self.declare_parameter("cartesian_execution_replan_settle_s", 0.25)
            self.declare_parameter(
                "arm_execution_endpoint_acceptance_tolerance_rad",
                0.020,
            )
            self.declare_parameter("cartesian_jump_threshold", 0.0)
            self.declare_parameter("cartesian_min_fraction", 0.9)
            self.declare_parameter("cartesian_avoid_collisions", True)
            self.declare_parameter("diagnose_direct_path_to_pre_grasp", False)
            self.declare_parameter("require_direct_path_blocked", False)
            self.declare_parameter("direct_path_blocked_fraction_threshold", 0.95)
            self.declare_parameter("use_ompl_for_place", False)
            self.declare_parameter("use_cartesian_pre_place_transfer", False)
            self.declare_parameter("pre_place_transfer_stage_count", 4)
            self.declare_parameter("cartesian_velocity_scaling_factor", 0.15)
            self.declare_parameter("cartesian_acceleration_scaling_factor", 0.15)
            self.declare_parameter("contact_approach_velocity_scaling_factor", 0.05)
            self.declare_parameter("contact_approach_acceleration_scaling_factor", 0.05)
            self.declare_parameter("enable_cartesian_approach_recovery", True)
            self.declare_parameter("approach_recovery_ratio", 0.5)
            self.declare_parameter(
                "approach_recovery_ratios",
                [0.25, 0.5, 0.75, 0.9],
            )
            self.declare_parameter("enable_pre_grasp_alignment_recovery", True)
            self.declare_parameter("pre_grasp_alignment_staging_height", 0.20)
            self.declare_parameter(
                "pre_grasp_alignment_recovery_min_fraction",
                0.95,
            )
            self.declare_parameter(
                "pre_grasp_alignment_prefix_min_fraction",
                0.50,
            )
            self.declare_parameter(
                "pre_grasp_alignment_prefix_max_attempts",
                2,
            )
            self.declare_parameter("contact_prefix_min_fraction", 0.50)
            self.declare_parameter("contact_prefix_max_attempts", 2)
            self.declare_parameter("execute_trajectories", False)
            self.declare_parameter("visualize_only_execution", False)
            self.declare_parameter("ompl_execution_backend", "execute_trajectory")
            self.declare_parameter("execute_gripper", False)
            self.declare_parameter("gripper_group_name", "hand")
            self.declare_parameter("gripper_execution_backend", "move_group")
            self.declare_parameter(
                "gripper_action_name",
                "/panda_hand_controller/follow_joint_trajectory",
            )
            self.declare_parameter("gripper_motion_duration_s", 1.5)
            self.declare_parameter("gripper_free_motion_max_retries", 1)
            self.declare_parameter("gripper_goal_rejection_max_retries", 2)
            self.declare_parameter("gripper_goal_rejection_retry_delay_s", 0.5)
            self.declare_parameter("gripper_close_in_place_max_retries", 1)
            self.declare_parameter("gripper_close_min_movement_m", 0.002)
            self.declare_parameter("gripper_endpoint_acceptance_tolerance_m", 0.001)
            self.declare_parameter("gripper_open_position", 0.04)
            self.declare_parameter("gripper_release_position", 0.028)
            self.declare_parameter("gripper_release_settle_s", 0.5)
            # Each Panda finger joint contributes half of the total opening.
            # Keep a 50 mm target cube between the pads instead of closing through it.
            self.declare_parameter("gripper_closed_position", 0.026)
            self.declare_parameter("enable_tactile_grasp_supervision", False)
            self.declare_parameter(
                "target_contact_topic",
                "/gazebo/target_cube/contacts",
            )
            self.declare_parameter("tactile_left_finger_token", "panda_leftfinger")
            self.declare_parameter("tactile_right_finger_token", "panda_rightfinger")
            self.declare_parameter("tactile_contact_settle_s", 0.2)
            self.declare_parameter("tactile_contact_max_age_s", 0.75)
            self.declare_parameter("tactile_release_max_retries", 1)
            self.declare_parameter("release_object_max_horizontal_motion_m", 0.05)
            self.declare_parameter("release_object_max_vertical_motion_m", 0.06)
            self.declare_parameter("gripper_allowed_planning_time", 2.0)
            self.declare_parameter("gripper_planning_attempts", 5)
            self.declare_parameter("enable_physical_grasp_verification", False)
            self.declare_parameter("physical_target_pose_topic", "/gazebo/target_pose")
            self.declare_parameter("grasp_probe_lift_m", 0.03)
            self.declare_parameter("grasp_probe_min_object_lift_m", 0.015)
            self.declare_parameter("grasp_probe_settle_s", 0.5)
            self.declare_parameter("release_settle_s", 0.0)
            self.declare_parameter("release_clearance_height", 0.12)
            self.declare_parameter("pre_place_height_offset", 0.12)
            self.declare_parameter("pre_place_candidate_attempts", 4)
            self.declare_parameter("pre_place_candidate_height_step_m", 0.0)
            self.declare_parameter("cube_symmetric_placement", False)
            self.declare_parameter("diagnose_rejected_descent", False)
            self.declare_parameter("pre_place_joint_replay_positions_csv", "")
            self.declare_parameter("pre_place_joint_replay_tolerance_rad", 0.005)
            self.declare_parameter("direct_place_candidate_attempts", 4)
            self.declare_parameter("place_transfer_max_joint_path_length", 4.5)
            self.declare_parameter("payload_transfer_tolerance_m", 0.06)
            self.declare_parameter("place_object_clearance_m", 0.008)
            self.declare_parameter("place_feedback_tolerance_m", 0.05)
            self.declare_parameter("place_feedback_max_correction_m", 0.12)
            self.declare_parameter("place_feedback_max_retries", 1)
            self.declare_parameter("place_feedback_settle_s", 0.5)
            self.declare_parameter("place_feedback_recovery_height_m", 0.05)
            self.declare_parameter("place_feedback_min_improvement_m", 0.005)
            self.declare_parameter("place_descent_stage_count", 3)
            self.declare_parameter(
                "place_descent_max_trajectory_duration_s",
                15.0,
            )
            self.declare_parameter(
                "place_descent_max_joint_path_length_rad",
                0.5,
            )
            self.declare_parameter(
                "place_descent_stage_max_horizontal_motion_m",
                0.01,
            )
            self.declare_parameter("place_descent_stage_payload_tolerance_m", 0.015)
            self.declare_parameter(
                "place_descent_endpoint_orientation_tolerance_rad",
                0.10,
            )
            self.declare_parameter("place_descent_start_tolerance_m", 0.01)
            self.declare_parameter("place_descent_start_poll_s", 0.2)
            self.declare_parameter("place_descent_start_timeout_s", 3.0)
            self.declare_parameter("place_descent_start_max_alignment_retries", 2)
            self.declare_parameter("place_descent_backend", "cartesian")
            self.declare_parameter(
                "servo_cartesian_command_topic",
                "/servo_node/delta_twist_cmds",
            )
            self.declare_parameter("servo_status_topic", "/servo_node/status")
            self.declare_parameter(
                "servo_switch_service",
                "/servo_node/switch_command_type",
            )
            self.declare_parameter(
                "servo_pause_service",
                "/servo_node/pause_servo",
            )
            self.declare_parameter("servo_descent_speed_mps", 0.01)
            self.declare_parameter("servo_command_period_s", 0.02)
            self.declare_parameter("servo_stage_position_tolerance_m", 0.002)
            self.declare_parameter("servo_stage_timeout_margin_s", 2.0)
            self.declare_parameter("servo_watchdog_period_s", 0.10)
            self.declare_parameter("servo_fault_injection_type", "none")
            self.declare_parameter("servo_fault_injection_stage", 2)
            self.declare_parameter("servo_fault_injection_delay_s", 0.50)
            self.declare_parameter(
                "servo_fault_injection_payload_drift_m",
                0.030,
            )
            self.declare_parameter(
                "servo_collision_limited_acceptance_m",
                0.012,
            )
            self.declare_parameter("enable_servo_cartesian_fallback", True)
            self.declare_parameter("payload_transfer_velocity_scaling_factor", 0.05)
            self.declare_parameter("place_descent_velocity_scaling_factor", 0.01)
            self.declare_parameter(
                "place_descent_recovery_velocity_scaling_factor",
                0.02,
            )
            self.declare_parameter(
                "payload_transfer_execution_replan_max_retries",
                2,
            )
            self.declare_parameter(
                "payload_transfer_execution_replan_settle_s",
                0.25,
            )
            self.declare_parameter("allow_ompl_release_retreat_fallback", False)
            self.declare_parameter("release_retreat_max_joint_path_length", 1.5)
            self.declare_parameter("return_home_after_success", False)
            self.declare_parameter("return_home_scene_settle_s", 1.0)
            self.declare_parameter("return_home_max_retries", 3)
            self.declare_parameter("approach_scene_settle_s", 0.0)
            self.declare_parameter("grasp_probe_max_retries", 1)
            self.declare_parameter("preclose_recenter_max_retries", 2)
            self.declare_parameter("grasp_preclose_max_target_drift_m", 0.015)
            self.declare_parameter("grasp_recovery_max_translation_m", 0.10)
            self.declare_parameter("grasp_finger_asymmetry_correction_gain", 1.0)
            self.declare_parameter("grasp_finger_asymmetry_min_m", 0.001)
            self.declare_parameter("grasp_finger_max_correction_m", 0.004)
            self.declare_parameter("reset_to_home_before_execute", False)
            self.declare_parameter("execution_start_tolerance", 0.05)
            self.declare_parameter("fail_on_start_state_mismatch", True)
            self.declare_parameter("joint_state_wait_timeout_s", 5.0)
            self.declare_parameter("enable_dynamic_replanning", False)
            self.declare_parameter("dynamic_replan_request_topic", "/dynamic_replan_request")
            self.declare_parameter("maximum_dynamic_replans", 2)
            self.declare_parameter("dynamic_replan_settle_s", 0.75)
            self.declare_parameter("plan_once", True)

            self.move_action_name = str(self.get_parameter("move_action_name").value)
            self.execute_trajectory_action_name = str(
                self.get_parameter("execute_trajectory_action_name").value
            )
            self.arm_controller_action_name = str(
                self.get_parameter("arm_controller_action_name").value
            )
            self.cartesian_service_name = str(self.get_parameter("cartesian_service_name").value)
            self.group_name = str(self.get_parameter("planning_group").value)
            self.end_effector_link = str(self.get_parameter("end_effector_link").value)
            self.planner_id = str(self.get_parameter("planner_id").value)
            self.planner_ids = normalize_planner_ids(
                list(self.get_parameter("planner_ids").value),
                str(self.get_parameter("planner_ids_csv").value),
                self.planner_id,
            )
            self.planner_id = self.planner_ids[0]
            self.allowed_planning_time = float(self.get_parameter("allowed_planning_time").value)
            self.planning_attempts = int(self.get_parameter("planning_attempts").value)
            self.ompl_velocity_scaling_factor = max(
                0.01,
                min(
                    1.0,
                    float(
                        self.get_parameter(
                            "ompl_velocity_scaling_factor"
                        ).value
                    ),
                ),
            )
            self.ompl_invalid_plan_retries = max(
                0,
                int(self.get_parameter("ompl_invalid_plan_retries").value),
            )
            self.recovery_ompl_retries = max(
                0,
                int(self.get_parameter("recovery_ompl_retries").value),
            )
            self.recovery_planner_id = str(
                self.get_parameter("recovery_planner_id").value
            ).strip() or self.planner_id
            self.position_tolerance = float(self.get_parameter("position_tolerance").value)
            self.use_fixed_home_start = bool(self.get_parameter("use_fixed_home_start").value)
            self.pre_grasp_orientation_tolerance = float(
                self.get_parameter("pre_grasp_orientation_tolerance").value
            )
            self.recovery_orientation_tolerance = min(
                self.pre_grasp_orientation_tolerance,
                max(
                    0.01,
                    float(
                        self.get_parameter(
                            "recovery_orientation_tolerance"
                        ).value
                    ),
                ),
            )
            self.recovery_orientation_tolerance_step = max(
                0.0,
                float(
                    self.get_parameter(
                        "recovery_orientation_tolerance_step"
                    ).value
                ),
            )
            self.equivalent_grasp_yaw_offsets_deg = [
                float(value)
                for value in self.get_parameter(
                    "equivalent_grasp_yaw_offsets_deg"
                ).value
            ]
            self.restrict_grasp_yaw_to_nominal = bool(
                self.get_parameter("restrict_grasp_yaw_to_nominal").value
            )
            self.force_grasp_yaw_offset = bool(
                self.get_parameter("force_grasp_yaw_offset").value
            )
            self.forced_grasp_yaw_offset_deg = float(
                self.get_parameter("forced_grasp_yaw_offset_deg").value
            )
            if self.restrict_grasp_yaw_to_nominal or self.force_grasp_yaw_offset:
                self.equivalent_grasp_yaw_offsets_deg = []
            self.enable_grasp_candidate_prevalidation = bool(
                self.get_parameter("enable_grasp_candidate_prevalidation").value
            )
            if (bool(self.get_parameter("enable_place_endpoint_precheck").value)
                    and not self.enable_grasp_candidate_prevalidation):
                raise ValueError("placement endpoint precheck requires grasp candidate prevalidation")
            if (bool(self.get_parameter("enable_loaded_path_precheck").value)
                    and not bool(self.get_parameter("enable_place_endpoint_precheck").value)):
                raise ValueError("loaded path precheck requires placement endpoint precheck")
            pitch = float(self.get_parameter("grasp_pitch_offset_deg").value)
            if not isfinite(pitch) or abs(pitch) > 20:
                raise ValueError("grasp pitch must be finite and within +/-20 degrees")
            if pitch and (
                not bool(self.get_parameter("enable_loaded_path_precheck").value)
                or bool(self.get_parameter("defer_grasp_candidate_prevalidation_until_dynamic_replan").value)
            ):
                raise ValueError("pitched grasps require immediate loaded-path prevalidation")
            self.grasp_candidate_prevalidation_min_fraction = min(
                1.0,
                max(
                    0.0,
                    float(
                        self.get_parameter(
                            "grasp_candidate_prevalidation_min_fraction"
                        ).value
                    ),
                ),
            )
            self.grasp_candidate_prevalidation_min_joint_limit_margin = min(
                1.0,
                max(
                    0.0,
                    float(
                        self.get_parameter(
                            "grasp_candidate_prevalidation_min_joint_limit_margin"
                        ).value
                    ),
                ),
            )
            self.grasp_candidate_prevalidation_scene_settle_s = max(
                0.0,
                float(
                    self.get_parameter(
                        "grasp_candidate_prevalidation_scene_settle_s"
                    ).value
                ),
            )
            self.grasp_candidate_prevalidation_max_rounds = max(
                1,
                int(
                    self.get_parameter(
                        "grasp_candidate_prevalidation_max_rounds"
                    ).value
                ),
            )
            self.defer_grasp_candidate_prevalidation_until_dynamic_replan = bool(
                self.get_parameter(
                    "defer_grasp_candidate_prevalidation_until_dynamic_replan"
                ).value
            )
            self.cartesian_max_step = float(self.get_parameter("cartesian_max_step").value)
            self.cartesian_retry_max_step = float(self.get_parameter("cartesian_retry_max_step").value)
            configured_timing_retry_steps = [
                float(value)
                for value in self.get_parameter(
                    "cartesian_timing_retry_max_steps"
                ).value
            ]
            self.cartesian_timing_retry_max_steps = [
                min(self.cartesian_retry_max_step, max(0.001, value))
                for value in configured_timing_retry_steps
            ]
            self.cartesian_start_state_replan_max_retries = max(
                0,
                int(
                    self.get_parameter(
                        "cartesian_start_state_replan_max_retries"
                    ).value
                ),
            )
            self.cartesian_execution_replan_max_retries = max(
                0,
                int(
                    self.get_parameter(
                        "cartesian_execution_replan_max_retries"
                    ).value
                ),
            )
            self.cartesian_execution_replan_settle_s = max(
                0.0,
                float(
                    self.get_parameter(
                        "cartesian_execution_replan_settle_s"
                    ).value
                ),
            )
            self.arm_execution_endpoint_acceptance_tolerance_rad = max(
                0.0,
                float(
                    self.get_parameter(
                        "arm_execution_endpoint_acceptance_tolerance_rad"
                    ).value
                ),
            )
            self.cartesian_jump_threshold = float(self.get_parameter("cartesian_jump_threshold").value)
            self.cartesian_min_fraction = float(self.get_parameter("cartesian_min_fraction").value)
            self.cartesian_avoid_collisions = bool(self.get_parameter("cartesian_avoid_collisions").value)
            self.diagnose_direct_path_to_pre_grasp = bool(
                self.get_parameter("diagnose_direct_path_to_pre_grasp").value
            )
            self.require_direct_path_blocked = bool(
                self.get_parameter("require_direct_path_blocked").value
            )
            self.direct_path_blocked_fraction_threshold = float(
                self.get_parameter("direct_path_blocked_fraction_threshold").value
            )
            self.use_ompl_for_place = bool(
                self.get_parameter("use_ompl_for_place").value
            )
            self.use_cartesian_pre_place_transfer = bool(
                self.get_parameter("use_cartesian_pre_place_transfer").value
            )
            self.pre_place_transfer_stage_count = max(
                1,
                int(self.get_parameter("pre_place_transfer_stage_count").value),
            )
            self.cartesian_velocity_scaling_factor = float(
                self.get_parameter("cartesian_velocity_scaling_factor").value
            )
            self.cartesian_acceleration_scaling_factor = float(
                self.get_parameter("cartesian_acceleration_scaling_factor").value
            )
            self.contact_approach_velocity_scaling_factor = max(
                0.01,
                min(
                    1.0,
                    float(
                        self.get_parameter(
                            "contact_approach_velocity_scaling_factor"
                        ).value
                    ),
                ),
            )
            self.contact_approach_acceleration_scaling_factor = max(
                0.01,
                min(
                    1.0,
                    float(
                        self.get_parameter(
                            "contact_approach_acceleration_scaling_factor"
                        ).value
                    ),
                ),
            )
            self.enable_cartesian_approach_recovery = bool(
                self.get_parameter("enable_cartesian_approach_recovery").value
            )
            self.approach_recovery_ratio = float(self.get_parameter("approach_recovery_ratio").value)
            configured_approach_ratios = [
                float(value)
                for value in self.get_parameter("approach_recovery_ratios").value
            ]
            self.approach_recovery_ratios = [
                min(0.9, max(0.1, value))
                for value in configured_approach_ratios
            ] or [min(0.9, max(0.1, self.approach_recovery_ratio))]
            self.enable_pre_grasp_alignment_recovery = bool(
                self.get_parameter("enable_pre_grasp_alignment_recovery").value
            )
            self.pre_grasp_alignment_staging_height = float(
                self.get_parameter("pre_grasp_alignment_staging_height").value
            )
            self.pre_grasp_alignment_recovery_min_fraction = float(
                self.get_parameter(
                    "pre_grasp_alignment_recovery_min_fraction"
                ).value
            )
            self.pre_grasp_alignment_prefix_min_fraction = min(
                self.pre_grasp_alignment_recovery_min_fraction,
                max(
                    0.1,
                    float(
                        self.get_parameter(
                            "pre_grasp_alignment_prefix_min_fraction"
                        ).value
                    ),
                ),
            )
            self.pre_grasp_alignment_prefix_max_attempts = max(
                0,
                int(
                    self.get_parameter(
                        "pre_grasp_alignment_prefix_max_attempts"
                    ).value
                ),
            )
            self.contact_prefix_min_fraction = min(
                self.cartesian_min_fraction,
                max(
                    0.1,
                    float(
                        self.get_parameter(
                            "contact_prefix_min_fraction"
                        ).value
                    ),
                ),
            )
            self.contact_prefix_max_attempts = max(
                0,
                int(self.get_parameter("contact_prefix_max_attempts").value),
            )
            self.execute_trajectories = bool(self.get_parameter("execute_trajectories").value)
            self.visualize_only_execution = bool(self.get_parameter("visualize_only_execution").value)
            self.ompl_execution_backend = str(self.get_parameter("ompl_execution_backend").value)
            self.execute_gripper = bool(self.get_parameter("execute_gripper").value)
            self.gripper_group_name = str(self.get_parameter("gripper_group_name").value)
            self.gripper_execution_backend = str(
                self.get_parameter("gripper_execution_backend").value
            )
            self.gripper_action_name = str(self.get_parameter("gripper_action_name").value)
            self.gripper_motion_duration_s = float(
                self.get_parameter("gripper_motion_duration_s").value
            )
            self.gripper_free_motion_max_retries = int(
                self.get_parameter("gripper_free_motion_max_retries").value
            )
            self.gripper_goal_rejection_max_retries = max(
                0,
                int(
                    self.get_parameter(
                        "gripper_goal_rejection_max_retries"
                    ).value
                ),
            )
            self.gripper_goal_rejection_retry_delay_s = max(
                0.1,
                float(
                    self.get_parameter(
                        "gripper_goal_rejection_retry_delay_s"
                    ).value
                ),
            )
            self.gripper_close_in_place_max_retries = max(
                0,
                int(
                    self.get_parameter(
                        "gripper_close_in_place_max_retries"
                    ).value
                ),
            )
            self.gripper_close_min_movement_m = max(
                0.0,
                float(
                    self.get_parameter("gripper_close_min_movement_m").value
                ),
            )
            self.gripper_endpoint_acceptance_tolerance_m = max(
                0.0,
                float(
                    self.get_parameter(
                        "gripper_endpoint_acceptance_tolerance_m"
                    ).value
                ),
            )
            self.gripper_open_position = float(self.get_parameter("gripper_open_position").value)
            self.gripper_release_position = float(
                self.get_parameter("gripper_release_position").value
            )
            self.gripper_release_settle_s = max(
                0.0,
                float(self.get_parameter("gripper_release_settle_s").value),
            )
            self.gripper_closed_position = float(self.get_parameter("gripper_closed_position").value)
            self.enable_tactile_grasp_supervision = bool(
                self.get_parameter("enable_tactile_grasp_supervision").value
            )
            self.target_contact_topic = str(
                self.get_parameter("target_contact_topic").value
            )
            self.tactile_finger_tokens = {
                "left": str(self.get_parameter("tactile_left_finger_token").value),
                "right": str(self.get_parameter("tactile_right_finger_token").value),
            }
            self.tactile_contact_settle_s = max(
                0.0,
                float(self.get_parameter("tactile_contact_settle_s").value),
            )
            self.tactile_contact_max_age_s = max(
                0.01,
                float(self.get_parameter("tactile_contact_max_age_s").value),
            )
            self.tactile_release_max_retries = max(
                0,
                int(self.get_parameter("tactile_release_max_retries").value),
            )
            self.release_object_max_horizontal_motion_m = max(
                0.0,
                float(
                    self.get_parameter(
                        "release_object_max_horizontal_motion_m"
                    ).value
                ),
            )
            self.release_object_max_vertical_motion_m = max(
                0.0,
                float(
                    self.get_parameter(
                        "release_object_max_vertical_motion_m"
                    ).value
                ),
            )
            self.gripper_allowed_planning_time = float(
                self.get_parameter("gripper_allowed_planning_time").value
            )
            self.gripper_planning_attempts = int(self.get_parameter("gripper_planning_attempts").value)
            self.enable_physical_grasp_verification = bool(
                self.get_parameter("enable_physical_grasp_verification").value
            )
            self.physical_target_pose_topic = str(
                self.get_parameter("physical_target_pose_topic").value
            )
            self.grasp_probe_lift_m = float(self.get_parameter("grasp_probe_lift_m").value)
            self.grasp_probe_min_object_lift_m = float(
                self.get_parameter("grasp_probe_min_object_lift_m").value
            )
            self.grasp_probe_settle_s = float(
                self.get_parameter("grasp_probe_settle_s").value
            )
            self.release_settle_s = float(
                self.get_parameter("release_settle_s").value
            )
            self.release_clearance_height = float(
                self.get_parameter("release_clearance_height").value
            )
            self.pre_place_height_offset = float(
                self.get_parameter("pre_place_height_offset").value
            )
            self.pre_place_candidate_attempts = max(
                1,
                int(self.get_parameter("pre_place_candidate_attempts").value),
            )
            self.pre_place_joint_replay_positions = parse_joint_positions_csv(
                str(
                    self.get_parameter(
                        "pre_place_joint_replay_positions_csv"
                    ).value
                )
            )
            self.pre_place_joint_replay_tolerance_rad = max(
                0.0001,
                float(
                    self.get_parameter(
                        "pre_place_joint_replay_tolerance_rad"
                    ).value
                ),
            )
            self.direct_place_candidate_attempts = max(
                1,
                int(self.get_parameter("direct_place_candidate_attempts").value),
            )
            self.place_transfer_max_joint_path_length = float(
                self.get_parameter("place_transfer_max_joint_path_length").value
            )
            self.payload_transfer_tolerance_m = float(
                self.get_parameter("payload_transfer_tolerance_m").value
            )
            self.place_object_clearance_m = max(
                0.0,
                float(self.get_parameter("place_object_clearance_m").value),
            )
            self.place_feedback_tolerance_m = max(
                0.0,
                float(self.get_parameter("place_feedback_tolerance_m").value),
            )
            self.place_feedback_max_correction_m = max(
                self.place_feedback_tolerance_m,
                float(
                    self.get_parameter("place_feedback_max_correction_m").value
                ),
            )
            self.place_feedback_max_retries = max(
                0,
                int(self.get_parameter("place_feedback_max_retries").value),
            )
            self.place_feedback_settle_s = max(
                0.0,
                float(self.get_parameter("place_feedback_settle_s").value),
            )
            self.place_feedback_recovery_height_m = max(
                0.0,
                float(
                    self.get_parameter("place_feedback_recovery_height_m").value
                ),
            )
            self.place_feedback_min_improvement_m = max(
                0.0,
                float(
                    self.get_parameter("place_feedback_min_improvement_m").value
                ),
            )
            self.place_descent_stage_count = max(
                1,
                int(self.get_parameter("place_descent_stage_count").value),
            )
            self.place_descent_max_trajectory_duration_s = max(
                1.0,
                float(
                    self.get_parameter(
                        "place_descent_max_trajectory_duration_s"
                    ).value
                ),
            )
            self.place_descent_max_joint_path_length_rad = max(
                0.05,
                float(
                    self.get_parameter(
                        "place_descent_max_joint_path_length_rad"
                    ).value
                ),
            )
            self.place_descent_stage_max_horizontal_motion_m = max(
                0.0,
                float(
                    self.get_parameter(
                        "place_descent_stage_max_horizontal_motion_m"
                    ).value
                ),
            )
            self.place_descent_stage_payload_tolerance_m = max(
                0.0,
                float(
                    self.get_parameter(
                        "place_descent_stage_payload_tolerance_m"
                    ).value
                ),
            )
            self.place_descent_endpoint_orientation_tolerance_rad = max(
                0.0,
                float(
                    self.get_parameter(
                        "place_descent_endpoint_orientation_tolerance_rad"
                    ).value
                ),
            )
            self.place_descent_start_tolerance_m = max(
                0.0,
                float(
                    self.get_parameter("place_descent_start_tolerance_m").value
                ),
            )
            self.place_descent_start_poll_s = max(
                0.05,
                float(self.get_parameter("place_descent_start_poll_s").value),
            )
            self.place_descent_start_timeout_s = max(
                self.place_descent_start_poll_s,
                float(self.get_parameter("place_descent_start_timeout_s").value),
            )
            self.place_descent_start_max_alignment_retries = max(
                0,
                int(
                    self.get_parameter(
                        "place_descent_start_max_alignment_retries"
                    ).value
                ),
            )
            self.place_descent_backend = str(
                self.get_parameter("place_descent_backend").value
            ).strip().lower()
            if self.place_descent_backend not in {"cartesian", "servo"}:
                raise ValueError(
                    "place_descent_backend must be 'cartesian' or 'servo'"
                )
            self.servo_cartesian_command_topic = str(
                self.get_parameter("servo_cartesian_command_topic").value
            )
            self.servo_status_topic = str(
                self.get_parameter("servo_status_topic").value
            )
            self.servo_switch_service = str(
                self.get_parameter("servo_switch_service").value
            )
            self.servo_pause_service = str(
                self.get_parameter("servo_pause_service").value
            )
            self.servo_descent_speed_mps = max(
                0.001,
                min(
                    0.03,
                    float(self.get_parameter("servo_descent_speed_mps").value),
                ),
            )
            self.servo_command_period_s = max(
                0.01,
                min(
                    0.10,
                    float(self.get_parameter("servo_command_period_s").value),
                ),
            )
            self.servo_stage_position_tolerance_m = max(
                0.0005,
                min(
                    0.005,
                    float(
                        self.get_parameter(
                            "servo_stage_position_tolerance_m"
                        ).value
                    ),
                ),
            )
            self.servo_stage_timeout_margin_s = max(
                0.5,
                float(
                    self.get_parameter("servo_stage_timeout_margin_s").value
                ),
            )
            self.servo_watchdog_period_s = max(
                self.servo_command_period_s,
                float(self.get_parameter("servo_watchdog_period_s").value),
            )
            self.servo_fault_injection_type = str(
                self.get_parameter("servo_fault_injection_type").value
            ).strip().lower()
            if self.servo_fault_injection_type not in {
                "none",
                "contact_loss",
                "payload_drift",
            }:
                raise ValueError(
                    "servo_fault_injection_type must be none, contact_loss, "
                    "or payload_drift"
                )
            self.servo_fault_injection_stage = max(
                1,
                int(self.get_parameter("servo_fault_injection_stage").value),
            )
            self.servo_fault_injection_delay_s = max(
                0.0,
                float(
                    self.get_parameter("servo_fault_injection_delay_s").value
                ),
            )
            self.servo_fault_injection_payload_drift_m = max(
                0.0,
                float(
                    self.get_parameter(
                        "servo_fault_injection_payload_drift_m"
                    ).value
                ),
            )
            self.servo_collision_limited_acceptance_m = max(
                self.servo_stage_position_tolerance_m,
                min(
                    0.015,
                    float(
                        self.get_parameter(
                            "servo_collision_limited_acceptance_m"
                        ).value
                    ),
                ),
            )
            self.enable_servo_cartesian_fallback = bool(
                self.get_parameter("enable_servo_cartesian_fallback").value
            )
            self.payload_transfer_velocity_scaling_factor = max(
                0.01,
                min(
                    1.0,
                    float(
                        self.get_parameter(
                            "payload_transfer_velocity_scaling_factor"
                        ).value
                    ),
                ),
            )
            self.place_descent_velocity_scaling_factor = max(
                0.005,
                min(
                    0.1,
                    float(
                        self.get_parameter(
                            "place_descent_velocity_scaling_factor"
                        ).value
                    ),
                ),
            )
            self.place_descent_recovery_velocity_scaling_factor = max(
                self.place_descent_velocity_scaling_factor,
                min(
                    0.1,
                    float(
                        self.get_parameter(
                            "place_descent_recovery_velocity_scaling_factor"
                        ).value
                    ),
                ),
            )
            self.payload_transfer_execution_replan_max_retries = max(
                0,
                int(
                    self.get_parameter(
                        "payload_transfer_execution_replan_max_retries"
                    ).value
                ),
            )
            self.payload_transfer_execution_replan_settle_s = max(
                0.0,
                float(
                    self.get_parameter(
                        "payload_transfer_execution_replan_settle_s"
                    ).value
                ),
            )
            self.allow_ompl_release_retreat_fallback = bool(
                self.get_parameter("allow_ompl_release_retreat_fallback").value
            )
            self.release_retreat_max_joint_path_length = float(
                self.get_parameter("release_retreat_max_joint_path_length").value
            )
            self.return_home_after_success = bool(
                self.get_parameter("return_home_after_success").value
            )
            self.return_home_scene_settle_s = float(
                self.get_parameter("return_home_scene_settle_s").value
            )
            self.return_home_max_retries = max(
                0,
                int(self.get_parameter("return_home_max_retries").value),
            )
            self.approach_scene_settle_s = float(
                self.get_parameter("approach_scene_settle_s").value
            )
            self.grasp_probe_max_retries = int(
                self.get_parameter("grasp_probe_max_retries").value
            )
            self.preclose_recenter_max_retries = max(
                0,
                int(
                    self.get_parameter(
                        "preclose_recenter_max_retries"
                    ).value
                ),
            )
            self.grasp_preclose_max_target_drift_m = float(
                self.get_parameter("grasp_preclose_max_target_drift_m").value
            )
            self.grasp_recovery_max_translation_m = float(
                self.get_parameter("grasp_recovery_max_translation_m").value
            )
            self.grasp_finger_asymmetry_correction_gain = max(
                0.0,
                float(
                    self.get_parameter(
                        "grasp_finger_asymmetry_correction_gain"
                    ).value
                ),
            )
            self.grasp_finger_asymmetry_min_m = max(
                0.0,
                float(
                    self.get_parameter("grasp_finger_asymmetry_min_m").value
                ),
            )
            self.grasp_finger_max_correction_m = max(
                0.0,
                float(
                    self.get_parameter("grasp_finger_max_correction_m").value
                ),
            )
            self.reset_to_home_before_execute = bool(
                self.get_parameter("reset_to_home_before_execute").value
            )
            self.execution_start_tolerance = float(
                self.get_parameter("execution_start_tolerance").value
            )
            self.fail_on_start_state_mismatch = bool(
                self.get_parameter("fail_on_start_state_mismatch").value
            )
            self.joint_state_wait_timeout_s = float(
                self.get_parameter("joint_state_wait_timeout_s").value
            )
            self.enable_dynamic_replanning = bool(
                self.get_parameter("enable_dynamic_replanning").value
            )
            self.dynamic_replan_request_topic = str(
                self.get_parameter("dynamic_replan_request_topic").value
            )
            self.maximum_dynamic_replans = max(
                0,
                int(self.get_parameter("maximum_dynamic_replans").value),
            )
            self.dynamic_replan_settle_s = max(
                0.0,
                float(self.get_parameter("dynamic_replan_settle_s").value),
            )
            self.plan_once = bool(self.get_parameter("plan_once").value)
            if self.execute_trajectories and self.use_fixed_home_start:
                self.get_logger().warn(
                    "execute_trajectories=true requires planning from the current robot state; "
                    "overriding use_fixed_home_start=false."
                )
                self.use_fixed_home_start = False

            result_qos = QoSProfile(depth=1)
            result_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.result_publisher = self.create_publisher(String, "/pick_plan_result", result_qos)
            demo_qos = QoSProfile(depth=10)
            demo_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.demo_state_publisher = self.create_publisher(String, "/pick_demo_state", demo_qos)
            self.active_candidate_publisher = self.create_publisher(
                String,
                "/active_grasp_candidate",
                demo_qos,
            )
            self.trajectory_execution_event_publisher = self.create_publisher(
                String,
                "/trajectory_execution_event",
                10,
            )
            self.latest_physical_target_pose: Optional[Tuple[float, float, float]] = None
            self.physical_target_reference_pose: Optional[Tuple[float, float, float]] = None
            self.grasp_probe_initial_z: Optional[float] = None
            self.grasp_probe_retry_count = 0
            self.preclose_recenter_retry_count = 0
            self.gripper_free_motion_retry_count = 0
            self.gripper_goal_rejection_retry_count = 0
            self.gripper_goal_rejection_retry_total = 0
            self.gripper_close_in_place_retry_count = 0
            self.gripper_close_in_place_retry_total = 0
            self.pending_finger_center_correction = (0.0, 0.0, 0.0)
            self.pre_place_candidate_attempt_count = 0
            self.pre_place_joint_target_positions: List[float] = []
            self.pre_place_joint_replay_max_plan_delta_rad: Optional[float] = None
            self.direct_place_candidate_attempt_count = 0
            self.return_home_retry_count = 0
            self.payload_place_compensation_applied = False
            self.payload_place_compensation_m = 0.0
            self.payload_grasp_calibration_delta_m = 0.0
            self.place_descent_object_in_hand_pose: Optional[Dict[str, object]] = None
            self.place_descent_payload_reanchor_delta_m = 0.0
            self.place_feedback_retry_count = 0
            self.place_feedback_error_m: Optional[float] = None
            self.place_feedback_previous_error_m: Optional[float] = None
            self.place_feedback_improvement_m: Optional[float] = None
            self.place_descent_reference_target_pose: Optional[
                Tuple[float, float, float]
            ] = None
            self.place_descent_horizontal_motion_m: Optional[float] = None
            self.place_descent_vertical_motion_m: Optional[float] = None
            self.place_descent_motion_m: Optional[float] = None
            self.place_descent_diagnostic_recorded = False
            self.place_descent_stage_index = 0
            self.place_descent_stage_pose_keys: List[str] = []
            self.pre_place_transfer_stage_index = 0
            self.pre_place_transfer_stage_pose_keys: List[str] = []
            self.place_descent_stage_reference_target_pose: Optional[
                Tuple[float, float, float]
            ] = None
            self.place_descent_stage_reference_hand_pose: Optional[
                Tuple[float, float, float]
            ] = None
            self.place_descent_stage_diagnostics: List[Dict[str, object]] = []
            self.place_descent_start_wait_count = 0
            self.place_descent_start_alignment_retry_count = 0
            self.place_descent_start_wait_timer = None
            self.servo_descent_timer = None
            self.servo_descent_active = False
            self.servo_descent_target_z: Optional[float] = None
            self.servo_descent_frame_id = ""
            self.servo_descent_stage_started_ns: Optional[int] = None
            self.servo_descent_stage_timeout_s = 0.0
            self.servo_watchdog_last_check_ns: Optional[int] = None
            self.servo_watchdog_check_count = 0
            self.servo_collision_deceleration_seen_in_stage = False
            self.servo_cartesian_fallback_used = False
            self.servo_cartesian_fallback_count = 0
            self.servo_fault_injection_triggered = False
            self.servo_fault_injection_observed_stage: Optional[int] = None
            self.servo_fault_injection_expected_ns: Optional[int] = None
            self.servo_fault_stop_command_ns: Optional[int] = None
            self.servo_watchdog_stop_latency_s: Optional[float] = None
            self.servo_watchdog_abort_stop_command_ns: Optional[int] = None
            self.servo_watchdog_abort_stop_monotonic_ns: Optional[int] = None
            self.servo_watchdog_abort_stage: Optional[int] = None
            self.servo_watchdog_abort_reason: Optional[str] = None
            self.latest_servo_status_code: Optional[int] = None
            self.latest_servo_status_message = ""
            self.servo_status_history: List[Dict[str, object]] = []
            self.grasp_probe_timer = None
            self.place_feedback_timer = None
            self.tactile_contact_settle_timer = None
            self.last_target_contact_ns: Dict[str, Optional[int]] = {
                "left": None,
                "right": None,
            }
            self.latest_tactile_target_collisions: Dict[str, List[str]] = {
                "left": [],
                "right": [],
            }
            self.tactile_grasp_verified = False
            self.tactile_release_verified = False
            self.tactile_release_retry_count = 0
            self.release_reference_target_pose: Optional[
                Tuple[float, float, float]
            ] = None
            self.release_object_horizontal_motion_m: Optional[float] = None
            self.release_object_vertical_motion_m: Optional[float] = None
            self.release_object_motion_m: Optional[float] = None
            self.release_settle_timer = None
            self.gripper_release_settle_timer = None
            self.return_home_scene_settle_timer = None
            self.approach_scene_settle_timer = None
            self.gripper_goal_rejection_retry_timer = None
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.servo_twist_publisher = self.create_publisher(
                TwistStamped,
                self.servo_cartesian_command_topic,
                10,
            )
            self.servo_switch_client = self.create_client(
                ServoCommandType,
                self.servo_switch_service,
            )
            self.servo_pause_client = self.create_client(
                SetBool,
                self.servo_pause_service,
            )
            if self.place_descent_backend == "servo":
                self.create_subscription(
                    ServoStatus,
                    self.servo_status_topic,
                    self.on_servo_status,
                    10,
                )
            if self.enable_physical_grasp_verification:
                self.create_subscription(
                    Pose,
                    self.physical_target_pose_topic,
                    self.on_physical_target_pose,
                    qos_profile_sensor_data,
                )
            if self.enable_tactile_grasp_supervision:
                self.create_subscription(
                    Contacts,
                    self.target_contact_topic,
                    self.on_target_object_contacts,
                    qos_profile_sensor_data,
                )
            self.display_publisher = self.create_publisher(DisplayTrajectory, "/display_planned_path", 10)
            self.cpp_trajectory_metrics_history: List[Dict[str, object]] = []
            self.move_client = ActionClient(self, MoveGroup, self.move_action_name)
            self.execute_client = ActionClient(
                self,
                ExecuteTrajectory,
                self.execute_trajectory_action_name,
            )
            self.arm_trajectory_client = ActionClient(
                self,
                FollowJointTrajectory,
                self.arm_controller_action_name,
            )
            self.gripper_trajectory_client = ActionClient(
                self,
                FollowJointTrajectory,
                self.gripper_action_name,
            )
            self.cartesian_client = self.create_client(GetCartesianPath, self.cartesian_service_name)
            self.diagnostic_ik_client = self.create_client(GetPositionIK, "/compute_ik")
            self.precheck_scene_client = self.create_client(GetPlanningScene, "/get_planning_scene")
            self.precheck_fk_client = self.create_client(GetPositionFK, "/compute_fk")
            self.diagnostic_validity_client = self.create_client(GetStateValidity, "/check_state_validity")
            self.create_subscription(String, "/grasp_candidates", self.on_grasp_candidates, 10)
            self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
            self.create_subscription(
                String,
                "/trajectory_quality_metrics",
                self.on_cpp_trajectory_metrics,
                10,
            )
            if self.enable_dynamic_replanning:
                self.create_subscription(
                    String,
                    self.dynamic_replan_request_topic,
                    self.on_dynamic_replan_request,
                    10,
                )

            self.has_started = False
            self.active = False
            self.candidate: Optional[Dict[str, object]] = None
            self.base_candidate: Optional[Dict[str, object]] = None
            self.steps: List[Dict[str, object]] = []
            self.cartesian_trajectory_metrics: List[Dict[str, object]] = []
            self.current_start_state: Optional[RobotState] = None
            self.latest_joint_state: Optional[JointState] = None
            self.pending_candidate: Optional[Dict[str, object]] = None
            self.waiting_for_joint_state = False
            self.joint_state_wait_started_ns: Optional[int] = None
            self.joint_state_wait_timer = None
            self.last_execution_start_check = ""
            self.pre_grasp_fallback_used = False
            self.home_diagnostic_used = False
            self.approach_recovery_used = False
            self.approach_recovery_attempt_count = 0
            self.active_approach_recovery_ratio: Optional[float] = None
            self.pre_grasp_alignment_recovery_used = False
            self.pre_grasp_alignment_prefix_attempt_count = 0
            self.contact_prefix_attempt_counts: Dict[str, int] = {}
            self.cartesian_retry_used: Dict[str, bool] = {}
            self.cartesian_timing_retry_counts: Dict[str, int] = {}
            self.cartesian_start_state_replan_counts: Dict[str, int] = {}
            self.ompl_invalid_plan_retry_counts: Dict[str, int] = {}
            self.recovery_ompl_retry_counts: Dict[str, int] = {}
            self.equivalent_grasp_attempt_count = 0
            self.active_grasp_yaw_offset_deg = 0.0
            self.payload_place_compensation_applied = False
            self.payload_place_compensation_m = 0.0
            self.payload_grasp_calibration_delta_m = 0.0
            self.place_descent_object_in_hand_pose = None
            self.place_descent_payload_reanchor_delta_m = 0.0
            self.place_feedback_retry_count = 0
            self.place_feedback_error_m = None
            self.place_feedback_previous_error_m = None
            self.place_feedback_improvement_m = None
            self.place_descent_reference_target_pose = None
            self.place_descent_horizontal_motion_m = None
            self.place_descent_vertical_motion_m = None
            self.place_descent_motion_m = None
            self.place_descent_diagnostic_recorded = False
            self.place_descent_stage_index = 0
            self.place_descent_stage_pose_keys = []
            self.pre_place_transfer_stage_index = 0
            self.pre_place_transfer_stage_pose_keys = []
            self.place_descent_stage_reference_target_pose = None
            self.place_descent_stage_reference_hand_pose = None
            self.place_descent_stage_diagnostics = []
            self.place_descent_start_wait_count = 0
            self.place_descent_start_alignment_retry_count = 0
            self.stop_servo_descent_output()
            self.latest_servo_status_code = None
            self.latest_servo_status_message = ""
            self.servo_status_history = []
            self.servo_watchdog_check_count = 0
            self.servo_collision_deceleration_seen_in_stage = False
            self.servo_cartesian_fallback_used = False
            self.servo_cartesian_fallback_count = 0
            self.servo_fault_injection_triggered = False
            self.servo_fault_injection_observed_stage = None
            self.servo_fault_injection_expected_ns = None
            self.servo_fault_stop_command_ns = None
            self.servo_watchdog_stop_latency_s = None
            self.servo_watchdog_abort_stop_command_ns = None
            self.servo_watchdog_abort_stop_monotonic_ns = None
            self.servo_watchdog_abort_stage = None
            self.servo_watchdog_abort_reason = None
            self.equivalent_grasp_recovery_yaw_offsets_deg = list(
                self.equivalent_grasp_yaw_offsets_deg
            )
            self.grasp_candidate_prevalidation_results: List[Dict[str, object]] = []
            self.grasp_candidate_prevalidation_candidates: List[
                Tuple[float, Dict[str, object]]
            ] = []
            self.grasp_candidate_prevalidation_index = 0
            self.grasp_candidate_prevalidation_start_state: Optional[RobotState] = None
            self.grasp_candidate_cartesian_prevalidation: List[
                Tuple[float, Dict[str, object], Dict[str, object], RobotState]
            ] = []
            self.grasp_candidate_cartesian_prevalidation_index = 0
            self.grasp_candidate_prevalidation_timer = None
            self.grasp_candidate_prevalidation_next_callback = None
            self.grasp_candidate_prevalidation_round = 0
            self.grasp_candidate_prevalidation_history: List[
                Dict[str, object]
            ] = []
            self.prevalidated_pre_grasp_trajectory = None
            self.prevalidated_pre_grasp_start_state = None
            self.prevalidated_pre_grasp_metrics: Optional[Dict[str, object]] = None
            self.retreat_ompl_fallback_used = False
            self.direct_path_fraction: Optional[float] = None
            self.direct_path_unchecked_fraction: Optional[float] = None
            self.direct_path_blocked: Optional[bool] = None
            self.ompl_metrics: List[Dict[str, object]] = []
            self.active_execute_goal_handle = None
            self.active_execution_step: Optional[str] = None
            self.dynamic_replan_requested = False
            self.dynamic_replan_count = 0
            self.dynamic_replan_events: List[Dict[str, object]] = []
            self.dynamic_replan_timer = None
            self.payload_transfer_execution_replan_count = 0
            self.payload_transfer_execution_replan_timer = None
            self.cartesian_execution_replan_counts: Dict[str, int] = {}
            self.cartesian_execution_replan_timer = None
            self.pre_place_transfer_ompl_fallback_used = False

            self.get_logger().info("Pick plan pipeline waiting for /grasp_candidates.")

        def on_joint_state(self, message: JointState) -> None:
            self.latest_joint_state = message
            if self.waiting_for_joint_state and self.pending_candidate is not None:
                candidate = self.pending_candidate
                self.pending_candidate = None
                self.waiting_for_joint_state = False
                self.joint_state_wait_started_ns = None
                self.stop_joint_state_wait_timer()
                self.start_pipeline(candidate)

        def on_cpp_trajectory_metrics(self, message: String) -> None:
            try:
                payload = json.loads(message.data)
            except (TypeError, ValueError):
                return
            if not isinstance(payload, dict) or payload.get("schema_version") != 1:
                return
            self.cpp_trajectory_metrics_history.append(payload)
            self.cpp_trajectory_metrics_history = self.cpp_trajectory_metrics_history[-100:]

        def on_grasp_candidates(self, message: String) -> None:
            if self.plan_once and self.has_started:
                return
            if self.active:
                return

            payload = json.loads(message.data)
            candidate = selected_candidate(payload)
            if candidate is None:
                self.publish_result("FAILED", "no selected candidate")
                return

            if not self.move_client.wait_for_server(timeout_sec=1.0):
                self.publish_result("WAITING", f"MoveGroup action {self.move_action_name} is not available")
                return
            if not self.cartesian_client.wait_for_service(timeout_sec=1.0):
                self.publish_result("WAITING", f"Cartesian service {self.cartesian_service_name} is not available")
                return
            if self.execute_trajectories and not self.execute_client.wait_for_server(timeout_sec=1.0):
                self.publish_result(
                    "WAITING",
                    f"ExecuteTrajectory action {self.execute_trajectory_action_name} is not available",
                )
                return
            if (
                self.execute_trajectories
                and not self.visualize_only_execution
                and not self.arm_trajectory_client.wait_for_server(timeout_sec=1.0)
            ):
                self.publish_result(
                    "WAITING",
                    f"arm controller action {self.arm_controller_action_name} is not available",
                )
                return
            if (
                self.execute_gripper
                and self.execute_trajectories
                and self.gripper_execution_backend == "follow_joint_trajectory"
                and not self.gripper_trajectory_client.wait_for_server(timeout_sec=1.0)
            ):
                self.publish_result(
                    "WAITING",
                    f"gripper action {self.gripper_action_name} is not available",
                )
                return

            if self.requires_joint_state_before_start() and self.latest_joint_state is None:
                self.pending_candidate = candidate
                self.waiting_for_joint_state = True
                self.joint_state_wait_started_ns = self.get_clock().now().nanoseconds
                self.get_logger().info("Waiting for /joint_states before execution planning.")
                if self.joint_state_wait_timer is None:
                    self.joint_state_wait_timer = self.create_timer(0.2, self.check_joint_state_wait_timeout)
                return

            self.start_pipeline(candidate)

        def start_pipeline(self, candidate: Dict[str, object]) -> None:
            if self.plan_once and self.has_started:
                return
            if self.active:
                return

            self.has_started = True
            self.active = True
            self.base_candidate = deepcopy(candidate)
            self.base_candidate["cube_symmetric_placement"] = bool(
                self.get_parameter("cube_symmetric_placement").value
            )
            pitch = float(self.get_parameter("grasp_pitch_offset_deg").value)
            if pitch:
                self.base_candidate = tilted_grasp_candidate(self.base_candidate, pitch)
                self.get_logger().info(f"Experimental closing-axis grasp pitch: {pitch:.1f}deg")
            self.candidate = deepcopy(self.base_candidate)
            self.steps = []
            self.cartesian_trajectory_metrics = []
            self.cpp_trajectory_metrics_history = []
            self.pre_grasp_fallback_used = False
            self.home_diagnostic_used = False
            self.approach_recovery_used = False
            self.approach_recovery_attempt_count = 0
            self.active_approach_recovery_ratio = None
            self.pre_grasp_alignment_recovery_used = False
            self.pre_grasp_alignment_prefix_attempt_count = 0
            self.contact_prefix_attempt_counts = {}
            self.cartesian_retry_used = {}
            self.cartesian_timing_retry_counts = {}
            self.cartesian_start_state_replan_counts = {}
            self.ompl_invalid_plan_retry_counts = {}
            self.recovery_ompl_retry_counts = {}
            self.equivalent_grasp_attempt_count = 0
            self.active_grasp_yaw_offset_deg = 0.0
            self.equivalent_grasp_recovery_yaw_offsets_deg = list(
                self.equivalent_grasp_yaw_offsets_deg
            )
            self.grasp_candidate_prevalidation_results = []
            self.grasp_candidate_prevalidation_candidates = []
            self.grasp_candidate_prevalidation_index = 0
            self.grasp_candidate_prevalidation_start_state = None
            self.grasp_candidate_cartesian_prevalidation = []
            self.grasp_candidate_cartesian_prevalidation_index = 0
            if self.grasp_candidate_prevalidation_timer is not None:
                self.grasp_candidate_prevalidation_timer.cancel()
                self.grasp_candidate_prevalidation_timer = None
            self.grasp_candidate_prevalidation_next_callback = None
            self.grasp_candidate_prevalidation_round = 0
            self.grasp_candidate_prevalidation_history = []
            self.prevalidated_pre_grasp_trajectory = None
            self.prevalidated_pre_grasp_start_state = None
            self.prevalidated_pre_grasp_metrics = None
            self.retreat_ompl_fallback_used = False
            self.direct_path_fraction = None
            self.direct_path_unchecked_fraction = None
            self.direct_path_blocked = None
            self.ompl_metrics = []
            self.active_execute_goal_handle = None
            self.active_execution_step = None
            self.dynamic_replan_requested = False
            self.dynamic_replan_count = 0
            self.dynamic_replan_events = []
            if self.dynamic_replan_timer is not None:
                self.dynamic_replan_timer.cancel()
                self.dynamic_replan_timer = None
            self.payload_transfer_execution_replan_count = 0
            self.pre_place_transfer_ompl_fallback_used = False
            if self.payload_transfer_execution_replan_timer is not None:
                self.payload_transfer_execution_replan_timer.cancel()
                self.payload_transfer_execution_replan_timer = None
            self.cartesian_execution_replan_counts = {}
            if self.cartesian_execution_replan_timer is not None:
                self.cartesian_execution_replan_timer.cancel()
                self.cartesian_execution_replan_timer = None
            self.gripper_free_motion_retry_count = 0
            self.gripper_goal_rejection_retry_count = 0
            self.gripper_goal_rejection_retry_total = 0
            if self.gripper_goal_rejection_retry_timer is not None:
                self.gripper_goal_rejection_retry_timer.cancel()
                self.gripper_goal_rejection_retry_timer = None
            self.gripper_close_in_place_retry_count = 0
            self.gripper_close_in_place_retry_total = 0
            self.pending_finger_center_correction = (0.0, 0.0, 0.0)
            self.pre_place_candidate_attempt_count = 0
            self.pre_place_joint_target_positions = []
            self.pre_place_joint_replay_max_plan_delta_rad = None
            self.direct_place_candidate_attempt_count = 0
            self.return_home_retry_count = 0
            self.last_execution_start_check = ""
            self.current_start_state = self.fixed_home_state() if self.use_fixed_home_start else None
            self.publish_active_candidate()
            if self.execute_trajectories and self.reset_to_home_before_execute:
                self.plan_reset_to_home()
                return
            self.begin_grasp_candidate_prevalidation()

        def prepare_gripper_for_approach(self) -> None:
            self.run_gripper_step(
                "gripper_open",
                self.gripper_open_position,
                None,
                self.after_gripper_prepared,
            )

        def begin_grasp_candidate_prevalidation(self, next_callback=None) -> None:
            if not self.enable_grasp_candidate_prevalidation:
                (next_callback or self.prepare_gripper_for_approach)()
                return
            if (
                self.enable_dynamic_replanning
                and self.defer_grasp_candidate_prevalidation_until_dynamic_replan
                and self.dynamic_replan_count == 0
            ):
                self.get_logger().info(
                    "Deferring grasp candidate prevalidation until the sensed "
                    "obstacle cancels the initial trajectory."
                )
                (next_callback or self.prepare_gripper_for_approach)()
                return
            if self.base_candidate is None:
                self.finish_failed("grasp candidate prevalidation: base candidate missing")
                return

            self.grasp_candidate_prevalidation_next_callback = (
                next_callback or self.prepare_gripper_for_approach
            )

            if self.force_grasp_yaw_offset:
                yaw_offsets = [self.forced_grasp_yaw_offset_deg]
            else:
                yaw_offsets = list(
                    dict.fromkeys(
                        [0.0]
                        + [
                            float(value)
                            for value in self.equivalent_grasp_yaw_offsets_deg
                        ]
                    )
                )
            self.grasp_candidate_prevalidation_candidates = [
                (
                    yaw_offset,
                    deepcopy(self.base_candidate)
                    if abs(yaw_offset) <= 1e-9
                    else equivalent_grasp_candidate(
                        self.base_candidate,
                        yaw_offset,
                    ),
                )
                for yaw_offset in yaw_offsets
            ]
            self.grasp_candidate_prevalidation_results = []
            self.grasp_candidate_prevalidation_index = 0
            self.grasp_candidate_prevalidation_round = 1
            self.grasp_candidate_prevalidation_history = []
            self.grasp_candidate_prevalidation_start_state = (
                self.measured_robot_state() or self.current_start_state
            )
            self.get_logger().info(
                "Prevalidating grasp orientations before execution: "
                + ", ".join(f"{value:.1f}deg" for value in yaw_offsets)
            )
            self.prevalidate_next_grasp_candidate()

        def prevalidate_next_grasp_candidate(self) -> None:
            if self.grasp_candidate_prevalidation_index >= len(
                self.grasp_candidate_prevalidation_candidates
            ):
                self.begin_grasp_candidate_cartesian_prevalidation()
                return

            yaw_offset, candidate = self.grasp_candidate_prevalidation_candidates[
                self.grasp_candidate_prevalidation_index
            ]
            pre_grasp = candidate.get("pre_grasp_pose")
            grasp = candidate.get("grasp_pose")
            if not isinstance(pre_grasp, dict) or not isinstance(grasp, dict):
                self.complete_grasp_candidate_prevalidation(
                    {
                        "yaw_offset_deg": yaw_offset,
                        "feasible": False,
                        "reason": "pre_grasp_pose or grasp_pose missing",
                    }
                )
                return

            goal = MoveGroup.Goal()
            prevalidation_planner_id = self.grasp_prevalidation_planner_id()
            goal.request = self.make_motion_plan_request(
                self.to_pose_stamped(pre_grasp),
                self.grasp_candidate_prevalidation_start_state,
                constrain_orientation=True,
                planner_id=prevalidation_planner_id,
                orientation_tolerance=self.pre_grasp_orientation_tolerance,
            )
            # Candidate evaluation must never move the robot.
            goal.planning_options = self.make_planning_options()
            future = self.move_client.send_goal_async(goal)
            future.add_done_callback(
                lambda done_future: self.on_grasp_prevalidation_goal_response(
                    done_future,
                    yaw_offset,
                    candidate,
                )
            )

        def on_grasp_prevalidation_goal_response(
            self,
            future,
            yaw_offset: float,
            candidate: Dict[str, object],
        ) -> None:
            try:
                goal_handle = future.result()
            except Exception as exc:
                self.complete_grasp_candidate_prevalidation(
                    {
                        "yaw_offset_deg": yaw_offset,
                        "feasible": False,
                        "reason": f"OMPL goal error: {exc}",
                    }
                )
                return
            if not goal_handle.accepted:
                self.complete_grasp_candidate_prevalidation(
                    {
                        "yaw_offset_deg": yaw_offset,
                        "feasible": False,
                        "reason": "OMPL goal rejected",
                    }
                )
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda done_future: self.on_grasp_prevalidation_ompl_result(
                    done_future,
                    yaw_offset,
                    candidate,
                )
            )

        def on_grasp_prevalidation_ompl_result(
            self,
            future,
            yaw_offset: float,
            candidate: Dict[str, object],
        ) -> None:
            try:
                result_wrapper = future.result()
                error_code = int(result_wrapper.result.error_code.val)
            except Exception as exc:
                self.complete_grasp_candidate_prevalidation(
                    {
                        "yaw_offset_deg": yaw_offset,
                        "feasible": False,
                        "reason": f"OMPL result error: {exc}",
                    }
                )
                return
            if (
                result_wrapper.status != GoalStatus.STATUS_SUCCEEDED
                or error_code != 1
            ):
                self.complete_grasp_candidate_prevalidation(
                    {
                        "yaw_offset_deg": yaw_offset,
                        "feasible": False,
                        "reason": (
                            f"OMPL failed: action_status={result_wrapper.status}, "
                            f"moveit_error_code={error_code}"
                        ),
                    }
                )
                return

            trajectory = result_wrapper.result.planned_trajectory
            end_state = self.end_state_from_trajectory(trajectory)
            points = trajectory.joint_trajectory.points
            if end_state is None or not points:
                self.complete_grasp_candidate_prevalidation(
                    {
                        "yaw_offset_deg": yaw_offset,
                        "feasible": False,
                        "reason": "OMPL trajectory has no endpoint",
                    }
                )
                return

            position_rows = [
                [float(value) for value in point.positions]
                for point in points
            ]
            provisional = {
                "yaw_offset_deg": yaw_offset,
                "feasible": False,
                "planner_id": self.grasp_prevalidation_planner_id(),
                "planning_time_s": float(
                    getattr(result_wrapper.result, "planning_time", 0.0)
                ),
                "joint_path_length_rad": joint_path_length(position_rows),
                "joint_limit_margin": normalized_joint_limit_margin(
                    list(trajectory.joint_trajectory.joint_names),
                    position_rows[-1],
                ),
                "pre_grasp_trajectory": trajectory,
            }
            self.grasp_candidate_cartesian_prevalidation.append(
                (yaw_offset, candidate, provisional, end_state)
            )
            self.grasp_candidate_prevalidation_index += 1
            self.prevalidate_next_grasp_candidate()

        def grasp_prevalidation_planner_id(self) -> str:
            if self.enable_dynamic_replanning and self.dynamic_replan_count > 0:
                return self.recovery_planner_id
            return self.planner_ids[0]

        def begin_grasp_candidate_cartesian_prevalidation(self) -> None:
            if bool(self.get_parameter("enable_place_endpoint_precheck").value):
                request = GetPlanningScene.Request()
                request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY

                def on_scene(response):
                    try:
                        self.precheck_target_dimensions = target_box_dimensions(response.scene)
                    except ValueError as exc:
                        self.finish_failed(f"place_endpoint_precheck: {exc}")
                        return
                    self.start_grasp_candidate_cartesian_prevalidation()

                self.call_place_precheck_service(
                    self.precheck_scene_client, request, on_scene,
                    lambda reason: self.finish_failed(f"place_endpoint_precheck: {reason}"),
                )
                return
            self.start_grasp_candidate_cartesian_prevalidation()

        def call_place_precheck_service(self, client, request, success, failure, timeout_s=10.0) -> None:
            # Bound observations with wall time even if Gazebo's clock stops.
            if not client.service_is_ready():
                failure("service unavailable")
                return
            finished = False
            timer = None

            def complete(response=None, reason=None):
                nonlocal finished
                if finished:
                    return
                finished = True
                if timer is not None:
                    timer.cancel()
                    self.destroy_timer(timer)
                if reason is not None:
                    failure(reason)
                else:
                    success(response)

            def receive(done):
                try:
                    response = done.result()
                except Exception as exc:
                    complete(reason=f"service error: {exc}")
                    return
                complete(response=response)

            try:
                future = client.call_async(request)
            except Exception as exc:
                complete(reason=f"service request error: {exc}")
                return
            timer = self.create_timer(
                timeout_s, lambda: complete(reason="service response timed out"),
                clock=Clock(clock_type=ClockType.STEADY_TIME),
            )
            future.add_done_callback(receive)

        def start_grasp_candidate_cartesian_prevalidation(self) -> None:
            if not self.grasp_candidate_cartesian_prevalidation:
                self.select_prevalidated_grasp_candidate()
                return
            self.grasp_candidate_cartesian_prevalidation_index = 0
            self.publish_demo_state(
                "grasp_candidate_prevalidation_contact",
                None,
                "planning",
            )
            self.schedule_grasp_candidate_prevalidation(
                self.prevalidate_next_grasp_cartesian_candidate
            )

        def prevalidate_next_grasp_cartesian_candidate(self) -> None:
            if self.grasp_candidate_cartesian_prevalidation_index >= len(
                self.grasp_candidate_cartesian_prevalidation
            ):
                self.publish_demo_state(
                    "grasp_candidate_prevalidation_restore",
                    None,
                    "planning",
                )
                self.schedule_grasp_candidate_prevalidation(
                    self.select_prevalidated_grasp_candidate
                )
                return

            yaw_offset, candidate, provisional, end_state = (
                self.grasp_candidate_cartesian_prevalidation[
                    self.grasp_candidate_cartesian_prevalidation_index
                ]
            )
            grasp = candidate["grasp_pose"]
            request = self.make_cartesian_request(
                grasp,
                self.cartesian_retry_max_step,
                start_state_override=end_state,
                step_name="grasp_candidate_prevalidation",
            )
            # Validate the exact alignment and contact approach, not just the
            # tolerance-bounded OMPL endpoint.
            request.waypoints = [
                self.to_pose(candidate["pre_grasp_pose"]),
                self.to_pose(grasp),
            ]
            request.avoid_collisions = True
            from concurrent.futures import Future

            def received(response):
                completed = Future()
                completed.set_result(response)
                self.on_grasp_prevalidation_cartesian_result(
                    completed, yaw_offset, candidate, provisional
                )

            def failed(reason):
                provisional.update(candidate=candidate, feasible=False,
                                   reason=f"Cartesian prevalidation: {reason}")
                self.complete_grasp_cartesian_candidate(provisional)

            self.call_place_precheck_service(
                self.cartesian_client, request, received, failed
            )

        def schedule_grasp_candidate_prevalidation(self, callback) -> None:
            if self.grasp_candidate_prevalidation_timer is not None:
                self.grasp_candidate_prevalidation_timer.cancel()
                self.grasp_candidate_prevalidation_timer = None
            if self.grasp_candidate_prevalidation_scene_settle_s <= 0.0:
                callback()
                return

            def invoke() -> None:
                if self.grasp_candidate_prevalidation_timer is not None:
                    self.grasp_candidate_prevalidation_timer.cancel()
                    self.grasp_candidate_prevalidation_timer = None
                callback()

            self.grasp_candidate_prevalidation_timer = self.create_timer(
                self.grasp_candidate_prevalidation_scene_settle_s,
                invoke,
            )

        def on_grasp_prevalidation_cartesian_result(
            self,
            future,
            yaw_offset: float,
            candidate: Dict[str, object],
            result: Dict[str, object],
        ) -> None:
            try:
                response = future.result()
                fraction = float(response.fraction)
                error_code = int(response.error_code.val)
                if not isfinite(fraction) or not 0.0 <= fraction <= 1.0:
                    raise ValueError("invalid Cartesian fraction")
                if not response.solution.joint_trajectory.points:
                    raise ValueError("empty Cartesian trajectory")
            except Exception as exc:
                result["reason"] = f"Cartesian result error: {exc}"
                result["candidate"] = candidate
                result["feasible"] = False
                self.complete_grasp_cartesian_candidate(result)
                return

            trajectory = response.solution.joint_trajectory
            if trajectory.points:
                contact_path_margin = min(
                    normalized_joint_limit_margin(
                        list(trajectory.joint_names),
                        [float(value) for value in point.positions],
                    )
                    for point in trajectory.points
                )
                result["joint_limit_margin"] = min(
                    float(result["joint_limit_margin"]),
                    contact_path_margin,
                )
            feasible = grasp_candidate_prevalidation_feasible(
                error_code,
                fraction,
                self.grasp_candidate_prevalidation_min_fraction,
                float(result["joint_limit_margin"]),
                self.grasp_candidate_prevalidation_min_joint_limit_margin,
            )
            result["cartesian_fraction"] = fraction
            result["moveit_error_code"] = error_code
            result["feasible"] = feasible
            result["score"] = grasp_candidate_prevalidation_score(
                fraction,
                float(result["joint_limit_margin"]),
                float(result["joint_path_length_rad"]),
                float(result["planning_time_s"]),
                yaw_offset,
            )
            if feasible:
                result["reason"] = (
                    "collision-checked OMPL and Cartesian paths valid, "
                    f"joint_limit_margin={float(result['joint_limit_margin']):.4f}"
                )
            else:
                result["reason"] = (
                    f"Cartesian fraction={fraction:.3f}, required="
                    f"{self.grasp_candidate_prevalidation_min_fraction:.3f}, "
                    f"joint_limit_margin={float(result['joint_limit_margin']):.4f}, "
                    "required_joint_limit_margin="
                    f"{self.grasp_candidate_prevalidation_min_joint_limit_margin:.4f}, "
                    f"moveit_error_code={error_code}"
                )
            result["candidate"] = candidate
            if bool(self.get_parameter("enable_place_endpoint_precheck").value):
                self.precheck_grasp_place_endpoint(
                    candidate, result, contact_end_state=self.end_state_from_trajectory(response.solution)
                )
                return
            self.complete_grasp_cartesian_candidate(result)

        def complete_grasp_cartesian_candidate(self, result) -> None:
            self.grasp_candidate_prevalidation_results.append(result)
            self.grasp_candidate_cartesian_prevalidation_index += 1
            self.prevalidate_next_grasp_cartesian_candidate()

        def precheck_grasp_place_endpoint(self, candidate, result, seed_index=0, contact_end_state=None) -> None:
            # Nominal loaded endpoint only: not a transfer-path or physical-grasp proof.
            observations = result.setdefault("place_endpoint_checks", [])
            if seed_index >= 3:
                result["feasible"] = False
                result["place_endpoint_valid"] = False
                result["reason"] = "no collision-free loaded placement endpoint in bounded seed search"
                self.complete_grasp_cartesian_candidate(result)
                return
            state = deepcopy(self.grasp_candidate_prevalidation_start_state)
            if seed_index > 0:
                state = self.fixed_home_state()
                sign = 1.0 if seed_index == 1 else -1.0
                state.joint_state.position[0] = 0.5 * sign
                state.joint_state.position[2] = 1.5 * sign
                state.joint_state.position[4] = -1.0 * sign
            try:
                request = placement_precheck_request(
                    candidate, state, self.precheck_target_dimensions,
                    self.group_name, self.end_effector_link,
                    self.gripper_closed_position, self.place_object_clearance_m,
                )
            except (ValueError, KeyError, TypeError) as exc:
                result.update(feasible=False, place_endpoint_valid=False,
                              reason=f"placement precheck input invalid: {exc}")
                self.complete_grasp_cartesian_candidate(result)
                return
            report = {"seed_index": seed_index, "executed": False,
                      "nominal_payload": True, "endpoint_valid": False}
            observations.append(report)

            def rejected(reason):
                report["reason"] = reason
                self.precheck_grasp_place_endpoint(candidate, result, seed_index + 1, contact_end_state)

            def on_validity(response):
                report["endpoint_valid"] = bool(response.valid)
                report["contacts"] = [[c.contact_body_1, c.contact_body_2]
                                      for c in response.contacts]
                if not response.valid:
                    rejected("loaded endpoint validity check rejected")
                    return
                result["place_endpoint_valid"] = True
                if bool(self.get_parameter("enable_loaded_path_precheck").value):
                    if not result["feasible"]:
                        result.update(loaded_path_valid=False,
                                      reason="complete lookahead requires a valid contact approach")
                        if abs(float(candidate.get("grasp_pitch_offset_deg", 0.0))) > 0:
                            branches = result.get("_reverse_connection_branches", [])
                            next_index = int(result.get("_reverse_connection_index", 0))
                            if branches and next_index < len(branches):
                                self.connect_prevalidated_reverse_branch(
                                    candidate, result, branches, next_index)
                                return
                            if result.get("reverse_connection_attempted", False):
                                self.complete_grasp_cartesian_candidate(result)
                                return
                            def probed(reports):
                                result["reverse_approach_probe"] = reports
                                valid = [r for r in reports if r["reverse_valid"]]
                                if valid:
                                    result["_reverse_connection_branches"] = valid
                                    result["_reverse_connection_index"] = 0
                                    self.connect_prevalidated_reverse_branch(
                                        candidate, result, valid, 0)
                                    return
                                self.complete_grasp_cartesian_candidate(result)
                            ReverseApproachProbe(
                                self, candidate, self.grasp_candidate_prevalidation_start_state,
                                normalized_joint_limit_margin, probed,
                                collect_all=True).start()
                            return
                        self.complete_grasp_cartesian_candidate(result)
                        return

                    def completed(passed, checks, reason):
                        result["loaded_path_valid"] = passed
                        result["loaded_path_checks"] = checks
                        result["reason"] = reason
                        result["feasible"] = bool(result["feasible"] and passed)
                        self.complete_grasp_cartesian_candidate(result)

                    LoadedPathPrecheck(self, candidate, contact_end_state, completed).start()
                    return
                self.complete_grasp_cartesian_candidate(result)

            def on_ik(response):
                report["ik_error_code"] = int(response.error_code.val)
                if response.error_code.val != 1:
                    rejected("collision-aware endpoint IK did not find a solution")
                    return
                check = GetStateValidity.Request()
                check.robot_state = response.solution
                check.robot_state.is_diff = True
                check.robot_state.attached_collision_objects = deepcopy(
                    request.ik_request.robot_state.attached_collision_objects
                )
                check.group_name = self.group_name
                self.call_place_precheck_service(
                    self.diagnostic_validity_client, check, on_validity, rejected
                )

            self.call_place_precheck_service(
                self.diagnostic_ik_client, request, on_ik, rejected
            )

        def connect_prevalidated_reverse_branch(self, candidate, result, branches, index) -> None:
            result["reverse_connection_attempted"] = True
            result["_reverse_connection_index"] = index + 1
            branch = branches[index]

            def failed(reason):
                result.update(feasible=False, loaded_path_valid=False,
                              reason=f"branch {index + 1}/{len(branches)}: {reason}")
                if index + 1 < len(branches):
                    self.connect_prevalidated_reverse_branch(
                        candidate, result, branches, index + 1)
                    return
                self.complete_grasp_cartesian_candidate(result)

            def connected(response):
                trajectory = response.planned_trajectory
                points = trajectory.joint_trajectory.points
                end_state = self.end_state_from_trajectory(trajectory)
                if not points or end_state is None:
                    failed("reverse connection returned an empty trajectory")
                    return
                rows = [list(p.positions) for p in points]
                result.update(
                    feasible=False, loaded_path_valid=False,
                    pre_grasp_trajectory=trajectory,
                    joint_path_length_rad=joint_path_length(rows),
                    planning_time_s=float(response.planning_time),
                    joint_limit_margin=min(normalized_joint_limit_margin(
                        list(trajectory.joint_trajectory.joint_names), row) for row in rows))
                self.grasp_candidate_cartesian_prevalidation[
                    self.grasp_candidate_cartesian_prevalidation_index
                ] = (result["yaw_offset_deg"], candidate, result, end_state)
                # The reverse path is not an executable substitute for forward checks.
                self.prevalidate_next_grasp_cartesian_candidate()

            connect_reverse_branch(self, candidate, branch, connected, failed)

        def complete_grasp_candidate_prevalidation(
            self,
            result: Dict[str, object],
        ) -> None:
            self.grasp_candidate_prevalidation_results.append(result)
            self.grasp_candidate_prevalidation_index += 1
            self.prevalidate_next_grasp_candidate()

        def select_prevalidated_grasp_candidate(self) -> None:
            require_place_endpoint = bool(self.get_parameter("enable_place_endpoint_precheck").value)
            require_loaded_path = bool(self.get_parameter("enable_loaded_path_precheck").value)
            feasible = [
                result
                for result in self.grasp_candidate_prevalidation_results
                if bool(result.get("feasible"))
                and (not require_place_endpoint or result.get("place_endpoint_valid") is True)
                and (not require_loaded_path or result.get("loaded_path_valid") is True)
            ]
            bounded_recovery_required = False
            if not feasible:
                planned_candidates = [
                    result
                    for result in self.grasp_candidate_prevalidation_results
                    if "score" in result and "candidate" in result
                    and (not require_place_endpoint or result.get("place_endpoint_valid") is True)
                    and (not require_loaded_path or result.get("loaded_path_valid") is True)
                ]
                margin_rejected = [
                    result
                    for result in planned_candidates
                    if float(result.get("joint_limit_margin", 0.0))
                    < self.grasp_candidate_prevalidation_min_joint_limit_margin
                ]
                if (
                    margin_rejected
                    and len(margin_rejected) == len(planned_candidates)
                    and self.grasp_candidate_prevalidation_round
                    < self.grasp_candidate_prevalidation_max_rounds
                ):
                    self.resample_grasp_candidate_prevalidation()
                    return
                if not planned_candidates:
                    reasons = "; ".join(
                        f"yaw={float(result.get('yaw_offset_deg', 0.0)):.1f}: "
                        f"{result.get('reason', 'rejected')}"
                        for result in self.grasp_candidate_prevalidation_results
                    )
                    self.finish_failed(
                        "grasp_candidate_prevalidation: no eligible reachable candidate; "
                        + reasons
                    )
                    return
                if margin_rejected and len(margin_rejected) == len(
                    planned_candidates
                ):
                    margins = ", ".join(
                        f"yaw={float(result.get('yaw_offset_deg', 0.0)):.1f}:"
                        f"{float(result.get('joint_limit_margin', 0.0)):.4f}"
                        for result in planned_candidates
                    )
                    self.finish_failed(
                        "grasp_candidate_prevalidation: all planned candidates "
                        "violate the minimum joint-limit margin after "
                        f"{self.grasp_candidate_prevalidation_round} round(s); "
                        + margins
                    )
                    return
                selected = max(
                    planned_candidates,
                    key=lambda result: float(result["score"]),
                )
                bounded_recovery_required = True
            else:
                selected = max(feasible, key=lambda result: float(result["score"]))
            self.candidate = deepcopy(selected["candidate"])
            self.prevalidated_pre_grasp_trajectory = deepcopy(
                selected.get("pre_grasp_trajectory")
            )
            self.prevalidated_pre_grasp_start_state = deepcopy(
                self.grasp_candidate_prevalidation_start_state
            )
            self.prevalidated_pre_grasp_metrics = {
                "planner_id": str(selected.get("planner_id", self.planner_id)),
                "planning_time_s": float(selected.get("planning_time_s", 0.0)),
                "joint_path_length_rad": float(
                    selected.get("joint_path_length_rad", 0.0)
                ),
                "joint_limit_margin": float(
                    selected.get("joint_limit_margin", 0.0)
                ),
            }
            yaw_offset = float(selected["yaw_offset_deg"])
            self.active_grasp_yaw_offset_deg = yaw_offset
            self.equivalent_grasp_recovery_yaw_offsets_deg = [
                value
                for value in self.equivalent_grasp_yaw_offsets_deg
                if abs(float(value) - yaw_offset) > 1e-9
            ]
            # Keep result telemetry compact and JSON serializable.
            for result in self.grasp_candidate_prevalidation_results:
                result.pop("candidate", None)
                result.pop("pre_grasp_trajectory", None)
            self.publish_active_candidate()
            selection_step = (
                "grasp_candidate_prevalidation_fallback_selected"
                if bounded_recovery_required
                else "grasp_candidate_prevalidation_selected"
            )
            self.record_step(
                selection_step,
                True,
                (
                    f"yaw_offset_deg={yaw_offset:.1f}, score="
                    f"{float(selected['score']):.3f}, feasible="
                    f"{len(feasible)}/"
                    f"{len(self.grasp_candidate_prevalidation_results)}, "
                    f"bounded_recovery_required={str(bounded_recovery_required).lower()}"
                    f", prevalidation_round={self.grasp_candidate_prevalidation_round}"
                ),
            )
            log_message = (
                f"Selected prevalidated grasp yaw={yaw_offset:.1f}deg with "
                f"score={float(selected['score']):.3f}."
            )
            if bounded_recovery_required:
                self.get_logger().warn(
                    log_message
                    + " No candidate passed the strict direct-approach gate; "
                    "continuing with bounded collision-checked recovery."
                )
            else:
                self.get_logger().info(log_message)
            callback = self.grasp_candidate_prevalidation_next_callback
            self.grasp_candidate_prevalidation_next_callback = None
            (callback or self.prepare_gripper_for_approach)()

        def resample_grasp_candidate_prevalidation(self) -> None:
            for result in self.grasp_candidate_prevalidation_results:
                compact = grasp_candidate_report(result)
                compact["round"] = self.grasp_candidate_prevalidation_round
                self.grasp_candidate_prevalidation_history.append(compact)
            self.grasp_candidate_prevalidation_round += 1
            self.grasp_candidate_prevalidation_results = []
            self.grasp_candidate_prevalidation_index = 0
            self.grasp_candidate_cartesian_prevalidation = []
            self.grasp_candidate_cartesian_prevalidation_index = 0
            self.grasp_candidate_prevalidation_start_state = (
                self.measured_robot_state() or self.current_start_state
            )
            self.record_step(
                "grasp_candidate_prevalidation_resample",
                False,
                (
                    "all planned candidates were too close to a joint limit; "
                    f"round={self.grasp_candidate_prevalidation_round}/"
                    f"{self.grasp_candidate_prevalidation_max_rounds}"
                ),
            )
            self.get_logger().warn(
                "Resampling grasp candidate IK branches because every planned "
                "contact path violated the joint-limit margin."
            )
            self.schedule_grasp_candidate_prevalidation(
                self.prevalidate_next_grasp_candidate
            )

        def after_gripper_prepared(self) -> None:
            if self.diagnose_direct_path_to_pre_grasp:
                self.diagnose_direct_path()
                return
            self.plan_to_pre_grasp()

        def diagnose_direct_path(self) -> None:
            if self.candidate is None:
                self.finish_failed("direct_path_diagnostic: candidate missing")
                return
            target_pose = self.candidate.get("pre_grasp_pose")
            if not isinstance(target_pose, dict):
                self.finish_failed("direct_path_diagnostic: pre_grasp_pose missing")
                return
            request = self.make_cartesian_request(target_pose, self.cartesian_max_step)
            request.avoid_collisions = True
            self.get_logger().info(
                "Diagnosing collision-aware direct Cartesian path to pre_grasp."
            )
            future = self.cartesian_client.call_async(request)
            future.add_done_callback(self.on_direct_path_diagnostic)

        def on_direct_path_diagnostic(self, future) -> None:
            try:
                response = future.result()
            except Exception as exc:
                self.record_step("direct_path_obstacle_diagnostic", False, str(exc))
                self.finish_failed(f"direct_path_obstacle_diagnostic: {exc}")
                return
            error_code = int(response.error_code.val)
            self.direct_path_fraction = float(response.fraction)
            if error_code != 1:
                reason = (
                    f"collision_checked_fraction={self.direct_path_fraction:.3f}, "
                    f"moveit_error_code={error_code}"
                )
                self.record_step("direct_path_obstacle_diagnostic", False, reason)
                self.finish_failed(f"direct_path_obstacle_diagnostic: {reason}")
                return
            if self.candidate is None:
                self.finish_failed("direct_path_obstacle_diagnostic: candidate missing")
                return
            target_pose = self.candidate.get("pre_grasp_pose")
            if not isinstance(target_pose, dict):
                self.finish_failed("direct_path_obstacle_diagnostic: pre_grasp_pose missing")
                return
            unchecked_request = self.make_cartesian_request(
                target_pose,
                self.cartesian_max_step,
            )
            unchecked_request.avoid_collisions = False
            unchecked_future = self.cartesian_client.call_async(unchecked_request)
            unchecked_future.add_done_callback(self.on_unchecked_direct_path_diagnostic)

        def on_unchecked_direct_path_diagnostic(self, future) -> None:
            try:
                response = future.result()
            except Exception as exc:
                self.record_step("direct_path_obstacle_diagnostic", False, str(exc))
                self.finish_failed(f"direct_path_obstacle_diagnostic: {exc}")
                return
            error_code = int(response.error_code.val)
            self.direct_path_unchecked_fraction = float(response.fraction)
            collision_checked_blocked = (
                self.direct_path_fraction is not None
                and self.direct_path_fraction < self.direct_path_blocked_fraction_threshold
            )
            unchecked_reachable = (
                self.direct_path_unchecked_fraction
                >= self.direct_path_blocked_fraction_threshold
            )
            self.direct_path_blocked = (
                error_code == 1 and collision_checked_blocked and unchecked_reachable
            )
            reason = (
                f"collision_checked_fraction={self.direct_path_fraction:.3f}, "
                f"unchecked_fraction={self.direct_path_unchecked_fraction:.3f}, "
                f"collision_blocked={str(self.direct_path_blocked).lower()}, "
                f"threshold={self.direct_path_blocked_fraction_threshold:.3f}, "
                f"moveit_error_code={error_code}"
            )
            self.record_step(
                "direct_path_obstacle_diagnostic",
                error_code == 1,
                reason,
            )
            if error_code != 1:
                self.finish_failed(f"direct_path_obstacle_diagnostic: {reason}")
                return
            if self.require_direct_path_blocked and not self.direct_path_blocked:
                self.finish_failed(
                    "challenge scene invalid: direct path was not specifically blocked by collision; "
                    + reason
                )
                return
            self.plan_to_pre_grasp()

        def plan_to_pre_grasp(self) -> None:
            if self.prevalidated_pre_grasp_trajectory is not None:
                trajectory = self.prevalidated_pre_grasp_trajectory
                display_start_state = self.prevalidated_pre_grasp_start_state
                metrics = self.prevalidated_pre_grasp_metrics or {}
                self.prevalidated_pre_grasp_trajectory = None
                self.prevalidated_pre_grasp_start_state = None
                self.prevalidated_pre_grasp_metrics = None
                end_state = self.end_state_from_trajectory(trajectory)
                if end_state is None:
                    self.record_step(
                        "ompl_to_pre_grasp",
                        False,
                        "prevalidated trajectory has no joint endpoint",
                    )
                    self.finish_failed(
                        "ompl_to_pre_grasp: prevalidated trajectory has no joint endpoint"
                    )
                    return
                self.current_start_state = end_state
                self.ompl_metrics.append(
                    {
                        "step": "ompl_to_pre_grasp",
                        "planner_id": str(metrics.get("planner_id", self.planner_id)),
                        "planning_time_s": float(
                            metrics.get("planning_time_s", 0.0)
                        ),
                        "joint_path_length_rad": float(
                            metrics.get("joint_path_length_rad", 0.0)
                        ),
                        "trajectory_points": len(
                            trajectory.joint_trajectory.points
                        ),
                        "joint_limit_margin": float(
                            metrics.get("joint_limit_margin", 0.0)
                        ),
                        "prevalidated": True,
                    }
                )
                self.get_logger().info(
                    "Executing the exact pre-grasp trajectory selected by "
                    "candidate prevalidation."
                )
                self.handle_successful_trajectory(
                    "ompl_to_pre_grasp",
                    "pre_grasp_pose",
                    trajectory,
                    "prevalidated_trajectory",
                    self.after_pre_grasp,
                    display_start_state,
                    self.plan_to_pre_grasp,
                )
                return
            self.plan_ompl_step(
                "ompl_to_pre_grasp",
                "pre_grasp_pose",
                self.after_pre_grasp,
                constrain_orientation=True,
            )

        def requires_joint_state_before_start(self) -> bool:
            return (
                self.execute_trajectories
                and not self.visualize_only_execution
                and self.fail_on_start_state_mismatch
            )

        def check_joint_state_wait_timeout(self) -> None:
            if not self.waiting_for_joint_state:
                return
            if self.latest_joint_state is not None and self.pending_candidate is not None:
                candidate = self.pending_candidate
                self.pending_candidate = None
                self.waiting_for_joint_state = False
                self.joint_state_wait_started_ns = None
                self.stop_joint_state_wait_timer()
                self.start_pipeline(candidate)
                return
            if self.joint_state_wait_started_ns is None:
                self.joint_state_wait_started_ns = self.get_clock().now().nanoseconds
            elapsed_s = (self.get_clock().now().nanoseconds - self.joint_state_wait_started_ns) / 1e9
            if elapsed_s >= self.joint_state_wait_timeout_s:
                self.pending_candidate = None
                self.waiting_for_joint_state = False
                self.joint_state_wait_started_ns = None
                self.stop_joint_state_wait_timer()
                self.publish_result(
                    "FAILED",
                    f"/joint_states unavailable after {self.joint_state_wait_timeout_s:.1f}s",
                )

        def stop_joint_state_wait_timer(self) -> None:
            if self.joint_state_wait_timer is not None:
                self.joint_state_wait_timer.cancel()
                self.joint_state_wait_timer = None

        def plan_reset_to_home(self) -> None:
            goal = MoveGroup.Goal()
            goal.request = self.make_home_motion_plan_request()
            goal.planning_options = self.make_planning_options()

            self.get_logger().info("Planning reset_to_home before execution pipeline.")
            future = self.move_client.send_goal_async(goal)
            future.add_done_callback(self.on_reset_goal_response)

        def on_reset_goal_response(self, future) -> None:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.record_step("reset_to_home", False, "goal rejected")
                self.finish_failed("reset_to_home: goal rejected")
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.on_reset_result)

        def on_reset_result(self, future) -> None:
            result_wrapper = future.result()
            error_code = int(result_wrapper.result.error_code.val)
            success = result_wrapper.status == GoalStatus.STATUS_SUCCEEDED and error_code == 1
            if not success:
                reason = f"action_status={result_wrapper.status}, moveit_error_code={error_code}"
                self.record_step("reset_to_home", False, reason)
                self.finish_failed(f"reset_to_home: {reason}")
                return

            trajectory = result_wrapper.result.planned_trajectory
            display_start_state = getattr(result_wrapper.result, "trajectory_start", None)
            self.current_start_state = self.end_state_from_trajectory(trajectory)
            if self.current_start_state is None:
                self.record_step("reset_to_home", False, "planned trajectory has no joint endpoint")
                self.finish_failed("reset_to_home: planned trajectory has no joint endpoint")
                return

            self.handle_successful_trajectory(
                "reset_to_home",
                None,
                trajectory,
                "",
                self.after_reset_to_home,
                display_start_state,
            )

        def after_reset_to_home(self) -> None:
            self.current_start_state = None
            self.begin_grasp_candidate_prevalidation()

        def plan_ompl_step(
            self,
            step_name: str,
            pose_key: str,
            next_callback,
            constrain_orientation: bool = False,
            orientation_tolerance: Optional[float] = None,
            planner_id_override: Optional[str] = None,
        ) -> None:
            if self.candidate is None:
                self.finish_failed("candidate missing")
                return
            pose_data = self.candidate.get(pose_key)
            if pose_data is None:
                self.finish_failed(f"{pose_key} missing")
                return

            active_planner_id = planner_id_override or self.planner_ids[0]
            goal = MoveGroup.Goal()
            goal.request = self.make_motion_plan_request(
                self.to_pose_stamped(pose_data),
                self.current_start_state,
                constrain_orientation=constrain_orientation,
                planner_id=active_planner_id,
                orientation_tolerance=orientation_tolerance,
            )
            goal.planning_options = self.make_ompl_planning_options()

            self.get_logger().info(
                f"Planning {step_name} -> {pose_key} with {active_planner_id}."
            )
            future = self.move_client.send_goal_async(goal)
            future.add_done_callback(
                lambda done_future: self.on_ompl_goal_response(
                    done_future,
                    step_name,
                    pose_key,
                    next_callback,
                    constrain_orientation,
                    0,
                    orientation_tolerance,
                    planner_id_override,
                )
            )

        def retry_ompl_step(
            self,
            step_name: str,
            pose_key: str,
            next_callback,
            constrain_orientation: bool,
            planner_index: int,
            orientation_tolerance: Optional[float] = None,
            planner_id_override: Optional[str] = None,
        ) -> None:
            if self.candidate is None:
                self.finish_failed("candidate missing")
                return
            pose_data = self.candidate.get(pose_key)
            if pose_data is None:
                self.finish_failed(f"{pose_key} missing")
                return

            planner_id = planner_id_override or self.planner_ids[planner_index]
            goal = MoveGroup.Goal()
            goal.request = self.make_motion_plan_request(
                self.to_pose_stamped(pose_data),
                self.current_start_state,
                constrain_orientation=constrain_orientation,
                planner_id=planner_id,
                orientation_tolerance=orientation_tolerance,
            )
            goal.planning_options = self.make_ompl_planning_options()
            self.get_logger().warn(f"Retrying {step_name} -> {pose_key} with {planner_id}.")
            future = self.move_client.send_goal_async(goal)
            future.add_done_callback(
                lambda done_future: self.on_ompl_goal_response(
                    done_future,
                    step_name,
                    pose_key,
                    next_callback,
                    constrain_orientation,
                    planner_index,
                    orientation_tolerance,
                    planner_id_override,
                )
            )

        def on_ompl_goal_response(
            self,
            future,
            step_name: str,
            pose_key: str,
            next_callback,
            constrain_orientation: bool,
            planner_index: int,
            orientation_tolerance: Optional[float],
            planner_id_override: Optional[str],
        ) -> None:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.record_step(step_name, False, "goal rejected")
                self.finish_failed(f"{step_name} goal rejected")
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda done_future: self.on_ompl_result(
                    done_future,
                    step_name,
                    pose_key,
                    next_callback,
                    constrain_orientation,
                    planner_index,
                    orientation_tolerance,
                    planner_id_override,
                )
            )

        def on_ompl_result(
            self,
            future,
            step_name: str,
            pose_key: str,
            next_callback,
            constrain_orientation: bool,
            planner_index: int,
            orientation_tolerance: Optional[float],
            planner_id_override: Optional[str],
        ) -> None:
            result_wrapper = future.result()
            error_code = int(result_wrapper.result.error_code.val)
            success = result_wrapper.status == GoalStatus.STATUS_SUCCEEDED and error_code == 1
            if not success:
                planner_id = planner_id_override or self.planner_ids[planner_index]
                reason = (
                    f"planner_id={planner_id}, "
                    f"action_status={result_wrapper.status}, moveit_error_code={error_code}"
                )
                retry_key = f"{step_name}:{planner_id}"
                invalid_plan_retry_count = self.ompl_invalid_plan_retry_counts.get(
                    retry_key,
                    0,
                )
                # INVALID_MOTION_PLAN means MoveIt rejected the sampled path before
                # execution. A bounded replan is safe and gives stochastic planners
                # another chance without weakening collision validation.
                if (
                    error_code == -2
                    and invalid_plan_retry_count < self.ompl_invalid_plan_retries
                ):
                    invalid_plan_retry_count += 1
                    self.ompl_invalid_plan_retry_counts[retry_key] = (
                        invalid_plan_retry_count
                    )
                    self.record_step(
                        f"{step_name}_invalid_plan_replan",
                        False,
                        (
                            f"{reason}, retry={invalid_plan_retry_count}/"
                            f"{self.ompl_invalid_plan_retries}"
                        ),
                    )
                    self.get_logger().warn(
                        f"{step_name} returned INVALID_MOTION_PLAN; replanning with "
                        f"{planner_id} ({invalid_plan_retry_count}/"
                        f"{self.ompl_invalid_plan_retries})."
                    )
                    self.retry_ompl_step(
                        step_name,
                        pose_key,
                        next_callback,
                        constrain_orientation,
                        planner_index,
                        orientation_tolerance,
                        planner_id_override,
                    )
                    return
                recovery_steps = {
                    "ompl_to_pre_grasp_alignment_staging",
                    "ompl_to_strict_pre_grasp",
                    "ompl_to_approach_recovery",
                    "ompl_to_equivalent_grasp_staging",
                    "ompl_to_equivalent_grasp_pre_grasp",
                }
                recovery_suffix = (
                    (
                        f"orientation={self.equivalent_grasp_attempt_count}:"
                        f"approach={self.approach_recovery_attempt_count}"
                    )
                    if step_name == "ompl_to_approach_recovery"
                    else f"orientation={self.equivalent_grasp_attempt_count}"
                )
                recovery_retry_key = (
                    f"{step_name}:{planner_id}:{recovery_suffix}"
                )
                recovery_retry_count = self.recovery_ompl_retry_counts.get(
                    recovery_retry_key,
                    0,
                )
                if (
                    step_name in recovery_steps
                    and error_code == 99999
                    and recovery_retry_count < self.recovery_ompl_retries
                ):
                    recovery_retry_count += 1
                    self.recovery_ompl_retry_counts[recovery_retry_key] = (
                        recovery_retry_count
                    )
                    self.record_step(
                        f"{step_name}_replan",
                        False,
                        (
                            f"{reason}, retry={recovery_retry_count}/"
                            f"{self.recovery_ompl_retries}"
                        ),
                    )
                    self.retry_ompl_step(
                        step_name,
                        pose_key,
                        next_callback,
                        constrain_orientation,
                        planner_index,
                        orientation_tolerance,
                        planner_id_override,
                    )
                    return
                if (
                    step_name == "ompl_to_approach_recovery"
                    and self.has_approach_recovery_candidate()
                ):
                    ratio_text = (
                        "unknown"
                        if self.active_approach_recovery_ratio is None
                        else f"{self.active_approach_recovery_ratio:.2f}"
                    )
                    self.record_step(
                        "ompl_to_approach_recovery_candidate_failed",
                        False,
                        f"{reason}, ratio={ratio_text}",
                    )
                    self.plan_approach_recovery()
                    return
                if (
                    planner_id_override is None
                    and planner_index + 1 < len(self.planner_ids)
                ):
                    self.record_step(f"{step_name}_planner_fallback", False, reason)
                    self.retry_ompl_step(
                        step_name,
                        pose_key,
                        next_callback,
                        constrain_orientation,
                        planner_index + 1,
                        orientation_tolerance,
                        planner_id_override,
                    )
                    return
                if (
                    step_name in {
                        "ompl_to_pre_grasp_alignment_staging",
                        "ompl_to_strict_pre_grasp",
                        "ompl_to_approach_recovery",
                        "ompl_to_equivalent_grasp_staging",
                        "ompl_to_equivalent_grasp_pre_grasp",
                    }
                    and self.has_equivalent_grasp_orientation()
                ):
                    self.record_step(
                        "equivalent_grasp_orientation_triggered",
                        False,
                        f"{step_name}: {reason}",
                    )
                    self.plan_equivalent_grasp_orientation()
                    return
                if step_name == "ompl_to_pre_grasp" and not self.pre_grasp_fallback_used:
                    self.pre_grasp_fallback_used = True
                    self.record_step("ompl_to_pre_grasp_orientation_fallback", False, reason)
                    self.get_logger().warn(
                        "ompl_to_pre_grasp failed with orientation constraint; retrying position-only."
                    )
                    self.plan_ompl_step(
                        "ompl_to_pre_grasp",
                        "pre_grasp_pose",
                        self.after_pre_grasp,
                        constrain_orientation=False,
                    )
                    return
                if (
                    self.execute_trajectories
                    and not self.use_fixed_home_start
                    and step_name == "ompl_to_pre_grasp"
                    and not self.home_diagnostic_used
                ):
                    self.home_diagnostic_used = True
                    self.record_step("ompl_to_pre_grasp", False, reason)
                    self.plan_fixed_home_pre_grasp_diagnostic(reason)
                    return
                self.record_step(step_name, False, reason)
                self.finish_failed(f"{step_name}: {reason}")
                return

            trajectory = result_wrapper.result.planned_trajectory
            planner_id = planner_id_override or self.planner_ids[planner_index]
            position_rows = [
                [float(value) for value in point.positions]
                for point in trajectory.joint_trajectory.points
            ]
            self.ompl_metrics.append(
                {
                    "step": step_name,
                    "planner_id": planner_id,
                    "planning_time_s": float(
                        getattr(result_wrapper.result, "planning_time", 0.0)
                    ),
                    "joint_path_length_rad": joint_path_length(position_rows),
                    "trajectory_points": len(position_rows),
                }
            )
            display_start_state = getattr(result_wrapper.result, "trajectory_start", None)
            self.current_start_state = self.end_state_from_trajectory(trajectory)
            if self.current_start_state is None:
                self.record_step(step_name, False, "planned trajectory has no joint endpoint")
                self.finish_failed(f"{step_name}: planned trajectory has no joint endpoint")
                return

            if self.ompl_was_executed_by_move_group():
                self.record_step(step_name, True, "move_group_executed")
                if display_start_state is not None:
                    self.publish_display_trajectory(display_start_state, trajectory)
                self.publish_demo_state(step_name, pose_key, "executed")
                next_callback()
                return

            replan_callback = lambda: self.retry_ompl_step(
                step_name,
                pose_key,
                next_callback,
                constrain_orientation,
                planner_index,
                orientation_tolerance,
                planner_id_override,
            )
            if (
                step_name == "ompl_to_pre_grasp"
                and self.enable_grasp_candidate_prevalidation
                and self.enable_dynamic_replanning
            ):
                replan_callback = (
                    self.prevalidate_grasp_candidates_after_dynamic_replan
                )

            self.handle_successful_trajectory(
                step_name,
                pose_key,
                trajectory,
                "",
                next_callback,
                display_start_state,
                replan_callback,
            )

        def prevalidate_grasp_candidates_after_dynamic_replan(self) -> None:
            if self.base_candidate is None:
                self.finish_failed(
                    "dynamic grasp candidate prevalidation: base candidate missing"
                )
                return
            self.candidate = deepcopy(self.base_candidate)
            self.pre_grasp_fallback_used = False
            self.home_diagnostic_used = False
            self.approach_recovery_used = False
            self.approach_recovery_attempt_count = 0
            self.active_approach_recovery_ratio = None
            self.pre_grasp_alignment_recovery_used = False
            self.pre_grasp_alignment_prefix_attempt_count = 0
            self.contact_prefix_attempt_counts = {}
            self.cartesian_retry_used = {}
            self.cartesian_timing_retry_counts = {}
            self.equivalent_grasp_attempt_count = 0
            self.active_grasp_yaw_offset_deg = 0.0
            self.equivalent_grasp_recovery_yaw_offsets_deg = list(
                self.equivalent_grasp_yaw_offsets_deg
            )
            self.current_start_state = None
            self.publish_active_candidate()
            self.record_step(
                "grasp_candidate_prevalidation_triggered",
                True,
                (
                    "updated PlanningScene after dynamic cancellation, "
                    f"replan={self.dynamic_replan_count}/"
                    f"{self.maximum_dynamic_replans}"
                ),
            )
            self.begin_grasp_candidate_prevalidation(
                self.plan_prevalidated_dynamic_grasp
            )

        def plan_prevalidated_dynamic_grasp(self) -> None:
            self.plan_ompl_step(
                "ompl_to_pre_grasp",
                "pre_grasp_pose",
                self.after_pre_grasp,
                constrain_orientation=True,
                orientation_tolerance=self.pre_grasp_orientation_tolerance,
                planner_id_override=self.recovery_planner_id,
            )

        def plan_fixed_home_pre_grasp_diagnostic(self, original_reason: str) -> None:
            if self.candidate is None:
                self.finish_failed(f"ompl_to_pre_grasp: {original_reason}")
                return
            pose_data = self.candidate.get("pre_grasp_pose")
            if pose_data is None:
                self.finish_failed(f"ompl_to_pre_grasp: {original_reason}; pre_grasp_pose missing")
                return

            planner_id = self.planner_ids[0]
            goal = MoveGroup.Goal()
            goal.request = self.make_motion_plan_request(
                self.to_pose_stamped(pose_data),
                self.fixed_home_state(),
                constrain_orientation=False,
                planner_id=planner_id,
            )
            goal.planning_options = self.make_planning_options()
            self.get_logger().warn(
                "Current-state pre-grasp planning failed; running fixed-home diagnostic plan-only request."
            )
            future = self.move_client.send_goal_async(goal)
            future.add_done_callback(
                lambda done_future: self.on_fixed_home_diagnostic_goal_response(
                    done_future,
                    original_reason,
                    planner_id,
                )
            )

        def on_fixed_home_diagnostic_goal_response(
            self,
            future,
            original_reason: str,
            planner_id: str,
        ) -> None:
            goal_handle = future.result()
            if not goal_handle.accepted:
                reason = self.current_home_delta_summary()
                self.record_step(
                    "diagnostic_fixed_home_to_pre_grasp",
                    False,
                    f"goal rejected, {reason}",
                )
                self.finish_failed(
                    f"ompl_to_pre_grasp current-state failed: {original_reason}; "
                    f"fixed-home diagnostic goal rejected; {reason}"
                )
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda done_future: self.on_fixed_home_diagnostic_result(
                    done_future,
                    original_reason,
                    planner_id,
                )
            )

        def on_fixed_home_diagnostic_result(
            self,
            future,
            original_reason: str,
            planner_id: str,
        ) -> None:
            result_wrapper = future.result()
            error_code = int(result_wrapper.result.error_code.val)
            success = result_wrapper.status == GoalStatus.STATUS_SUCCEEDED and error_code == 1
            diagnostic_status = (
                f"planner_id={planner_id}, action_status={result_wrapper.status}, "
                f"moveit_error_code={error_code}, {self.current_home_delta_summary()}"
            )
            self.record_step(
                "diagnostic_fixed_home_to_pre_grasp",
                success,
                diagnostic_status,
            )
            if success:
                self.finish_failed(
                    "ompl_to_pre_grasp current-state planning failed, "
                    "but fixed-home diagnostic planning succeeded; "
                    f"current robot state is likely not a good execution start. "
                    f"original={original_reason}; {diagnostic_status}"
                )
                return
            self.finish_failed(
                "ompl_to_pre_grasp failed from current state and fixed-home diagnostic also failed; "
                f"original={original_reason}; diagnostic={diagnostic_status}"
            )

        def after_pre_grasp(self) -> None:
            self.plan_cartesian_step(
                "cartesian_pre_grasp_alignment",
                "pre_grasp_pose",
                self.after_pre_grasp_alignment,
            )

        def after_pre_grasp_alignment(self) -> None:
            if self.approach_scene_settle_s > 0.0:
                if self.approach_scene_settle_timer is not None:
                    self.approach_scene_settle_timer.cancel()
                self.approach_scene_settle_timer = self.create_timer(
                    self.approach_scene_settle_s,
                    self.after_approach_scene_settle,
                )
                return
            self.after_approach_scene_settle()

        def after_approach_scene_settle(self) -> None:
            if self.approach_scene_settle_timer is not None:
                self.approach_scene_settle_timer.cancel()
                self.approach_scene_settle_timer = None
            self.plan_cartesian_step("cartesian_approach", "grasp_pose", self.after_grasp)

        def after_grasp(self) -> None:
            # A new arm approach gets its own local gripper retry budget. A
            # same-pose gripper retry calls run_gripper_step() directly.
            self.gripper_close_in_place_retry_count = 0
            if self.enable_physical_grasp_verification:
                translation = self.current_target_translation()
                if translation is None:
                    self.finish_failed("preclose_target_stability: target pose unavailable")
                    return
                drift = translation_distance(translation)
                if drift > self.grasp_preclose_max_target_drift_m:
                    reason = (
                        f"target_drift={drift:.4f}m exceeds "
                        f"{self.grasp_preclose_max_target_drift_m:.4f}m"
                    )
                    self.record_step("preclose_target_stability", False, reason)
                    if (
                        self.preclose_recenter_retry_count
                        >= self.preclose_recenter_max_retries
                    ):
                        self.finish_failed(f"preclose_target_stability failed: {reason}")
                        return
                    self.preclose_recenter_retry_count += 1
                    self.after_grasp_recovery_open()
                    return
                self.record_step(
                    "preclose_target_stability",
                    True,
                    f"target_drift={drift:.4f}m",
                )
            if self.enable_tactile_grasp_supervision:
                self.last_target_contact_ns = {"left": None, "right": None}
                self.latest_tactile_target_collisions = {"left": [], "right": []}
            self.run_gripper_step(
                "gripper_closed",
                self.gripper_closed_position,
                "grasp_pose",
                self.after_gripper_closed,
            )

        def after_gripper_closed(self) -> None:
            if self.enable_tactile_grasp_supervision:
                if self.tactile_contact_settle_timer is not None:
                    self.tactile_contact_settle_timer.cancel()
                self.tactile_contact_settle_timer = self.create_timer(
                    self.tactile_contact_settle_s,
                    self.evaluate_tactile_grasp,
                )
                return
            self.continue_after_tactile_grasp()

        def on_target_object_contacts(self, message: Contacts) -> None:
            now_ns = self.get_clock().now().nanoseconds
            for side, token in self.tactile_finger_tokens.items():
                names = matching_contact_collision_names(message, token)
                self.latest_tactile_target_collisions[side] = names
                if names:
                    self.last_target_contact_ns[side] = now_ns

        def tactile_contact_status(self) -> Tuple[bool, str]:
            now_ns = self.get_clock().now().nanoseconds
            success = bilateral_contact_is_recent(
                self.last_target_contact_ns,
                now_ns,
                self.tactile_contact_max_age_s,
            )
            details = []
            for side in ("left", "right"):
                timestamp = self.last_target_contact_ns.get(side)
                age_text = "never"
                if timestamp is not None:
                    age_text = f"{max(0, now_ns - timestamp) / 1e9:.3f}s"
                names = self.latest_tactile_target_collisions.get(side, [])
                details.append(
                    f"{side}=target_contact:{bool(names)},age:{age_text}"
                )
            return success, "; ".join(details)

        def evaluate_tactile_grasp(self) -> None:
            if self.tactile_contact_settle_timer is not None:
                self.tactile_contact_settle_timer.cancel()
                self.tactile_contact_settle_timer = None
            success, reason = self.tactile_contact_status()
            self.record_step("tactile_grasp_check", success, reason)
            if success:
                self.tactile_grasp_verified = True
                self.continue_after_tactile_grasp()
                return
            if self.grasp_probe_retry_count >= self.grasp_probe_max_retries:
                self.finish_failed(f"tactile_grasp_check failed after retries: {reason}")
                return
            self.grasp_probe_retry_count += 1
            self.run_gripper_step(
                "gripper_recovery_open",
                self.gripper_open_position,
                None,
                self.after_grasp_recovery_open,
            )

        def continue_after_tactile_grasp(self) -> None:
            self.publish_demo_state("object_attached", "grasp_pose", "visualized")
            if self.enable_physical_grasp_verification:
                self.start_grasp_probe()
                return
            self.plan_cartesian_step("cartesian_lift", "lift_pose", self.after_lift)

        def on_physical_target_pose(self, message: Pose) -> None:
            pose = (
                float(message.position.x),
                float(message.position.y),
                float(message.position.z),
            )
            self.latest_physical_target_pose = pose
            if self.physical_target_reference_pose is None:
                self.physical_target_reference_pose = pose

        def start_grasp_probe(self) -> None:
            if self.candidate is None or self.latest_physical_target_pose is None:
                self.finish_failed("physical_grasp_check: target pose unavailable")
                return
            grasp_pose = self.candidate.get("grasp_pose")
            if not isinstance(grasp_pose, dict):
                self.finish_failed("physical_grasp_check: grasp_pose unavailable")
                return
            self.grasp_probe_initial_z = self.latest_physical_target_pose[2]
            probe_pose = {
                "frame_id": grasp_pose["frame_id"],
                "position": dict(grasp_pose["position"]),
                "orientation": dict(grasp_pose["orientation"]),
            }
            probe_pose["position"]["z"] = (
                float(probe_pose["position"]["z"]) + self.grasp_probe_lift_m
            )
            self.candidate["grasp_probe_pose"] = probe_pose
            self.plan_cartesian_step(
                "cartesian_grasp_probe",
                "grasp_probe_pose",
                self.after_grasp_probe_motion,
            )

        def after_grasp_probe_motion(self) -> None:
            if self.grasp_probe_timer is not None:
                self.grasp_probe_timer.cancel()
            self.grasp_probe_timer = self.create_timer(
                self.grasp_probe_settle_s,
                self.evaluate_grasp_probe_once,
            )

        def evaluate_grasp_probe_once(self) -> None:
            if self.grasp_probe_timer is not None:
                self.grasp_probe_timer.cancel()
                self.grasp_probe_timer = None
            initial_z = self.grasp_probe_initial_z
            current_z = (
                self.latest_physical_target_pose[2]
                if self.latest_physical_target_pose is not None
                else None
            )
            delta = None if initial_z is None or current_z is None else current_z - initial_z
            if physical_grasp_verified(
                initial_z,
                current_z,
                self.grasp_probe_min_object_lift_m,
            ):
                self.record_step(
                    "physical_grasp_check",
                    True,
                    f"object_lift_delta={delta:.4f}m",
                )
                if not self.calibrate_physical_object_in_hand_pose():
                    return
                self.plan_cartesian_step("cartesian_lift", "lift_pose", self.after_lift)
                return

            reason = "target pose unavailable" if delta is None else f"object_lift_delta={delta:.4f}m"
            self.record_step("physical_grasp_check", False, reason)
            if self.grasp_probe_retry_count >= self.grasp_probe_max_retries:
                self.finish_failed(f"physical_grasp_check failed after retries: {reason}")
                return
            self.grasp_probe_retry_count += 1
            self.run_gripper_step(
                "gripper_recovery_open",
                self.gripper_open_position,
                None,
                self.after_grasp_recovery_open,
            )

        def calibrate_physical_object_in_hand_pose(self) -> bool:
            if (
                self.candidate is None
                or self.latest_physical_target_pose is None
                or self.physical_target_reference_pose is None
            ):
                self.finish_failed("payload_grasp_calibration: target pose unavailable")
                return False
            object_pose = self.candidate.get("object_pose")
            nominal_local = self.candidate.get("object_in_hand_pose")
            if not isinstance(object_pose, dict) or not isinstance(nominal_local, dict):
                self.finish_failed("payload_grasp_calibration: nominal transform unavailable")
                return False
            frame_id = str(object_pose["frame_id"])
            try:
                transform = self.tf_buffer.lookup_transform(
                    frame_id,
                    self.end_effector_link,
                    Time(),
                )
            except TransformException as exc:
                self.finish_failed(f"payload_grasp_calibration: hand TF unavailable: {exc}")
                return False
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            hand_pose = {
                "frame_id": frame_id,
                "position": {
                    "x": float(translation.x),
                    "y": float(translation.y),
                    "z": float(translation.z),
                },
                "orientation": {
                    "x": float(rotation.x),
                    "y": float(rotation.y),
                    "z": float(rotation.z),
                    "w": float(rotation.w),
                },
            }
            measured_local = calibrated_object_in_hand_pose(
                hand_pose,
                object_pose,
                self.latest_physical_target_pose,
                self.physical_target_reference_pose,
            )
            nominal_position = nominal_local["position"]
            measured_position = measured_local["position"]
            self.payload_grasp_calibration_delta_m = translation_distance(
                tuple(
                    float(measured_position[axis]) - float(nominal_position[axis])
                    for axis in ("x", "y", "z")
                )
            )
            self.candidate["object_in_hand_pose"] = measured_local
            self.record_step(
                "payload_grasp_calibration",
                True,
                f"translation_delta={self.payload_grasp_calibration_delta_m:.4f}m",
            )
            return True

        def after_grasp_recovery_open(self) -> None:
            if not self.recenter_recovery_poses():
                return
            self.plan_cartesian_step(
                "cartesian_grasp_recovery_retreat",
                "pre_grasp_pose",
                self.after_grasp_recovery_retreat,
            )

        def after_grasp_recovery_retreat(self) -> None:
            self.plan_cartesian_step(
                "cartesian_grasp_recovery_reapproach",
                "grasp_pose",
                self.after_grasp,
            )

        def current_target_translation(self) -> Optional[Tuple[float, float, float]]:
            if (
                self.latest_physical_target_pose is None
                or self.physical_target_reference_pose is None
            ):
                return None
            return tuple(
                current - reference
                for current, reference in zip(
                    self.latest_physical_target_pose,
                    self.physical_target_reference_pose,
                )
            )

        def recenter_recovery_poses(self) -> bool:
            if (
                self.candidate is None
                or self.latest_physical_target_pose is None
                or self.physical_target_reference_pose is None
            ):
                self.finish_failed("grasp_recenter: physical target pose unavailable")
                return False
            translation = self.current_target_translation()
            if translation is None:
                self.finish_failed("grasp_recenter: target translation unavailable")
                return False
            distance = translation_distance(translation)
            if distance > self.grasp_recovery_max_translation_m:
                self.record_step(
                    "grasp_recenter",
                    False,
                    f"translation_distance={distance:.4f}m exceeds recovery limit "
                    f"{self.grasp_recovery_max_translation_m:.4f}m",
                )
                self.finish_failed(
                    f"grasp_recenter: target moved {distance:.4f}m outside recovery envelope"
                )
                return False
            finger_correction = self.pending_finger_center_correction
            applied_translation = tuple(
                observed + correction
                for observed, correction in zip(translation, finger_correction)
            )
            for pose_key in ("pre_grasp_pose", "grasp_pose", "lift_pose"):
                pose = self.candidate.get(pose_key)
                if not isinstance(pose, dict):
                    self.finish_failed(f"grasp_recenter: {pose_key} unavailable")
                    return False
                self.candidate[pose_key] = translated_pose_dict(
                    pose,
                    applied_translation,
                )
            self.physical_target_reference_pose = self.latest_physical_target_pose
            self.pending_finger_center_correction = (0.0, 0.0, 0.0)
            self.publish_active_candidate()
            self.record_step(
                "grasp_recenter",
                True,
                "observed_translation=["
                + ", ".join(f"{value:.4f}" for value in translation)
                + "]m, finger_correction=["
                + ", ".join(f"{value:.4f}" for value in finger_correction)
                + "]m, applied_translation=["
                + ", ".join(f"{value:.4f}" for value in applied_translation)
                + f"]m, distance={distance:.4f}m",
            )
            return True

        def plan_cartesian_step(self, step_name: str, pose_key: str, next_callback) -> None:
            if self.candidate is None:
                self.finish_failed("candidate missing")
                return
            target_pose = self.candidate.get(pose_key)
            if target_pose is None:
                self.finish_failed(f"{pose_key} missing")
                return

            request = self.make_cartesian_request(
                target_pose,
                self.cartesian_max_step,
                step_name=step_name,
            )

            self.get_logger().info(f"Planning {step_name} -> {pose_key}.")
            future = self.cartesian_client.call_async(request)
            future.add_done_callback(
                lambda done_future: self.on_cartesian_result(done_future, step_name, pose_key, next_callback)
            )

        def make_cartesian_request(
            self,
            target_pose: Dict[str, object],
            max_step: float,
            start_state_override: Optional[RobotState] = None,
            step_name: Optional[str] = None,
        ):
            request = GetCartesianPath.Request()
            request.header.stamp = self.get_clock().now().to_msg()
            request.header.frame_id = str(target_pose["frame_id"])
            measured_start_state = self.measured_robot_state()
            if start_state_override is not None:
                request.start_state = start_state_override
            elif measured_start_state is not None:
                request.start_state = measured_start_state
            elif self.current_start_state is not None:
                request.start_state = self.current_start_state
            else:
                request.start_state.is_diff = True
            request.group_name = self.group_name
            request.link_name = self.end_effector_link
            request.waypoints = [self.to_pose(target_pose)]
            request.max_step = max_step
            request.jump_threshold = self.cartesian_jump_threshold
            request.avoid_collisions = self.cartesian_avoid_collisions
            contact_approach_steps = {
                "cartesian_approach",
                "cartesian_grasp_recovery_reapproach",
            }
            payload_transfer_steps = {
                "cartesian_to_place",
                "cartesian_place_feedback_lift",
                "cartesian_place_feedback_translate",
            }
            place_descent_steps = {
                "cartesian_place_descent",
                "cartesian_place_correction",
            }
            if step_name in contact_approach_steps:
                request.max_velocity_scaling_factor = (
                    self.contact_approach_velocity_scaling_factor
                )
                request.max_acceleration_scaling_factor = (
                    self.contact_approach_acceleration_scaling_factor
                )
            elif step_name in place_descent_steps or (step_name or "").startswith(
                "cartesian_place_descent_stage_"
            ):
                descent_velocity = self.place_descent_velocity_scaling_factor
                stage_number = place_descent_stage_number(step_name)
                if (
                    stage_number is not None
                    and stage_number > 1
                    and self.payload_transfer_execution_replan_count > 0
                ):
                    descent_velocity = (
                        self.place_descent_recovery_velocity_scaling_factor
                    )
                request.max_velocity_scaling_factor = (
                    descent_velocity
                )
                request.max_acceleration_scaling_factor = (
                    descent_velocity
                )
            elif step_name in payload_transfer_steps:
                request.max_velocity_scaling_factor = (
                    self.payload_transfer_velocity_scaling_factor
                )
                request.max_acceleration_scaling_factor = (
                    self.payload_transfer_velocity_scaling_factor
                )
            else:
                request.max_velocity_scaling_factor = (
                    self.cartesian_velocity_scaling_factor
                )
                request.max_acceleration_scaling_factor = (
                    self.cartesian_acceleration_scaling_factor
                )
            return request

        def measured_robot_state(self) -> Optional[RobotState]:
            if (
                not self.execute_trajectories
                or self.visualize_only_execution
                or self.latest_joint_state is None
            ):
                return None
            if not self.latest_joint_state.name or not self.latest_joint_state.position:
                return None
            state = RobotState()
            state.joint_state.header.stamp = self.get_clock().now().to_msg()
            state.joint_state.name = list(self.latest_joint_state.name)
            state.joint_state.position = clamp_near_joint_limits(
                state.joint_state.name,
                list(self.latest_joint_state.position),
            )
            # Only joint values are supplied; retain the scene's attached payload.
            state.is_diff = True
            return state

        def on_cartesian_result(self, future, step_name: str, pose_key: str, next_callback) -> None:
            try:
                response = future.result()
            except Exception as exc:
                self.record_step(step_name, False, str(exc))
                self.finish_failed(f"{step_name}: {exc}")
                return

            fraction = float(response.fraction)
            error_code = int(response.error_code.val)
            required_fraction = self.required_cartesian_fraction(step_name)
            success = fraction >= required_fraction and error_code == 1
            if not success:
                reason = (
                    f"fraction={fraction:.3f}, required={required_fraction:.3f}, "
                    f"moveit_error_code={error_code}"
                )
                if error_code == 1 and not self.cartesian_retry_used.get(step_name, False):
                    self.cartesian_retry_used[step_name] = True
                    self.record_step(f"{step_name}_retry_max_step", False, reason)
                    self.get_logger().warn(
                        f"{step_name} fraction below threshold; retrying with max_step={self.cartesian_retry_max_step}."
                    )
                    if self.candidate is None or self.candidate.get(pose_key) is None:
                        self.record_step(step_name, False, "candidate missing during retry")
                        self.finish_failed(f"{step_name}: candidate missing during retry")
                        return
                    retry_request = self.make_cartesian_request(
                        self.candidate[pose_key],
                        self.cartesian_retry_max_step,
                        step_name=step_name,
                    )
                    retry_future = self.cartesian_client.call_async(retry_request)
                    retry_future.add_done_callback(
                        lambda done_future: self.on_cartesian_result(
                            done_future, step_name, pose_key, next_callback
                        )
                    )
                    return
                if self.try_execute_pre_grasp_alignment_prefix(
                    response,
                    step_name,
                    fraction,
                ):
                    return
                if self.try_execute_contact_prefix(
                    response,
                    step_name,
                    pose_key,
                    next_callback,
                    fraction,
                ):
                    return
                if self.should_try_pre_grasp_alignment_recovery(step_name):
                    self.pre_grasp_alignment_recovery_used = True
                    self.record_step("pre_grasp_alignment_recovery_triggered", False, reason)
                    self.plan_pre_grasp_alignment_recovery()
                    return
                if self.should_try_approach_recovery(step_name):
                    self.approach_recovery_used = True
                    self.record_step("cartesian_approach_recovery_triggered", False, reason)
                    self.plan_approach_recovery()
                    return
                if self.should_try_equivalent_grasp_orientation(step_name):
                    self.record_step(
                        "equivalent_grasp_orientation_triggered",
                        False,
                        reason,
                    )
                    self.plan_equivalent_grasp_orientation()
                    return
                if (
                    step_name == "cartesian_retreat"
                    and self.use_ompl_for_place
                    and self.allow_ompl_release_retreat_fallback
                    and not self.retreat_ompl_fallback_used
                ):
                    self.retreat_ompl_fallback_used = True
                    self.record_step("cartesian_retreat_ompl_fallback", False, reason)
                    self.get_logger().warn(
                        "Cartesian release retreat failed; planning a bounded OMPL clearance."
                    )
                    self.plan_bounded_release_retreat(pose_key, next_callback)
                    return
                self.record_step(step_name, False, reason)
                self.finish_failed(f"{step_name}: {reason}")
                return

            timing_error = self.trajectory_timing_error(response.solution)
            if timing_error is not None:
                reason = (
                    f"invalid trajectory timing: {timing_error}, "
                    f"fraction={fraction:.3f}"
                )
                if self.retry_cartesian_after_timing_error(
                    step_name,
                    pose_key,
                    next_callback,
                    reason,
                ):
                    return
                if self.should_try_pre_grasp_alignment_recovery(step_name):
                    self.pre_grasp_alignment_recovery_used = True
                    self.record_step(
                        "pre_grasp_alignment_recovery_triggered",
                        False,
                        reason,
                    )
                    self.plan_pre_grasp_alignment_recovery()
                    return
                if self.should_try_approach_recovery(step_name):
                    self.approach_recovery_used = True
                    self.record_step(
                        "cartesian_approach_recovery_triggered",
                        False,
                        reason,
                    )
                    self.plan_approach_recovery()
                    return
                if self.should_try_equivalent_grasp_orientation(step_name):
                    self.record_step(
                        "equivalent_grasp_orientation_triggered",
                        False,
                        reason,
                    )
                    self.plan_equivalent_grasp_orientation()
                    return
                self.record_step(step_name, False, reason)
                self.finish_failed(f"{step_name}: {reason}")
                return

            trajectory_metrics = joint_trajectory_quality_metrics(
                list(response.solution.joint_trajectory.joint_names),
                list(response.solution.joint_trajectory.points),
            )
            trajectory_metrics.update(
                {
                    "step": step_name,
                    "fraction": fraction,
                    "timing_retry": self.cartesian_timing_retry_counts.get(
                        step_name,
                        0,
                    ),
                }
            )
            self.cartesian_trajectory_metrics.append(trajectory_metrics)
            self.get_logger().info(
                "Cartesian trajectory quality: "
                + json.dumps(trajectory_metrics, sort_keys=True)
            )
            quality_error = place_descent_trajectory_quality_error(
                step_name,
                trajectory_metrics,
                self.place_descent_max_trajectory_duration_s,
                self.place_descent_max_joint_path_length_rad,
            )
            if quality_error is not None:
                reason = f"unsafe Cartesian trajectory: {quality_error}"
                self.record_step(
                    f"{step_name}_trajectory_quality",
                    False,
                    reason,
                )
                self.finish_failed(f"{step_name}: {reason}")
                return

            start_check = self.execution_start_check(response.solution)
            if not start_check["success"]:
                attempt = self.cartesian_start_state_replan_counts.get(step_name, 0)
                if (
                    attempt < self.cartesian_start_state_replan_max_retries
                    and self.candidate is not None
                    and self.candidate.get(pose_key) is not None
                ):
                    self.cartesian_start_state_replan_counts[step_name] = attempt + 1
                    reason = str(start_check["reason"])
                    self.record_step(
                        f"{step_name}_start_state_replan",
                        False,
                        reason,
                    )
                    self.get_logger().warn(
                        f"{step_name} was planned from a stale measured state; "
                        "discarding the trajectory and replanning from the latest "
                        f"/joint_states ({attempt + 1}/"
                        f"{self.cartesian_start_state_replan_max_retries})."
                    )
                    self.current_start_state = None
                    retry_request = self.make_cartesian_request(
                        self.candidate[pose_key],
                        self.cartesian_max_step,
                        step_name=step_name,
                    )
                    retry_future = self.cartesian_client.call_async(retry_request)
                    retry_future.add_done_callback(
                        lambda done_future: self.on_cartesian_result(
                            done_future,
                            step_name,
                            pose_key,
                            next_callback,
                        )
                    )
                    return

            display_start_state = response.start_state
            self.current_start_state = self.end_state_from_trajectory(response.solution)
            if self.current_start_state is None:
                self.record_step(step_name, False, "trajectory has no joint endpoint")
                self.finish_failed(f"{step_name}: trajectory has no joint endpoint")
                return

            self.handle_successful_trajectory(
                step_name,
                pose_key,
                response.solution,
                f"fraction={fraction:.3f}",
                next_callback,
                display_start_state,
                (
                    (
                        lambda: self.plan_cartesian_step(
                            step_name,
                            pose_key,
                            next_callback,
                        )
                    )
                    if (
                        step_name in {
                            "cartesian_pre_grasp_alignment",
                            "cartesian_approach",
                            "cartesian_grasp_recovery_reapproach",
                            "cartesian_to_pre_place",
                        }
                        or step_name.startswith(
                            "cartesian_pre_place_transfer_stage_"
                        )
                        or step_name.startswith(
                            "cartesian_place_descent_stage_"
                        )
                    )
                    else None
                ),
            )

        def retry_cartesian_after_timing_error(
            self,
            step_name: str,
            pose_key: str,
            next_callback,
            reason: str,
        ) -> bool:
            attempt = self.cartesian_timing_retry_counts.get(step_name, 0)
            if attempt >= len(self.cartesian_timing_retry_max_steps):
                return False
            if self.candidate is None or self.candidate.get(pose_key) is None:
                return False
            max_step = self.cartesian_timing_retry_max_steps[attempt]
            self.cartesian_timing_retry_counts[step_name] = attempt + 1
            self.record_step(
                f"{step_name}_timing_replan",
                False,
                f"{reason}, retry_max_step={max_step:.4f}",
            )
            self.get_logger().warn(
                f"{step_name} returned invalid timing; recomputing the "
                f"collision-checked path with max_step={max_step:.4f} "
                f"({attempt + 1}/{len(self.cartesian_timing_retry_max_steps)})."
            )
            retry_request = self.make_cartesian_request(
                self.candidate[pose_key],
                max_step,
                step_name=step_name,
            )
            retry_future = self.cartesian_client.call_async(retry_request)
            retry_future.add_done_callback(
                lambda done_future: self.on_cartesian_result(
                    done_future,
                    step_name,
                    pose_key,
                    next_callback,
                )
            )
            return True

        def try_execute_pre_grasp_alignment_prefix(
            self,
            response,
            step_name: str,
            fraction: float,
        ) -> bool:
            if (
                step_name != "cartesian_pre_grasp_alignment"
                or not self.pre_grasp_alignment_recovery_used
                or int(response.error_code.val) != 1
                or fraction < self.pre_grasp_alignment_prefix_min_fraction
                or self.pre_grasp_alignment_prefix_attempt_count
                >= self.pre_grasp_alignment_prefix_max_attempts
            ):
                return False
            timing_error = self.trajectory_timing_error(response.solution)
            if timing_error is not None:
                return False
            self.current_start_state = self.end_state_from_trajectory(response.solution)
            if self.current_start_state is None:
                return False
            self.pre_grasp_alignment_prefix_attempt_count += 1
            reason = (
                f"fraction={fraction:.3f}, prefix_attempt="
                f"{self.pre_grasp_alignment_prefix_attempt_count}/"
                f"{self.pre_grasp_alignment_prefix_max_attempts}"
            )
            self.get_logger().warn(
                "Executing a collision-checked pre-grasp alignment prefix, "
                "then replanning the residual motion from measured joints."
            )
            self.handle_successful_trajectory(
                "cartesian_pre_grasp_alignment_prefix",
                None,
                response.solution,
                reason,
                self.after_pre_grasp_alignment_prefix,
                response.start_state,
            )
            return True

        def try_execute_contact_prefix(
            self,
            response,
            step_name: str,
            pose_key: str,
            next_callback,
            fraction: float,
        ) -> bool:
            supported_steps = {
                "cartesian_approach",
                "cartesian_grasp_probe",
            }
            attempt = self.contact_prefix_attempt_counts.get(step_name, 0)
            if (
                step_name not in supported_steps
                or int(response.error_code.val) != 1
                or fraction < self.contact_prefix_min_fraction
                or attempt >= self.contact_prefix_max_attempts
            ):
                return False
            timing_error = self.trajectory_timing_error(response.solution)
            if timing_error is not None:
                return False
            self.current_start_state = self.end_state_from_trajectory(response.solution)
            if self.current_start_state is None:
                return False
            attempt += 1
            self.contact_prefix_attempt_counts[step_name] = attempt
            reason = (
                f"fraction={fraction:.3f}, prefix_attempt={attempt}/"
                f"{self.contact_prefix_max_attempts}"
            )
            self.get_logger().warn(
                f"Executing the collision-checked {step_name} prefix, then "
                "replanning the residual motion from measured joints."
            )
            self.handle_successful_trajectory(
                f"{step_name}_prefix",
                None,
                response.solution,
                reason,
                lambda: self.after_contact_prefix(
                    step_name,
                    pose_key,
                    next_callback,
                ),
                response.start_state,
            )
            return True

        def after_contact_prefix(
            self,
            step_name: str,
            pose_key: str,
            next_callback,
        ) -> None:
            self.cartesian_retry_used.pop(step_name, None)
            self.cartesian_timing_retry_counts.pop(step_name, None)
            self.plan_cartesian_step(step_name, pose_key, next_callback)

        def after_pre_grasp_alignment_prefix(self) -> None:
            self.cartesian_retry_used.pop("cartesian_pre_grasp_alignment", None)
            self.cartesian_timing_retry_counts.pop(
                "cartesian_pre_grasp_alignment",
                None,
            )
            self.plan_cartesian_step(
                "cartesian_pre_grasp_alignment",
                "pre_grasp_pose",
                self.after_pre_grasp_alignment,
            )

        def required_cartesian_fraction(self, step_name: str) -> float:
            if (
                step_name == "cartesian_pre_grasp_alignment"
                and self.pre_grasp_alignment_recovery_used
            ):
                # This is a non-contact staging move. Keep it stricter than the
                # general Cartesian threshold, while contact moves remain at 0.98.
                return max(
                    self.cartesian_min_fraction,
                    self.pre_grasp_alignment_recovery_min_fraction,
                    0.95,
                )
            critical_cartesian_steps = {
                "cartesian_pre_grasp_alignment",
                "cartesian_approach",
                "cartesian_grasp_probe",
                "cartesian_lift",
                "cartesian_to_pre_place",
                "cartesian_to_place",
                "cartesian_place_descent",
                "cartesian_place_correction",
                "cartesian_retreat",
                "cartesian_grasp_recovery_retreat",
                "cartesian_grasp_recovery_reapproach",
            }
            if (
                step_name in critical_cartesian_steps
                or step_name.startswith("cartesian_place_descent_stage_")
                or step_name.startswith("cartesian_pre_place_transfer_stage_")
            ):
                return max(self.cartesian_min_fraction, 0.98)
            return self.cartesian_min_fraction

        def trajectory_timing_error(self, trajectory) -> Optional[str]:
            previous_time_s = None
            for index, point in enumerate(trajectory.joint_trajectory.points):
                current_time_s = (
                    float(point.time_from_start.sec)
                    + float(point.time_from_start.nanosec) * 1e-9
                )
                if (
                    previous_time_s is not None
                    and current_time_s <= previous_time_s
                ):
                    return (
                        f"points {index - 1} and {index} are not strictly "
                        f"increasing ({previous_time_s:.9f}s, "
                        f"{current_time_s:.9f}s)"
                    )
                previous_time_s = current_time_s
            return None

        def after_lift(self) -> None:
            if self.candidate is None:
                self.finish_failed("place planning: candidate missing")
                return
            if not self.payload_place_compensation_applied:
                place_pose = self.candidate.get("place_pose")
                object_in_hand_pose = self.candidate.get("object_in_hand_pose")
                object_pose = self.candidate.get("object_pose")
                if not isinstance(place_pose, dict) or not isinstance(
                    object_in_hand_pose,
                    dict,
                ) or not isinstance(object_pose, dict):
                    self.finish_failed("place planning: payload transform unavailable")
                    return
                desired_object_pose = deepcopy(place_pose)
                desired_object_pose["position"]["z"] = (
                    float(object_pose["position"]["z"])
                    + self.place_object_clearance_m
                )
                self.candidate["desired_place_object_pose"] = deepcopy(
                    desired_object_pose
                )
                compensated_place = payload_compensated_place_pose(
                    desired_object_pose,
                    object_in_hand_pose,
                )
                original_position = place_pose["position"]
                compensated_position = compensated_place["position"]
                delta_x = float(compensated_position["x"]) - float(
                    original_position["x"]
                )
                delta_y = float(compensated_position["y"]) - float(
                    original_position["y"]
                )
                delta_z = float(compensated_position["z"]) - float(
                    original_position["z"]
                )
                self.payload_place_compensation_m = sqrt(
                    delta_x * delta_x + delta_y * delta_y + delta_z * delta_z
                )
                self.candidate["place_pose"] = compensated_place
                self.payload_place_compensation_applied = True
                self.record_step(
                    "payload_place_compensation",
                    True,
                    (
                        f"dx={delta_x:.4f}m, dy={delta_y:.4f}m, "
                        f"dz={delta_z:.4f}m, object_clearance="
                        f"{self.place_object_clearance_m:.4f}m, "
                        f"magnitude={self.payload_place_compensation_m:.4f}m"
                    ),
                )
            if self.use_ompl_for_place:
                place_pose = self.candidate.get("place_pose")
                if not isinstance(place_pose, dict):
                    self.finish_failed("pre-place planning: place_pose missing")
                    return
                self.candidate["pre_place_pose"] = translated_pose_dict(
                    place_pose,
                    (0.0, 0.0, max(0.05, self.pre_place_height_offset)),
                )
                if self.use_cartesian_pre_place_transfer:
                    self.begin_cartesian_pre_place_transfer()
                    return
                self.plan_pre_place_candidate()
                return
            self.plan_cartesian_step(
                "cartesian_to_place",
                "place_pose",
                self.after_place_descent,
            )

        def begin_cartesian_pre_place_transfer(self) -> None:
            if self.candidate is None:
                self.finish_failed("pre-place transfer: candidate missing")
                return
            lift_pose = self.candidate.get("lift_pose")
            pre_place_pose = self.candidate.get("pre_place_pose")
            if not isinstance(lift_pose, dict) or not isinstance(
                pre_place_pose,
                dict,
            ):
                self.finish_failed("pre-place transfer: staged poses unavailable")
                return
            stages = interpolated_place_descent_poses(
                lift_pose,
                pre_place_pose,
                self.pre_place_transfer_stage_count,
            )
            self.pre_place_transfer_stage_index = 0
            self.pre_place_transfer_stage_pose_keys = []
            for index, pose in enumerate(stages, start=1):
                key = f"pre_place_transfer_stage_{index}_pose"
                self.candidate[key] = pose
                self.pre_place_transfer_stage_pose_keys.append(key)
            self.plan_next_pre_place_transfer_stage()

        def plan_next_pre_place_transfer_stage(self) -> None:
            if self.pre_place_transfer_stage_index >= len(
                self.pre_place_transfer_stage_pose_keys
            ):
                self.after_pre_place()
                return
            key = self.pre_place_transfer_stage_pose_keys[
                self.pre_place_transfer_stage_index
            ]
            stage_number = self.pre_place_transfer_stage_index + 1
            self.plan_cartesian_step(
                (
                    f"cartesian_pre_place_transfer_stage_{stage_number}_of_"
                    f"{len(self.pre_place_transfer_stage_pose_keys)}"
                ),
                key,
                self.after_pre_place_transfer_stage,
            )

        def after_pre_place_transfer_stage(self) -> None:
            key = self.pre_place_transfer_stage_pose_keys[
                self.pre_place_transfer_stage_index
            ]
            if not self.verify_payload_after_transfer(
                key,
                tolerance_m=self.place_descent_stage_payload_tolerance_m,
                step_name="pre_place_transfer_payload_check",
            ):
                return
            tactile_success, tactile_reason = self.tactile_contact_status()
            self.record_step(
                "pre_place_transfer_tactile_check",
                tactile_success,
                (
                    f"stage={self.pre_place_transfer_stage_index + 1}/"
                    f"{len(self.pre_place_transfer_stage_pose_keys)}; "
                    f"{tactile_reason}"
                ),
            )
            if self.enable_tactile_grasp_supervision and not tactile_success:
                self.finish_failed(
                    "pre_place_transfer_tactile_check failed: "
                    f"{tactile_reason}"
                )
                return
            self.payload_transfer_execution_replan_count = 0
            self.pre_place_transfer_stage_index += 1
            self.plan_next_pre_place_transfer_stage()

        def plan_pre_place_candidate(self) -> None:
            if self.candidate is None:
                self.finish_failed("pre-place candidate planning: candidate missing")
                return
            pre_place_pose = self.candidate.get("pre_place_pose")
            if not isinstance(pre_place_pose, dict):
                self.finish_failed("pre-place candidate planning: pre_place_pose missing")
                return
            self.pre_place_candidate_attempt_count += 1
            height_step = float(self.get_parameter("pre_place_candidate_height_step_m").value)
            if height_step and not self.pre_place_joint_replay_positions:
                if self.pre_place_candidate_attempt_count == 1:
                    self.nominal_pre_place_candidate = deepcopy(pre_place_pose)
                pre_place_pose = pre_place_height_candidate(
                    self.nominal_pre_place_candidate, self.candidate["place_pose"],
                    self.pre_place_candidate_attempt_count, height_step,
                )
                self.candidate["pre_place_pose"] = pre_place_pose
                self.record_step("pre_place_height_candidate", True,
                                 f"attempt={self.pre_place_candidate_attempt_count}, "
                                 f"z={pre_place_pose['position']['z']:.4f}m")
            if (abs(float(self.candidate.get("grasp_pitch_offset_deg", 0.0))) > 0
                    and not self.pre_place_joint_replay_positions):
                self.probe_measured_pre_place_branch(pre_place_pose)
                return
            self.submit_pre_place_candidate(pre_place_pose)

        def probe_measured_pre_place_branch(self, pre_place_pose) -> None:
            start = self.measured_robot_state()
            if start is None:
                self.retry_or_fail_pre_place_candidate("measured state unavailable for reverse placement")
                return
            candidate = deepcopy(self.candidate)
            # place_pose is already compensated using the post-grasp payload transform.
            candidate["grasp_pose"] = deepcopy(candidate["place_pose"])
            candidate["pre_grasp_pose"] = deepcopy(pre_place_pose)

            def completed(checks):
                self.get_logger().info(json.dumps({
                    "diagnostic": "measured_reverse_place_branch",
                    "attempt": self.pre_place_candidate_attempt_count,
                    "checks": checks,
                }))
                branches = rank_reverse_joint_branches(start, checks)
                if not branches:
                    self.retry_or_fail_pre_place_candidate("no valid measured-payload reverse descent branch")
                    return
                selected_index = (self.pre_place_candidate_attempt_count - 1) % len(branches)
                branch = branches[selected_index]
                self.record_step(
                    "measured_reverse_place_branch_selected", True,
                    f"candidate={selected_index + 1}/{len(branches)}, "
                    f"max_joint_delta={branch['loaded_state_max_joint_delta_rad']:.4f}rad")
                positions = ordered_joint_positions(
                    branch["pre_grasp_joint_names"], branch["pre_grasp_joint_positions"])
                if positions is None:
                    self.retry_or_fail_pre_place_candidate("reverse placement joint endpoint incomplete")
                    return
                self.submit_pre_place_candidate(pre_place_pose, positions)

            ReverseApproachProbe(self, candidate, start, normalized_joint_limit_margin,
                                 completed, collect_all=True).start()

        def submit_pre_place_candidate(self, pre_place_pose, reverse_joint_target=None) -> None:
            planner_id = self.planner_ids[0]
            start_state = self.measured_robot_state() or self.current_start_state
            goal = MoveGroup.Goal()
            if reverse_joint_target is not None:
                goal.request = self.make_joint_motion_plan_request(
                    reverse_joint_target, start_state, planner_id, 0.002)
            elif self.pre_place_joint_replay_positions:
                goal.request = self.make_joint_motion_plan_request(
                    self.pre_place_joint_replay_positions,
                    start_state,
                    planner_id,
                    self.pre_place_joint_replay_tolerance_rad,
                )
            else:
                goal.request = self.make_motion_plan_request(
                    self.to_pose_stamped(pre_place_pose),
                    start_state,
                    constrain_orientation=True,
                    planner_id=planner_id,
                )
            goal.request.max_velocity_scaling_factor = (
                self.payload_transfer_velocity_scaling_factor
            )
            goal.request.max_acceleration_scaling_factor = (
                self.payload_transfer_velocity_scaling_factor
            )
            # The candidate must be inspected together with its Cartesian descent
            # before any arm motion is executed.
            goal.planning_options = self.make_planning_options()
            goal.planning_options.plan_only = True
            self.get_logger().info(
                "Planning pre-place candidate "
                f"{self.pre_place_candidate_attempt_count}/"
                f"{self.pre_place_candidate_attempts} with {planner_id}"
                + (
                    " using the paired joint-state replay target."
                    if self.pre_place_joint_replay_positions
                    else "."
                )
            )
            future = self.move_client.send_goal_async(goal)
            future.add_done_callback(self.on_pre_place_candidate_goal)

        def on_pre_place_candidate_goal(self, future) -> None:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.retry_or_fail_pre_place_candidate("goal rejected")
                return
            goal_handle.get_result_async().add_done_callback(
                self.on_pre_place_candidate_result
            )

        def on_pre_place_candidate_result(self, future) -> None:
            result_wrapper = future.result()
            error_code = int(result_wrapper.result.error_code.val)
            success = result_wrapper.status == GoalStatus.STATUS_SUCCEEDED and error_code == 1
            if not success:
                self.retry_or_fail_pre_place_candidate(
                    f"planner_id={self.planner_ids[0]}, "
                    f"action_status={result_wrapper.status}, moveit_error_code={error_code}"
                )
                return
            trajectory = result_wrapper.result.planned_trajectory
            endpoint_positions = ordered_joint_positions(
                list(trajectory.joint_trajectory.joint_names),
                list(trajectory.joint_trajectory.points[-1].positions)
                if trajectory.joint_trajectory.points
                else [],
            )
            if endpoint_positions is None:
                self.retry_or_fail_pre_place_candidate(
                    "planned trajectory has no complete Panda arm endpoint"
                )
                return
            replay_delta = None
            if self.pre_place_joint_replay_positions:
                replay_delta = max_joint_position_delta(
                    endpoint_positions,
                    self.pre_place_joint_replay_positions,
                )
                self.pre_place_joint_replay_max_plan_delta_rad = replay_delta
                if (
                    replay_delta is None
                    or replay_delta > self.pre_place_joint_replay_tolerance_rad
                ):
                    self.retry_or_fail_pre_place_candidate(
                        "joint replay endpoint mismatch: "
                        f"max_delta={replay_delta}, tolerance="
                        f"{self.pre_place_joint_replay_tolerance_rad:.4f}rad"
                    )
                    return
            position_rows = [
                [float(value) for value in point.positions]
                for point in trajectory.joint_trajectory.points
            ]
            path_length = joint_path_length(position_rows)
            if path_length > self.place_transfer_max_joint_path_length:
                self.retry_or_fail_pre_place_candidate(
                    f"joint_path_length={path_length:.3f}rad exceeds transfer limit "
                    f"{self.place_transfer_max_joint_path_length:.3f}rad"
                )
                return
            end_state = self.end_state_from_trajectory(trajectory)
            if end_state is None:
                self.retry_or_fail_pre_place_candidate(
                    "planned trajectory has no joint endpoint"
                )
                return
            if self.candidate is None or not isinstance(
                self.candidate.get("place_pose"),
                dict,
            ):
                self.finish_failed("pre-place descent validation: place_pose missing")
                return
            request = self.make_cartesian_request(
                self.candidate["place_pose"],
                self.cartesian_retry_max_step,
                start_state_override=end_state,
            )
            descent_future = self.cartesian_client.call_async(request)
            descent_future.add_done_callback(
                lambda done_future: self.on_pre_place_descent_validation(
                    done_future,
                    trajectory,
                    getattr(result_wrapper.result, "trajectory_start", None),
                    float(getattr(result_wrapper.result, "planning_time", 0.0)),
                    endpoint_positions,
                )
            )

        def diagnose_descent_endpoint(self, trajectory, seed_variant="pre_place", yaw_degrees=0.0) -> None:
            # Read-only IK is never submitted to an execution action.
            report = {"diagnostic": "rejected_descent_endpoint",
                      "attempt": self.pre_place_candidate_attempt_count,
                      "seed_variant": seed_variant,
                      "yaw_degrees": yaw_degrees,
                      "executed": False}

            def emit(reason):
                report["reason"] = reason
                self.get_logger().info(json.dumps(report))

            if not (self.diagnostic_ik_client.service_is_ready()
                    and self.diagnostic_validity_client.service_is_ready()):
                emit("diagnostic services unavailable")
                return
            state = self.end_state_from_trajectory(trajectory)
            if seed_variant != "pre_place":
                state = self.fixed_home_state()
                if seed_variant in ("elbow_positive", "elbow_negative"):
                    sign = 1.0 if seed_variant == "elbow_positive" else -1.0
                    state.joint_state.position[0] = 0.5 * sign
                    state.joint_state.position[2] = 1.5 * sign
                    state.joint_state.position[4] = -1.0 * sign
            if state is None or self.candidate is None:
                emit("diagnostic seed unavailable")
                return
            request = GetPositionIK.Request()
            request.ik_request.group_name = self.group_name
            request.ik_request.ik_link_name = self.end_effector_link
            request.ik_request.robot_state = state
            target = deepcopy(self.candidate["place_pose"])
            if yaw_degrees:
                target = cube_symmetric_hand_goal(target, self.candidate["object_in_hand_pose"], yaw_degrees)
            report["target_hand_pose"] = target
            request.ik_request.pose_stamped = self.to_pose_stamped(target)
            request.ik_request.timeout.sec = 1
            request.ik_request.avoid_collisions = False

            def on_validity(done):
                try:
                    response = done.result()
                    report["endpoint_valid"] = response.valid
                    report["contacts"] = [[c.contact_body_1, c.contact_body_2]
                                          for c in response.contacts]
                    emit("endpoint valid; intermediate path failure unresolved" if response.valid
                         else "endpoint IK solution is invalid; inspect contact pairs")
                except Exception as exc:
                    emit(f"validity diagnostic error: {exc}")

            def on_ik(done):
                try:
                    response = done.result()
                    report["ik_error_code"] = int(response.error_code.val)
                    if response.error_code.val != 1:
                        emit("no IK solution from this seed within diagnostic timeout")
                        return
                    check = GetStateValidity.Request()
                    report["ik_joint_names"] = list(response.solution.joint_state.name)
                    report["ik_joint_positions"] = list(response.solution.joint_state.position)
                    check.robot_state = response.solution
                    check.robot_state.is_diff = True
                    check.group_name = self.group_name
                    self.diagnostic_validity_client.call_async(check).add_done_callback(on_validity)
                except Exception as exc:
                    emit(f"IK diagnostic error: {exc}")

            self.diagnostic_ik_client.call_async(request).add_done_callback(on_ik)

        def on_pre_place_descent_validation(
            self,
            future,
            trajectory,
            display_start_state: Optional[RobotState],
            planning_time_s: float,
            endpoint_positions: List[float],
        ) -> None:
            try:
                response = future.result()
            except Exception as exc:
                self.retry_or_fail_pre_place_candidate(str(exc))
                return
            fraction = float(response.fraction)
            error_code = int(response.error_code.val)
            required_fraction = self.required_cartesian_fraction(
                "cartesian_place_descent"
            )
            reason = (
                f"fraction={fraction:.3f}, required={required_fraction:.3f}, "
                f"moveit_error_code={error_code}"
            )
            if fraction < required_fraction or error_code != 1:
                if bool(self.get_parameter("diagnose_rejected_descent").value):
                    for seed in ("pre_place", "home", "elbow_positive", "elbow_negative"):
                        self.diagnose_descent_endpoint(trajectory, seed)
                    if bool(self.get_parameter("cube_symmetric_placement").value):
                        for yaw in (90.0, 180.0, 270.0):
                            self.diagnose_descent_endpoint(trajectory, "home", yaw)
                self.retry_or_fail_pre_place_candidate(reason)
                return

            position_rows = [
                [float(value) for value in point.positions]
                for point in trajectory.joint_trajectory.points
            ]
            self.ompl_metrics.append(
                {
                    "step": "ompl_to_pre_place",
                    "planner_id": self.planner_ids[0],
                    "planning_time_s": planning_time_s,
                    "joint_path_length_rad": joint_path_length(position_rows),
                    "trajectory_points": len(position_rows),
                }
            )
            self.record_step("pre_place_descent_validation", True, reason)
            self.pre_place_joint_target_positions = list(endpoint_positions)
            if self.pre_place_joint_replay_positions:
                self.record_step(
                    "pre_place_joint_replay",
                    True,
                    "collision-checked joint target accepted, max_plan_delta="
                    f"{self.pre_place_joint_replay_max_plan_delta_rad:.6f}rad",
                )
            self.current_start_state = self.end_state_from_trajectory(trajectory)
            self.handle_successful_trajectory(
                "ompl_to_pre_place",
                "pre_place_pose",
                trajectory,
                "descent_prevalidated",
                self.after_pre_place,
                display_start_state,
                self.plan_pre_place_candidate,
            )

        def retry_or_fail_pre_place_candidate(self, reason: str) -> None:
            self.record_step(
                "pre_place_candidate_rejected",
                False,
                (
                    f"{reason}, candidate={self.pre_place_candidate_attempt_count}/"
                    f"{self.pre_place_candidate_attempts}"
                ),
            )
            if self.pre_place_candidate_attempt_count < self.pre_place_candidate_attempts:
                self.get_logger().warn(
                    "Pre-place candidate cannot complete the vertical descent; replanning."
                )
                self.plan_pre_place_candidate()
                return
            self.record_step(
                "direct_place_fallback_triggered",
                False,
                f"pre-place candidates exhausted: {reason}",
            )
            self.plan_direct_place_candidate()

        def plan_direct_place_candidate(self) -> None:
            if self.candidate is None:
                self.finish_failed("direct-place candidate planning: candidate missing")
                return
            place_pose = self.candidate.get("place_pose")
            if not isinstance(place_pose, dict):
                self.finish_failed("direct-place candidate planning: place_pose missing")
                return
            self.direct_place_candidate_attempt_count += 1
            planner_id = self.planner_ids[0]
            goal = MoveGroup.Goal()
            goal.request = self.make_motion_plan_request(
                self.to_pose_stamped(place_pose),
                self.measured_robot_state() or self.current_start_state,
                constrain_orientation=True,
                planner_id=planner_id,
            )
            goal.request.max_velocity_scaling_factor = (
                self.payload_transfer_velocity_scaling_factor
            )
            goal.request.max_acceleration_scaling_factor = (
                self.payload_transfer_velocity_scaling_factor
            )
            goal.planning_options = self.make_planning_options()
            self.get_logger().warn(
                "Planning direct-place candidate with retreat prevalidation "
                f"({self.direct_place_candidate_attempt_count}/"
                f"{self.direct_place_candidate_attempts})."
            )
            future = self.move_client.send_goal_async(goal)
            future.add_done_callback(self.on_direct_place_candidate_goal)

        def on_direct_place_candidate_goal(self, future) -> None:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.retry_or_fail_direct_place_candidate("goal rejected")
                return
            goal_handle.get_result_async().add_done_callback(
                self.on_direct_place_candidate_result
            )

        def on_direct_place_candidate_result(self, future) -> None:
            result_wrapper = future.result()
            error_code = int(result_wrapper.result.error_code.val)
            success = result_wrapper.status == GoalStatus.STATUS_SUCCEEDED and error_code == 1
            if not success:
                self.retry_or_fail_direct_place_candidate(
                    f"planner_id={self.planner_ids[0]}, "
                    f"action_status={result_wrapper.status}, moveit_error_code={error_code}"
                )
                return
            trajectory = result_wrapper.result.planned_trajectory
            position_rows = [
                [float(value) for value in point.positions]
                for point in trajectory.joint_trajectory.points
            ]
            path_length = joint_path_length(position_rows)
            if path_length > self.place_transfer_max_joint_path_length:
                self.retry_or_fail_direct_place_candidate(
                    f"joint_path_length={path_length:.3f}rad exceeds transfer limit "
                    f"{self.place_transfer_max_joint_path_length:.3f}rad"
                )
                return
            end_state = self.end_state_from_trajectory(trajectory)
            if end_state is None:
                self.retry_or_fail_direct_place_candidate(
                    "planned trajectory has no joint endpoint"
                )
                return
            if self.candidate is None or not isinstance(
                self.candidate.get("pre_place_pose"),
                dict,
            ):
                self.finish_failed("direct-place retreat validation: pre_place_pose missing")
                return
            request = self.make_cartesian_request(
                self.candidate["pre_place_pose"],
                self.cartesian_retry_max_step,
                start_state_override=end_state,
            )
            retreat_future = self.cartesian_client.call_async(request)
            retreat_future.add_done_callback(
                lambda done_future: self.on_direct_place_retreat_validation(
                    done_future,
                    trajectory,
                    getattr(result_wrapper.result, "trajectory_start", None),
                    float(getattr(result_wrapper.result, "planning_time", 0.0)),
                    path_length,
                )
            )

        def on_direct_place_retreat_validation(
            self,
            future,
            trajectory,
            display_start_state: Optional[RobotState],
            planning_time_s: float,
            path_length: float,
        ) -> None:
            try:
                response = future.result()
            except Exception as exc:
                self.retry_or_fail_direct_place_candidate(str(exc))
                return
            fraction = float(response.fraction)
            error_code = int(response.error_code.val)
            required_fraction = self.required_cartesian_fraction("cartesian_retreat")
            reason = (
                f"fraction={fraction:.3f}, required={required_fraction:.3f}, "
                f"moveit_error_code={error_code}"
            )
            if fraction < required_fraction or error_code != 1:
                self.retry_or_fail_direct_place_candidate(reason)
                return
            self.ompl_metrics.append(
                {
                    "step": "ompl_to_place_fallback",
                    "planner_id": self.planner_ids[0],
                    "planning_time_s": planning_time_s,
                    "joint_path_length_rad": path_length,
                    "trajectory_points": len(trajectory.joint_trajectory.points),
                }
            )
            self.record_step("direct_place_retreat_validation", True, reason)
            self.current_start_state = self.end_state_from_trajectory(trajectory)
            self.handle_successful_trajectory(
                "ompl_to_place_fallback",
                "place_pose",
                trajectory,
                "retreat_prevalidated",
                self.after_direct_place_transfer,
                display_start_state,
            )

        def retry_or_fail_direct_place_candidate(self, reason: str) -> None:
            self.record_step(
                "direct_place_candidate_rejected",
                False,
                (
                    f"{reason}, candidate={self.direct_place_candidate_attempt_count}/"
                    f"{self.direct_place_candidate_attempts}"
                ),
            )
            if self.direct_place_candidate_attempt_count < self.direct_place_candidate_attempts:
                self.plan_direct_place_candidate()
                return
            self.finish_failed(f"direct-place candidate validation failed: {reason}")

        def after_pre_place(self) -> None:
            self.place_descent_start_wait_count = 0
            self.wait_for_place_descent_start()

        def wait_for_place_descent_start(self) -> None:
            if self.place_descent_start_wait_timer is not None:
                self.place_descent_start_wait_timer.cancel()
                self.place_descent_start_wait_timer = None
            if self.candidate is None:
                self.finish_failed("place descent readiness: candidate missing")
                return
            pre_place_pose = self.candidate.get("pre_place_pose")
            if not isinstance(pre_place_pose, dict):
                self.finish_failed("place descent readiness: pre_place_pose missing")
                return
            if self.pre_place_joint_replay_positions:
                self.wait_for_replayed_place_descent_start()
                return
            current_hand = self.current_end_effector_position(
                str(pre_place_pose["frame_id"])
            )
            target_position = tuple(
                float(pre_place_pose["position"][axis])
                for axis in ("x", "y", "z")
            )
            error = (
                translation_distance(
                    tuple(
                        current - target
                        for current, target in zip(current_hand, target_position)
                    )
                )
                if current_hand is not None
                else None
            )
            delta = (
                tuple(
                    current - target
                    for current, target in zip(current_hand, target_position)
                )
                if current_hand is not None
                else None
            )
            if error is not None and error <= self.place_descent_start_tolerance_m:
                self.record_step(
                    "place_descent_start_readiness",
                    True,
                    (
                        f"position_error={error:.4f}m, tolerance="
                        f"{self.place_descent_start_tolerance_m:.4f}m, polls="
                        f"{self.place_descent_start_wait_count}, alignments="
                        f"{self.place_descent_start_alignment_retry_count}"
                    ),
                )
                self.begin_staged_place_descent()
                return
            maximum_polls = max(
                1,
                int(
                    self.place_descent_start_timeout_s
                    / self.place_descent_start_poll_s
                ),
            )
            if self.place_descent_start_wait_count >= maximum_polls:
                reason = (
                    "hand transform unavailable"
                    if error is None
                    else (
                        f"position_error={error:.4f}m exceeds tolerance="
                        f"{self.place_descent_start_tolerance_m:.4f}m, "
                        f"dx={delta[0]:.4f}m, dy={delta[1]:.4f}m, "
                        f"dz={delta[2]:.4f}m"
                    )
                )
                if (
                    error is not None
                    and self.place_descent_start_alignment_retry_count
                    < self.place_descent_start_max_alignment_retries
                ):
                    self.place_descent_start_alignment_retry_count += 1
                    self.record_step(
                        "place_descent_start_alignment_recovery",
                        True,
                        (
                            f"{reason}, attempt="
                            f"{self.place_descent_start_alignment_retry_count}/"
                            f"{self.place_descent_start_max_alignment_retries}"
                        ),
                    )
                    self.place_descent_start_wait_count = 0
                    self.plan_cartesian_step(
                        "cartesian_pre_place_alignment",
                        "pre_place_pose",
                        self.after_pre_place_alignment_recovery,
                    )
                    return
                self.record_step("place_descent_start_readiness", False, reason)
                self.finish_failed(f"place_descent_start_readiness failed: {reason}")
                return
            self.place_descent_start_wait_count += 1
            self.place_descent_start_wait_timer = self.create_timer(
                self.place_descent_start_poll_s,
                self.wait_for_place_descent_start,
            )

        def wait_for_replayed_place_descent_start(self) -> None:
            current_positions = None
            if self.latest_joint_state is not None:
                current_positions = ordered_joint_positions(
                    list(self.latest_joint_state.name),
                    list(self.latest_joint_state.position),
                )
            joint_error = max_joint_position_delta(
                current_positions or [],
                self.pre_place_joint_replay_positions,
            )
            tolerance = max(
                self.pre_place_joint_replay_tolerance_rad,
                self.arm_execution_endpoint_acceptance_tolerance_rad,
            )
            if joint_error is not None and joint_error <= tolerance:
                self.record_step(
                    "place_descent_start_readiness",
                    True,
                    (
                        "paired joint-state replay accepted without Cartesian "
                        f"realignment, max_joint_delta={joint_error:.4f}rad, "
                        f"tolerance={tolerance:.4f}rad, polls="
                        f"{self.place_descent_start_wait_count}"
                    ),
                )
                self.begin_staged_place_descent()
                return
            maximum_polls = max(
                1,
                int(
                    self.place_descent_start_timeout_s
                    / self.place_descent_start_poll_s
                ),
            )
            if self.place_descent_start_wait_count >= maximum_polls:
                reason = (
                    "measured Panda arm joints unavailable"
                    if joint_error is None
                    else (
                        f"max_joint_delta={joint_error:.4f}rad exceeds paired "
                        f"replay tolerance={tolerance:.4f}rad"
                    )
                )
                self.record_step("place_descent_start_readiness", False, reason)
                self.finish_failed(f"place_descent_start_readiness failed: {reason}")
                return
            self.place_descent_start_wait_count += 1
            self.place_descent_start_wait_timer = self.create_timer(
                self.place_descent_start_poll_s,
                self.wait_for_place_descent_start,
            )

        def after_pre_place_alignment_recovery(self) -> None:
            self.place_descent_start_wait_count = 0
            self.wait_for_place_descent_start()

        def on_servo_status(self, message: ServoStatus) -> None:
            code = int(message.code)
            text = str(message.message)
            self.latest_servo_status_code = code
            self.latest_servo_status_message = text
            item = {"code": code, "message": text}
            if not self.servo_status_history or item != self.servo_status_history[-1]:
                self.servo_status_history.append(item)
            if self.servo_descent_active and code == 4:
                self.servo_collision_deceleration_seen_in_stage = True

        def publish_servo_descent_velocity(self, velocity_z: float) -> None:
            message = TwistStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self.servo_descent_frame_id or "panda_link0"
            message.twist.linear.z = float(velocity_z)
            self.servo_twist_publisher.publish(message)

        def stop_servo_descent_output(self) -> None:
            if getattr(self, "servo_descent_timer", None) is not None:
                self.servo_descent_timer.cancel()
                self.servo_descent_timer = None
            if getattr(self, "servo_descent_active", False):
                self.publish_servo_descent_velocity(0.0)
            self.servo_descent_target_z = None
            self.servo_descent_stage_started_ns = None
            self.servo_watchdog_last_check_ns = None

        def begin_servo_place_descent(self) -> None:
            if not self.servo_switch_client.service_is_ready():
                self.finish_failed(
                    f"servo_place_descent: service unavailable: {self.servo_switch_service}"
                )
                return
            if not self.servo_pause_client.service_is_ready():
                self.finish_failed(
                    f"servo_place_descent: service unavailable: {self.servo_pause_service}"
                )
                return
            self.latest_servo_status_code = None
            self.latest_servo_status_message = ""
            self.servo_status_history = []
            request = ServoCommandType.Request()
            request.command_type = ServoCommandType.Request.TWIST
            self.servo_switch_client.call_async(request).add_done_callback(
                self.on_servo_command_type_switched
            )

        def on_servo_command_type_switched(self, future) -> None:
            try:
                response = future.result()
            except Exception as exc:
                self.finish_failed(f"servo_place_descent: command switch failed: {exc}")
                return
            if not response.success:
                self.finish_failed("servo_place_descent: TWIST command mode rejected")
                return
            request = SetBool.Request()
            request.data = False
            self.servo_pause_client.call_async(request).add_done_callback(
                self.on_servo_descent_unpaused
            )

        def on_servo_descent_unpaused(self, future) -> None:
            try:
                response = future.result()
            except Exception as exc:
                self.finish_failed(f"servo_place_descent: unpause failed: {exc}")
                return
            if not response.success:
                self.finish_failed(
                    f"servo_place_descent: unpause rejected: {response.message}"
                )
                return
            self.servo_descent_active = True
            self.record_step(
                "servo_place_descent_started",
                True,
                (
                    f"speed={self.servo_descent_speed_mps:.4f}m/s, "
                    f"period={self.servo_command_period_s:.3f}s"
                ),
            )
            self.plan_next_place_descent_stage()

        def pause_servo_place_descent(self, next_callback) -> None:
            self.stop_servo_descent_output()
            if not self.servo_descent_active:
                next_callback()
                return
            request = SetBool.Request()
            request.data = True

            def paused(future) -> None:
                try:
                    response = future.result()
                except Exception as exc:
                    self.servo_descent_active = False
                    self.finish_failed(f"servo_place_descent: pause failed: {exc}")
                    return
                self.servo_descent_active = False
                if not response.success:
                    self.finish_failed(
                        f"servo_place_descent: pause rejected: {response.message}"
                    )
                    return
                self.record_step(
                    "servo_place_descent_paused",
                    True,
                    "Servo paused before gripper release",
                )
                next_callback()

            self.servo_pause_client.call_async(request).add_done_callback(paused)

        def abort_servo_place_descent(self, reason: str) -> None:
            self.stop_servo_descent_output()
            stop_command_ns = self.get_clock().now().nanoseconds
            stop_monotonic_ns = time.monotonic_ns()
            if "watchdog:" in reason.lower():
                self.servo_watchdog_abort_stop_command_ns = stop_command_ns
                self.servo_watchdog_abort_stop_monotonic_ns = stop_monotonic_ns
                self.servo_watchdog_abort_stage = self.place_descent_stage_index + 1
                self.servo_watchdog_abort_reason = reason
            if (
                self.servo_fault_injection_triggered
                and self.servo_fault_injection_expected_ns is not None
                and self.servo_fault_stop_command_ns is None
            ):
                self.servo_fault_stop_command_ns = stop_command_ns
                self.servo_watchdog_stop_latency_s = max(
                    0.0,
                    (
                        self.servo_fault_stop_command_ns
                        - self.servo_fault_injection_expected_ns
                    )
                    * 1e-9,
                )
                self.record_step(
                    "servo_watchdog_safe_stop",
                    True,
                    (
                        f"fault={self.servo_fault_injection_type}, "
                        f"stage={self.servo_fault_injection_observed_stage}, "
                        f"latency={self.servo_watchdog_stop_latency_s:.4f}s"
                    ),
                )
            if self.servo_descent_active and self.servo_pause_client.service_is_ready():
                request = SetBool.Request()
                request.data = True
                self.servo_pause_client.call_async(request)
            self.servo_descent_active = False
            self.finish_failed(reason)

        def start_servo_cartesian_fallback(self, reason: str) -> None:
            """Pause Servo and replan the current and remaining stages from feedback."""
            self.stop_servo_descent_output()

            def continue_with_cartesian(pause_reason: str) -> None:
                self.servo_descent_active = False
                self.servo_cartesian_fallback_used = True
                self.servo_cartesian_fallback_count += 1
                self.record_step(
                    "servo_place_descent_cartesian_fallback",
                    False,
                    f"{reason}; {pause_reason}",
                )
                self.current_start_state = (
                    self.measured_robot_state() or self.current_start_state
                )
                self.plan_next_place_descent_stage()

            if not self.servo_pause_client.service_is_ready():
                self.servo_descent_active = False
                self.finish_failed(
                    f"{reason}; Servo pause service unavailable for fallback"
                )
                return
            request = SetBool.Request()
            request.data = True

            def paused(future) -> None:
                try:
                    response = future.result()
                except Exception as exc:
                    self.servo_descent_active = False
                    self.finish_failed(f"{reason}; Servo pause failed: {exc}")
                    return
                if not response.success:
                    self.servo_descent_active = False
                    self.finish_failed(
                        f"{reason}; Servo pause rejected: {response.message}"
                    )
                    return
                continue_with_cartesian("Servo paused; collision-checked replan")

            self.servo_pause_client.call_async(request).add_done_callback(paused)

        def start_servo_place_descent_stage(
            self,
            target_pose: Dict[str, object],
        ) -> None:
            position = target_pose.get("position")
            if not isinstance(position, dict):
                self.abort_servo_place_descent(
                    "servo_place_descent: target position unavailable"
                )
                return
            self.servo_descent_frame_id = str(target_pose["frame_id"])
            measured = self.current_end_effector_position(
                self.servo_descent_frame_id
            )
            if measured is None:
                self.abort_servo_place_descent(
                    "servo_place_descent: measured hand transform unavailable"
                )
                return
            horizontal_error = sqrt(
                (measured[0] - float(position["x"])) ** 2
                + (measured[1] - float(position["y"])) ** 2
            )
            if horizontal_error > self.place_descent_stage_max_horizontal_motion_m:
                self.abort_servo_place_descent(
                    "servo_place_descent: horizontal target mismatch "
                    f"{horizontal_error:.4f}m exceeds "
                    f"{self.place_descent_stage_max_horizontal_motion_m:.4f}m"
                )
                return
            self.servo_descent_target_z = float(position["z"])
            distance = abs(self.servo_descent_target_z - measured[2])
            self.servo_descent_stage_timeout_s = (
                distance / self.servo_descent_speed_mps
                + self.servo_stage_timeout_margin_s
            )
            self.servo_descent_stage_started_ns = self.get_clock().now().nanoseconds
            self.servo_watchdog_last_check_ns = None
            self.servo_collision_deceleration_seen_in_stage = False
            stage_number = self.place_descent_stage_index + 1
            self.publish_demo_state(
                (
                    f"servo_place_descent_stage_{stage_number}_of_"
                    f"{len(self.place_descent_stage_pose_keys)}"
                ),
                self.place_descent_stage_pose_keys[self.place_descent_stage_index],
                "executing",
            )
            self.servo_descent_timer = self.create_timer(
                self.servo_command_period_s,
                self.update_servo_place_descent_stage,
            )

        def update_servo_place_descent_stage(self) -> None:
            if (
                not self.active
                or not self.servo_descent_active
                or self.servo_descent_target_z is None
                or self.servo_descent_stage_started_ns is None
            ):
                self.stop_servo_descent_output()
                return
            if self.latest_servo_status_code in SERVO_HALT_STATUS_CODES:
                reason = (
                    "servo_place_descent: Servo halted with code="
                    f"{self.latest_servo_status_code}, message="
                    f"{self.latest_servo_status_message}"
                )
                if servo_halt_allows_cartesian_fallback(
                    self.latest_servo_status_code,
                    self.enable_servo_cartesian_fallback,
                ):
                    self.start_servo_cartesian_fallback(reason)
                else:
                    self.abort_servo_place_descent(reason)
                return
            measured = self.current_end_effector_position(
                self.servo_descent_frame_id
            )
            if measured is None:
                self.abort_servo_place_descent(
                    "servo_place_descent: hand transform lost during execution"
                )
                return
            elapsed_s = (
                self.get_clock().now().nanoseconds
                - self.servo_descent_stage_started_ns
            ) * 1e-9
            now_ns = self.get_clock().now().nanoseconds
            watchdog_due = (
                self.servo_watchdog_last_check_ns is None
                or (now_ns - self.servo_watchdog_last_check_ns) * 1e-9
                >= self.servo_watchdog_period_s
            )
            if watchdog_due:
                self.servo_watchdog_last_check_ns = now_ns
                self.servo_watchdog_check_count += 1
                injected_fault = "none"
                if (
                    not self.servo_fault_injection_triggered
                    and servo_fault_injection_due(
                        self.servo_fault_injection_type,
                        self.servo_fault_injection_stage,
                        self.place_descent_stage_index,
                        elapsed_s,
                        self.servo_fault_injection_delay_s,
                    )
                ):
                    injected_fault = self.servo_fault_injection_type
                    self.servo_fault_injection_triggered = True
                    self.servo_fault_injection_observed_stage = (
                        self.place_descent_stage_index + 1
                    )
                    self.servo_fault_injection_expected_ns = int(
                        self.servo_descent_stage_started_ns
                        + self.servo_fault_injection_delay_s * 1e9
                    )
                    self.record_step(
                        "servo_watchdog_fault_injected",
                        False,
                        (
                            f"fault={injected_fault}, "
                            f"stage={self.servo_fault_injection_observed_stage}, "
                            f"configured_delay={self.servo_fault_injection_delay_s:.3f}s"
                        ),
                    )
                if self.enable_tactile_grasp_supervision:
                    tactile_success, tactile_reason = self.tactile_contact_status()
                    if injected_fault == "contact_loss":
                        tactile_success = False
                        tactile_reason = "synthetic bilateral contact loss"
                    if not tactile_success:
                        self.abort_servo_place_descent(
                            "servo_place_descent watchdog: bilateral contact lost; "
                            + tactile_reason
                        )
                        return
                if self.enable_physical_grasp_verification:
                    pose_key = self.place_descent_stage_pose_keys[
                        self.place_descent_stage_index
                    ]
                    payload_error, unavailable_reason = (
                        self.current_payload_transfer_error(
                            pose_key,
                            self.place_descent_object_in_hand_pose,
                        )
                    )
                    if injected_fault == "payload_drift":
                        payload_error = (
                            self.servo_fault_injection_payload_drift_m
                        )
                        unavailable_reason = ""
                    if payload_error is None:
                        self.abort_servo_place_descent(
                            "servo_place_descent watchdog: " + unavailable_reason
                        )
                        return
                    if payload_error > self.place_descent_stage_payload_tolerance_m:
                        self.abort_servo_place_descent(
                            "servo_place_descent watchdog: payload drift="
                            f"{payload_error:.4f}m exceeds "
                            f"{self.place_descent_stage_payload_tolerance_m:.4f}m"
                        )
                        return
            remaining = self.servo_descent_target_z - measured[2]
            velocity = bounded_servo_velocity(
                remaining,
                self.servo_descent_speed_mps,
                self.servo_command_period_s,
                self.servo_stage_position_tolerance_m,
            )
            if velocity == 0.0:
                stage_number = self.place_descent_stage_index + 1
                final_error = abs(remaining)
                self.stop_servo_descent_output()
                self.record_step(
                    (
                        f"servo_place_descent_stage_{stage_number}_of_"
                        f"{len(self.place_descent_stage_pose_keys)}"
                    ),
                    True,
                    (
                        f"endpoint_error={final_error:.4f}m, "
                        f"elapsed={elapsed_s:.2f}s, status="
                        f"{self.latest_servo_status_code}"
                    ),
                )
                self.after_place_descent_stage()
                return
            if elapsed_s > self.servo_descent_stage_timeout_s:
                if servo_collision_limited_endpoint_acceptable(
                    remaining,
                    self.place_descent_stage_index,
                    len(self.place_descent_stage_pose_keys),
                    self.servo_collision_deceleration_seen_in_stage,
                    self.servo_collision_limited_acceptance_m,
                ):
                    stage_number = self.place_descent_stage_index + 1
                    self.stop_servo_descent_output()
                    self.record_step(
                        (
                            f"servo_place_descent_stage_{stage_number}_of_"
                            f"{len(self.place_descent_stage_pose_keys)}"
                        ),
                        True,
                        (
                            "collision_limited_endpoint, remaining="
                            f"{remaining:.4f}m, elapsed={elapsed_s:.2f}s, "
                            f"acceptance={self.servo_collision_limited_acceptance_m:.4f}m"
                        ),
                    )
                    self.after_place_descent_stage()
                    return
                self.abort_servo_place_descent(
                    "servo_place_descent: stage timeout, remaining="
                    f"{remaining:.4f}m after {elapsed_s:.2f}s"
                )
                return
            self.publish_servo_descent_velocity(velocity)

        def begin_staged_place_descent(self) -> None:
            if not self.verify_payload_after_transfer("pre_place_pose"):
                return
            if not self.capture_place_descent_payload_anchor():
                return
            self.capture_place_descent_reference()
            if self.candidate is None:
                self.finish_failed("place descent: candidate missing")
                return
            pre_place_pose = self.candidate.get("pre_place_pose")
            place_pose = self.candidate.get("place_pose")
            if not isinstance(pre_place_pose, dict) or not isinstance(
                place_pose,
                dict,
            ):
                self.finish_failed("place descent: staged poses unavailable")
                return
            measured_start_position = self.current_end_effector_position(
                str(pre_place_pose["frame_id"])
            )
            if measured_start_position is None:
                self.finish_failed("place descent: measured hand transform unavailable")
                return
            stages, path_anchor_offset = anchored_place_descent_poses(
                pre_place_pose,
                place_pose,
                measured_start_position,
                self.place_descent_stage_count,
            )
            self.record_step(
                "place_descent_path_anchor",
                True,
                (
                    f"dx={path_anchor_offset[0]:.4f}m, "
                    f"dy={path_anchor_offset[1]:.4f}m, "
                    f"dz={path_anchor_offset[2]:.4f}m, "
                    "contact-zone descent translated to measured pre-place pose"
                ),
            )
            self.place_descent_stage_pose_keys = []
            self.place_descent_stage_diagnostics = []
            self.place_descent_stage_index = 0
            for index, pose in enumerate(stages, start=1):
                key = f"place_descent_stage_{index}_pose"
                self.candidate[key] = pose
                self.place_descent_stage_pose_keys.append(key)
            if self.place_descent_backend == "servo":
                self.begin_servo_place_descent()
            else:
                self.plan_next_place_descent_stage()

        def plan_next_place_descent_stage(self) -> None:
            if self.place_descent_stage_index >= len(
                self.place_descent_stage_pose_keys
            ):
                if self.place_descent_backend == "servo":
                    self.pause_servo_place_descent(self.after_place_descent)
                else:
                    self.after_place_descent()
                return
            key = self.place_descent_stage_pose_keys[
                self.place_descent_stage_index
            ]
            target_pose = self.candidate.get(key) if self.candidate is not None else None
            if not isinstance(target_pose, dict):
                self.finish_failed(f"place descent: {key} unavailable")
                return
            if self.latest_physical_target_pose is not None:
                self.place_descent_stage_reference_target_pose = tuple(
                    float(value) for value in self.latest_physical_target_pose
                )
            self.place_descent_stage_reference_hand_pose = (
                self.current_end_effector_position(str(target_pose["frame_id"]))
            )
            if self.place_descent_stage_reference_hand_pose is None:
                self.finish_failed("place descent: hand transform unavailable")
                return
            stage_number = self.place_descent_stage_index + 1
            if (
                self.place_descent_backend == "servo"
                and not self.servo_cartesian_fallback_used
            ):
                self.start_servo_place_descent_stage(target_pose)
                return
            self.plan_cartesian_step(
                (
                    f"cartesian_place_descent_stage_{stage_number}_of_"
                    f"{len(self.place_descent_stage_pose_keys)}"
                ),
                key,
                self.after_place_descent_stage,
            )

        def after_place_descent_stage(self) -> None:
            stage_number = self.place_descent_stage_index + 1
            key = self.place_descent_stage_pose_keys[
                self.place_descent_stage_index
            ]
            if not self.verify_place_descent_stage_motion(stage_number):
                return
            if not self.verify_payload_after_transfer(
                key,
                tolerance_m=self.place_descent_stage_payload_tolerance_m,
                step_name="place_descent_payload_check",
                object_in_hand_pose_override=(
                    self.place_descent_object_in_hand_pose
                ),
            ):
                return
            tactile_success, tactile_reason = self.tactile_contact_status()
            self.record_step(
                "place_descent_tactile_check",
                tactile_success,
                f"stage={stage_number}; {tactile_reason}",
            )
            if self.enable_tactile_grasp_supervision and not tactile_success:
                self.finish_failed(
                    f"place_descent_tactile_check failed at stage {stage_number}: "
                    f"{tactile_reason}"
                )
                return
            self.payload_transfer_execution_replan_count = 0
            self.place_descent_stage_index += 1
            self.plan_next_place_descent_stage()

        def verify_place_descent_stage_motion(self, stage_number: int) -> bool:
            if not self.enable_physical_grasp_verification:
                self.record_step(
                    "place_descent_stage_motion",
                    True,
                    (
                        f"stage={stage_number}/{len(self.place_descent_stage_pose_keys)}; "
                        "payload-motion check skipped for unloaded control trial"
                    ),
                )
                return True
            before = self.place_descent_stage_reference_target_pose
            after = self.latest_physical_target_pose
            before_hand = self.place_descent_stage_reference_hand_pose
            key = self.place_descent_stage_pose_keys[
                self.place_descent_stage_index
            ]
            target_pose = self.candidate.get(key) if self.candidate is not None else None
            frame_id = (
                str(target_pose["frame_id"])
                if isinstance(target_pose, dict)
                else ""
            )
            after_hand = self.current_end_effector_position(frame_id)
            if (
                before is None
                or after is None
                or before_hand is None
                or after_hand is None
            ):
                self.finish_failed(
                    f"place_descent_stage_{stage_number}: target or hand pose unavailable"
                )
                return False
            motion = place_descent_stage_motion_diagnostic(
                before,
                after,
                before_hand,
                after_hand,
            )
            horizontal = motion["horizontal_motion_m"]
            success = place_descent_stage_path_is_safe(
                motion,
                self.place_descent_stage_max_horizontal_motion_m,
            )
            diagnostic = {"stage": stage_number, **motion, "success": success}
            self.place_descent_stage_diagnostics.append(diagnostic)
            reason = (
                f"stage={stage_number}/{len(self.place_descent_stage_pose_keys)}, "
                f"dx={motion['dx_m']:.4f}m, dy={motion['dy_m']:.4f}m, "
                f"dz={motion['dz_m']:.4f}m, "
                f"horizontal={horizontal:.4f}m, max_horizontal="
                f"{self.place_descent_stage_max_horizontal_motion_m:.4f}m, "
                f"hand_horizontal={motion['hand_horizontal_motion_m']:.4f}m, "
                f"translation_only_payload_residual="
                f"{motion['payload_relative_horizontal_m']:.4f}m"
            )
            self.record_step("place_descent_stage_motion", success, reason)
            if not success:
                self.finish_failed(f"place_descent_stage_motion failed: {reason}")
            return success

        def current_end_effector_position(
            self,
            frame_id: str,
        ) -> Optional[Tuple[float, float, float]]:
            if not frame_id:
                return None
            try:
                transform = self.tf_buffer.lookup_transform(
                    frame_id,
                    self.end_effector_link,
                    Time(),
                )
            except TransformException as exc:
                self.get_logger().warn(
                    f"End-effector transform unavailable in {frame_id}: {exc}"
                )
                return None
            translation = transform.transform.translation
            return (
                float(translation.x),
                float(translation.y),
                float(translation.z),
            )

        def after_direct_place_transfer(self) -> None:
            if not self.verify_payload_after_transfer("place_pose"):
                return
            self.capture_place_descent_reference()
            self.after_place_descent()

        def capture_place_descent_reference(self) -> None:
            if self.latest_physical_target_pose is None:
                return
            self.place_descent_reference_target_pose = tuple(
                float(value) for value in self.latest_physical_target_pose
            )
            self.place_descent_diagnostic_recorded = False

        def capture_place_descent_payload_anchor(self) -> bool:
            if not self.enable_physical_grasp_verification:
                if self.candidate is None:
                    self.finish_failed(
                        "place_descent_payload_reanchor: candidate unavailable"
                    )
                    return False
                nominal_local = self.candidate.get("object_in_hand_pose")
                if not isinstance(nominal_local, dict):
                    self.finish_failed(
                        "place_descent_payload_reanchor: payload transform unavailable"
                    )
                    return False
                self.place_descent_object_in_hand_pose = deepcopy(nominal_local)
                self.place_descent_payload_reanchor_delta_m = 0.0
                self.record_step(
                    "place_descent_payload_reanchor",
                    True,
                    "nominal hand/object anchor used for unloaded control trial",
                )
                return True
            if (
                self.candidate is None
                or self.latest_physical_target_pose is None
                or self.physical_target_reference_pose is None
            ):
                self.finish_failed("place_descent_payload_reanchor: target unavailable")
                return False
            object_pose = self.candidate.get("object_pose")
            original_local = self.candidate.get("object_in_hand_pose")
            if not isinstance(object_pose, dict) or not isinstance(
                original_local,
                dict,
            ):
                self.finish_failed(
                    "place_descent_payload_reanchor: payload transform unavailable"
                )
                return False
            frame_id = str(object_pose["frame_id"])
            try:
                transform = self.tf_buffer.lookup_transform(
                    frame_id,
                    self.end_effector_link,
                    Time(),
                )
            except TransformException as exc:
                self.finish_failed(
                    f"place_descent_payload_reanchor: hand TF unavailable: {exc}"
                )
                return False
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            measured_hand_pose = {
                "frame_id": frame_id,
                "position": {
                    "x": float(translation.x),
                    "y": float(translation.y),
                    "z": float(translation.z),
                },
                "orientation": {
                    "x": float(rotation.x),
                    "y": float(rotation.y),
                    "z": float(rotation.z),
                    "w": float(rotation.w),
                },
            }
            measured_local = calibrated_object_in_hand_pose(
                measured_hand_pose,
                object_pose,
                self.latest_physical_target_pose,
                self.physical_target_reference_pose,
            )
            original_position = original_local["position"]
            measured_position = measured_local["position"]
            self.place_descent_payload_reanchor_delta_m = translation_distance(
                tuple(
                    float(measured_position[axis]) - float(original_position[axis])
                    for axis in ("x", "y", "z")
                )
            )
            self.place_descent_object_in_hand_pose = measured_local
            self.record_step(
                "place_descent_payload_reanchor",
                True,
                (
                    "fixed pre-place hand/object anchor captured, inherited_offset="
                    f"{self.place_descent_payload_reanchor_delta_m:.4f}m"
                ),
            )
            return True

        def record_place_descent_diagnostics(self) -> None:
            if (
                self.place_descent_diagnostic_recorded
                or self.place_descent_reference_target_pose is None
                or self.latest_physical_target_pose is None
            ):
                return
            before = self.place_descent_reference_target_pose
            after = self.latest_physical_target_pose
            dx = float(after[0]) - float(before[0])
            dy = float(after[1]) - float(before[1])
            dz = float(after[2]) - float(before[2])
            horizontal = sqrt(dx * dx + dy * dy)
            total = sqrt(horizontal * horizontal + dz * dz)
            self.place_descent_horizontal_motion_m = horizontal
            self.place_descent_vertical_motion_m = abs(dz)
            self.place_descent_motion_m = total
            tactile_success, tactile_reason = self.tactile_contact_status()
            self.record_step(
                "place_descent_payload_motion",
                horizontal <= self.place_feedback_tolerance_m,
                (
                    f"dx={dx:.4f}m, dy={dy:.4f}m, dz={dz:.4f}m, "
                    f"horizontal={horizontal:.4f}m, total={total:.4f}m, "
                    f"bilateral_contact={tactile_success}; {tactile_reason}"
                ),
            )
            self.place_descent_diagnostic_recorded = True

        def after_place_descent(self) -> None:
            if not self.enable_physical_grasp_verification:
                self.finish_success()
                return
            if self.place_feedback_settle_s <= 0.0:
                self.evaluate_place_feedback()
                return
            if self.place_feedback_timer is not None:
                self.place_feedback_timer.cancel()
            self.place_feedback_timer = self.create_timer(
                self.place_feedback_settle_s,
                self.evaluate_place_feedback,
            )

        def evaluate_place_feedback(self) -> None:
            if self.place_feedback_timer is not None:
                self.place_feedback_timer.cancel()
                self.place_feedback_timer = None
            if (
                self.candidate is None
                or self.latest_physical_target_pose is None
                or self.physical_target_reference_pose is None
            ):
                self.finish_failed("place_feedback_check: target pose unavailable")
                return
            self.record_place_descent_diagnostics()
            object_pose = self.candidate.get("object_pose")
            desired_pose = self.candidate.get("desired_place_object_pose")
            if not isinstance(object_pose, dict) or not isinstance(desired_pose, dict):
                self.finish_failed("place_feedback_check: desired object pose unavailable")
                return
            actual = target_position_in_measurement_frame(
                self.latest_physical_target_pose,
                self.physical_target_reference_pose,
                object_pose,
            )
            desired_position = desired_pose["position"]
            desired = tuple(
                float(desired_position[axis]) for axis in ("x", "y", "z")
            )
            correction = tuple(
                desired_value - actual_value
                for desired_value, actual_value in zip(desired, actual)
            )
            error = translation_distance(correction)
            self.place_feedback_error_m = error
            if self.place_feedback_previous_error_m is not None:
                self.place_feedback_improvement_m = (
                    self.place_feedback_previous_error_m - error
                )
            if error <= self.place_feedback_tolerance_m:
                self.record_step(
                    "place_feedback_check",
                    True,
                    (
                        f"position_error={error:.4f}m, tolerance="
                        f"{self.place_feedback_tolerance_m:.4f}m, retries="
                        f"{self.place_feedback_retry_count}"
                    ),
                )
                self.finish_success()
                return
            if (
                self.place_feedback_retry_count > 0
                and self.place_feedback_previous_error_m is not None
                and self.place_feedback_improvement_m is not None
                and self.place_feedback_improvement_m
                < self.place_feedback_min_improvement_m
            ):
                reason = (
                    "correction ineffective: before="
                    f"{self.place_feedback_previous_error_m:.4f}m, after="
                    f"{error:.4f}m, improvement="
                    f"{self.place_feedback_improvement_m:.4f}m, required="
                    f"{self.place_feedback_min_improvement_m:.4f}m"
                )
                self.record_step("place_feedback_check", False, reason)
                self.finish_failed(f"place_feedback_check failed: {reason}")
                return
            if (
                error > self.place_feedback_max_correction_m
                or self.place_feedback_retry_count >= self.place_feedback_max_retries
            ):
                reason = (
                    f"position_error={error:.4f}m, tolerance="
                    f"{self.place_feedback_tolerance_m:.4f}m, max_correction="
                    f"{self.place_feedback_max_correction_m:.4f}m, retries="
                    f"{self.place_feedback_retry_count}/"
                    f"{self.place_feedback_max_retries}"
                )
                self.record_step("place_feedback_check", False, reason)
                self.finish_failed(f"place_feedback_check failed: {reason}")
                return
            frame_id = str(desired_pose["frame_id"])
            try:
                transform = self.tf_buffer.lookup_transform(
                    frame_id,
                    self.end_effector_link,
                    Time(),
                )
            except TransformException as exc:
                self.finish_failed(f"place_feedback_check: hand TF unavailable: {exc}")
                return
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            current_hand_pose = {
                "frame_id": frame_id,
                "position": {
                    "x": float(translation.x),
                    "y": float(translation.y),
                    "z": float(translation.z),
                },
                "orientation": {
                    "x": float(rotation.x),
                    "y": float(rotation.y),
                    "z": float(rotation.z),
                    "w": float(rotation.w),
                },
            }
            raised_pose, translated_pose, correction_pose = (
                staged_place_feedback_poses(
                    current_hand_pose,
                    correction,
                    self.place_feedback_recovery_height_m,
                )
            )
            self.place_feedback_previous_error_m = error
            self.place_feedback_retry_count += 1
            self.candidate["place_feedback_raise_pose"] = raised_pose
            self.candidate["place_feedback_translate_pose"] = translated_pose
            self.candidate["place_feedback_correction_pose"] = correction_pose
            self.record_step(
                "place_feedback_correction",
                True,
                (
                    f"dx={correction[0]:.4f}m, dy={correction[1]:.4f}m, "
                    f"dz={correction[2]:.4f}m, attempt="
                    f"{self.place_feedback_retry_count}/"
                    f"{self.place_feedback_max_retries}, recovery_height="
                    f"{self.place_feedback_recovery_height_m:.4f}m"
                ),
            )
            self.plan_cartesian_step(
                "cartesian_place_feedback_lift",
                "place_feedback_raise_pose",
                self.after_place_feedback_lift,
            )

        def after_place_feedback_lift(self) -> None:
            self.plan_cartesian_step(
                "cartesian_place_feedback_translate",
                "place_feedback_translate_pose",
                self.after_place_feedback_translate,
            )

        def after_place_feedback_translate(self) -> None:
            self.plan_cartesian_step(
                "cartesian_place_correction",
                "cartesian_pre_place_alignment",
                "place_feedback_correction_pose",
                self.after_place_descent,
            )

        def current_payload_transfer_error(
            self,
            hand_pose_key: str,
            object_in_hand_pose_override: Optional[Dict[str, object]] = None,
        ) -> Tuple[Optional[float], str]:
            """Return current rigid hand-object translation error without side effects."""
            if not self.enable_physical_grasp_verification:
                return 0.0, "physical payload verification disabled"
            if (
                self.candidate is None
                or self.latest_physical_target_pose is None
                or self.physical_target_reference_pose is None
            ):
                return None, "target pose unavailable"
            hand_pose = self.candidate.get(hand_pose_key)
            object_in_hand_pose = (
                object_in_hand_pose_override
                if object_in_hand_pose_override is not None
                else self.candidate.get("object_in_hand_pose")
            )
            object_pose = self.candidate.get("object_pose")
            if (
                not isinstance(hand_pose, dict)
                or not isinstance(object_in_hand_pose, dict)
                or not isinstance(object_pose, dict)
            ):
                return None, "payload transform unavailable"
            frame_id = str(hand_pose["frame_id"])
            try:
                transform = self.tf_buffer.lookup_transform(
                    frame_id,
                    self.end_effector_link,
                    Time(),
                )
            except TransformException as exc:
                return None, f"hand TF unavailable: {exc}"
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            measured_hand_pose = {
                "frame_id": frame_id,
                "position": {
                    "x": float(translation.x),
                    "y": float(translation.y),
                    "z": float(translation.z),
                },
                "orientation": {
                    "x": float(rotation.x),
                    "y": float(rotation.y),
                    "z": float(rotation.z),
                    "w": float(rotation.w),
                },
            }
            object_position = object_pose["position"]
            measurement_frame_translation = tuple(
                reference - float(object_position[axis])
                for reference, axis in zip(
                    self.physical_target_reference_pose,
                    ("x", "y", "z"),
                )
            )
            error = rigid_payload_translation_error(
                self.latest_physical_target_pose,
                measured_hand_pose,
                object_in_hand_pose,
                measurement_frame_translation,
            )
            return error, ""

        def verify_payload_after_transfer(
            self,
            hand_pose_key: str,
            tolerance_m: Optional[float] = None,
            step_name: str = "physical_transfer_check",
            object_in_hand_pose_override: Optional[Dict[str, object]] = None,
        ) -> bool:
            if not self.enable_physical_grasp_verification:
                return True
            error, unavailable_reason = self.current_payload_transfer_error(
                hand_pose_key,
                object_in_hand_pose_override,
            )
            if error is None:
                self.finish_failed(
                    f"physical_transfer_check: {unavailable_reason}"
                )
                return False
            tolerance = (
                self.payload_transfer_tolerance_m
                if tolerance_m is None
                else max(0.0, float(tolerance_m))
            )
            success = error <= tolerance
            self.record_step(
                step_name,
                success,
                (
                    f"pose_error={error:.4f}m, tolerance="
                    f"{tolerance:.4f}m, waypoint={hand_pose_key}"
                ),
            )
            if not success:
                self.finish_failed(
                    f"{step_name} failed: payload pose error {error:.4f}m"
                )
            return success

        def has_equivalent_grasp_orientation(self) -> bool:
            return self.equivalent_grasp_attempt_count < len(
                self.equivalent_grasp_recovery_yaw_offsets_deg
            )

        def should_try_equivalent_grasp_orientation(
            self,
            step_name: str,
        ) -> bool:
            return (
                step_name in {
                    "cartesian_pre_grasp_alignment",
                    "cartesian_approach",
                }
                and self.has_equivalent_grasp_orientation()
                and self.base_candidate is not None
            )

        def plan_equivalent_grasp_orientation(self) -> None:
            if self.base_candidate is None:
                self.finish_failed("equivalent grasp orientation: base candidate missing")
                return
            if not self.has_equivalent_grasp_orientation():
                self.finish_failed("equivalent grasp orientation: candidates exhausted")
                return
            yaw_offset = self.equivalent_grasp_recovery_yaw_offsets_deg[
                self.equivalent_grasp_attempt_count
            ]
            self.equivalent_grasp_attempt_count += 1
            self.active_grasp_yaw_offset_deg = yaw_offset
            self.candidate = equivalent_grasp_candidate(
                self.base_candidate,
                yaw_offset,
            )
            pre_grasp = self.candidate.get("pre_grasp_pose")
            if not isinstance(pre_grasp, dict):
                self.finish_failed("equivalent grasp orientation: pre_grasp_pose missing")
                return
            self.candidate["equivalent_grasp_staging_pose"] = translated_pose_dict(
                pre_grasp,
                (0.0, 0.0, max(0.05, self.pre_grasp_alignment_staging_height)),
            )
            self.approach_recovery_attempt_count = 0
            self.active_approach_recovery_ratio = None
            self.pre_grasp_alignment_recovery_used = True
            self.pre_grasp_alignment_prefix_attempt_count = 0
            self.contact_prefix_attempt_counts.pop("cartesian_approach", None)
            for step in (
                "cartesian_pre_grasp_alignment",
                "cartesian_approach",
            ):
                self.cartesian_retry_used.pop(step, None)
                self.cartesian_timing_retry_counts.pop(step, None)
            self.current_start_state = (
                self.measured_robot_state() or self.current_start_state
            )
            self.publish_active_candidate()
            self.record_step(
                "equivalent_grasp_orientation_selected",
                True,
                (
                    f"yaw_offset_deg={yaw_offset:.1f}, candidate="
                    f"{self.equivalent_grasp_attempt_count}/"
                    f"{len(self.equivalent_grasp_recovery_yaw_offsets_deg)}"
                ),
            )
            self.get_logger().warn(
                "Switching to cube-equivalent grasp orientation "
                f"yaw_offset={yaw_offset:.1f}deg."
            )
            self.plan_ompl_step(
                "ompl_to_equivalent_grasp_staging",
                "equivalent_grasp_staging_pose",
                self.after_equivalent_grasp_staging,
                constrain_orientation=True,
                orientation_tolerance=self.recovery_orientation_tolerance,
                planner_id_override=self.recovery_planner_id,
            )

        def after_equivalent_grasp_staging(self) -> None:
            self.plan_ompl_step(
                "ompl_to_equivalent_grasp_pre_grasp",
                "pre_grasp_pose",
                self.after_equivalent_grasp_pre_grasp,
                constrain_orientation=True,
                orientation_tolerance=self.pre_grasp_orientation_tolerance,
                planner_id_override=self.recovery_planner_id,
            )

        def after_equivalent_grasp_pre_grasp(self) -> None:
            self.record_step(
                "equivalent_grasp_pre_grasp_ready",
                True,
                (
                    "OMPL pose goal reached; contact approach remains subject "
                    "to the strict Cartesian threshold"
                ),
            )
            self.after_pre_grasp_alignment()

        def should_try_approach_recovery(self, step_name: str) -> bool:
            return (
                self.enable_cartesian_approach_recovery
                and step_name == "cartesian_approach"
                and self.has_approach_recovery_candidate()
                and self.candidate is not None
                and self.candidate.get("pre_grasp_pose") is not None
                and self.candidate.get("grasp_pose") is not None
            )

        def has_approach_recovery_candidate(self) -> bool:
            return self.approach_recovery_attempt_count < len(
                self.approach_recovery_ratios
            )

        def should_try_pre_grasp_alignment_recovery(self, step_name: str) -> bool:
            return (
                self.enable_pre_grasp_alignment_recovery
                and step_name == "cartesian_pre_grasp_alignment"
                and not self.pre_grasp_alignment_recovery_used
                and self.candidate is not None
                and self.candidate.get("pre_grasp_pose") is not None
            )

        def plan_pre_grasp_alignment_recovery(self) -> None:
            if self.candidate is None:
                self.finish_failed("pre-grasp alignment recovery: candidate missing")
                return
            pre_grasp = self.candidate.get("pre_grasp_pose")
            if not isinstance(pre_grasp, dict):
                self.finish_failed("pre-grasp alignment recovery: pre_grasp_pose missing")
                return
            staging_height = max(0.05, self.pre_grasp_alignment_staging_height)
            self.candidate["pre_grasp_alignment_staging_pose"] = translated_pose_dict(
                pre_grasp,
                (0.0, 0.0, staging_height),
            )
            self.get_logger().warn(
                "Pre-grasp alignment failed; moving above the obstacle to align before descending."
            )
            self.plan_ompl_step(
                "ompl_to_pre_grasp_alignment_staging",
                "pre_grasp_alignment_staging_pose",
                self.after_pre_grasp_alignment_staging,
                constrain_orientation=True,
                orientation_tolerance=self.recovery_orientation_tolerance,
                planner_id_override=self.recovery_planner_id,
            )

        def after_pre_grasp_alignment_staging(self) -> None:
            self.cartesian_retry_used.pop("cartesian_pre_grasp_alignment", None)
            self.plan_ompl_step(
                "ompl_to_strict_pre_grasp",
                "pre_grasp_pose",
                self.after_strict_pre_grasp,
                constrain_orientation=True,
                orientation_tolerance=self.recovery_orientation_tolerance,
                planner_id_override=self.recovery_planner_id,
            )

        def after_strict_pre_grasp(self) -> None:
            self.record_step(
                "strict_pre_grasp_ready",
                True,
                "OMPL reached the strict pose goal without Cartesian IK branch switching",
            )
            self.after_pre_grasp_alignment()

        def plan_approach_recovery(self) -> None:
            if self.candidate is None:
                self.finish_failed("cartesian_approach recovery: candidate missing")
                return
            if not self.has_approach_recovery_candidate():
                self.finish_failed("cartesian_approach recovery: candidates exhausted")
                return
            ratio = self.approach_recovery_ratios[
                self.approach_recovery_attempt_count
            ]
            orientation_tolerance = min(
                self.pre_grasp_orientation_tolerance,
                self.recovery_orientation_tolerance
                + self.recovery_orientation_tolerance_step
                * self.approach_recovery_attempt_count,
            )
            self.approach_recovery_attempt_count += 1
            self.active_approach_recovery_ratio = ratio
            recovery_pose = self.make_approach_recovery_pose(ratio)
            if recovery_pose is None:
                self.finish_failed("cartesian_approach recovery: recovery pose missing")
                return
            self.candidate["approach_recovery_pose"] = recovery_pose
            self.get_logger().warn(
                "Cartesian approach failed; planning to intermediate recovery "
                f"candidate {self.approach_recovery_attempt_count}/"
                f"{len(self.approach_recovery_ratios)} at ratio={ratio:.2f}, "
                f"orientation_tolerance={orientation_tolerance:.3f}rad."
            )
            self.plan_ompl_step(
                "ompl_to_approach_recovery",
                "approach_recovery_pose",
                self.after_approach_recovery,
                constrain_orientation=True,
                orientation_tolerance=orientation_tolerance,
                planner_id_override=self.recovery_planner_id,
            )

        def after_approach_recovery(self) -> None:
            self.cartesian_retry_used.pop("cartesian_approach", None)
            self.plan_cartesian_step("cartesian_approach", "grasp_pose", self.after_grasp)

        def make_approach_recovery_pose(
            self,
            ratio: float,
        ) -> Optional[Dict[str, object]]:
            if self.candidate is None:
                return None
            pre_grasp = self.candidate.get("pre_grasp_pose")
            grasp = self.candidate.get("grasp_pose")
            if not isinstance(pre_grasp, dict) or not isinstance(grasp, dict):
                return None
            ratio = min(0.9, max(0.1, ratio))
            pre_position = pre_grasp["position"]
            grasp_position = grasp["position"]
            position = {
                axis: float(pre_position[axis])
                + (float(grasp_position[axis]) - float(pre_position[axis])) * ratio
                for axis in ("x", "y", "z")
            }
            return {
                "frame_id": grasp["frame_id"],
                "position": position,
                "orientation": grasp["orientation"],
            }

        def run_gripper_step(
            self,
            step_name: str,
            finger_position: float,
            pose_key: Optional[str],
            next_callback,
        ) -> None:
            if not self.execute_gripper or not self.execute_trajectories or self.visualize_only_execution:
                self.publish_demo_state(step_name, pose_key, "visualized")
                next_callback()
                return

            if self.gripper_execution_backend == "follow_joint_trajectory":
                self.run_gripper_trajectory_step(
                    step_name,
                    finger_position,
                    pose_key,
                    next_callback,
                )
                return
            if self.gripper_execution_backend != "move_group":
                self.record_step(
                    step_name,
                    False,
                    f"unknown gripper_execution_backend={self.gripper_execution_backend}",
                )
                self.finish_failed(
                    f"{step_name}: unknown gripper backend {self.gripper_execution_backend}"
                )
                return

            goal = MoveGroup.Goal()
            goal.request = self.make_gripper_motion_plan_request(finger_position)
            goal.planning_options = self.make_gripper_planning_options()
            self.get_logger().info(
                f"Executing {step_name} with {self.gripper_group_name} "
                f"finger_position={finger_position:.3f}."
            )
            future = self.move_client.send_goal_async(goal)
            future.add_done_callback(
                lambda done_future: self.on_gripper_goal_response(
                    done_future,
                    step_name,
                    pose_key,
                    next_callback,
                )
            )

        def run_gripper_trajectory_step(
            self,
            step_name: str,
            finger_position: float,
            pose_key: Optional[str],
            next_callback,
        ) -> None:
            if self.latest_joint_state is None:
                self.record_step(step_name, False, "/joint_states unavailable for gripper")
                self.finish_failed(f"{step_name}: /joint_states unavailable for gripper")
                return
            current_by_name = dict(
                zip(self.latest_joint_state.name, self.latest_joint_state.position)
            )
            observed_finger_joints = ["panda_finger_joint1", "panda_finger_joint2"]
            missing = [name for name in observed_finger_joints if name not in current_by_name]
            if missing:
                self.record_step(step_name, False, f"missing gripper joints: {missing}")
                self.finish_failed(f"{step_name}: missing gripper joints: {missing}")
                return
            start_finger_positions = [
                float(current_by_name[name]) for name in observed_finger_joints
            ]

            start_point = JointTrajectoryPoint()
            start_point.positions = list(start_finger_positions)
            start_point.time_from_start = Duration(sec=0, nanosec=100_000_000)

            duration_s = self.gripper_motion_duration_s
            seconds = int(duration_s)
            target_point = JointTrajectoryPoint()
            target_point.positions = [finger_position, finger_position]
            target_point.time_from_start = Duration(
                sec=seconds,
                nanosec=int((duration_s - seconds) * 1_000_000_000),
            )

            goal = FollowJointTrajectory.Goal()
            # Gazebo's default physics engine does not create URDF mimic
            # constraints, so command both physical finger joints explicitly.
            goal.trajectory.joint_names = list(observed_finger_joints)
            goal.trajectory.points = [start_point, target_point]
            self.get_logger().info(
                f"Executing {step_name} through {self.gripper_action_name}: "
                f"finger_position={finger_position:.3f}."
            )
            future = self.gripper_trajectory_client.send_goal_async(goal)
            future.add_done_callback(
                lambda done_future: self.on_gripper_trajectory_goal_response(
                    done_future,
                    step_name,
                    finger_position,
                    start_finger_positions,
                    pose_key,
                    next_callback,
                )
            )

        def on_gripper_trajectory_goal_response(
            self,
            future,
            step_name: str,
            target_position: float,
            start_finger_positions: List[float],
            pose_key: Optional[str],
            next_callback,
        ) -> None:
            goal_handle = future.result()
            if not goal_handle.accepted:
                if (
                    self.gripper_goal_rejection_retry_count
                    < self.gripper_goal_rejection_max_retries
                ):
                    self.gripper_goal_rejection_retry_count += 1
                    self.gripper_goal_rejection_retry_total += 1
                    reason = (
                        "gripper trajectory goal rejected; retry="
                        f"{self.gripper_goal_rejection_retry_count}/"
                        f"{self.gripper_goal_rejection_max_retries}"
                    )
                    self.record_step(f"{step_name}_goal_rejected_retry", False, reason)
                    self.get_logger().warn(
                        "Gripper controller rejected a goal during activation; "
                        f"retrying in {self.gripper_goal_rejection_retry_delay_s:.1f}s."
                    )
                    self.schedule_gripper_trajectory_retry(
                        step_name,
                        target_position,
                        pose_key,
                        next_callback,
                    )
                    return
                self.record_step(step_name, False, "gripper trajectory goal rejected")
                self.finish_failed(f"{step_name}: gripper trajectory goal rejected")
                return
            self.gripper_goal_rejection_retry_count = 0
            goal_handle.get_result_async().add_done_callback(
                lambda done_future: self.on_gripper_trajectory_result(
                    done_future,
                    step_name,
                    target_position,
                    start_finger_positions,
                    pose_key,
                    next_callback,
                )
            )

        def schedule_gripper_trajectory_retry(
            self,
            step_name: str,
            target_position: float,
            pose_key: Optional[str],
            next_callback,
        ) -> None:
            if self.gripper_goal_rejection_retry_timer is not None:
                self.gripper_goal_rejection_retry_timer.cancel()

            def retry() -> None:
                if self.gripper_goal_rejection_retry_timer is not None:
                    self.gripper_goal_rejection_retry_timer.cancel()
                    self.gripper_goal_rejection_retry_timer = None
                self.run_gripper_trajectory_step(
                    step_name,
                    target_position,
                    pose_key,
                    next_callback,
                )

            self.gripper_goal_rejection_retry_timer = self.create_timer(
                self.gripper_goal_rejection_retry_delay_s,
                retry,
            )

        def on_gripper_trajectory_result(
            self,
            future,
            step_name: str,
            target_position: float,
            start_finger_positions: List[float],
            pose_key: Optional[str],
            next_callback,
        ) -> None:
            result_wrapper = future.result()
            error_code = int(result_wrapper.result.error_code)
            success = (
                result_wrapper.status == GoalStatus.STATUS_SUCCEEDED
                and error_code == FollowJointTrajectory.Result.SUCCESSFUL
            )
            if not success:
                current_by_name = {}
                if self.latest_joint_state is not None:
                    current_by_name = dict(
                        zip(self.latest_joint_state.name, self.latest_joint_state.position)
                    )
                finger_positions = [
                    float(current_by_name[name])
                    for name in ("panda_finger_joint1", "panda_finger_joint2")
                    if name in current_by_name
                ]
                if (
                    step_name == "gripper_closed"
                    and self.candidate is not None
                    and isinstance(self.candidate.get("grasp_pose"), dict)
                ):
                    grasp_pose = self.candidate["grasp_pose"]
                    orientation = grasp_pose.get("orientation")
                    if isinstance(orientation, dict):
                        self.pending_finger_center_correction = (
                            finger_asymmetry_correction(
                                finger_positions,
                                orientation,
                                self.grasp_finger_asymmetry_correction_gain,
                                self.grasp_finger_asymmetry_min_m,
                                self.grasp_finger_max_correction_m,
                            )
                        )
                endpoint_outcome = None
                contact_outcome = None
                if error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED:
                    endpoint_outcome = measured_gripper_endpoint_outcome(
                        target_position,
                        finger_positions,
                        self.gripper_endpoint_acceptance_tolerance_m,
                    )
                    contact_outcome = contact_limited_gripper_outcome(
                        step_name,
                        pose_key,
                        finger_positions,
                    )
                if endpoint_outcome is not None:
                    self.record_step(step_name, True, endpoint_outcome)
                    self.publish_demo_state(step_name, pose_key, "measured_endpoint")
                    next_callback()
                    return
                if contact_outcome is not None:
                    self.record_step(step_name, True, contact_outcome)
                    self.publish_demo_state(step_name, pose_key, "contact_limited")
                    next_callback()
                    return
                finger_debug = ""
                if finger_positions:
                    position_text = ", ".join(
                        f"{position:.4f}" for position in finger_positions
                    )
                    opening = sum(finger_positions)
                    finger_debug = (
                        f", fingers=[{position_text}], opening={opening:.4f}m"
                    )
                reason = (
                    f"gripper_status={result_wrapper.status}, "
                    f"follow_joint_trajectory_error={error_code}, "
                    f"message={result_wrapper.result.error_string}"
                    f"{finger_debug}"
                )
                self.record_step(step_name, False, reason)
                close_timed_out_without_motion = (
                    error_code
                    == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                    and gripper_close_stalled(
                        step_name,
                        target_position,
                        start_finger_positions,
                        finger_positions,
                        self.gripper_close_min_movement_m,
                    )
                )
                if (
                    close_timed_out_without_motion
                    and self.gripper_close_in_place_retry_count
                    < self.gripper_close_in_place_max_retries
                ):
                    self.gripper_close_in_place_retry_count += 1
                    self.gripper_close_in_place_retry_total += 1
                    retry_reason = (
                        "close command produced no measurable finger motion; "
                        f"retry={self.gripper_close_in_place_retry_count}/"
                        f"{self.gripper_close_in_place_max_retries}"
                    )
                    self.record_step(
                        "gripper_closed_in_place_retry",
                        True,
                        retry_reason,
                    )
                    self.get_logger().warn(
                        "Gripper close timed out without finger motion; "
                        "retrying at the current grasp pose."
                    )
                    self.run_gripper_step(
                        step_name,
                        target_position,
                        pose_key,
                        next_callback,
                    )
                    return
                if (
                    step_name == "gripper_open"
                    and pose_key is None
                    and self.gripper_free_motion_retry_count
                    < self.gripper_free_motion_max_retries
                ):
                    self.gripper_free_motion_retry_count += 1
                    self.get_logger().warn(
                        "Retrying free-space gripper open "
                        f"({self.gripper_free_motion_retry_count}/"
                        f"{self.gripper_free_motion_max_retries})."
                    )
                    self.run_gripper_step(
                        step_name,
                        self.gripper_open_position,
                        pose_key,
                        next_callback,
                    )
                    return
                if (
                    step_name == "gripper_closed"
                    and self.enable_physical_grasp_verification
                    and self.grasp_probe_retry_count < self.grasp_probe_max_retries
                ):
                    self.grasp_probe_retry_count += 1
                    self.run_gripper_step(
                        "gripper_recovery_open",
                        self.gripper_open_position,
                        None,
                        self.after_grasp_recovery_open,
                    )
                    return
                self.finish_failed(f"{step_name}: {reason}")
                return
            current_by_name = {}
            if self.latest_joint_state is not None:
                current_by_name = dict(
                    zip(self.latest_joint_state.name, self.latest_joint_state.position)
                )
            finger_positions = [
                float(current_by_name[name])
                for name in ("panda_finger_joint1", "panda_finger_joint2")
                if name in current_by_name
            ]
            outcome = "follow_joint_trajectory_executed"
            if len(finger_positions) == 2:
                outcome += (
                    ", fingers=["
                    + ", ".join(f"{position:.4f}" for position in finger_positions)
                    + f"], opening={sum(finger_positions):.4f}m"
                )
            self.record_step(step_name, True, outcome)
            self.publish_demo_state(step_name, pose_key, "executed")
            next_callback()

        def on_gripper_goal_response(
            self,
            future,
            step_name: str,
            pose_key: Optional[str],
            next_callback,
        ) -> None:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.record_step(step_name, False, "gripper goal rejected")
                self.finish_failed(f"{step_name}: gripper goal rejected")
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda done_future: self.on_gripper_result(
                    done_future,
                    step_name,
                    pose_key,
                    next_callback,
                )
            )

        def on_gripper_result(
            self,
            future,
            step_name: str,
            pose_key: Optional[str],
            next_callback,
        ) -> None:
            result_wrapper = future.result()
            error_code = int(result_wrapper.result.error_code.val)
            success = result_wrapper.status == GoalStatus.STATUS_SUCCEEDED and error_code == 1
            if not success:
                failure = f"gripper_status={result_wrapper.status}, moveit_error_code={error_code}"
                self.record_step(step_name, False, failure)
                self.finish_failed(f"{step_name}: {failure}")
                return
            self.record_step(step_name, True, "move_group_executed")
            self.publish_demo_state(step_name, pose_key, "executed")
            next_callback()

        def make_gripper_motion_plan_request(self, finger_position: float) -> MotionPlanRequest:
            request = MotionPlanRequest()
            request.group_name = self.gripper_group_name
            request.start_state.is_diff = True
            request.num_planning_attempts = self.gripper_planning_attempts
            request.allowed_planning_time = self.gripper_allowed_planning_time
            request.max_velocity_scaling_factor = 0.2
            request.max_acceleration_scaling_factor = 0.2

            constraints = Constraints()
            constraints.name = f"{self.gripper_group_name}_joint_goal"
            for joint_name in ("panda_finger_joint1", "panda_finger_joint2"):
                joint_constraint = JointConstraint()
                joint_constraint.joint_name = joint_name
                joint_constraint.position = finger_position
                joint_constraint.tolerance_above = 0.002
                joint_constraint.tolerance_below = 0.002
                joint_constraint.weight = 1.0
                constraints.joint_constraints.append(joint_constraint)
            request.goal_constraints = [constraints]
            return request

        def make_gripper_planning_options(self) -> PlanningOptions:
            options = PlanningOptions()
            options.plan_only = False
            options.look_around = False
            options.replan = False
            return options

        def finish_success(self) -> None:
            self.tactile_release_verified = False
            self.tactile_release_retry_count = 0
            self.release_reference_target_pose = self.latest_physical_target_pose
            self.run_gripper_step(
                "gripper_release",
                self.gripper_release_position,
                "place_pose",
                self.after_gripper_release_stage,
            )

        def after_gripper_release_stage(self) -> None:
            if self.gripper_release_settle_s <= 0.0:
                self.complete_release_open()
                return
            if self.gripper_release_settle_timer is not None:
                self.gripper_release_settle_timer.cancel()
            self.gripper_release_settle_timer = self.create_timer(
                self.gripper_release_settle_s,
                self.complete_release_open,
            )

        def complete_release_open(self) -> None:
            if self.gripper_release_settle_timer is not None:
                self.gripper_release_settle_timer.cancel()
                self.gripper_release_settle_timer = None
            if self.enable_tactile_grasp_supervision:
                self.last_target_contact_ns = {"left": None, "right": None}
                self.latest_tactile_target_collisions = {"left": [], "right": []}
            self.run_gripper_step(
                "gripper_open",
                self.gripper_open_position,
                "place_pose",
                self.after_release_gripper_open,
            )

        def after_release_gripper_open(self) -> None:
            self.publish_demo_state("object_released", "place_pose", "visualized")
            if self.release_settle_s > 0.0:
                if self.release_settle_timer is not None:
                    self.release_settle_timer.cancel()
                self.release_settle_timer = self.create_timer(
                    self.release_settle_s,
                    self.after_release_settle,
                )
                return
            self.after_release_settle()

        def after_release_settle(self) -> None:
            if self.release_settle_timer is not None:
                self.release_settle_timer.cancel()
                self.release_settle_timer = None
            if self.enable_tactile_grasp_supervision:
                released, contact_reason = self.tactile_release_status()
                self.record_step("tactile_release_check", released, contact_reason)
                if not released:
                    if (
                        self.tactile_release_retry_count
                        >= self.tactile_release_max_retries
                    ):
                        self.finish_failed(
                            "tactile_release_check failed after retries: "
                            f"{contact_reason}"
                        )
                        return
                    self.tactile_release_retry_count += 1
                    self.last_target_contact_ns = {"left": None, "right": None}
                    self.latest_tactile_target_collisions = {
                        "left": [],
                        "right": [],
                    }
                    self.run_gripper_step(
                        "gripper_open",
                        self.gripper_open_position,
                        "place_pose",
                        self.after_release_gripper_open,
                    )
                    return
                self.tactile_release_verified = True
            if not self.release_object_motion_is_safe():
                return
            if self.candidate is None:
                self.finish_failed("release retreat: candidate missing")
                return
            clearance_pose = self.current_release_clearance_pose()
            if clearance_pose is None:
                self.finish_failed("release retreat: current panda_hand transform unavailable")
                return
            self.candidate["release_clearance_pose"] = clearance_pose
            self.plan_cartesian_step(
                "cartesian_retreat",
                "release_clearance_pose",
                self.after_release_retreat,
            )

        def tactile_release_status(self) -> Tuple[bool, str]:
            now_ns = self.get_clock().now().nanoseconds
            maximum_age_ns = int(
                self.tactile_contact_max_age_s * 1_000_000_000
            )
            active_sides = []
            details = []
            for side in ("left", "right"):
                timestamp = self.last_target_contact_ns.get(side)
                recent = (
                    timestamp is not None
                    and 0 <= now_ns - timestamp <= maximum_age_ns
                )
                if recent:
                    active_sides.append(side)
                age_text = "never"
                if timestamp is not None:
                    age_text = f"{max(0, now_ns - timestamp) / 1e9:.3f}s"
                details.append(
                    f"{side}=recent_target_contact:{recent},age:{age_text}"
                )
            return not active_sides, "; ".join(details)

        def release_object_motion_is_safe(self) -> bool:
            reference = self.release_reference_target_pose
            settled = self.latest_physical_target_pose
            if not self.enable_physical_grasp_verification:
                return True
            if reference is None or settled is None:
                self.record_step(
                    "release_object_motion_check",
                    False,
                    "target pose unavailable",
                )
                self.finish_failed("release_object_motion_check: target pose unavailable")
                return False
            horizontal, vertical, total = release_object_motion(reference, settled)
            self.release_object_horizontal_motion_m = horizontal
            self.release_object_vertical_motion_m = vertical
            self.release_object_motion_m = total
            success = (
                horizontal <= self.release_object_max_horizontal_motion_m
                and vertical <= self.release_object_max_vertical_motion_m
            )
            reason = (
                f"horizontal={horizontal:.4f}m, vertical={vertical:.4f}m, "
                f"total={total:.4f}m, limits=["
                f"{self.release_object_max_horizontal_motion_m:.4f}, "
                f"{self.release_object_max_vertical_motion_m:.4f}]m"
            )
            self.record_step("release_object_motion_check", success, reason)
            if not success:
                self.finish_failed(f"release_object_motion_check failed: {reason}")
            return success

        def after_release_retreat(self) -> None:
            if not self.return_home_after_success:
                self.publish_result("SUCCESS", "")
                return
            # The physical cube can settle away from the nominal place pose.
            # After a collision-checked vertical retreat, remove that stale
            # nominal PlanningScene copy before planning the return motion.
            self.publish_demo_state(
                "object_collision_cleared",
                "release_clearance_pose",
                "visualized",
            )
            if self.return_home_scene_settle_s <= 0.0:
                self.plan_return_home()
                return
            if self.return_home_scene_settle_timer is not None:
                self.return_home_scene_settle_timer.cancel()
            self.return_home_scene_settle_timer = self.create_timer(
                self.return_home_scene_settle_s,
                self.after_return_home_scene_settle,
            )

        def plan_bounded_release_retreat(self, pose_key: str, next_callback) -> None:
            if self.candidate is None or not isinstance(self.candidate.get(pose_key), dict):
                self.finish_failed("ompl_retreat_fallback: clearance pose missing")
                return
            goal = MoveGroup.Goal()
            goal.request = self.make_motion_plan_request(
                self.to_pose_stamped(self.candidate[pose_key]),
                self.current_start_state,
                constrain_orientation=True,
                planner_id=self.planner_ids[0],
            )
            # Always inspect this contact-zone trajectory before execution.
            goal.planning_options = self.make_planning_options()
            self.get_logger().info(
                "Planning bounded ompl_retreat_fallback to release clearance pose."
            )
            future = self.move_client.send_goal_async(goal)
            future.add_done_callback(
                lambda done_future: self.on_bounded_release_retreat_goal(
                    done_future, pose_key, next_callback
                )
            )

        def on_bounded_release_retreat_goal(self, future, pose_key: str, next_callback) -> None:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.record_step("ompl_retreat_fallback", False, "goal rejected")
                self.finish_failed("ompl_retreat_fallback: goal rejected")
                return
            goal_handle.get_result_async().add_done_callback(
                lambda done_future: self.on_bounded_release_retreat_result(
                    done_future, pose_key, next_callback
                )
            )

        def on_bounded_release_retreat_result(
            self,
            future,
            pose_key: str,
            next_callback,
        ) -> None:
            result_wrapper = future.result()
            error_code = int(result_wrapper.result.error_code.val)
            success = result_wrapper.status == GoalStatus.STATUS_SUCCEEDED and error_code == 1
            if not success:
                reason = (
                    f"planner_id={self.planner_ids[0]}, action_status={result_wrapper.status}, "
                    f"moveit_error_code={error_code}"
                )
                self.record_step("ompl_retreat_fallback", False, reason)
                self.finish_failed(f"ompl_retreat_fallback: {reason}")
                return
            trajectory = result_wrapper.result.planned_trajectory
            position_rows = [
                [float(value) for value in point.positions]
                for point in trajectory.joint_trajectory.points
            ]
            path_length = joint_path_length(position_rows)
            if path_length > self.release_retreat_max_joint_path_length:
                reason = (
                    f"joint_path_length={path_length:.3f}rad exceeds safe limit "
                    f"{self.release_retreat_max_joint_path_length:.3f}rad"
                )
                self.record_step("ompl_retreat_fallback", False, reason)
                self.finish_failed(f"ompl_retreat_fallback: {reason}")
                return
            self.ompl_metrics.append(
                {
                    "step": "ompl_retreat_fallback",
                    "planner_id": self.planner_ids[0],
                    "planning_time_s": float(
                        getattr(result_wrapper.result, "planning_time", 0.0)
                    ),
                    "joint_path_length_rad": path_length,
                    "trajectory_points": len(position_rows),
                }
            )
            self.current_start_state = self.end_state_from_trajectory(trajectory)
            if self.current_start_state is None:
                self.record_step(
                    "ompl_retreat_fallback", False, "planned trajectory has no joint endpoint"
                )
                self.finish_failed(
                    "ompl_retreat_fallback: planned trajectory has no joint endpoint"
                )
                return
            self.handle_successful_trajectory(
                "ompl_retreat_fallback",
                pose_key,
                trajectory,
                f"joint_path_length={path_length:.3f}rad within safe limit",
                next_callback,
                getattr(result_wrapper.result, "trajectory_start", None),
            )

        def after_return_home_scene_settle(self) -> None:
            if self.return_home_scene_settle_timer is not None:
                self.return_home_scene_settle_timer.cancel()
                self.return_home_scene_settle_timer = None
            self.plan_return_home()

        def current_release_clearance_pose(self) -> Optional[Dict[str, object]]:
            if self.candidate is None:
                return None
            pre_place_pose = self.candidate.get("pre_place_pose")
            if isinstance(pre_place_pose, dict):
                # Reverse the known collision-checked place descent. This avoids
                # solving a fresh IK branch beside the newly released object.
                place_pose = self.candidate.get("place_pose")
                extra_clearance = 0.0
                if isinstance(place_pose, dict):
                    extra_clearance = max(
                        0.0,
                        float(place_pose["position"]["z"])
                        + self.release_clearance_height
                        - float(pre_place_pose["position"]["z"]),
                    )
                return translated_pose_dict(
                    pre_place_pose,
                    (0.0, 0.0, extra_clearance),
                )
            place_pose = self.candidate.get("place_pose")
            if not isinstance(place_pose, dict):
                return None
            frame_id = str(place_pose["frame_id"])
            try:
                transform = self.tf_buffer.lookup_transform(
                    frame_id,
                    self.end_effector_link,
                    Time(),
                )
            except TransformException as exc:
                self.get_logger().error(f"Release clearance TF lookup failed: {exc}")
                return None
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            return {
                "frame_id": frame_id,
                "position": {
                    "x": float(translation.x),
                    "y": float(translation.y),
                    "z": float(translation.z) + max(0.05, self.release_clearance_height),
                },
                "orientation": {
                    "x": float(rotation.x),
                    "y": float(rotation.y),
                    "z": float(rotation.z),
                    "w": float(rotation.w),
                },
            }

        def plan_return_home(self) -> None:
            goal = MoveGroup.Goal()
            goal.request = self.make_home_motion_plan_request()
            goal.planning_options = self.make_planning_options()
            self.get_logger().info("Planning collision-aware return_home after release clearance.")
            future = self.move_client.send_goal_async(goal)
            future.add_done_callback(self.on_return_home_goal_response)

        def on_return_home_goal_response(self, future) -> None:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.retry_or_fail_return_home("goal rejected")
                return
            goal_handle.get_result_async().add_done_callback(self.on_return_home_result)

        def on_return_home_result(self, future) -> None:
            result_wrapper = future.result()
            error_code = int(result_wrapper.result.error_code.val)
            success = result_wrapper.status == GoalStatus.STATUS_SUCCEEDED and error_code == 1
            if not success:
                reason = f"action_status={result_wrapper.status}, moveit_error_code={error_code}"
                self.retry_or_fail_return_home(reason)
                return
            trajectory = result_wrapper.result.planned_trajectory
            position_rows = [
                [float(value) for value in point.positions]
                for point in trajectory.joint_trajectory.points
            ]
            self.ompl_metrics.append(
                {
                    "step": "return_home",
                    "planner_id": self.planner_ids[0],
                    "planning_time_s": float(
                        getattr(result_wrapper.result, "planning_time", 0.0)
                    ),
                    "joint_path_length_rad": joint_path_length(position_rows),
                    "trajectory_points": len(position_rows),
                }
            )
            self.current_start_state = self.end_state_from_trajectory(trajectory)
            if self.current_start_state is None:
                self.record_step("return_home", False, "planned trajectory has no joint endpoint")
                self.finish_failed("return_home: planned trajectory has no joint endpoint")
                return
            self.handle_successful_trajectory(
                "return_home",
                None,
                trajectory,
                "",
                self.after_return_home,
                getattr(result_wrapper.result, "trajectory_start", None),
            )

        def retry_or_fail_return_home(self, reason: str) -> None:
            # The MoveGroup goal above is plan-only. If planning or final path
            # validation rejects it, the robot has not moved and replanning from
            # the same measured state is safe. Execution failures are handled by
            # handle_successful_trajectory and deliberately do not enter here.
            if self.return_home_retry_count < self.return_home_max_retries:
                self.return_home_retry_count += 1
                retry_reason = (
                    f"{reason}, retry={self.return_home_retry_count}/"
                    f"{self.return_home_max_retries}"
                )
                self.record_step("return_home_replan", False, retry_reason)
                self.get_logger().warn(
                    "return_home planning was rejected before execution; "
                    f"replanning from the unchanged state "
                    f"({self.return_home_retry_count}/"
                    f"{self.return_home_max_retries})."
                )
                self.plan_return_home()
                return
            self.record_step("return_home", False, reason)
            self.finish_failed(f"return_home: {reason}")

        def after_return_home(self) -> None:
            self.publish_result("SUCCESS", "")

        def finish_failed(self, reason: str) -> None:
            if self.servo_descent_active:
                self.stop_servo_descent_output()
                if self.servo_pause_client.service_is_ready():
                    request = SetBool.Request()
                    request.data = True
                    self.servo_pause_client.call_async(request)
                self.servo_descent_active = False
            self.publish_result("FAILED", reason)

        def make_motion_plan_request(
            self,
            target_pose: PoseStamped,
            start_state: Optional[RobotState],
            constrain_orientation: bool,
            planner_id: str,
            orientation_tolerance: Optional[float] = None,
        ) -> MotionPlanRequest:
            request = MotionPlanRequest()
            request.group_name = self.group_name
            if start_state is not None:
                request.start_state = start_state
            else:
                request.start_state.is_diff = True
            request.num_planning_attempts = self.planning_attempts
            request.allowed_planning_time = self.allowed_planning_time
            request.max_velocity_scaling_factor = (
                self.ompl_velocity_scaling_factor
            )
            request.max_acceleration_scaling_factor = (
                self.ompl_velocity_scaling_factor
            )
            request.planner_id = planner_id
            request.goal_constraints = [
                self.make_pose_constraint(
                    target_pose,
                    constrain_orientation,
                    orientation_tolerance,
                )
            ]
            return request

        def make_joint_motion_plan_request(
            self,
            joint_positions: List[float],
            start_state: Optional[RobotState],
            planner_id: str,
            tolerance_rad: float,
        ) -> MotionPlanRequest:
            request = MotionPlanRequest()
            request.group_name = self.group_name
            if start_state is not None:
                request.start_state = start_state
            else:
                request.start_state.is_diff = True
            request.num_planning_attempts = self.planning_attempts
            request.allowed_planning_time = self.allowed_planning_time
            request.max_velocity_scaling_factor = (
                self.ompl_velocity_scaling_factor
            )
            request.max_acceleration_scaling_factor = (
                self.ompl_velocity_scaling_factor
            )
            request.planner_id = planner_id
            constraints = Constraints()
            constraints.name = "paired_pre_place_joint_replay"
            for joint_name, position in zip(PANDA_ARM_JOINTS, joint_positions):
                joint_constraint = JointConstraint()
                joint_constraint.joint_name = joint_name
                joint_constraint.position = float(position)
                joint_constraint.tolerance_above = tolerance_rad
                joint_constraint.tolerance_below = tolerance_rad
                joint_constraint.weight = 1.0
                constraints.joint_constraints.append(joint_constraint)
            request.goal_constraints = [constraints]
            return request

        def make_home_motion_plan_request(self) -> MotionPlanRequest:
            request = MotionPlanRequest()
            request.group_name = self.group_name
            request.start_state.is_diff = True
            request.num_planning_attempts = self.planning_attempts
            request.allowed_planning_time = self.allowed_planning_time
            request.max_velocity_scaling_factor = (
                self.ompl_velocity_scaling_factor
            )
            request.max_acceleration_scaling_factor = (
                self.ompl_velocity_scaling_factor
            )
            request.planner_id = self.planner_ids[0]

            home_state = self.fixed_home_state()
            constraints = Constraints()
            constraints.name = "panda_home_joint_goal"
            for joint_name, position in zip(
                home_state.joint_state.name,
                home_state.joint_state.position,
            ):
                joint_constraint = JointConstraint()
                joint_constraint.joint_name = joint_name
                joint_constraint.position = float(position)
                joint_constraint.tolerance_above = 0.01
                joint_constraint.tolerance_below = 0.01
                joint_constraint.weight = 1.0
                constraints.joint_constraints.append(joint_constraint)
            request.goal_constraints = [constraints]
            return request

        def make_pose_constraint(
            self,
            target_pose: PoseStamped,
            constrain_orientation: bool,
            orientation_tolerance: Optional[float] = None,
        ) -> Constraints:
            constraints = Constraints()
            constraints.name = f"{self.end_effector_link}_pose_goal"

            sphere = SolidPrimitive()
            sphere.type = SolidPrimitive.SPHERE
            sphere.dimensions = [self.position_tolerance]

            position_constraint = PositionConstraint()
            position_constraint.header = target_pose.header
            position_constraint.link_name = self.end_effector_link
            position_constraint.constraint_region.primitives.append(sphere)
            position_constraint.constraint_region.primitive_poses.append(target_pose.pose)
            position_constraint.weight = 1.0

            constraints.position_constraints.append(position_constraint)
            if constrain_orientation:
                tolerance = (
                    self.pre_grasp_orientation_tolerance
                    if orientation_tolerance is None
                    else max(0.001, float(orientation_tolerance))
                )
                orientation_constraint = OrientationConstraint()
                orientation_constraint.header = target_pose.header
                orientation_constraint.link_name = self.end_effector_link
                orientation_constraint.orientation = target_pose.pose.orientation
                orientation_constraint.absolute_x_axis_tolerance = tolerance
                orientation_constraint.absolute_y_axis_tolerance = tolerance
                orientation_constraint.absolute_z_axis_tolerance = tolerance
                orientation_constraint.weight = 1.0
                constraints.orientation_constraints.append(orientation_constraint)
            return constraints

        def make_planning_options(self) -> PlanningOptions:
            options = PlanningOptions()
            options.plan_only = True
            options.look_around = False
            options.replan = False
            return options

        def make_ompl_planning_options(self) -> PlanningOptions:
            options = self.make_planning_options()
            if self.ompl_was_executed_by_move_group():
                options.plan_only = False
            return options

        def ompl_was_executed_by_move_group(self) -> bool:
            return (
                self.execute_trajectories
                and not self.visualize_only_execution
                and self.ompl_execution_backend == "move_group"
            )

        def to_pose_stamped(self, pose_data: Dict[str, object]) -> PoseStamped:
            message = PoseStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = str(pose_data["frame_id"])
            message.pose = self.to_pose(pose_data)
            return message

        def to_pose(self, pose_data: Dict[str, object]) -> Pose:
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

        def end_state_from_trajectory(self, trajectory) -> Optional[RobotState]:
            joint_trajectory = trajectory.joint_trajectory
            if not joint_trajectory.points:
                return None
            last_point = joint_trajectory.points[-1]
            state = RobotState()
            state.joint_state.header.stamp = self.get_clock().now().to_msg()
            state.joint_state.name = list(joint_trajectory.joint_names)
            state.joint_state.position = list(last_point.positions)
            # A trajectory endpoint does not describe attached collision bodies.
            state.is_diff = True
            return state

        def fixed_home_state(self) -> RobotState:
            state = RobotState()
            state.joint_state.header.stamp = self.get_clock().now().to_msg()
            state.joint_state.name = [
                "panda_joint1",
                "panda_joint2",
                "panda_joint3",
                "panda_joint4",
                "panda_joint5",
                "panda_joint6",
                "panda_joint7",
            ]
            state.joint_state.position = [
                0.0,
                -0.785,
                0.0,
                -2.356,
                0.0,
                1.571,
                0.785,
            ]
            # Home joints are a joint-only override, not an empty-payload scene.
            state.is_diff = True
            return state

        def publish_display_trajectory(self, start_state: RobotState, trajectory) -> None:
            message = DisplayTrajectory()
            message.trajectory_start = start_state
            message.trajectory.append(trajectory)
            self.display_publisher.publish(message)

        def on_dynamic_replan_request(self, message: String) -> None:
            if not self.enable_dynamic_replanning or self.active_execute_goal_handle is None:
                return
            try:
                payload = json.loads(message.data)
            except json.JSONDecodeError:
                return
            if (
                payload.get("event") != "trajectory_invalidated"
                or payload.get("step") != self.active_execution_step
                or self.dynamic_replan_requested
            ):
                return
            self.dynamic_replan_requested = True
            event = dict(payload)
            event["received_time_ns"] = self.get_clock().now().nanoseconds
            self.dynamic_replan_events.append(event)
            self.get_logger().warn(
                f"Cancelling unsafe execution {self.active_execution_step}; "
                f"dynamic replan {self.dynamic_replan_count + 1}/"
                f"{self.maximum_dynamic_replans}."
            )
            self.publish_demo_state(
                str(self.active_execution_step),
                None,
                "canceling_for_dynamic_obstacle",
            )
            stop_message = String()
            stop_message.data = "stop"
            self.trajectory_execution_event_publisher.publish(stop_message)
            cancel_future = self.active_execute_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.on_dynamic_cancel_response)

        def on_dynamic_cancel_response(self, future) -> None:
            now_ns = self.get_clock().now().nanoseconds
            try:
                response = future.result()
                accepted = bool(response.goals_canceling)
            except Exception as exc:
                self.get_logger().warn(f"Dynamic execution cancellation failed: {exc}")
                return
            if self.dynamic_replan_events:
                event = self.dynamic_replan_events[-1]
                event["cancel_accepted"] = accepted
                event["cancel_response_time_ns"] = now_ns
                received_ns = int(event.get("received_time_ns", now_ns))
                event["cancel_ack_latency_s"] = (now_ns - received_ns) / 1e9
            self.get_logger().warn(
                "Dynamic execution cancellation "
                + ("accepted." if accepted else "was not accepted; stop event published.")
            )

        def handle_successful_trajectory(
            self,
            step_name: str,
            pose_key: Optional[str],
            trajectory,
            reason: str,
            next_callback,
            display_start_state: Optional[RobotState] = None,
            replan_callback=None,
        ) -> None:
            if not self.execute_trajectories or self.visualize_only_execution:
                if display_start_state is not None:
                    self.publish_display_trajectory(display_start_state, trajectory)
                self.record_step(step_name, True, reason)
                self.publish_demo_state(step_name, pose_key, "visualized")
                next_callback()
                return

            start_check = self.execution_start_check(trajectory)
            self.last_execution_start_check = str(start_check["reason"])
            if not start_check["success"]:
                failure = str(start_check["reason"])
                self.record_step(step_name, False, failure)
                self.finish_failed(f"{step_name}: {failure}")
                return
            if start_check["reason"]:
                self.get_logger().info(str(start_check["reason"]))

            goal = ExecuteTrajectory.Goal()
            goal.trajectory = trajectory
            self.get_logger().info(f"Executing {step_name}.")
            self.publish_demo_state(step_name, pose_key, "executing")
            if display_start_state is not None:
                self.publish_display_trajectory(display_start_state, trajectory)
            future = self.execute_client.send_goal_async(goal)
            future.add_done_callback(
                lambda done_future: self.on_execute_goal_response(
                    done_future,
                    step_name,
                    pose_key,
                    reason,
                    next_callback,
                    replan_callback,
                    trajectory,
                )
            )

        def on_execute_goal_response(
            self,
            future,
            step_name: str,
            pose_key: Optional[str],
            reason: str,
            next_callback,
            replan_callback,
            trajectory,
        ) -> None:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.record_step(step_name, False, "execution goal rejected")
                self.finish_failed(f"{step_name}: execution goal rejected")
                return
            self.active_execute_goal_handle = goal_handle
            self.active_execution_step = step_name
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda done_future: self.on_execute_result(
                    done_future,
                    step_name,
                    pose_key,
                    reason,
                    next_callback,
                    replan_callback,
                    trajectory,
                )
            )

        def on_execute_result(
            self,
            future,
            step_name: str,
            pose_key: Optional[str],
            reason: str,
            next_callback,
            replan_callback,
            trajectory,
        ) -> None:
            if not self.active:
                return
            result_wrapper = future.result()
            error_code = int(result_wrapper.result.error_code.val)
            success = result_wrapper.status == GoalStatus.STATUS_SUCCEEDED and error_code == 1
            self.active_execute_goal_handle = None
            self.active_execution_step = None
            if not success:
                failure = f"execution_status={result_wrapper.status}, moveit_error_code={error_code}"
                if (
                    self.dynamic_replan_requested
                    and replan_callback is not None
                    and self.dynamic_replan_count < self.maximum_dynamic_replans
                ):
                    self.dynamic_replan_count += 1
                    self.dynamic_replan_requested = False
                    self.current_start_state = None
                    retry_reason = (
                        f"{failure}, cancelled for sensed obstacle, "
                        f"replan={self.dynamic_replan_count}/"
                        f"{self.maximum_dynamic_replans}"
                    )
                    self.record_step(f"{step_name}_dynamic_replan", False, retry_reason)
                    self.get_logger().warn(
                        f"{step_name} stopped at the measured state; replanning "
                        "against the updated PlanningScene."
                    )
                    self.schedule_dynamic_replan(replan_callback)
                    return
                if self.should_replan_cartesian_execution(
                    step_name,
                    error_code,
                    replan_callback,
                ):
                    attempt = self.cartesian_execution_replan_counts.get(
                        step_name,
                        0,
                    ) + 1
                    self.cartesian_execution_replan_counts[step_name] = attempt
                    self.current_start_state = None
                    retry_reason = (
                        f"{failure}, measured-state residual replan="
                        f"{attempt}/{self.cartesian_execution_replan_max_retries}"
                    )
                    self.record_step(
                        f"{step_name}_execution_replan",
                        False,
                        retry_reason,
                    )
                    self.get_logger().warn(
                        f"{step_name} missed the controller endpoint tolerance; "
                        "replanning the collision-checked residual motion from "
                        "measured joints."
                    )
                    self.schedule_cartesian_execution_replan(replan_callback)
                    return
                endpoint_outcome = self.arm_endpoint_outcome_after_failure(
                    step_name,
                    error_code,
                    trajectory,
                )
                if endpoint_outcome is not None:
                    self.record_step(step_name, True, endpoint_outcome)
                    self.publish_demo_state(
                        step_name,
                        pose_key,
                        "measured_endpoint",
                    )
                    next_callback()
                    return
                if self.should_replan_payload_transfer_execution(
                    step_name,
                    pose_key,
                    replan_callback,
                ):
                    self.payload_transfer_execution_replan_count += 1
                    self.current_start_state = None
                    retry_reason = (
                        f"{failure}, measured-state replan="
                        f"{self.payload_transfer_execution_replan_count}/"
                        f"{self.payload_transfer_execution_replan_max_retries}"
                    )
                    self.record_step(
                        f"{step_name}_execution_replan",
                        False,
                        retry_reason,
                    )
                    self.get_logger().warn(
                        f"{step_name} stopped before the pre-place waypoint; "
                        "the payload is still secure, so the remaining transfer "
                        "will be replanned from measured joints."
                    )
                    self.schedule_payload_transfer_execution_replan(
                        replan_callback
                    )
                    return
                if not self.active:
                    return
                payload_endpoint_outcome = (
                    self.payload_endpoint_outcome_after_failure(
                        step_name,
                        pose_key,
                        error_code,
                        trajectory,
                    )
                )
                if not self.active:
                    return
                if payload_endpoint_outcome is not None:
                    self.record_step(
                        step_name,
                        True,
                        payload_endpoint_outcome,
                    )
                    self.publish_demo_state(
                        step_name,
                        pose_key,
                        "measured_payload_endpoint",
                    )
                    next_callback()
                    return
                if self.try_pre_place_transfer_ompl_fallback(
                    step_name,
                    pose_key,
                ):
                    return
                if not self.active:
                    return
                if self.last_execution_start_check:
                    failure = f"{failure}, {self.last_execution_start_check}"
                self.record_step(step_name, False, failure)
                self.finish_failed(f"{step_name}: {failure}")
                return
            if (
                self.dynamic_replan_requested
                and replan_callback is not None
                and self.dynamic_replan_count < self.maximum_dynamic_replans
            ):
                self.dynamic_replan_count += 1
                self.dynamic_replan_requested = False
                self.current_start_state = None
                retry_reason = (
                    "execution completed while obstacle cancellation was in flight, "
                    f"replan={self.dynamic_replan_count}/"
                    f"{self.maximum_dynamic_replans}"
                )
                self.record_step(f"{step_name}_dynamic_replan", False, retry_reason)
                self.get_logger().warn(
                    f"{step_name} completed during cancellation; validating the "
                    "updated scene with a fresh plan."
                )
                self.schedule_dynamic_replan(replan_callback)
                return
            self.dynamic_replan_requested = False
            suffix = "executed"
            if reason:
                suffix = f"{reason}, executed"
            self.record_step(step_name, True, suffix)
            self.publish_demo_state(step_name, pose_key, "executed")
            next_callback()

        def arm_endpoint_outcome_after_failure(
            self,
            step_name: str,
            error_code: int,
            trajectory,
        ) -> Optional[str]:
            if (
                step_name
                not in {
                    "cartesian_approach",
                    "cartesian_grasp_recovery_reapproach",
                }
                or error_code != -4
            ):
                return None
            joint_trajectory = trajectory.joint_trajectory
            if not joint_trajectory.points or self.latest_joint_state is None:
                return None
            names = list(joint_trajectory.joint_names)
            target_positions = [
                float(value)
                for value in joint_trajectory.points[-1].positions
            ]
            current_by_name = dict(
                zip(
                    self.latest_joint_state.name,
                    self.latest_joint_state.position,
                )
            )
            if any(name not in current_by_name for name in names):
                return None
            return measured_arm_endpoint_outcome(
                names,
                target_positions,
                [float(current_by_name[name]) for name in names],
                self.arm_execution_endpoint_acceptance_tolerance_rad,
                self.grasp_candidate_prevalidation_min_joint_limit_margin,
            )

        def should_replan_cartesian_execution(
            self,
            step_name: str,
            error_code: int,
            replan_callback,
        ) -> bool:
            return (
                step_name
                in {
                    "cartesian_pre_grasp_alignment",
                    "cartesian_approach",
                    "cartesian_grasp_recovery_reapproach",
                }
                and error_code == -4
                and replan_callback is not None
                and self.cartesian_execution_replan_counts.get(step_name, 0)
                < self.cartesian_execution_replan_max_retries
            )

        def schedule_cartesian_execution_replan(self, replan_callback) -> None:
            if self.cartesian_execution_replan_settle_s <= 0.0:
                replan_callback()
                return
            if self.cartesian_execution_replan_timer is not None:
                self.cartesian_execution_replan_timer.cancel()

            def invoke() -> None:
                if self.cartesian_execution_replan_timer is not None:
                    self.cartesian_execution_replan_timer.cancel()
                    self.cartesian_execution_replan_timer = None
                if self.active:
                    replan_callback()

            self.cartesian_execution_replan_timer = self.create_timer(
                self.cartesian_execution_replan_settle_s,
                invoke,
            )

        def should_replan_payload_transfer_execution(
            self,
            step_name: str,
            pose_key: Optional[str],
            replan_callback,
        ) -> bool:
            if (
                (
                    step_name not in {
                        "ompl_to_pre_place",
                        "cartesian_to_pre_place",
                    }
                    and not step_name.startswith(
                        "cartesian_pre_place_transfer_stage_"
                    )
                    and not step_name.startswith(
                        "cartesian_place_descent_stage_"
                    )
                )
                or replan_callback is None
                or self.payload_transfer_execution_replan_count
                >= self.payload_transfer_execution_replan_max_retries
            ):
                return False
            if self.enable_tactile_grasp_supervision:
                tactile_success, tactile_reason = self.tactile_contact_status()
                self.record_step(
                    "payload_transfer_replan_tactile_check",
                    tactile_success,
                    tactile_reason,
                )
                if not tactile_success:
                    return False
            hand_pose_key = pose_key or "pre_place_pose"
            tolerance_m = self.payload_transfer_tolerance_m
            if step_name.startswith("cartesian_place_descent_stage_"):
                tolerance_m = self.place_descent_stage_payload_tolerance_m
            return self.verify_payload_after_transfer(
                hand_pose_key,
                tolerance_m=tolerance_m,
                step_name="payload_transfer_replan_rigid_check",
                object_in_hand_pose_override=(
                    self.place_descent_object_in_hand_pose
                    if step_name.startswith("cartesian_place_descent_stage_")
                    else None
                ),
            )

        def payload_endpoint_outcome_after_failure(
            self,
            step_name: str,
            pose_key: Optional[str],
            error_code: int,
            trajectory,
        ) -> Optional[str]:
            if (
                not step_name.startswith("cartesian_place_descent_stage_")
                or pose_key is None
                or error_code != -4
                or self.latest_joint_state is None
                or not self.enable_tactile_grasp_supervision
                or not self.enable_physical_grasp_verification
            ):
                return None
            joint_trajectory = trajectory.joint_trajectory
            if not joint_trajectory.points:
                return None
            names = list(joint_trajectory.joint_names)
            current_by_name = dict(
                zip(
                    self.latest_joint_state.name,
                    self.latest_joint_state.position,
                )
            )
            if any(name not in current_by_name for name in names):
                return None
            current_margin = normalized_joint_limit_margin(
                names,
                [float(current_by_name[name]) for name in names],
            )
            hand_pose = (
                self.candidate.get(pose_key)
                if self.candidate is not None
                else None
            )
            if not isinstance(hand_pose, dict):
                return None
            frame_id = str(hand_pose["frame_id"])
            try:
                transform = self.tf_buffer.lookup_transform(
                    frame_id,
                    self.end_effector_link,
                    Time(),
                )
            except TransformException as exc:
                self.get_logger().warn(
                    f"Payload endpoint hand transform unavailable: {exc}"
                )
                return None
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            target_position = hand_pose["position"]
            target_orientation = hand_pose["orientation"]
            position_error = translation_distance(
                (
                    float(translation.x) - float(target_position["x"]),
                    float(translation.y) - float(target_position["y"]),
                    float(translation.z) - float(target_position["z"]),
                )
            )
            orientation_error = quaternion_angular_distance(
                (
                    float(rotation.x),
                    float(rotation.y),
                    float(rotation.z),
                    float(rotation.w),
                ),
                (
                    float(target_orientation["x"]),
                    float(target_orientation["y"]),
                    float(target_orientation["z"]),
                    float(target_orientation["w"]),
                ),
            )
            task_space_success = (
                position_error <= self.place_descent_stage_payload_tolerance_m
                and orientation_error
                <= self.place_descent_endpoint_orientation_tolerance_rad
                and current_margin
                >= self.grasp_candidate_prevalidation_min_joint_limit_margin
            )
            task_space_reason = (
                f"position_error={position_error:.4f}m, position_tolerance="
                f"{self.place_descent_stage_payload_tolerance_m:.4f}m, "
                f"orientation_error={orientation_error:.4f}rad, "
                "orientation_tolerance="
                f"{self.place_descent_endpoint_orientation_tolerance_rad:.4f}rad, "
                f"joint_limit_margin={current_margin:.4f}"
            )
            self.record_step(
                "payload_endpoint_task_space_check",
                task_space_success,
                task_space_reason,
            )
            if not task_space_success:
                return None
            if self.enable_tactile_grasp_supervision:
                tactile_success, tactile_reason = self.tactile_contact_status()
                self.record_step(
                    "payload_endpoint_tactile_check",
                    tactile_success,
                    tactile_reason,
                )
                if not tactile_success:
                    return None
            if not self.verify_payload_after_transfer(
                pose_key,
                tolerance_m=self.place_descent_stage_payload_tolerance_m,
                step_name="payload_endpoint_rigid_check",
                object_in_hand_pose_override=(
                    self.place_descent_object_in_hand_pose
                ),
            ):
                return None
            return (
                "measured_task_space_endpoint_after_control_failed, "
                + task_space_reason
                + ", tactile_and_rigid_payload_checks_passed"
            )

        def try_pre_place_transfer_ompl_fallback(
            self,
            step_name: str,
            pose_key: Optional[str],
        ) -> bool:
            if (
                not step_name.startswith(
                    "cartesian_pre_place_transfer_stage_"
                )
                or self.pre_place_transfer_ompl_fallback_used
                or pose_key is None
            ):
                return False
            if self.enable_tactile_grasp_supervision:
                tactile_success, tactile_reason = self.tactile_contact_status()
                self.record_step(
                    "pre_place_transfer_fallback_tactile_check",
                    tactile_success,
                    tactile_reason,
                )
                if not tactile_success:
                    return False
            if not self.verify_payload_after_transfer(
                pose_key,
                tolerance_m=self.payload_transfer_tolerance_m,
                step_name="pre_place_transfer_fallback_rigid_check",
            ):
                return False
            self.pre_place_transfer_ompl_fallback_used = True
            self.pre_place_candidate_attempt_count = 0
            self.payload_transfer_execution_replan_count = 0
            self.current_start_state = None
            self.record_step(
                "pre_place_transfer_ompl_fallback",
                False,
                (
                    f"{step_name} exhausted supervised Cartesian execution "
                    "replans; switching once to measured-state OMPL"
                ),
            )
            self.schedule_payload_transfer_execution_replan(
                self.plan_pre_place_candidate
            )
            return True

        def schedule_payload_transfer_execution_replan(
            self,
            replan_callback,
        ) -> None:
            if self.payload_transfer_execution_replan_settle_s <= 0.0:
                replan_callback()
                return
            if self.payload_transfer_execution_replan_timer is not None:
                self.payload_transfer_execution_replan_timer.cancel()

            def invoke() -> None:
                if self.payload_transfer_execution_replan_timer is not None:
                    self.payload_transfer_execution_replan_timer.cancel()
                    self.payload_transfer_execution_replan_timer = None
                if self.active:
                    replan_callback()

            self.payload_transfer_execution_replan_timer = self.create_timer(
                self.payload_transfer_execution_replan_settle_s,
                invoke,
            )

        def schedule_dynamic_replan(self, replan_callback) -> None:
            if self.dynamic_replan_settle_s <= 0.0:
                self.mark_dynamic_replan_started()
                replan_callback()
                return
            if self.dynamic_replan_timer is not None:
                self.dynamic_replan_timer.cancel()

            def invoke() -> None:
                if self.dynamic_replan_timer is not None:
                    self.dynamic_replan_timer.cancel()
                    self.dynamic_replan_timer = None
                self.mark_dynamic_replan_started()
                replan_callback()

            self.get_logger().info(
                f"Waiting {self.dynamic_replan_settle_s:.2f}s for the sensed "
                "PlanningScene to settle before replanning."
            )
            self.dynamic_replan_timer = self.create_timer(
                self.dynamic_replan_settle_s,
                invoke,
            )

        def mark_dynamic_replan_started(self) -> None:
            if not self.dynamic_replan_events:
                return
            now_ns = self.get_clock().now().nanoseconds
            event = self.dynamic_replan_events[-1]
            event["replan_started_time_ns"] = now_ns
            received_ns = int(event.get("received_time_ns", now_ns))
            event["replan_trigger_latency_s"] = (now_ns - received_ns) / 1e9

        def publish_demo_state(self, step_name: str, pose_key: Optional[str], mode: str) -> None:
            message = String()
            message.data = json.dumps(
                {
                    "step": step_name,
                    "pose_key": pose_key,
                    "mode": mode,
                }
            )
            self.demo_state_publisher.publish(message)

        def publish_active_candidate(self) -> None:
            if self.candidate is None:
                return
            message = String()
            message.data = json.dumps(self.candidate)
            self.active_candidate_publisher.publish(message)

        def record_step(self, step_name: str, success: bool, reason: str) -> None:
            self.steps.append({"step": step_name, "success": success, "reason": reason})

        def publish_result(self, final_state: str, reason: str) -> None:
            self.active = False
            output = String()
            output.data = json.dumps(
                {
                    "final_state": final_state,
                    "planner_id": self.planner_id,
                    "planner_ids": self.planner_ids,
                    "planning_group": self.group_name,
                    "end_effector_link": self.end_effector_link,
                    "position_tolerance": self.position_tolerance,
                    "use_fixed_home_start": self.use_fixed_home_start,
                    "pre_grasp_orientation_tolerance": self.pre_grasp_orientation_tolerance,
                    "grasp_pitch_offset_deg": float(self.get_parameter("grasp_pitch_offset_deg").value),
                    "enable_place_endpoint_precheck": bool(self.get_parameter("enable_place_endpoint_precheck").value),
                    "enable_loaded_path_precheck": bool(self.get_parameter("enable_loaded_path_precheck").value),
                    "recovery_orientation_tolerance": (
                        self.recovery_orientation_tolerance
                    ),
                    "recovery_orientation_tolerance_step": (
                        self.recovery_orientation_tolerance_step
                    ),
                    "equivalent_grasp_yaw_offsets_deg": (
                        self.equivalent_grasp_yaw_offsets_deg
                    ),
                    "enable_grasp_candidate_prevalidation": (
                        self.enable_grasp_candidate_prevalidation
                    ),
                    "grasp_candidate_prevalidation_min_fraction": (
                        self.grasp_candidate_prevalidation_min_fraction
                    ),
                    "grasp_candidate_prevalidation_min_joint_limit_margin": (
                        self.grasp_candidate_prevalidation_min_joint_limit_margin
                    ),
                    "grasp_candidate_prevalidation_scene_settle_s": (
                        self.grasp_candidate_prevalidation_scene_settle_s
                    ),
                    "defer_grasp_candidate_prevalidation_until_dynamic_replan": (
                        self.defer_grasp_candidate_prevalidation_until_dynamic_replan
                    ),
                    "grasp_candidate_prevalidation_results": (
                        [
                            grasp_candidate_report(result)
                            for result in self.grasp_candidate_prevalidation_results
                        ]
                    ),
                    "grasp_candidate_prevalidation_round": (
                        self.grasp_candidate_prevalidation_round
                    ),
                    "grasp_candidate_prevalidation_max_rounds": (
                        self.grasp_candidate_prevalidation_max_rounds
                    ),
                    "grasp_candidate_prevalidation_history": (
                        [
                            grasp_candidate_report(result)
                            for result in self.grasp_candidate_prevalidation_history
                        ]
                    ),
                    "equivalent_grasp_attempt_count": (
                        self.equivalent_grasp_attempt_count
                    ),
                    "active_grasp_yaw_offset_deg": (
                        self.active_grasp_yaw_offset_deg
                    ),
                    "cartesian_min_fraction": self.cartesian_min_fraction,
                    "cartesian_retry_max_step": self.cartesian_retry_max_step,
                    "cartesian_timing_retry_max_steps": (
                        self.cartesian_timing_retry_max_steps
                    ),
                    "cartesian_timing_retry_counts": (
                        self.cartesian_timing_retry_counts
                    ),
                    "cartesian_trajectory_metrics": (
                        self.cartesian_trajectory_metrics
                    ),
                    "cpp_trajectory_metrics_history": (
                        self.cpp_trajectory_metrics_history
                    ),
                    "cpp_trajectory_metrics_summary": (
                        summarize_cpp_trajectory_metrics(
                            self.cpp_trajectory_metrics_history
                        )
                    ),
                    "cartesian_start_state_replan_max_retries": (
                        self.cartesian_start_state_replan_max_retries
                    ),
                    "cartesian_start_state_replan_counts": (
                        self.cartesian_start_state_replan_counts
                    ),
                    "cartesian_execution_replan_max_retries": (
                        self.cartesian_execution_replan_max_retries
                    ),
                    "cartesian_execution_replan_counts": (
                        self.cartesian_execution_replan_counts
                    ),
                    "arm_execution_endpoint_acceptance_tolerance_rad": (
                        self.arm_execution_endpoint_acceptance_tolerance_rad
                    ),
                    "cartesian_avoid_collisions": self.cartesian_avoid_collisions,
                    "diagnose_direct_path_to_pre_grasp": (
                        self.diagnose_direct_path_to_pre_grasp
                    ),
                    "require_direct_path_blocked": self.require_direct_path_blocked,
                    "direct_path_blocked_fraction_threshold": (
                        self.direct_path_blocked_fraction_threshold
                    ),
                    "direct_path_fraction": self.direct_path_fraction,
                    "direct_path_unchecked_fraction": (
                        self.direct_path_unchecked_fraction
                    ),
                    "direct_path_blocked": self.direct_path_blocked,
                    "use_ompl_for_place": self.use_ompl_for_place,
                    "use_cartesian_pre_place_transfer": (
                        self.use_cartesian_pre_place_transfer
                    ),
                    "pre_place_transfer_stage_count": (
                        self.pre_place_transfer_stage_count
                    ),
                    "pre_place_transfer_ompl_fallback_used": (
                        self.pre_place_transfer_ompl_fallback_used
                    ),
                    "ompl_velocity_scaling_factor": (
                        self.ompl_velocity_scaling_factor
                    ),
                    "ompl_invalid_plan_retries": self.ompl_invalid_plan_retries,
                    "ompl_invalid_plan_retry_counts": (
                        self.ompl_invalid_plan_retry_counts
                    ),
                    "recovery_ompl_retries": self.recovery_ompl_retries,
                    "recovery_planner_id": self.recovery_planner_id,
                    "recovery_ompl_retry_counts": (
                        self.recovery_ompl_retry_counts
                    ),
                    "ompl_metrics": self.ompl_metrics,
                    "ompl_planning_time_s": sum(
                        float(metric["planning_time_s"])
                        for metric in self.ompl_metrics
                    ),
                    "ompl_joint_path_length_rad": sum(
                        float(metric["joint_path_length_rad"])
                        for metric in self.ompl_metrics
                    ),
                    "cartesian_velocity_scaling_factor": (
                        self.cartesian_velocity_scaling_factor
                    ),
                    "cartesian_acceleration_scaling_factor": (
                        self.cartesian_acceleration_scaling_factor
                    ),
                    "contact_approach_velocity_scaling_factor": (
                        self.contact_approach_velocity_scaling_factor
                    ),
                    "contact_approach_acceleration_scaling_factor": (
                        self.contact_approach_acceleration_scaling_factor
                    ),
                    "enable_cartesian_approach_recovery": self.enable_cartesian_approach_recovery,
                    "approach_recovery_ratio": self.approach_recovery_ratio,
                    "approach_recovery_ratios": self.approach_recovery_ratios,
                    "approach_recovery_attempt_count": (
                        self.approach_recovery_attempt_count
                    ),
                    "enable_pre_grasp_alignment_recovery": (
                        self.enable_pre_grasp_alignment_recovery
                    ),
                    "pre_grasp_alignment_staging_height": (
                        self.pre_grasp_alignment_staging_height
                    ),
                    "pre_grasp_alignment_recovery_min_fraction": (
                        self.pre_grasp_alignment_recovery_min_fraction
                    ),
                    "pre_grasp_alignment_prefix_min_fraction": (
                        self.pre_grasp_alignment_prefix_min_fraction
                    ),
                    "pre_grasp_alignment_prefix_attempt_count": (
                        self.pre_grasp_alignment_prefix_attempt_count
                    ),
                    "contact_prefix_min_fraction": (
                        self.contact_prefix_min_fraction
                    ),
                    "contact_prefix_max_attempts": (
                        self.contact_prefix_max_attempts
                    ),
                    "contact_prefix_attempt_counts": (
                        self.contact_prefix_attempt_counts
                    ),
                    "execute_trajectories": self.execute_trajectories,
                    "visualize_only_execution": self.visualize_only_execution,
                    "ompl_execution_backend": self.ompl_execution_backend,
                    "enable_dynamic_replanning": self.enable_dynamic_replanning,
                    "maximum_dynamic_replans": self.maximum_dynamic_replans,
                    "dynamic_replan_count": self.dynamic_replan_count,
                    "dynamic_replan_events": self.dynamic_replan_events,
                    "execute_gripper": self.execute_gripper,
                    "gripper_group_name": self.gripper_group_name,
                    "gripper_execution_backend": self.gripper_execution_backend,
                    "gripper_action_name": self.gripper_action_name,
                    "gripper_open_position": self.gripper_open_position,
                    "gripper_release_position": self.gripper_release_position,
                    "gripper_release_settle_s": self.gripper_release_settle_s,
                    "gripper_closed_position": self.gripper_closed_position,
                    "enable_tactile_grasp_supervision": (
                        self.enable_tactile_grasp_supervision
                    ),
                    "tactile_contact_max_age_s": self.tactile_contact_max_age_s,
                    "tactile_grasp_verified": self.tactile_grasp_verified,
                    "tactile_release_verified": self.tactile_release_verified,
                    "tactile_release_retry_count": (
                        self.tactile_release_retry_count
                    ),
                    "release_object_horizontal_motion_m": (
                        self.release_object_horizontal_motion_m
                    ),
                    "release_object_vertical_motion_m": (
                        self.release_object_vertical_motion_m
                    ),
                    "release_object_motion_m": self.release_object_motion_m,
                    "tactile_target_contacts": {
                        side: bool(names)
                        for side, names in self.latest_tactile_target_collisions.items()
                    },
                    "gripper_free_motion_max_retries": (
                        self.gripper_free_motion_max_retries
                    ),
                    "gripper_goal_rejection_retry_total": (
                        self.gripper_goal_rejection_retry_total
                    ),
                    "gripper_close_in_place_max_retries": (
                        self.gripper_close_in_place_max_retries
                    ),
                    "gripper_close_min_movement_m": (
                        self.gripper_close_min_movement_m
                    ),
                    "gripper_endpoint_acceptance_tolerance_m": (
                        self.gripper_endpoint_acceptance_tolerance_m
                    ),
                    "gripper_close_in_place_retry_total": (
                        self.gripper_close_in_place_retry_total
                    ),
                    "enable_physical_grasp_verification": (
                        self.enable_physical_grasp_verification
                    ),
                    "grasp_probe_lift_m": self.grasp_probe_lift_m,
                    "grasp_probe_min_object_lift_m": self.grasp_probe_min_object_lift_m,
                    "grasp_probe_max_retries": self.grasp_probe_max_retries,
                    "preclose_recenter_max_retries": (
                        self.preclose_recenter_max_retries
                    ),
                    "preclose_recenter_retry_count": (
                        self.preclose_recenter_retry_count
                    ),
                    "release_settle_s": self.release_settle_s,
                    "release_clearance_height": self.release_clearance_height,
                    "pre_place_height_offset": self.pre_place_height_offset,
                    "pre_place_candidate_attempts": self.pre_place_candidate_attempts,
                    "pre_place_joint_replay_positions": (
                        self.pre_place_joint_replay_positions
                    ),
                    "pre_place_joint_replay_tolerance_rad": (
                        self.pre_place_joint_replay_tolerance_rad
                    ),
                    "pre_place_joint_target_positions": (
                        self.pre_place_joint_target_positions
                    ),
                    "pre_place_joint_replay_max_plan_delta_rad": (
                        self.pre_place_joint_replay_max_plan_delta_rad
                    ),
                    "direct_place_candidate_attempts": self.direct_place_candidate_attempts,
                    "place_transfer_max_joint_path_length": (
                        self.place_transfer_max_joint_path_length
                    ),
                    "payload_transfer_tolerance_m": self.payload_transfer_tolerance_m,
                    "place_object_clearance_m": self.place_object_clearance_m,
                    "place_feedback_tolerance_m": self.place_feedback_tolerance_m,
                    "place_feedback_max_correction_m": (
                        self.place_feedback_max_correction_m
                    ),
                    "place_feedback_retry_count": self.place_feedback_retry_count,
                    "place_feedback_error_m": self.place_feedback_error_m,
                    "place_feedback_previous_error_m": (
                        self.place_feedback_previous_error_m
                    ),
                    "place_feedback_improvement_m": (
                        self.place_feedback_improvement_m
                    ),
                    "place_feedback_recovery_height_m": (
                        self.place_feedback_recovery_height_m
                    ),
                    "place_descent_horizontal_motion_m": (
                        self.place_descent_horizontal_motion_m
                    ),
                    "place_descent_vertical_motion_m": (
                        self.place_descent_vertical_motion_m
                    ),
                    "place_descent_motion_m": self.place_descent_motion_m,
                    "place_descent_stage_count": self.place_descent_stage_count,
                    "place_descent_backend": self.place_descent_backend,
                    "servo_descent_speed_mps": self.servo_descent_speed_mps,
                    "servo_command_period_s": self.servo_command_period_s,
                    "servo_stage_position_tolerance_m": (
                        self.servo_stage_position_tolerance_m
                    ),
                    "servo_status_history": self.servo_status_history,
                    "servo_watchdog_period_s": self.servo_watchdog_period_s,
                    "servo_watchdog_check_count": self.servo_watchdog_check_count,
                    "servo_fault_injection_type": (
                        self.servo_fault_injection_type
                    ),
                    "servo_fault_injection_stage": (
                        self.servo_fault_injection_stage
                    ),
                    "servo_fault_injection_delay_s": (
                        self.servo_fault_injection_delay_s
                    ),
                    "servo_fault_injection_triggered": (
                        self.servo_fault_injection_triggered
                    ),
                    "servo_fault_injection_observed_stage": (
                        self.servo_fault_injection_observed_stage
                    ),
                    "servo_watchdog_stop_latency_s": (
                        self.servo_watchdog_stop_latency_s
                    ),
                    "servo_watchdog_abort_stop_command_ns": (
                        self.servo_watchdog_abort_stop_command_ns
                    ),
                    "servo_watchdog_abort_stop_monotonic_ns": (
                        self.servo_watchdog_abort_stop_monotonic_ns
                    ),
                    "servo_watchdog_abort_stage": (
                        self.servo_watchdog_abort_stage
                    ),
                    "servo_watchdog_abort_reason": (
                        self.servo_watchdog_abort_reason
                    ),
                    "servo_collision_limited_acceptance_m": (
                        self.servo_collision_limited_acceptance_m
                    ),
                    "enable_servo_cartesian_fallback": (
                        self.enable_servo_cartesian_fallback
                    ),
                    "servo_cartesian_fallback_used": (
                        self.servo_cartesian_fallback_used
                    ),
                    "servo_cartesian_fallback_count": (
                        self.servo_cartesian_fallback_count
                    ),
                    "place_descent_max_trajectory_duration_s": (
                        self.place_descent_max_trajectory_duration_s
                    ),
                    "place_descent_max_joint_path_length_rad": (
                        self.place_descent_max_joint_path_length_rad
                    ),
                    "place_descent_stage_max_horizontal_motion_m": (
                        self.place_descent_stage_max_horizontal_motion_m
                    ),
                    "place_descent_stage_payload_tolerance_m": (
                        self.place_descent_stage_payload_tolerance_m
                    ),
                    "place_descent_endpoint_orientation_tolerance_rad": (
                        self.place_descent_endpoint_orientation_tolerance_rad
                    ),
                    "place_descent_start_tolerance_m": (
                        self.place_descent_start_tolerance_m
                    ),
                    "place_descent_start_poll_s": self.place_descent_start_poll_s,
                    "place_descent_start_timeout_s": (
                        self.place_descent_start_timeout_s
                    ),
                    "place_descent_start_max_alignment_retries": (
                        self.place_descent_start_max_alignment_retries
                    ),
                    "place_descent_start_alignment_retry_count": (
                        self.place_descent_start_alignment_retry_count
                    ),
                    "place_descent_stage_diagnostics": (
                        self.place_descent_stage_diagnostics
                    ),
                    "payload_grasp_calibration_delta_m": (
                        self.payload_grasp_calibration_delta_m
                    ),
                    "place_descent_payload_reanchor_delta_m": (
                        self.place_descent_payload_reanchor_delta_m
                    ),
                    "payload_place_compensation_m": (
                        self.payload_place_compensation_m
                    ),
                    "payload_transfer_velocity_scaling_factor": (
                        self.payload_transfer_velocity_scaling_factor
                    ),
                    "place_descent_velocity_scaling_factor": (
                        self.place_descent_velocity_scaling_factor
                    ),
                    "place_descent_recovery_velocity_scaling_factor": (
                        self.place_descent_recovery_velocity_scaling_factor
                    ),
                    "payload_transfer_execution_replan_max_retries": (
                        self.payload_transfer_execution_replan_max_retries
                    ),
                    "payload_transfer_execution_replan_count": (
                        self.payload_transfer_execution_replan_count
                    ),
                    "allow_ompl_release_retreat_fallback": (
                        self.allow_ompl_release_retreat_fallback
                    ),
                    "release_retreat_max_joint_path_length": (
                        self.release_retreat_max_joint_path_length
                    ),
                    "return_home_after_success": self.return_home_after_success,
                    "return_home_scene_settle_s": self.return_home_scene_settle_s,
                    "return_home_max_retries": self.return_home_max_retries,
                    "return_home_retry_count": self.return_home_retry_count,
                    "approach_scene_settle_s": self.approach_scene_settle_s,
                    "grasp_preclose_max_target_drift_m": (
                        self.grasp_preclose_max_target_drift_m
                    ),
                    "grasp_recovery_max_translation_m": (
                        self.grasp_recovery_max_translation_m
                    ),
                    "grasp_finger_asymmetry_correction_gain": (
                        self.grasp_finger_asymmetry_correction_gain
                    ),
                    "grasp_finger_asymmetry_min_m": (
                        self.grasp_finger_asymmetry_min_m
                    ),
                    "grasp_finger_max_correction_m": (
                        self.grasp_finger_max_correction_m
                    ),
                    "reset_to_home_before_execute": self.reset_to_home_before_execute,
                    "execution_start_tolerance": self.execution_start_tolerance,
                    "fail_on_start_state_mismatch": self.fail_on_start_state_mismatch,
                    "joint_state_wait_timeout_s": self.joint_state_wait_timeout_s,
                    "steps": self.steps,
                    "failure_reason": reason or None,
                }
            )
            self.result_publisher.publish(output)
            self.get_logger().info(output.data)

        def execution_start_check(self, trajectory) -> Dict[str, object]:
            joint_trajectory = trajectory.joint_trajectory
            if not joint_trajectory.points:
                return {"success": False, "reason": "execution_start_check: trajectory has no points"}
            if self.latest_joint_state is None:
                reason = "execution_start_check: /joint_states unavailable"
                return {"success": not self.fail_on_start_state_mismatch, "reason": reason}

            current_by_name = {
                name: position
                for name, position in zip(
                    self.latest_joint_state.name,
                    self.latest_joint_state.position,
                )
            }
            first_point = joint_trajectory.points[0]
            missing = [name for name in joint_trajectory.joint_names if name not in current_by_name]
            if missing:
                reason = f"execution_start_check: missing joints in /joint_states: {missing}"
                return {"success": not self.fail_on_start_state_mismatch, "reason": reason}

            deltas = []
            for name, target_position in zip(joint_trajectory.joint_names, first_point.positions):
                current_position = float(current_by_name[name])
                delta = abs(float(target_position) - current_position)
                deltas.append((name, delta, float(target_position), current_position))
            if not deltas:
                return {"success": False, "reason": "execution_start_check: no joint deltas computed"}

            max_name, max_delta, target_position, current_position = max(deltas, key=lambda item: item[1])
            reason = (
                "execution_start_check: "
                f"max_delta={max_delta:.4f} rad at {max_name}, "
                f"trajectory_start={target_position:.4f}, current={current_position:.4f}, "
                f"tolerance={self.execution_start_tolerance:.4f}"
            )
            if max_delta > self.execution_start_tolerance:
                return {"success": not self.fail_on_start_state_mismatch, "reason": reason}
            return {"success": True, "reason": reason}

        def current_home_delta_summary(self) -> str:
            if self.latest_joint_state is None:
                return "current_vs_home: /joint_states unavailable"

            current_by_name = {
                name: position
                for name, position in zip(
                    self.latest_joint_state.name,
                    self.latest_joint_state.position,
                )
            }
            home_state = self.fixed_home_state()
            deltas = []
            missing = []
            for name, home_position in zip(home_state.joint_state.name, home_state.joint_state.position):
                if name not in current_by_name:
                    missing.append(name)
                    continue
                current_position = float(current_by_name[name])
                delta = abs(float(home_position) - current_position)
                deltas.append((name, delta, float(home_position), current_position))
            if missing:
                return f"current_vs_home: missing joints in /joint_states: {missing}"
            if not deltas:
                return "current_vs_home: no joint deltas computed"

            max_name, max_delta, home_position, current_position = max(deltas, key=lambda item: item[1])
            return (
                "current_vs_home: "
                f"max_delta={max_delta:.4f} rad at {max_name}, "
                f"home={home_position:.4f}, current={current_position:.4f}"
            )

    rclpy.init(args=args)
    node = PickPlanPipeline()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
