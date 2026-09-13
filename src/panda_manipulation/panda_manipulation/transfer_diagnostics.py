"""Record synchronized arm, gripper, contact, and payload-transfer telemetry."""

from __future__ import annotations

import csv
import json
from math import ceil, floor, isfinite, sqrt
from pathlib import Path
from statistics import median
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from .physics_benchmark import wilson_interval
from .ros_helpers import require_ros2


ARM_JOINTS = [f"panda_joint{index}" for index in range(1, 8)]
FINGER_JOINTS = ["panda_finger_joint1", "panda_finger_joint2"]


def finite_values(values: Iterable[object]) -> List[float]:
    """Return finite numeric values while tolerating partially populated messages."""
    output: List[float] = []
    for value in values:
        try:
            number = float(value)
        except (TypeError, ValueError):
            continue
        if isfinite(number):
            output.append(number)
    return output


def max_abs(values: Iterable[object]) -> Optional[float]:
    """Return the largest finite absolute value, if one exists."""
    numbers = finite_values(values)
    return max((abs(value) for value in numbers), default=None)


def root_mean_square(values: Iterable[object]) -> Optional[float]:
    """Return the RMS of finite values, if one exists."""
    numbers = finite_values(values)
    if not numbers:
        return None
    return sqrt(sum(value * value for value in numbers) / len(numbers))


def percentile(values: Iterable[object], quantile: float) -> Optional[float]:
    """Return a linearly interpolated percentile of finite numeric values."""
    if not 0.0 <= quantile <= 1.0:
        raise ValueError("quantile must be between zero and one")
    numbers = sorted(finite_values(values))
    if not numbers:
        return None
    index = (len(numbers) - 1) * quantile
    lower = floor(index)
    upper = ceil(index)
    if lower == upper:
        return numbers[lower]
    weight = index - lower
    return numbers[lower] * (1.0 - weight) + numbers[upper] * weight


def decode_joint_positions(value: object) -> Optional[List[float]]:
    """Decode a complete seven-joint vector from memory or a CSV JSON cell."""
    if isinstance(value, str):
        try:
            value = json.loads(value)
        except json.JSONDecodeError:
            return None
    if not isinstance(value, (list, tuple)) or len(value) != len(ARM_JOINTS):
        return None
    positions = finite_values(value)
    return positions if len(positions) == len(ARM_JOINTS) else None


def first_stage_arm_positions(
    rows: Sequence[Dict[str, object]],
    stage_prefix: object = (
        "cartesian_place_descent_stage_",
        "servo_place_descent_stage_",
    ),
) -> Optional[List[float]]:
    """Return measured arm positions at the first sample of a stage family."""
    prefixes = (
        (str(stage_prefix),)
        if isinstance(stage_prefix, str)
        else tuple(str(prefix) for prefix in stage_prefix)
    )
    for row in rows:
        if not str(row.get("stage") or "").startswith(prefixes):
            continue
        positions = decode_joint_positions(
            [row.get(f"arm_feedback_{joint}") for joint in ARM_JOINTS]
        )
        if positions is not None:
            return positions
    return None


def selected_grasp_yaw_offset(result: object) -> Optional[float]:
    """Return the active yaw only when candidate selection actually succeeded."""
    if not isinstance(result, dict):
        return None
    selected = any(
        isinstance(step, dict)
        and step.get("step") == "grasp_candidate_prevalidation_selected"
        and bool(step.get("success"))
        for step in result.get("steps", [])
    )
    if not selected:
        return None
    values = finite_values([result.get("active_grasp_yaw_offset_deg")])
    return values[0] if values else None


def translation_distance(values: Sequence[float]) -> float:
    """Return the Euclidean magnitude of a translation vector."""
    return sqrt(sum(float(value) * float(value) for value in values))


def relative_motion_drift(
    baseline_hand: Sequence[float],
    baseline_object: Sequence[float],
    current_hand: Sequence[float],
    current_object: Sequence[float],
) -> float:
    """Measure object motion not explained by hand motion.

    The hand and Gazebo object positions may have different constant origins. Taking
    deltas from a synchronized baseline removes that translation while preserving
    payload slip in their aligned world/base axes.
    """
    hand_delta = tuple(
        float(current) - float(baseline)
        for baseline, current in zip(baseline_hand, current_hand)
    )
    object_delta = tuple(
        float(current) - float(baseline)
        for baseline, current in zip(baseline_object, current_object)
    )
    return translation_distance(
        tuple(
            object_value - hand_value
            for object_value, hand_value in zip(object_delta, hand_delta)
        )
    )


def rotate_vector_by_quaternion(
    vector: Sequence[float],
    quaternion: Sequence[float],
) -> Tuple[float, float, float]:
    """Rotate a vector by an ``(x, y, z, w)`` quaternion."""
    vx, vy, vz = (float(value) for value in vector)
    qx, qy, qz, qw = (float(value) for value in quaternion)
    norm = sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        raise ValueError("quaternion norm must be non-zero")
    qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def point_in_frame(
    frame_position: Sequence[float],
    frame_orientation: Sequence[float],
    point_position: Sequence[float],
) -> Tuple[float, float, float]:
    """Express a world-frame point in a translated and rotated local frame."""
    delta = tuple(
        float(point) - float(origin)
        for point, origin in zip(point_position, frame_position)
    )
    qx, qy, qz, qw = (float(value) for value in frame_orientation)
    return rotate_vector_by_quaternion(delta, (-qx, -qy, -qz, qw))


def local_point_drift(
    baseline_point: Sequence[float],
    current_point: Sequence[float],
) -> float:
    """Measure payload translation in the hand frame from a synchronized baseline."""
    return translation_distance(
        tuple(
            float(current) - float(baseline)
            for baseline, current in zip(baseline_point, current_point)
        )
    )


def controller_point_values(message: object, field: str) -> Dict[str, float]:
    """Map a JointTrajectoryControllerState point field by joint name."""
    names = list(getattr(message, "joint_names", []))
    point = getattr(message, field, None)
    positions = list(getattr(point, "positions", [])) if point is not None else []
    if field == "output" and not positions and point is not None:
        positions = list(getattr(point, "effort", []))
    return {
        str(name): float(value)
        for name, value in zip(names, positions)
        if isfinite(float(value))
    }


def contact_sides(
    contacts: object,
    left_token: str = "panda_leftfinger",
    right_token: str = "panda_rightfinger",
) -> Tuple[bool, bool, List[str]]:
    """Classify target contacts by finger collision names."""
    left = False
    right = False
    names: List[str] = []
    for contact in getattr(contacts, "contacts", []):
        pair = [
            str(getattr(getattr(contact, "collision1", None), "name", "")),
            str(getattr(getattr(contact, "collision2", None), "name", "")),
        ]
        names.extend(name for name in pair if name)
        joined = " ".join(pair)
        left = left or left_token in joined
        right = right or right_token in joined
    return left, right, sorted(set(names))


def payload_phase_event(stage: str, mode: str) -> str:
    """Return whether a stage starts, resets, or preserves payload tracking."""
    normalized_stage = str(stage).lower()
    normalized_mode = str(mode).lower()
    if normalized_stage == "object_attached":
        return "transfer_start"
    if normalized_stage == "object_released" or (
        "gripper" in normalized_stage and "open" in normalized_stage
    ):
        return "reset"
    if normalized_stage == "gripper_closed":
        if normalized_mode == "executing":
            return "reset"
        if normalized_mode in {
            "executed",
            "measured_endpoint",
            "contact_limited",
        }:
            return "probe_start"
    return "keep"


def summarize_rows(
    rows: Sequence[Dict[str, object]],
    payload_drift_threshold_m: float,
) -> Dict[str, object]:
    """Build task and per-stage tracking metrics from synchronized samples."""
    arm_errors = [
        row.get(f"arm_error_{joint}")
        for row in rows
        for joint in ARM_JOINTS
    ]
    gripper_errors = [
        row.get(f"gripper_error_{joint}")
        for row in rows
        for joint in FINGER_JOINTS
    ]
    payload_drifts = finite_values(row.get("payload_relative_drift_m") for row in rows)
    payload_rows = [row for row in rows if bool(row.get("payload_phase"))]
    transfer_rows = [row for row in rows if row.get("payload_epoch") == "transfer"]
    probe_rows = [row for row in rows if row.get("payload_epoch") == "grasp_probe"]
    transfer_drifts = finite_values(
        row.get("payload_relative_drift_m") for row in transfer_rows
    )
    probe_drifts = finite_values(
        row.get("payload_relative_drift_m") for row in probe_rows
    )
    transfer_arm_errors = [
        row.get(f"arm_error_{joint}")
        for row in transfer_rows
        for joint in ARM_JOINTS
    ]
    first_crossing = next(
        (
            {
                "elapsed_s": row.get("elapsed_s"),
                "stage": row.get("stage"),
                "mode": row.get("mode"),
                "payload_relative_drift_m": row.get("payload_relative_drift_m"),
            }
            for row in rows
            if isinstance(row.get("payload_relative_drift_m"), (int, float))
            and float(row["payload_relative_drift_m"]) > payload_drift_threshold_m
        ),
        None,
    )
    first_transfer_crossing = next(
        (
            {
                "elapsed_s": row.get("elapsed_s"),
                "stage": row.get("stage"),
                "mode": row.get("mode"),
                "payload_relative_drift_m": row.get("payload_relative_drift_m"),
            }
            for row in transfer_rows
            if isinstance(row.get("payload_relative_drift_m"), (int, float))
            and float(row["payload_relative_drift_m"]) > payload_drift_threshold_m
        ),
        None,
    )
    stage_names = list(dict.fromkeys(str(row.get("stage") or "UNSET") for row in rows))
    stage_metrics: Dict[str, object] = {}
    for stage in stage_names:
        stage_rows = [row for row in rows if str(row.get("stage") or "UNSET") == stage]
        stage_arm_errors = [
            row.get(f"arm_error_{joint}")
            for row in stage_rows
            for joint in ARM_JOINTS
        ]
        stage_drifts = finite_values(
            row.get("payload_relative_drift_m") for row in stage_rows
        )
        stage_metrics[stage] = {
            "samples": len(stage_rows),
            "max_arm_position_error_rad": max_abs(stage_arm_errors),
            "rms_arm_position_error_rad": root_mean_square(stage_arm_errors),
            "max_payload_relative_drift_m": max(stage_drifts, default=None),
            "bilateral_contact_ratio": (
                sum(bool(row.get("bilateral_contact")) for row in stage_rows)
                / len(stage_rows)
                if stage_rows
                else None
            ),
        }
    return {
        "samples": len(rows),
        "duration_s": rows[-1].get("elapsed_s") if rows else 0.0,
        "max_arm_position_error_rad": max_abs(arm_errors),
        "rms_arm_position_error_rad": root_mean_square(arm_errors),
        "max_gripper_position_error_m": max_abs(gripper_errors),
        "max_payload_relative_drift_m": max(payload_drifts, default=None),
        "max_grasp_probe_relative_drift_m": max(probe_drifts, default=None),
        "max_transfer_relative_drift_m": max(transfer_drifts, default=None),
        "max_transfer_arm_position_error_rad": max_abs(transfer_arm_errors),
        "rms_transfer_arm_position_error_rad": root_mean_square(
            transfer_arm_errors
        ),
        "payload_drift_threshold_m": payload_drift_threshold_m,
        "first_payload_drift_threshold_crossing": first_crossing,
        "first_transfer_drift_threshold_crossing": first_transfer_crossing,
        "payload_bilateral_contact_ratio": (
            sum(bool(row.get("bilateral_contact")) for row in payload_rows)
            / len(payload_rows)
            if payload_rows
            else None
        ),
        "pre_descent_arm_positions_rad": first_stage_arm_positions(rows),
        "stages": stage_metrics,
    }


def summarize_comparison(
    rows: Sequence[Dict[str, object]],
    matched_state_tolerance_rad: float = 0.05,
) -> Dict[str, object]:
    """Aggregate paired unloaded/loaded runs without hiding failed trials."""
    modes: Dict[str, object] = {}
    for mode in ("unloaded", "loaded"):
        mode_rows = [row for row in rows if row.get("mode") == mode]
        arm_maxima = finite_values(
            row.get("max_transfer_arm_position_error_rad")
            if row.get("max_transfer_arm_position_error_rad") is not None
            else row.get("max_arm_position_error_rad")
            for row in mode_rows
        )
        arm_rms = finite_values(
            row.get("rms_transfer_arm_position_error_rad")
            if row.get("rms_transfer_arm_position_error_rad") is not None
            else row.get("rms_arm_position_error_rad")
            for row in mode_rows
        )
        payload_drifts = finite_values(
            row.get("max_payload_relative_drift_m") for row in mode_rows
        )
        durations = finite_values(
            row.get("elapsed_s")
            if row.get("elapsed_s") is not None
            else row.get("duration_s")
            for row in mode_rows
        )
        successes = sum(row.get("final_state") == "SUCCESS" for row in mode_rows)
        modes[mode] = {
            "trials": len(mode_rows),
            "successes": successes,
            "success_rate": successes / len(mode_rows) if mode_rows else None,
            "success_rate_ci": wilson_interval(successes, len(mode_rows)),
            "mean_max_transfer_arm_position_error_rad": (
                sum(arm_maxima) / len(arm_maxima) if arm_maxima else None
            ),
            "median_max_transfer_arm_position_error_rad": (
                median(arm_maxima) if arm_maxima else None
            ),
            "p95_max_transfer_arm_position_error_rad": percentile(
                arm_maxima, 0.95
            ),
            "mean_rms_transfer_arm_position_error_rad": (
                sum(arm_rms) / len(arm_rms) if arm_rms else None
            ),
            "median_rms_transfer_arm_position_error_rad": (
                median(arm_rms) if arm_rms else None
            ),
            "mean_max_payload_relative_drift_m": (
                sum(payload_drifts) / len(payload_drifts) if payload_drifts else None
            ),
            "median_max_payload_relative_drift_m": (
                median(payload_drifts) if payload_drifts else None
            ),
            "mean_elapsed_s": (
                sum(durations) / len(durations) if durations else None
            ),
            "failure_reasons": sorted(
                str(row.get("failure_reason"))
                for row in mode_rows
                if row.get("final_state") != "SUCCESS"
            ),
        }
    unloaded_error = modes["unloaded"]["mean_rms_transfer_arm_position_error_rad"]
    loaded_error = modes["loaded"]["mean_rms_transfer_arm_position_error_rad"]
    tracking_error_ratio = None
    tracking_error_absolute_delta = None
    ratio_denominator_near_zero = False
    if (
        isinstance(unloaded_error, (int, float))
        and unloaded_error > 0.0
        and isinstance(loaded_error, (int, float))
    ):
        tracking_error_ratio = loaded_error / unloaded_error
        tracking_error_absolute_delta = loaded_error - unloaded_error
        ratio_denominator_near_zero = unloaded_error < 1e-4

    paired_rows: Dict[object, Dict[str, Dict[str, object]]] = {}
    for row in rows:
        pair = row.get("pair")
        mode = str(row.get("mode") or "")
        if pair is None or mode not in {"unloaded", "loaded"}:
            continue
        paired_rows.setdefault(pair, {})[mode] = row
    pair_diagnostics: List[Dict[str, object]] = []
    matched_unloaded_rms: List[float] = []
    matched_loaded_rms: List[float] = []
    for pair, modes_for_pair in sorted(paired_rows.items(), key=lambda item: str(item[0])):
        unloaded_row = modes_for_pair.get("unloaded")
        loaded_row = modes_for_pair.get("loaded")
        unloaded_positions = decode_joint_positions(
            unloaded_row.get("pre_descent_arm_positions_rad")
            if unloaded_row is not None
            else None
        )
        loaded_positions = decode_joint_positions(
            loaded_row.get("pre_descent_arm_positions_rad")
            if loaded_row is not None
            else None
        )
        max_delta = None
        if unloaded_positions is not None and loaded_positions is not None:
            max_delta = max(
                abs(loaded - unloaded)
                for unloaded, loaded in zip(unloaded_positions, loaded_positions)
            )
        yaw_delta = None
        if unloaded_row is not None and loaded_row is not None:
            yaw_values = finite_values(
                (
                    unloaded_row.get("grasp_yaw_offset_deg"),
                    loaded_row.get("grasp_yaw_offset_deg"),
                )
            )
            if len(yaw_values) == 2:
                yaw_delta = abs(yaw_values[1] - yaw_values[0])
        matched = (
            max_delta is not None
            and max_delta <= matched_state_tolerance_rad
            and (yaw_delta is None or yaw_delta <= 1e-6)
        )
        pair_diagnostics.append(
            {
                "pair": pair,
                "complete": unloaded_row is not None and loaded_row is not None,
                "matched": matched,
                "max_pre_descent_joint_delta_rad": max_delta,
                "grasp_yaw_delta_deg": yaw_delta,
            }
        )
        if not matched or unloaded_row is None or loaded_row is None:
            continue
        unloaded_rms = finite_values(
            [unloaded_row.get("rms_transfer_arm_position_error_rad")]
        )
        loaded_rms = finite_values(
            [loaded_row.get("rms_transfer_arm_position_error_rad")]
        )
        if unloaded_rms and loaded_rms:
            matched_unloaded_rms.append(unloaded_rms[0])
            matched_loaded_rms.append(loaded_rms[0])
    matched_ratio = None
    matched_absolute_delta = None
    matched_ratio_denominator_near_zero = False
    if matched_unloaded_rms and matched_loaded_rms:
        unloaded_mean = sum(matched_unloaded_rms) / len(matched_unloaded_rms)
        loaded_mean = sum(matched_loaded_rms) / len(matched_loaded_rms)
        matched_absolute_delta = loaded_mean - unloaded_mean
        if unloaded_mean > 0.0:
            matched_ratio = loaded_mean / unloaded_mean
            matched_ratio_denominator_near_zero = unloaded_mean < 1e-4
    pair_state_deltas = finite_values(
        item.get("max_pre_descent_joint_delta_rad") for item in pair_diagnostics
    )
    return {
        "total_trials": len(rows),
        "modes": modes,
        "loaded_to_unloaded_rms_tracking_error_ratio": tracking_error_ratio,
        "loaded_minus_unloaded_mean_rms_tracking_error_rad": (
            tracking_error_absolute_delta
        ),
        "ratio_denominator_near_zero": ratio_denominator_near_zero,
        "matched_state_tolerance_rad": matched_state_tolerance_rad,
        "complete_pair_count": sum(item["complete"] for item in pair_diagnostics),
        "matched_state_pair_count": sum(item["matched"] for item in pair_diagnostics),
        "mean_pre_descent_joint_delta_rad": (
            sum(pair_state_deltas) / len(pair_state_deltas)
            if pair_state_deltas
            else None
        ),
        "max_pre_descent_joint_delta_rad": max(pair_state_deltas, default=None),
        "matched_pair_rms_tracking_error_ratio": matched_ratio,
        "matched_pair_loaded_minus_unloaded_mean_rms_tracking_error_rad": (
            matched_absolute_delta
        ),
        "matched_ratio_denominator_near_zero": (
            matched_ratio_denominator_near_zero
        ),
        "pair_state_diagnostics": pair_diagnostics,
    }


def render_comparison_markdown(summary: Dict[str, object]) -> str:
    """Render the paired payload diagnostic summary as a compact report."""
    lines = [
        "# Payload Transfer Diagnostics",
        "",
        "The unloaded and loaded runs use the same fixed obstacle task and controller settings.",
        "",
        (
            "| Mode | Trials | Success rate (95% CI) | Mean / median max error | "
            "P95 max error | Mean / median RMS error | Mean payload drift | Mean time |"
        ),
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    modes = summary.get("modes", {})
    for mode in ("unloaded", "loaded"):
        values = modes.get(mode, {})
        success_rate = values.get("success_rate")
        success_ci = values.get("success_rate_ci")
        max_error = values.get("mean_max_transfer_arm_position_error_rad")
        median_max_error = values.get("median_max_transfer_arm_position_error_rad")
        p95_max_error = values.get("p95_max_transfer_arm_position_error_rad")
        rms_error = values.get("mean_rms_transfer_arm_position_error_rad")
        median_rms_error = values.get("median_rms_transfer_arm_position_error_rad")
        drift = values.get("mean_max_payload_relative_drift_m")
        elapsed = values.get("mean_elapsed_s")
        rate_text = "N/A"
        if success_rate is not None:
            rate_text = f"{100.0 * success_rate:.1f}%"
            if success_ci is not None:
                rate_text += (
                    f" ({100.0 * success_ci[0]:.1f}%-"
                    f"{100.0 * success_ci[1]:.1f}%)"
                )
        lines.append(
            "| {mode} | {trials} | {rate} | {max_error} | {p95_max_error} | "
            "{rms_error} | {drift} | {elapsed} |".format(
                mode=mode,
                trials=values.get("trials", 0),
                rate=rate_text,
                max_error=(
                    f"{max_error:.5f} / {median_max_error:.5f} rad"
                    if max_error is not None and median_max_error is not None
                    else "N/A"
                ),
                p95_max_error=(
                    f"{p95_max_error:.5f} rad"
                    if p95_max_error is not None
                    else "N/A"
                ),
                rms_error=(
                    f"{rms_error:.5f} / {median_rms_error:.5f} rad"
                    if rms_error is not None and median_rms_error is not None
                    else "N/A"
                ),
                drift=(f"{drift:.4f} m" if drift is not None else "N/A"),
                elapsed=(f"{elapsed:.1f} s" if elapsed is not None else "N/A"),
            )
        )
    ratio = summary.get("loaded_to_unloaded_rms_tracking_error_ratio")
    matched_ratio = summary.get("matched_pair_rms_tracking_error_ratio")
    matched_count = summary.get("matched_state_pair_count", 0)
    complete_count = summary.get("complete_pair_count", 0)
    tolerance = summary.get("matched_state_tolerance_rad", 0.05)
    absolute_delta = summary.get(
        "loaded_minus_unloaded_mean_rms_tracking_error_rad"
    )
    matched_absolute_delta = summary.get(
        "matched_pair_loaded_minus_unloaded_mean_rms_tracking_error_rad"
    )
    mean_state_delta = summary.get("mean_pre_descent_joint_delta_rad")
    max_state_delta = summary.get("max_pre_descent_joint_delta_rad")
    lines.extend(
        [
            "",
            "## Interpretation Gate",
            "",
            (
                f"- Loaded/unloaded RMS tracking-error ratio: {ratio:.2f}x"
                if ratio is not None
                else "- Loaded/unloaded RMS tracking-error ratio: N/A"
            ),
            (
                f"- Matched pre-descent states: {matched_count}/{complete_count} "
                f"pairs (required max joint delta <= {tolerance:.3f} rad)."
            ),
            (
                f"- Pair start-state joint delta: mean {mean_state_delta:.5f} rad, "
                f"max {max_state_delta:.5f} rad."
                if mean_state_delta is not None and max_state_delta is not None
                else "- Pair start-state joint delta: N/A"
            ),
            (
                f"- Matched-pair RMS tracking-error ratio: {matched_ratio:.2f}x"
                if matched_ratio is not None
                else "- Matched-pair RMS tracking-error ratio: N/A"
            ),
            (
                f"- Loaded-minus-unloaded mean RMS error: {absolute_delta:.6f} rad; "
                f"matched pairs: {matched_absolute_delta:.6f} rad."
                if absolute_delta is not None and matched_absolute_delta is not None
                else "- Loaded-minus-unloaded mean RMS error: N/A"
            ),
            (
                "- Ratio caution: unloaded RMS is below 0.0001 rad, so the relative "
                "multiplier is numerically sensitive; prioritize absolute loaded "
                "error, P95, task success, and payload drift."
                if summary.get("ratio_denominator_near_zero")
                or summary.get("matched_ratio_denominator_near_zero")
                else "- Ratio denominator is large enough for direct relative comparison."
            ),
            (
                "- Attribute differences to payload only from matched-state pairs."
            ),
            "- Unloaded tracking failures point to controller timing or execution.",
            "- Loaded-only degradation points to grasp force, friction, depth, or wrist pose.",
            (
                "- Normal tracking with rising hand-object drift points to contact "
                "or collision modeling."
            ),
            "",
        ]
    )
    return "\n".join(lines)


def summarize_backend_comparison(
    rows: Sequence[Dict[str, object]],
    matched_state_tolerance_rad: float = 0.05,
) -> Dict[str, object]:
    """Aggregate matched loaded Cartesian/Servo contact-zone trials."""
    backends: Dict[str, object] = {}
    for backend in ("cartesian", "servo"):
        backend_rows = [
            row for row in rows if row.get("place_descent_backend") == backend
        ]
        successes = sum(row.get("final_state") == "SUCCESS" for row in backend_rows)
        descent_times = finite_values(
            row.get("place_descent_execution_time_s") for row in backend_rows
        )
        elapsed_times = finite_values(row.get("elapsed_s") for row in backend_rows)
        tracking_rms = finite_values(
            row.get("rms_transfer_arm_position_error_rad") for row in backend_rows
        )
        payload_drifts = finite_values(
            row.get("max_payload_relative_drift_m") for row in backend_rows
        )
        placement_errors = finite_values(
            row.get("physical_place_error_m") for row in backend_rows
        )
        final_tilts = finite_values(
            row.get("physical_final_tilt_deg") for row in backend_rows
        )
        backends[backend] = {
            "trials": len(backend_rows),
            "successes": successes,
            "success_rate": successes / len(backend_rows) if backend_rows else None,
            "success_rate_ci": wilson_interval(successes, len(backend_rows)),
            "mean_place_descent_execution_time_s": (
                sum(descent_times) / len(descent_times) if descent_times else None
            ),
            "median_place_descent_execution_time_s": (
                median(descent_times) if descent_times else None
            ),
            "mean_elapsed_s": (
                sum(elapsed_times) / len(elapsed_times) if elapsed_times else None
            ),
            "mean_rms_transfer_arm_position_error_rad": (
                sum(tracking_rms) / len(tracking_rms) if tracking_rms else None
            ),
            "mean_max_payload_relative_drift_m": (
                sum(payload_drifts) / len(payload_drifts) if payload_drifts else None
            ),
            "mean_physical_place_error_m": (
                sum(placement_errors) / len(placement_errors)
                if placement_errors
                else None
            ),
            "p95_physical_place_error_m": percentile(placement_errors, 0.95),
            "mean_physical_final_tilt_deg": (
                sum(final_tilts) / len(final_tilts) if final_tilts else None
            ),
            "servo_cartesian_fallback_trials": sum(
                bool(row.get("servo_cartesian_fallback_used"))
                for row in backend_rows
            ),
            "servo_cartesian_fallback_count": sum(
                int(row.get("servo_cartesian_fallback_count") or 0)
                for row in backend_rows
            ),
            "failure_reasons": sorted(
                str(row.get("failure_reason"))
                for row in backend_rows
                if row.get("final_state") != "SUCCESS"
            ),
        }

    paired_rows: Dict[object, Dict[str, Dict[str, object]]] = {}
    for row in rows:
        pair = row.get("pair")
        backend = str(row.get("place_descent_backend") or "")
        if pair is None or backend not in {"cartesian", "servo"}:
            continue
        paired_rows.setdefault(pair, {})[backend] = row
    pair_diagnostics: List[Dict[str, object]] = []
    descent_time_deltas: List[float] = []
    descent_speedup_ratios: List[float] = []
    placement_error_deltas: List[float] = []
    payload_drift_deltas: List[float] = []
    tracking_rms_deltas: List[float] = []
    for pair, rows_for_pair in sorted(
        paired_rows.items(), key=lambda item: str(item[0])
    ):
        cartesian = rows_for_pair.get("cartesian")
        servo = rows_for_pair.get("servo")
        cartesian_positions = decode_joint_positions(
            cartesian.get("pre_descent_arm_positions_rad")
            if cartesian is not None
            else None
        )
        servo_positions = decode_joint_positions(
            servo.get("pre_descent_arm_positions_rad")
            if servo is not None
            else None
        )
        max_delta = None
        if cartesian_positions is not None and servo_positions is not None:
            max_delta = max(
                abs(servo_value - cartesian_value)
                for cartesian_value, servo_value in zip(
                    cartesian_positions, servo_positions
                )
            )
        yaw_delta = None
        if cartesian is not None and servo is not None:
            yaw_values = finite_values(
                (
                    cartesian.get("grasp_yaw_offset_deg"),
                    servo.get("grasp_yaw_offset_deg"),
                )
            )
            if len(yaw_values) == 2:
                yaw_delta = abs(yaw_values[1] - yaw_values[0])
        matched = (
            max_delta is not None
            and max_delta <= matched_state_tolerance_rad
            and (yaw_delta is None or yaw_delta <= 1e-6)
        )
        pair_diagnostics.append(
            {
                "pair": pair,
                "complete": cartesian is not None and servo is not None,
                "matched": matched,
                "both_successful": (
                    cartesian is not None
                    and servo is not None
                    and cartesian.get("final_state") == "SUCCESS"
                    and servo.get("final_state") == "SUCCESS"
                ),
                "max_pre_descent_joint_delta_rad": max_delta,
                "grasp_yaw_delta_deg": yaw_delta,
            }
        )
        if not matched or cartesian is None or servo is None:
            continue
        metric_pairs = (
            (
                "place_descent_execution_time_s",
                descent_time_deltas,
            ),
            ("physical_place_error_m", placement_error_deltas),
            ("max_payload_relative_drift_m", payload_drift_deltas),
            (
                "rms_transfer_arm_position_error_rad",
                tracking_rms_deltas,
            ),
        )
        for field, differences in metric_pairs:
            cartesian_value = finite_values([cartesian.get(field)])
            servo_value = finite_values([servo.get(field)])
            if cartesian_value and servo_value:
                differences.append(servo_value[0] - cartesian_value[0])
        cartesian_time = finite_values(
            [cartesian.get("place_descent_execution_time_s")]
        )
        servo_time = finite_values([servo.get("place_descent_execution_time_s")])
        if cartesian_time and servo_time and servo_time[0] > 0.0:
            descent_speedup_ratios.append(cartesian_time[0] / servo_time[0])
    state_deltas = finite_values(
        item.get("max_pre_descent_joint_delta_rad") for item in pair_diagnostics
    )
    return {
        "total_trials": len(rows),
        "backends": backends,
        "matched_state_tolerance_rad": matched_state_tolerance_rad,
        "complete_pair_count": sum(item["complete"] for item in pair_diagnostics),
        "matched_state_pair_count": sum(item["matched"] for item in pair_diagnostics),
        "matched_both_successful_pair_count": sum(
            item["matched"] and item["both_successful"]
            for item in pair_diagnostics
        ),
        "mean_pre_descent_joint_delta_rad": (
            sum(state_deltas) / len(state_deltas) if state_deltas else None
        ),
        "max_pre_descent_joint_delta_rad": max(state_deltas, default=None),
        "mean_servo_minus_cartesian_descent_time_s": (
            sum(descent_time_deltas) / len(descent_time_deltas)
            if descent_time_deltas
            else None
        ),
        "mean_cartesian_to_servo_descent_speedup_ratio": (
            sum(descent_speedup_ratios) / len(descent_speedup_ratios)
            if descent_speedup_ratios
            else None
        ),
        "mean_servo_minus_cartesian_place_error_m": (
            sum(placement_error_deltas) / len(placement_error_deltas)
            if placement_error_deltas
            else None
        ),
        "mean_servo_minus_cartesian_payload_drift_m": (
            sum(payload_drift_deltas) / len(payload_drift_deltas)
            if payload_drift_deltas
            else None
        ),
        "mean_servo_minus_cartesian_tracking_rms_rad": (
            sum(tracking_rms_deltas) / len(tracking_rms_deltas)
            if tracking_rms_deltas
            else None
        ),
        "pair_state_diagnostics": pair_diagnostics,
    }


def render_backend_comparison_markdown(summary: Dict[str, object]) -> str:
    """Render the matched Cartesian/Servo comparison report."""
    lines = [
        "# Place Descent Backend Comparison",
        "",
        (
            "All rows are physically loaded trials. Servo replays the Cartesian "
            "grasp yaw and pre-place joint endpoint for each pair."
        ),
        "",
        (
            "| Backend | Trials | Success rate (95% CI) | Descent mean / median | "
            "Transfer RMS | Payload drift | Place error mean / P95 | Mean tilt | "
            "Fallbacks |"
        ),
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    backends = summary.get("backends", {})
    for backend in ("cartesian", "servo"):
        values = backends.get(backend, {})
        rate = values.get("success_rate")
        interval = values.get("success_rate_ci")
        rate_text = "N/A"
        if rate is not None:
            rate_text = f"{100.0 * rate:.1f}%"
            if interval is not None:
                rate_text += (
                    f" ({100.0 * interval[0]:.1f}%-"
                    f"{100.0 * interval[1]:.1f}%)"
                )
        mean_descent = values.get("mean_place_descent_execution_time_s")
        median_descent = values.get("median_place_descent_execution_time_s")
        rms = values.get("mean_rms_transfer_arm_position_error_rad")
        drift = values.get("mean_max_payload_relative_drift_m")
        mean_place = values.get("mean_physical_place_error_m")
        p95_place = values.get("p95_physical_place_error_m")
        tilt = values.get("mean_physical_final_tilt_deg")
        lines.append(
            "| {backend} | {trials} | {rate} | {descent} | {rms} | {drift} | "
            "{place} | {tilt} | {fallbacks} |".format(
                backend=backend,
                trials=values.get("trials", 0),
                rate=rate_text,
                descent=(
                    f"{mean_descent:.2f} / {median_descent:.2f} s"
                    if mean_descent is not None and median_descent is not None
                    else "N/A"
                ),
                rms=f"{rms:.6f} rad" if rms is not None else "N/A",
                drift=f"{drift:.4f} m" if drift is not None else "N/A",
                place=(
                    f"{mean_place:.4f} / {p95_place:.4f} m"
                    if mean_place is not None and p95_place is not None
                    else "N/A"
                ),
                tilt=f"{tilt:.3f} deg" if tilt is not None else "N/A",
                fallbacks=values.get("servo_cartesian_fallback_count", 0),
            )
        )
    matched = summary.get("matched_state_pair_count", 0)
    complete = summary.get("complete_pair_count", 0)
    both_successful = summary.get("matched_both_successful_pair_count", 0)
    tolerance = summary.get("matched_state_tolerance_rad", 0.05)
    mean_delta = summary.get("mean_pre_descent_joint_delta_rad")
    max_delta = summary.get("max_pre_descent_joint_delta_rad")
    time_delta = summary.get("mean_servo_minus_cartesian_descent_time_s")
    speedup = summary.get("mean_cartesian_to_servo_descent_speedup_ratio")
    place_delta = summary.get("mean_servo_minus_cartesian_place_error_m")
    drift_delta = summary.get("mean_servo_minus_cartesian_payload_drift_m")
    rms_delta = summary.get("mean_servo_minus_cartesian_tracking_rms_rad")
    lines.extend(
        [
            "",
            "## Pairing Gate",
            "",
            (
                f"- Matched pre-descent states: {matched}/{complete} pairs "
                f"(required max joint delta <= {tolerance:.3f} rad)."
            ),
            f"- Matched pairs with both backends successful: {both_successful}/{matched}.",
            (
                f"- Pair state delta: mean {mean_delta:.5f} rad, "
                f"max {max_delta:.5f} rad."
                if mean_delta is not None and max_delta is not None
                else "- Pair state delta: N/A"
            ),
            (
                f"- Matched-pair descent: Servo {abs(time_delta):.2f} s faster "
                f"on average; Cartesian/Servo speedup {speedup:.2f}x."
                if time_delta is not None and time_delta < 0.0 and speedup is not None
                else "- Matched-pair descent speedup: N/A"
            ),
            (
                f"- Servo-minus-Cartesian mean deltas: place error "
                f"{place_delta:+.4f} m, payload drift {drift_delta:+.4f} m, "
                f"tracking RMS {rms_delta:+.6f} rad."
                if place_delta is not None
                and drift_delta is not None
                and rms_delta is not None
                else "- Matched-pair quality deltas: N/A"
            ),
            "- Do not attribute backend differences from unmatched pairs.",
            (
                "- Three matched pairs are an engineering checkpoint, not a "
                "final reliability claim; retain the Wilson intervals."
            ),
            "",
        ]
    )
    for backend in ("cartesian", "servo"):
        failures = backends.get(backend, {}).get("failure_reasons", [])
        if failures:
            lines.extend([f"## {backend.title()} Failures", ""])
            lines.extend(f"- {reason}" for reason in failures)
            lines.append("")
    return "\n".join(lines)


def summarize_servo_watchdog_safety(
    rows: Sequence[Dict[str, object]],
) -> Dict[str, object]:
    """Aggregate expected-failure Servo watchdog injection trials."""
    faults: Dict[str, object] = {}
    for fault_type in ("contact_loss", "payload_drift"):
        fault_rows = [
            row
            for row in rows
            if row.get("servo_fault_injection_type") == fault_type
        ]
        passed = sum(
            row.get("servo_watchdog_safe_stop_passed") is True
            for row in fault_rows
        )
        latencies = finite_values(
            row.get("servo_watchdog_stop_latency_s") for row in fault_rows
        )
        faults[fault_type] = {
            "trials": len(fault_rows),
            "safe_stops": passed,
            "safe_stop_rate": passed / len(fault_rows) if fault_rows else None,
            "safe_stop_rate_ci": wilson_interval(passed, len(fault_rows)),
            "mean_stop_latency_s": (
                sum(latencies) / len(latencies) if latencies else None
            ),
            "median_stop_latency_s": median(latencies) if latencies else None,
            "p95_stop_latency_s": percentile(latencies, 0.95),
            "max_stop_latency_s": max(latencies, default=None),
            "failed_verdict_reasons": [
                str(row.get("failure_reason"))
                for row in fault_rows
                if row.get("servo_watchdog_safe_stop_passed") is not True
            ],
        }
    nominal_rows = [
        row
        for row in rows
        if row.get("place_descent_backend") == "servo"
        and row.get("servo_fault_injection_type") in {None, "", "none"}
    ]
    nominal_watchdog_aborts = [
        row
        for row in nominal_rows
        if "servo_place_descent watchdog" in str(
            row.get("failure_reason") or ""
        ).lower()
    ]
    nominal_successes = sum(
        str(row.get("final_state")) == "SUCCESS" for row in nominal_rows
    )
    nominal = {
        "trials": len(nominal_rows),
        "successful_completions": nominal_successes,
        "completion_rate": (
            nominal_successes / len(nominal_rows) if nominal_rows else None
        ),
        "completion_rate_ci": wilson_interval(
            nominal_successes,
            len(nominal_rows),
        ),
        "watchdog_aborts": len(nominal_watchdog_aborts),
        "watchdog_abort_rate": (
            len(nominal_watchdog_aborts) / len(nominal_rows)
            if nominal_rows
            else None
        ),
        "watchdog_abort_reasons": [
            str(row.get("failure_reason")) for row in nominal_watchdog_aborts
        ],
    }
    limits = finite_values(
        row.get("servo_watchdog_stop_latency_limit_s") for row in rows
    )
    cases: List[Dict[str, object]] = []
    case_keys = sorted(
        {
            (
                str(row.get("servo_fault_injection_type")),
                int(row.get("servo_fault_injection_stage")),
            )
            for row in rows
            if row.get("servo_fault_injection_type")
            in {"contact_loss", "payload_drift"}
            and isinstance(row.get("servo_fault_injection_stage"), (int, float))
        }
    )
    for fault_type, stage in case_keys:
        case_rows = [
            row
            for row in rows
            if row.get("servo_fault_injection_type") == fault_type
            and int(row.get("servo_fault_injection_stage")) == stage
        ]
        case_latencies = finite_values(
            row.get("servo_watchdog_stop_latency_s") for row in case_rows
        )
        cases.append(
            {
                "fault_type": fault_type,
                "stage": stage,
                "trials": len(case_rows),
                "safe_stops": sum(
                    row.get("servo_watchdog_safe_stop_passed") is True
                    for row in case_rows
                ),
                "mean_stop_latency_s": (
                    sum(case_latencies) / len(case_latencies)
                    if case_latencies
                    else None
                ),
                "max_stop_latency_s": max(case_latencies, default=None),
            }
        )
    return {
        "total_trials": sum(values["trials"] for values in faults.values()),
        "safe_stops": sum(values["safe_stops"] for values in faults.values()),
        "stop_latency_limit_s": min(limits) if limits else None,
        "faults": faults,
        "cases": cases,
        "nominal": nominal,
    }


def render_servo_watchdog_safety_markdown(summary: Dict[str, object]) -> str:
    """Render the deterministic Servo watchdog expected-failure report."""
    lines = [
        "# Servo Watchdog Safety Injection",
        "",
        (
            "These are expected-failure trials. A pass requires the configured "
            "fault to trigger, the pipeline to stop for the matching reason, and "
            "the zero-command latency to remain within the configured limit."
        ),
        "",
        (
            "| Fault | Trials | Safe-stop rate (95% CI) | Mean / median latency | "
            "P95 / max latency |"
        ),
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    faults = summary.get("faults", {})
    for fault_type in ("contact_loss", "payload_drift"):
        values = faults.get(fault_type, {})
        rate = values.get("safe_stop_rate")
        interval = values.get("safe_stop_rate_ci")
        rate_text = "N/A"
        if rate is not None:
            rate_text = f"{100.0 * rate:.1f}%"
            if interval is not None:
                rate_text += (
                    f" ({100.0 * interval[0]:.1f}%-"
                    f"{100.0 * interval[1]:.1f}%)"
                )
        mean_latency = values.get("mean_stop_latency_s")
        median_latency = values.get("median_stop_latency_s")
        p95_latency = values.get("p95_stop_latency_s")
        max_latency = values.get("max_stop_latency_s")
        lines.append(
            "| {fault} | {trials} | {rate} | {mean_median} | {p95_max} |".format(
                fault=fault_type,
                trials=values.get("trials", 0),
                rate=rate_text,
                mean_median=(
                    f"{mean_latency:.4f} / {median_latency:.4f} s"
                    if mean_latency is not None and median_latency is not None
                    else "N/A"
                ),
                p95_max=(
                    f"{p95_latency:.4f} / {max_latency:.4f} s"
                    if p95_latency is not None and max_latency is not None
                    else "N/A"
                ),
            )
        )
    limit = summary.get("stop_latency_limit_s")
    lines.extend(
        [
            "",
            "## Acceptance Gate",
            "",
            f"- Safe stops: {summary.get('safe_stops', 0)}/{summary.get('total_trials', 0)}.",
            (
                f"- Configured stop-latency limit: {limit:.3f} s."
                if limit is not None
                else "- Configured stop-latency limit: N/A"
            ),
            (
                "- Injection is synthetic sensor-path testing; retain separate "
                "physical collision and payload-slip validation."
            ),
            "",
        ]
    )
    nominal = summary.get("nominal", {})
    nominal_trials = nominal.get("trials", 0)
    if nominal_trials:
        completion_rate = nominal.get("completion_rate")
        completion_interval = nominal.get("completion_rate_ci")
        completion_text = f"{100.0 * completion_rate:.1f}%"
        if completion_interval is not None:
            completion_text += (
                f" ({100.0 * completion_interval[0]:.1f}%-"
                f"{100.0 * completion_interval[1]:.1f}%)"
            )
        lines.extend(
            [
                "## Nominal Servo Baseline",
                "",
                (
                    "- Successful nominal completions: "
                    f"{nominal.get('successful_completions', 0)}/"
                    f"{nominal_trials} ({completion_text})."
                ),
                (
                    "- Nominal watchdog aborts: "
                    f"{nominal.get('watchdog_aborts', 0)}/{nominal_trials}."
                ),
                (
                    "- A nominal watchdog abort is not automatically a false "
                    "positive; inspect physical contact and payload telemetry."
                ),
                "",
            ]
        )
    cases = summary.get("cases", [])
    if cases:
        lines.extend(
            [
                "## Stage Matrix",
                "",
                "| Fault | Stage | Trials | Safe stops | Mean / max latency |",
                "| --- | ---: | ---: | ---: | ---: |",
            ]
        )
        for case in cases:
            mean_latency = case.get("mean_stop_latency_s")
            max_latency = case.get("max_stop_latency_s")
            lines.append(
                "| {fault} | {stage} | {trials} | {safe} | {latency} |".format(
                    fault=case.get("fault_type"),
                    stage=case.get("stage"),
                    trials=case.get("trials"),
                    safe=case.get("safe_stops"),
                    latency=(
                        f"{mean_latency:.4f} / {max_latency:.4f} s"
                        if mean_latency is not None and max_latency is not None
                        else "N/A"
                    ),
                )
            )
        lines.append("")
    for fault_type in ("contact_loss", "payload_drift"):
        failures = faults.get(fault_type, {}).get("failed_verdict_reasons", [])
        if failures:
            lines.extend([f"## {fault_type} Verdict Failures", ""])
            lines.extend(f"- {reason}" for reason in failures)
            lines.append("")
    return "\n".join(lines)


def summarize_physical_disturbance_safety(
    rows: Sequence[Dict[str, object]],
) -> Dict[str, object]:
    """Aggregate real Gazebo wrench-disturbance safe-stop trials."""
    disturbance_rows = [
        row for row in rows if row.get("physical_disturbance_enabled") is True
    ]
    safe_stops = sum(
        row.get("physical_disturbance_safe_stop_passed") is True
        for row in disturbance_rows
    )
    latencies = finite_values(
        row.get("physical_disturbance_stop_latency_s")
        for row in disturbance_rows
    )
    limits = finite_values(
        row.get("physical_disturbance_stop_latency_limit_s")
        for row in disturbance_rows
    )
    strictest_limit = min(limits) if limits else None
    stages: List[Dict[str, object]] = []
    configurations: List[Dict[str, object]] = []
    configuration_keys = sorted(
        {
            (
                float(row.get("physical_disturbance_force_x_n") or 0.0),
                float(row.get("physical_disturbance_force_y_n") or 0.0),
                float(row.get("physical_disturbance_force_z_n") or 0.0),
                float(row.get("physical_disturbance_duration_s") or 0.0),
                float(row.get("physical_disturbance_delay_s") or 0.0),
            )
            for row in disturbance_rows
        }
    )
    for force_x, force_y, force_z, duration_s, delay_s in configuration_keys:
        matching_rows = [
            row
            for row in disturbance_rows
            if (
                float(row.get("physical_disturbance_force_x_n") or 0.0),
                float(row.get("physical_disturbance_force_y_n") or 0.0),
                float(row.get("physical_disturbance_force_z_n") or 0.0),
                float(row.get("physical_disturbance_duration_s") or 0.0),
                float(row.get("physical_disturbance_delay_s") or 0.0),
            )
            == (force_x, force_y, force_z, duration_s, delay_s)
        ]
        configurations.append(
            {
                "force_n": [force_x, force_y, force_z],
                "duration_s": duration_s,
                "delay_s": delay_s,
                "trials": len(matching_rows),
                "safe_stops": sum(
                    row.get("physical_disturbance_safe_stop_passed") is True
                    for row in matching_rows
                ),
            }
        )
    stage_ids = sorted(
        {
            int(row["physical_disturbance_stage"])
            for row in disturbance_rows
            if isinstance(row.get("physical_disturbance_stage"), (int, float))
        }
    )
    for stage in stage_ids:
        stage_rows = [
            row
            for row in disturbance_rows
            if int(row.get("physical_disturbance_stage", -1)) == stage
        ]
        stage_latencies = finite_values(
            row.get("physical_disturbance_stop_latency_s") for row in stage_rows
        )
        stages.append(
            {
                "stage": stage,
                "trials": len(stage_rows),
                "applied": sum(
                    row.get("physical_disturbance_applied") is True
                    for row in stage_rows
                ),
                "cleared": sum(
                    row.get("physical_disturbance_cleared") is True
                    for row in stage_rows
                ),
                "safe_stops": sum(
                    row.get("physical_disturbance_safe_stop_passed") is True
                    for row in stage_rows
                ),
                "mean_stop_latency_s": (
                    sum(stage_latencies) / len(stage_latencies)
                    if stage_latencies
                    else None
                ),
                "max_stop_latency_s": max(stage_latencies, default=None),
            }
        )
    return {
        "total_trials": len(disturbance_rows),
        "disturbances_applied": sum(
            row.get("physical_disturbance_applied") is True
            for row in disturbance_rows
        ),
        "disturbances_cleared": sum(
            row.get("physical_disturbance_cleared") is True
            for row in disturbance_rows
        ),
        "safe_stops": safe_stops,
        "safe_stop_rate": (
            safe_stops / len(disturbance_rows) if disturbance_rows else None
        ),
        "safe_stop_rate_ci": wilson_interval(safe_stops, len(disturbance_rows)),
        "mean_stop_latency_s": (
            sum(latencies) / len(latencies) if latencies else None
        ),
        "median_stop_latency_s": median(latencies) if latencies else None,
        "p95_stop_latency_s": percentile(latencies, 0.95),
        "max_stop_latency_s": max(latencies, default=None),
        "stop_latency_limit_s": strictest_limit,
        "stop_latency_limits_s": sorted(set(limits)),
        "all_meet_strictest_stop_latency_limit": (
            len(latencies) == len(disturbance_rows)
            and strictest_limit is not None
            and all(latency <= strictest_limit for latency in latencies)
        ),
        "configurations": configurations,
        "stages": stages,
        "failed_verdict_reasons": [
            str(row.get("failure_reason") or "disturbance verdict incomplete")
            for row in disturbance_rows
            if row.get("physical_disturbance_safe_stop_passed") is not True
        ],
    }


def render_physical_disturbance_safety_markdown(
    summary: Dict[str, object],
) -> str:
    """Render the real-physics Servo disturbance safety report."""
    total = int(summary.get("total_trials", 0))
    safe_stops = int(summary.get("safe_stops", 0))
    rate = summary.get("safe_stop_rate")
    interval = summary.get("safe_stop_rate_ci")
    rate_text = "N/A"
    if rate is not None:
        rate_text = f"{100.0 * float(rate):.1f}%"
        if interval is not None:
            rate_text += (
                f" ({100.0 * interval[0]:.1f}%-"
                f"{100.0 * interval[1]:.1f}%)"
            )
    limit = summary.get("stop_latency_limit_s")
    limits = summary.get("stop_latency_limits_s", [])
    lines = [
        "# Gazebo Physical Disturbance Safety",
        "",
        (
            "A pass requires the real Gazebo wrench to be applied and cleared, "
            "the Servo watchdog to stop the matching descent stage for measured "
            "contact loss or payload drift, and the zero command to meet the "
            "configured latency limit. Task failure alone is not a pass."
        ),
        "",
        f"- Safe stops: {safe_stops}/{total} ({rate_text}).",
        (
            "- Disturbance lifecycle: "
            f"{summary.get('disturbances_applied', 0)} applied, "
            f"{summary.get('disturbances_cleared', 0)} cleared."
        ),
        (
            "- Configured per-trial stop-latency limits: "
            + ", ".join(f"{float(value):.3f} s" for value in limits)
            + "."
            if limits
            else "- Configured per-trial stop-latency limits: N/A."
        ),
        (
            f"- All observed stops meet the strictest {float(limit):.3f} s "
            "limit: "
            f"{'yes' if summary.get('all_meet_strictest_stop_latency_limit') else 'no'}."
            if limit is not None
            else "- Strictest stop-latency comparison: N/A."
        ),
        "",
        "## Disturbance Configurations",
        "",
        "| Force [x, y, z] | Duration | Stage delay | Trials | Safe stops |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for configuration in summary.get("configurations", []):
        force = configuration.get("force_n", [0.0, 0.0, 0.0])
        lines.append(
            "| [{:.1f}, {:.1f}, {:.1f}] N | {:.3f} s | {:.3f} s | {} | {} |".format(
                float(force[0]),
                float(force[1]),
                float(force[2]),
                float(configuration.get("duration_s", 0.0)),
                float(configuration.get("delay_s", 0.0)),
                configuration.get("trials", 0),
                configuration.get("safe_stops", 0),
            )
        )
    lines.extend(
        [
        "",
        "## Stage Matrix",
        "",
        "| Stage | Trials | Applied | Cleared | Safe stops | Mean / max latency |",
        "| ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for stage in summary.get("stages", []):
        mean_latency = stage.get("mean_stop_latency_s")
        max_latency = stage.get("max_stop_latency_s")
        latency_text = "N/A"
        if mean_latency is not None and max_latency is not None:
            latency_text = f"{mean_latency:.4f} / {max_latency:.4f} s"
        lines.append(
            "| {stage} | {trials} | {applied} | {cleared} | {safe} | {latency} |".format(
                stage=stage.get("stage"),
                trials=stage.get("trials"),
                applied=stage.get("applied"),
                cleared=stage.get("cleared"),
                safe=stage.get("safe_stops"),
                latency=latency_text,
            )
        )
    lines.append("")
    failures = summary.get("failed_verdict_reasons", [])
    if failures:
        lines.extend(["## Verdict Failures", ""])
        lines.extend(f"- {reason}" for reason in failures)
        lines.append("")
    return "\n".join(lines)


def csv_fields() -> List[str]:
    """Return the stable synchronized telemetry CSV schema."""
    fields = [
        "timestamp_s",
        "elapsed_s",
        "label",
        "stage",
        "mode",
        "payload_phase",
        "payload_epoch",
        "left_contact",
        "right_contact",
        "bilateral_contact",
        "contact_names",
        "target_x",
        "target_y",
        "target_z",
        "hand_x",
        "hand_y",
        "hand_z",
        "hand_qx",
        "hand_qy",
        "hand_qz",
        "hand_qw",
        "object_in_hand_x",
        "object_in_hand_y",
        "object_in_hand_z",
        "payload_relative_drift_m",
    ]
    for prefix, joints in (
        ("arm_reference", ARM_JOINTS),
        ("arm_feedback", ARM_JOINTS),
        ("arm_error", ARM_JOINTS),
        ("arm_output", ARM_JOINTS),
        ("gripper_reference", FINGER_JOINTS),
        ("gripper_feedback", FINGER_JOINTS),
        ("gripper_error", FINGER_JOINTS),
        ("gripper_output", FINGER_JOINTS),
        ("joint_state", FINGER_JOINTS),
    ):
        fields.extend(f"{prefix}_{joint}" for joint in joints)
    return fields


def main(args: List[str] | None = None) -> None:
    """Run the ROS2 transfer diagnostics recorder."""
    rclpy = require_ros2()
    from control_msgs.msg import JointTrajectoryControllerState
    from geometry_msgs.msg import Pose
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from ros_gz_interfaces.msg import Contacts
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformException, TransformListener
    from rclpy.time import Time

    class TransferDiagnosticsRecorder(Node):
        def __init__(self) -> None:
            super().__init__("transfer_diagnostics_recorder")
            self.declare_parameter("csv_path", "transfer_diagnostics.csv")
            self.declare_parameter("summary_path", "transfer_diagnostics.json")
            self.declare_parameter("label", "loaded")
            self.declare_parameter("sample_rate_hz", 20.0)
            self.declare_parameter("target_frame", "world")
            self.declare_parameter("end_effector_link", "panda_hand")
            self.declare_parameter("target_pose_topic", "/gazebo/target_pose")
            self.declare_parameter("contact_topic", "/gazebo/target_cube/contacts")
            self.declare_parameter("left_finger_token", "panda_leftfinger")
            self.declare_parameter("right_finger_token", "panda_rightfinger")
            self.declare_parameter("contact_max_age_s", 0.25)
            self.declare_parameter("payload_drift_threshold_m", 0.015)
            self.declare_parameter("finalize_delay_s", 0.5)

            self.label = str(self.get_parameter("label").value)
            self.target_frame = str(self.get_parameter("target_frame").value)
            self.end_effector_link = str(self.get_parameter("end_effector_link").value)
            self.left_finger_token = str(self.get_parameter("left_finger_token").value)
            self.right_finger_token = str(self.get_parameter("right_finger_token").value)
            self.contact_max_age_s = max(
                0.0, float(self.get_parameter("contact_max_age_s").value)
            )
            self.payload_drift_threshold_m = max(
                0.0, float(self.get_parameter("payload_drift_threshold_m").value)
            )
            self.csv_path = Path(str(self.get_parameter("csv_path").value)).expanduser()
            self.summary_path = Path(
                str(self.get_parameter("summary_path").value)
            ).expanduser()
            self.csv_path.parent.mkdir(parents=True, exist_ok=True)
            self.summary_path.parent.mkdir(parents=True, exist_ok=True)
            self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
            self.writer = csv.DictWriter(self.csv_file, fieldnames=csv_fields())
            self.writer.writeheader()

            self.rows: List[Dict[str, object]] = []
            self.first_sample_ns: Optional[int] = None
            self.stage = "WAITING"
            self.mode = "waiting"
            self.payload_phase = False
            self.payload_epoch = "none"
            self.target_position: Optional[Tuple[float, float, float]] = None
            self.hand_position: Optional[Tuple[float, float, float]] = None
            self.hand_orientation: Optional[Tuple[float, float, float, float]] = None
            self.object_in_hand: Optional[Tuple[float, float, float]] = None
            self.baseline_object_in_hand: Optional[Tuple[float, float, float]] = None
            self.arm_state = None
            self.gripper_state = None
            self.joint_positions: Dict[str, float] = {}
            self.left_contact = False
            self.right_contact = False
            self.contact_names: List[str] = []
            self.last_contact_ns: Optional[int] = None
            self.terminal_result: Optional[Dict[str, object]] = None
            self.finalize_timer = None
            self.finalized = False
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

            durable_qos = QoSProfile(depth=10)
            durable_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.summary_publisher = self.create_publisher(
                String, "/transfer_diagnostics_summary", durable_qos
            )
            self.create_subscription(
                JointTrajectoryControllerState,
                "/panda_arm_controller/controller_state",
                self.on_arm_state,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                JointTrajectoryControllerState,
                "/panda_hand_controller/controller_state",
                self.on_gripper_state,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                JointState, "/joint_states", self.on_joint_state, qos_profile_sensor_data
            )
            self.create_subscription(
                Pose,
                str(self.get_parameter("target_pose_topic").value),
                self.on_target_pose,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Contacts,
                str(self.get_parameter("contact_topic").value),
                self.on_contacts,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                String, "/pick_demo_state", self.on_demo_state, durable_qos
            )
            self.create_subscription(
                String, "/pick_plan_result", self.on_result, durable_qos
            )
            period = 1.0 / max(1.0, float(self.get_parameter("sample_rate_hz").value))
            self.sample_timer = self.create_timer(period, self.sample)
            self.get_logger().info(
                f"Recording transfer diagnostics to {self.csv_path} ({self.label})."
            )

        def on_arm_state(self, message) -> None:
            self.arm_state = message

        def on_gripper_state(self, message) -> None:
            self.gripper_state = message

        def on_joint_state(self, message) -> None:
            self.joint_positions = {
                str(name): float(position)
                for name, position in zip(message.name, message.position)
            }

        def on_target_pose(self, message) -> None:
            self.target_position = (
                float(message.position.x),
                float(message.position.y),
                float(message.position.z),
            )

        def on_contacts(self, message) -> None:
            left, right, names = contact_sides(
                message, self.left_finger_token, self.right_finger_token
            )
            self.left_contact = left
            self.right_contact = right
            self.contact_names = names
            if left or right:
                self.last_contact_ns = self.get_clock().now().nanoseconds

        def on_demo_state(self, message) -> None:
            try:
                payload = json.loads(message.data)
            except json.JSONDecodeError:
                return
            self.stage = str(payload.get("step") or "UNSET")
            self.mode = str(payload.get("mode") or "unset")
            event = payload_phase_event(self.stage, self.mode)
            if event == "reset":
                self.payload_phase = False
                self.payload_epoch = "none"
                self.baseline_object_in_hand = None
            elif event == "probe_start":
                self.payload_phase = True
                self.payload_epoch = "grasp_probe"
                self.baseline_object_in_hand = None
            elif event == "transfer_start":
                self.payload_phase = True
                self.payload_epoch = "transfer"
                self.baseline_object_in_hand = None

        def on_result(self, message) -> None:
            try:
                payload = json.loads(message.data)
            except json.JSONDecodeError:
                return
            if payload.get("final_state") not in {"SUCCESS", "FAILED"}:
                return
            self.terminal_result = payload
            if self.finalize_timer is not None:
                self.finalize_timer.cancel()
            delay = max(0.0, float(self.get_parameter("finalize_delay_s").value))
            self.finalize_timer = self.create_timer(delay or 0.01, self.finalize)

        def recent_contacts(self, now_ns: int) -> Tuple[bool, bool]:
            if self.last_contact_ns is None:
                return False, False
            if (now_ns - self.last_contact_ns) / 1e9 > self.contact_max_age_s:
                return False, False
            return self.left_contact, self.right_contact

        def update_hand_pose(self) -> None:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame, self.end_effector_link, Time()
                )
            except TransformException:
                return
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            self.hand_position = (
                float(translation.x),
                float(translation.y),
                float(translation.z),
            )
            self.hand_orientation = (
                float(rotation.x),
                float(rotation.y),
                float(rotation.z),
                float(rotation.w),
            )
            if self.target_position is not None:
                self.object_in_hand = point_in_frame(
                    self.hand_position,
                    self.hand_orientation,
                    self.target_position,
                )

        def sample(self) -> None:
            if self.finalized:
                return
            now_ns = self.get_clock().now().nanoseconds
            if self.first_sample_ns is None:
                self.first_sample_ns = now_ns
            self.update_hand_pose()
            left, right = self.recent_contacts(now_ns)
            bilateral = left and right
            if (
                self.payload_phase
                and bilateral
                and self.baseline_object_in_hand is None
                and self.object_in_hand is not None
            ):
                self.baseline_object_in_hand = self.object_in_hand

            row: Dict[str, object] = {field: None for field in csv_fields()}
            row.update(
                {
                    "timestamp_s": now_ns / 1e9,
                    "elapsed_s": (now_ns - self.first_sample_ns) / 1e9,
                    "label": self.label,
                    "stage": self.stage,
                    "mode": self.mode,
                    "payload_phase": self.payload_phase,
                    "payload_epoch": self.payload_epoch,
                    "left_contact": left,
                    "right_contact": right,
                    "bilateral_contact": bilateral,
                    "contact_names": "|".join(self.contact_names),
                }
            )
            if self.target_position is not None:
                row.update(dict(zip(("target_x", "target_y", "target_z"), self.target_position)))
            if self.hand_position is not None:
                row.update(dict(zip(("hand_x", "hand_y", "hand_z"), self.hand_position)))
            if self.hand_orientation is not None:
                row.update(
                    dict(
                        zip(
                            ("hand_qx", "hand_qy", "hand_qz", "hand_qw"),
                            self.hand_orientation,
                        )
                    )
                )
            if self.object_in_hand is not None:
                row.update(
                    dict(
                        zip(
                            ("object_in_hand_x", "object_in_hand_y", "object_in_hand_z"),
                            self.object_in_hand,
                        )
                    )
                )
            if self.baseline_object_in_hand is not None and self.object_in_hand is not None:
                row["payload_relative_drift_m"] = local_point_drift(
                    self.baseline_object_in_hand,
                    self.object_in_hand,
                )
            self.add_controller_values(row, "arm", self.arm_state, ARM_JOINTS)
            self.add_controller_values(
                row, "gripper", self.gripper_state, FINGER_JOINTS
            )
            for joint in FINGER_JOINTS:
                row[f"joint_state_{joint}"] = self.joint_positions.get(joint)
            self.rows.append(row)
            self.writer.writerow(row)
            self.csv_file.flush()

        @staticmethod
        def add_controller_values(row, prefix, message, joints) -> None:
            if message is None:
                return
            for field in ("reference", "feedback", "error", "output"):
                values = controller_point_values(message, field)
                for joint in joints:
                    key = f"{prefix}_{field}_{joint}"
                    if key in row:
                        row[key] = values.get(joint)

        def finalize(self) -> None:
            if self.finalized:
                return
            if self.finalize_timer is not None:
                self.finalize_timer.cancel()
                self.finalize_timer = None
            self.sample()
            self.finalized = True
            summary = summarize_rows(self.rows, self.payload_drift_threshold_m)
            summary.update(
                {
                    "label": self.label,
                    "csv_path": str(self.csv_path),
                    "final_state": (
                        self.terminal_result.get("final_state")
                        if self.terminal_result is not None
                        else None
                    ),
                    "failure_reason": (
                        self.terminal_result.get("failure_reason")
                        if self.terminal_result is not None
                        else None
                    ),
                }
            )
            self.summary_path.write_text(
                json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8"
            )
            self.csv_file.flush()
            output = String()
            output.data = json.dumps(summary)
            self.summary_publisher.publish(output)
            self.get_logger().info(json.dumps(summary))

        def destroy_node(self) -> bool:
            if not self.finalized and self.rows:
                self.finalize()
            if not self.csv_file.closed:
                self.csv_file.close()
            return super().destroy_node()

    rclpy.init(args=args)
    node = TransferDiagnosticsRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
