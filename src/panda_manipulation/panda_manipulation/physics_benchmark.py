"""Aggregate and report repeated Gazebo physical pick validations."""

from __future__ import annotations

import csv
import html
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from statistics import fmean, pstdev
from typing import Any, Dict, Iterable, List


CSV_FIELDS = [
    "trial",
    "scenario_index",
    "scenario_name",
    "perception_mode",
    "planner_id",
    "obstacle_x",
    "obstacle_y",
    "obstacle_z",
    "obstacle_size_x",
    "obstacle_size_y",
    "obstacle_size_z",
    "final_state",
    "pipeline_state",
    "initial_x",
    "initial_y",
    "initial_z",
    "max_z",
    "final_x",
    "final_y",
    "final_z",
    "lift_delta_m",
    "place_error_m",
    "final_tilt_deg",
    "direct_path_fraction",
    "direct_path_unchecked_fraction",
    "direct_path_blocked",
    "perception_validation_state",
    "perception_samples",
    "perception_mean_error_m",
    "perception_max_error_m",
    "perception_std_error_m",
    "perception_mean_x_error_m",
    "perception_mean_y_error_m",
    "perception_mean_z_error_m",
    "perception_first_detection_latency_s",
    "perception_validation_duration_s",
    "perception_detection_rate",
    "ompl_planning_time_s",
    "ompl_joint_path_length_rad",
    "cpp_trajectory_message_count",
    "cpp_trajectory_segment_count",
    "cpp_joint_path_length_rad",
    "cpp_max_joint_step_rad",
    "cpp_integrated_squared_acceleration",
    "cpp_min_normalized_joint_limit_margin",
    "elapsed_s",
    "failure_category",
    "reason",
]


@dataclass(frozen=True)
class ChallengeScenario:
    name: str
    target_x: float
    target_y: float
    obstacle_pose_xyz: tuple[float, float, float]
    obstacle_size_xyz: tuple[float, float, float]


CHALLENGE_SCENARIOS = [
    ChallengeScenario(
        "center_tall_barrier",
        0.55,
        0.0,
        (0.37, 0.0, 0.18),
        (0.12, 0.18, 0.36),
    ),
    ChallengeScenario(
        "positive_y_barrier",
        0.54,
        -0.10,
        (0.37, 0.08, 0.17),
        (0.12, 0.20, 0.34),
    ),
    ChallengeScenario(
        "negative_y_barrier",
        0.54,
        0.10,
        (0.37, -0.08, 0.17),
        (0.12, 0.20, 0.34),
    ),
]


def pipeline_process_exit_reason(line: str) -> str | None:
    """Detect a launch child crash so a benchmark can fail without timing out."""
    if "pick_plan_pipeline" in line and "process has died" in line:
        return "pick pipeline process exited before physical validation"
    if "aruco_pose_estimator" in line and "process has died" in line:
        return "perception failed: ArUco pose estimator process exited"
    if "rgbd_target_pose_estimator" in line and "process has died" in line:
        return "perception failed: RGB-D target pose estimator process exited"
    if (
        "spawner-" in line
        and "process has died" in line
        and any(
            controller in line
            for controller in (
                "joint_state_broadcaster",
                "panda_arm_controller",
                "panda_hand_controller",
            )
        )
    ):
        return "infrastructure startup failed: controller spawner exited before activation"
    return None


def failure_category(reason: str | None, final_state: str = "FAILED") -> str | None:
    """Map detailed failure text to a stable report category."""
    if final_state == "SUCCESS":
        return None
    normalized = str(reason or "").lower()
    if (
        "infrastructure startup" in normalized
        or "controller spawner exited" in normalized
        or ("controller action" in normalized and "not available" in normalized)
    ):
        return "infrastructure_startup"
    if "timed out" in normalized:
        return "timeout"
    if "perception" in normalized or "aruco" in normalized:
        return "perception"
    if (
        "gripper_status" in normalized
        or "follow_joint_trajectory_error" in normalized
        or "goal_time_tolerance" in normalized
    ):
        return "controller_execution"
    if "physical_grasp_check" in normalized or "grasp_recenter" in normalized:
        return "physical_grasp"
    if "cartesian_" in normalized or "fraction=" in normalized:
        return "cartesian_motion"
    if "candidate validation" in normalized or "joint_path_length" in normalized:
        return "planner_constraint"
    if "moveit_error_code" in normalized or "planning failed" in normalized:
        return "motion_planning"
    return "other"


def wilson_interval(successes: int, total: int) -> tuple[float, float] | None:
    """Return a two-sided 95% Wilson score interval for a binomial rate."""
    if total <= 0:
        return None
    z = 1.959963984540054
    proportion = successes / total
    denominator = 1.0 + z * z / total
    center = (proportion + z * z / (2.0 * total)) / denominator
    margin = (
        z
        * math.sqrt(
            proportion * (1.0 - proportion) / total
            + z * z / (4.0 * total * total)
        )
        / denominator
    )
    return max(0.0, center - margin), min(1.0, center + margin)


def reclassify_infrastructure_failures(
    rows: Iterable[Dict[str, Any]],
    log_dir: str | Path,
) -> List[Dict[str, Any]]:
    """Use saved launch logs to distinguish startup failures from task failures."""
    classified_rows = [dict(row) for row in rows]
    log_root = Path(log_dir)
    for row in classified_rows:
        final_state = str(row.get("final_state") or "FAILED")
        reason = str(row.get("reason") or "unknown failure")
        if final_state != "SUCCESS" and "timed out" in reason.lower():
            trial = int(row["trial"])
            log_path = log_root / f"trial_{trial:03d}.log"
            if log_path.exists():
                log_text = log_path.read_text(encoding="utf-8", errors="replace")
                dead_controllers = [
                    controller
                    for controller in (
                        "joint_state_broadcaster",
                        "panda_arm_controller",
                        "panda_hand_controller",
                    )
                    if controller in log_text
                    and any(
                        controller in line and "process has died" in line
                        for line in log_text.splitlines()
                    )
                ]
                if dead_controllers:
                    reason = (
                        "infrastructure startup failed: controller spawners exited "
                        f"before activation ({', '.join(dead_controllers)})"
                    )
                    row["reason"] = reason
        row["failure_category"] = failure_category(reason, final_state)
    return classified_rows


def write_trial_world(
    source_world: str | Path,
    output_world: str | Path,
    target_x: float,
    target_y: float,
    obstacle_pose_xyz: tuple[float, float, float] | None = None,
    obstacle_size_xyz: tuple[float, float, float] | None = None,
) -> None:
    tree = ET.parse(source_world)
    target = tree.getroot().find(".//model[@name='target_cube']/pose")
    if target is None or target.text is None:
        raise RuntimeError("target_cube pose is missing from the Gazebo world")
    values = target.text.split()
    if len(values) != 6:
        raise RuntimeError(f"target_cube pose must contain 6 values, got: {target.text}")
    values[0] = f"{target_x:.6f}"
    values[1] = f"{target_y:.6f}"
    target.text = " ".join(values)
    if obstacle_pose_xyz is not None or obstacle_size_xyz is not None:
        if obstacle_pose_xyz is None or obstacle_size_xyz is None:
            raise ValueError("obstacle pose and size must be provided together")
        obstacle = tree.getroot().find(".//model[@name='obstacle_block']")
        if obstacle is None:
            raise RuntimeError("obstacle_block is missing from the Gazebo world")
        obstacle_pose = obstacle.find("pose")
        if obstacle_pose is None:
            raise RuntimeError("obstacle_block pose is missing from the Gazebo world")
        world_pose = (
            obstacle_pose_xyz[0],
            obstacle_pose_xyz[1],
            obstacle_pose_xyz[2] + 0.72,
        )
        obstacle_pose.text = " ".join(
            [*(f"{value:.6f}" for value in world_pose), "0", "0", "0"]
        )
        size_text = " ".join(f"{value:.6f}" for value in obstacle_size_xyz)
        size_elements = obstacle.findall(".//box/size")
        if not size_elements:
            raise RuntimeError("obstacle_block box size is missing from the Gazebo world")
        for size_element in size_elements:
            size_element.text = size_text
    output_path = Path(output_world)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_path, encoding="utf-8", xml_declaration=True)


def validation_to_row(
    trial: int,
    payload: Dict[str, Any],
    elapsed_s: float,
) -> Dict[str, Any]:
    """Flatten a validator payload into a stable CSV row."""
    initial = payload.get("initial_pose") or [None, None, None]
    final = payload.get("final_pose") or [None, None, None]
    cpp_metrics = payload.get("cpp_trajectory_metrics_summary") or {}
    return {
        "trial": trial,
        "scenario_index": payload.get("scenario_index"),
        "scenario_name": payload.get("scenario_name"),
        "perception_mode": payload.get("perception_mode", "synthetic"),
        "planner_id": payload.get("planner_id"),
        "obstacle_x": payload.get("obstacle_x"),
        "obstacle_y": payload.get("obstacle_y"),
        "obstacle_z": payload.get("obstacle_z"),
        "obstacle_size_x": payload.get("obstacle_size_x"),
        "obstacle_size_y": payload.get("obstacle_size_y"),
        "obstacle_size_z": payload.get("obstacle_size_z"),
        "final_state": payload.get("final_state", "FAILED"),
        "pipeline_state": payload.get("pipeline_state"),
        "initial_x": initial[0],
        "initial_y": initial[1],
        "initial_z": initial[2],
        "max_z": payload.get("max_z"),
        "final_x": final[0],
        "final_y": final[1],
        "final_z": final[2],
        "lift_delta_m": payload.get("lift_delta_m"),
        "place_error_m": payload.get("place_error_m"),
        "final_tilt_deg": payload.get("final_tilt_deg"),
        "direct_path_fraction": payload.get("direct_path_fraction"),
        "direct_path_unchecked_fraction": payload.get(
            "direct_path_unchecked_fraction"
        ),
        "direct_path_blocked": payload.get("direct_path_blocked"),
        "perception_validation_state": payload.get(
            "perception_validation_state"
        ),
        "perception_samples": payload.get("perception_samples"),
        "perception_mean_error_m": payload.get("perception_mean_error_m"),
        "perception_max_error_m": payload.get("perception_max_error_m"),
        "perception_std_error_m": payload.get("perception_std_error_m"),
        "perception_mean_x_error_m": payload.get(
            "perception_mean_x_error_m"
        ),
        "perception_mean_y_error_m": payload.get(
            "perception_mean_y_error_m"
        ),
        "perception_mean_z_error_m": payload.get(
            "perception_mean_z_error_m"
        ),
        "perception_first_detection_latency_s": payload.get(
            "perception_first_detection_latency_s"
        ),
        "perception_validation_duration_s": payload.get(
            "perception_validation_duration_s"
        ),
        "perception_detection_rate": payload.get(
            "perception_detection_rate"
        ),
        "ompl_planning_time_s": payload.get("ompl_planning_time_s"),
        "ompl_joint_path_length_rad": payload.get("ompl_joint_path_length_rad"),
        "cpp_trajectory_message_count": cpp_metrics.get("message_count"),
        "cpp_trajectory_segment_count": cpp_metrics.get(
            "trajectory_segment_count"
        ),
        "cpp_joint_path_length_rad": cpp_metrics.get("joint_path_length_rad"),
        "cpp_max_joint_step_rad": cpp_metrics.get("max_joint_step_rad"),
        "cpp_integrated_squared_acceleration": cpp_metrics.get(
            "integrated_squared_acceleration"
        ),
        "cpp_min_normalized_joint_limit_margin": cpp_metrics.get(
            "min_normalized_joint_limit_margin"
        ),
        "elapsed_s": elapsed_s,
        "failure_category": failure_category(
            payload.get("reason"),
            payload.get("final_state", "FAILED"),
        ),
        "reason": payload.get("reason", "unknown failure"),
    }


def summarize(rows: Iterable[Dict[str, Any]]) -> Dict[str, Any]:
    rows = list(rows)
    successful = [row for row in rows if row["final_state"] == "SUCCESS"]
    infrastructure_failures = [
        row
        for row in rows
        if row.get("failure_category") == "infrastructure_startup"
    ]
    valid_start_rows = [
        row
        for row in rows
        if row.get("failure_category") != "infrastructure_startup"
    ]
    valid_start_successes = [
        row for row in valid_start_rows if row["final_state"] == "SUCCESS"
    ]
    lift_values = [float(row["lift_delta_m"]) for row in successful]
    error_values = [float(row["place_error_m"]) for row in successful]
    tilt_values = [
        float(row["final_tilt_deg"])
        for row in successful
        if row.get("final_tilt_deg") is not None
    ]
    elapsed_values = [float(row["elapsed_s"]) for row in rows]
    valid_start_elapsed_values = [
        float(row["elapsed_s"]) for row in valid_start_rows
    ]
    direct_path_rows = [row for row in rows if row.get("direct_path_blocked") is not None]
    blocked_direct_paths = [row for row in direct_path_rows if row["direct_path_blocked"] is True]
    planning_time_values = [
        float(row["ompl_planning_time_s"])
        for row in successful
        if row.get("ompl_planning_time_s") is not None
    ]
    path_length_values = [
        float(row["ompl_joint_path_length_rad"])
        for row in successful
        if row.get("ompl_joint_path_length_rad") is not None
    ]
    cpp_telemetry_rows = [
        row
        for row in successful
        if int(row.get("cpp_trajectory_message_count") or 0) > 0
    ]
    cpp_segment_counts = [
        float(row["cpp_trajectory_segment_count"])
        for row in cpp_telemetry_rows
        if row.get("cpp_trajectory_segment_count") is not None
    ]
    cpp_path_lengths = [
        float(row["cpp_joint_path_length_rad"])
        for row in cpp_telemetry_rows
        if row.get("cpp_joint_path_length_rad") is not None
    ]
    cpp_max_steps = [
        float(row["cpp_max_joint_step_rad"])
        for row in cpp_telemetry_rows
        if row.get("cpp_max_joint_step_rad") is not None
    ]
    cpp_smoothness_costs = [
        float(row["cpp_integrated_squared_acceleration"])
        for row in cpp_telemetry_rows
        if row.get("cpp_integrated_squared_acceleration") is not None
    ]
    cpp_joint_limit_margins = [
        float(row["cpp_min_normalized_joint_limit_margin"])
        for row in cpp_telemetry_rows
        if row.get("cpp_min_normalized_joint_limit_margin") is not None
    ]
    initial_x_values = [float(row["initial_x"]) for row in rows if row.get("initial_x") is not None]
    initial_y_values = [float(row["initial_y"]) for row in rows if row.get("initial_y") is not None]

    def mean(values: List[float]) -> float | None:
        return fmean(values) if values else None

    def stddev(values: List[float]) -> float | None:
        return pstdev(values) if values else None

    failures: Dict[str, int] = {}
    failure_categories: Dict[str, int] = {}
    for row in rows:
        if row["final_state"] != "SUCCESS":
            reason = str(row.get("reason") or "unknown failure")
            failures[reason] = failures.get(reason, 0) + 1
            category = str(
                row.get("failure_category")
                or failure_category(reason, str(row.get("final_state")))
                or "other"
            )
            failure_categories[category] = failure_categories.get(category, 0) + 1

    planner_results: Dict[str, Dict[str, Any]] = {}
    planner_names = sorted(
        {str(row["planner_id"]) for row in rows if row.get("planner_id")}
    )
    for planner_name in planner_names:
        planner_rows = [row for row in rows if row.get("planner_id") == planner_name]
        planner_successes = [
            row for row in planner_rows if row["final_state"] == "SUCCESS"
        ]
        planner_valid_start_rows = [
            row
            for row in planner_rows
            if row.get("failure_category") != "infrastructure_startup"
        ]
        planner_valid_start_successes = [
            row
            for row in planner_valid_start_rows
            if row["final_state"] == "SUCCESS"
        ]
        planner_infrastructure_failures = [
            row
            for row in planner_rows
            if row.get("failure_category") == "infrastructure_startup"
        ]
        planner_elapsed = [float(row["elapsed_s"]) for row in planner_rows]
        planner_valid_start_elapsed = [
            float(row["elapsed_s"]) for row in planner_valid_start_rows
        ]
        planner_errors = [
            float(row["place_error_m"])
            for row in planner_successes
            if row.get("place_error_m") is not None
        ]
        planner_planning_times = [
            float(row["ompl_planning_time_s"])
            for row in planner_successes
            if row.get("ompl_planning_time_s") is not None
        ]
        planner_path_lengths = [
            float(row["ompl_joint_path_length_rad"])
            for row in planner_successes
            if row.get("ompl_joint_path_length_rad") is not None
        ]
        planner_cpp_rows = [
            row
            for row in planner_successes
            if int(row.get("cpp_trajectory_message_count") or 0) > 0
        ]
        planner_cpp_path_lengths = [
            float(row["cpp_joint_path_length_rad"])
            for row in planner_cpp_rows
            if row.get("cpp_joint_path_length_rad") is not None
        ]
        planner_cpp_max_steps = [
            float(row["cpp_max_joint_step_rad"])
            for row in planner_cpp_rows
            if row.get("cpp_max_joint_step_rad") is not None
        ]
        planner_cpp_smoothness_costs = [
            float(row["cpp_integrated_squared_acceleration"])
            for row in planner_cpp_rows
            if row.get("cpp_integrated_squared_acceleration") is not None
        ]
        planner_cpp_joint_limit_margins = [
            float(row["cpp_min_normalized_joint_limit_margin"])
            for row in planner_cpp_rows
            if row.get("cpp_min_normalized_joint_limit_margin") is not None
        ]
        planner_tilts = [
            float(row["final_tilt_deg"])
            for row in planner_successes
            if row.get("final_tilt_deg") is not None
        ]
        planner_results[planner_name] = {
            "total": len(planner_rows),
            "successes": len(planner_successes),
            "success_rate": (
                len(planner_successes) / len(planner_rows) if planner_rows else 0.0
            ),
            "success_rate_ci": wilson_interval(
                len(planner_successes), len(planner_rows)
            ),
            "valid_start_trials": len(planner_valid_start_rows),
            "valid_start_successes": len(planner_valid_start_successes),
            "valid_start_success_rate": (
                len(planner_valid_start_successes) / len(planner_valid_start_rows)
                if planner_valid_start_rows
                else 0.0
            ),
            "valid_start_success_rate_ci": wilson_interval(
                len(planner_valid_start_successes),
                len(planner_valid_start_rows),
            ),
            "infrastructure_failures": len(planner_infrastructure_failures),
            "mean_elapsed_s": mean(planner_elapsed),
            "std_elapsed_s": stddev(planner_elapsed),
            "mean_valid_start_elapsed_s": mean(planner_valid_start_elapsed),
            "std_valid_start_elapsed_s": stddev(planner_valid_start_elapsed),
            "mean_place_error_m": mean(planner_errors),
            "std_place_error_m": stddev(planner_errors),
            "mean_final_tilt_deg": mean(planner_tilts),
            "mean_planning_time_s": mean(planner_planning_times),
            "std_planning_time_s": stddev(planner_planning_times),
            "mean_joint_path_length_rad": mean(planner_path_lengths),
            "std_joint_path_length_rad": stddev(planner_path_lengths),
            "cpp_telemetry_trials": len(planner_cpp_rows),
            "mean_cpp_joint_path_length_rad": mean(
                planner_cpp_path_lengths
            ),
            "mean_cpp_max_joint_step_rad": mean(planner_cpp_max_steps),
            "mean_cpp_integrated_squared_acceleration": mean(
                planner_cpp_smoothness_costs
            ),
            "mean_cpp_min_normalized_joint_limit_margin": mean(
                planner_cpp_joint_limit_margins
            ),
            "worst_cpp_normalized_joint_limit_margin": (
                min(planner_cpp_joint_limit_margins)
                if planner_cpp_joint_limit_margins
                else None
            ),
        }

    scenario_results = []
    scenario_keys = sorted(
        {
            (
                str(row.get("perception_mode") or "synthetic"),
                str(row.get("planner_id") or "unknown"),
                str(row.get("scenario_name") or "unknown"),
            )
            for row in rows
        }
    )
    for perception_mode, planner_name, scenario_name in scenario_keys:
        scenario_rows = [
            row
            for row in rows
            if str(row.get("perception_mode") or "synthetic")
            == perception_mode
            and str(row.get("planner_id") or "unknown") == planner_name
            and str(row.get("scenario_name") or "unknown") == scenario_name
        ]
        scenario_valid_rows = [
            row
            for row in scenario_rows
            if row.get("failure_category") != "infrastructure_startup"
        ]
        scenario_successes = [
            row for row in scenario_rows if row["final_state"] == "SUCCESS"
        ]
        scenario_results.append(
            {
                "perception_mode": perception_mode,
                "planner_id": planner_name,
                "scenario_name": scenario_name,
                "total": len(scenario_rows),
                "successes": len(scenario_successes),
                "raw_success_rate": (
                    len(scenario_successes) / len(scenario_rows)
                    if scenario_rows
                    else 0.0
                ),
                "valid_start_trials": len(scenario_valid_rows),
                "valid_start_success_rate": (
                    len(scenario_successes) / len(scenario_valid_rows)
                    if scenario_valid_rows
                    else 0.0
                ),
            }
        )

    perception_results: Dict[str, Dict[str, Any]] = {}
    perception_modes = sorted(
        {
            str(row.get("perception_mode") or "synthetic")
            for row in rows
        }
    )
    for perception_mode in perception_modes:
        mode_rows = [
            row
            for row in rows
            if str(row.get("perception_mode") or "synthetic")
            == perception_mode
        ]
        mode_successes = [
            row for row in mode_rows if row["final_state"] == "SUCCESS"
        ]
        mode_valid_rows = [
            row
            for row in mode_rows
            if row.get("failure_category") != "infrastructure_startup"
        ]
        perception_errors = [
            float(row["perception_mean_error_m"])
            for row in mode_rows
            if row.get("perception_mean_error_m") is not None
        ]
        perception_validation_rows = [
            row
            for row in mode_rows
            if row.get("perception_validation_state") is not None
        ]
        perception_validation_successes = [
            row
            for row in perception_validation_rows
            if row.get("perception_validation_state") == "SUCCESS"
        ]
        detection_rates = [
            float(row["perception_detection_rate"])
            for row in mode_rows
            if row.get("perception_detection_rate") is not None
        ]
        x_errors = [
            float(row["perception_mean_x_error_m"])
            for row in mode_rows
            if row.get("perception_mean_x_error_m") is not None
        ]
        y_errors = [
            float(row["perception_mean_y_error_m"])
            for row in mode_rows
            if row.get("perception_mean_y_error_m") is not None
        ]
        z_errors = [
            float(row["perception_mean_z_error_m"])
            for row in mode_rows
            if row.get("perception_mean_z_error_m") is not None
        ]
        detection_latencies = [
            float(row["perception_first_detection_latency_s"])
            for row in mode_rows
            if row.get("perception_first_detection_latency_s") is not None
        ]
        placement_errors = [
            float(row["place_error_m"])
            for row in mode_successes
            if row.get("place_error_m") is not None
        ]
        elapsed = [
            float(row["elapsed_s"])
            for row in mode_valid_rows
        ]
        perception_results[perception_mode] = {
            "total": len(mode_rows),
            "successes": len(mode_successes),
            "raw_success_rate": (
                len(mode_successes) / len(mode_rows) if mode_rows else 0.0
            ),
            "valid_start_trials": len(mode_valid_rows),
            "valid_start_success_rate": (
                len(mode_successes) / len(mode_valid_rows)
                if mode_valid_rows
                else 0.0
            ),
            "mean_perception_error_m": mean(perception_errors),
            "std_perception_error_m": stddev(perception_errors),
            "max_perception_error_m": (
                max(perception_errors) if perception_errors else None
            ),
            "perception_validation_trials": len(
                perception_validation_rows
            ),
            "perception_validation_successes": len(
                perception_validation_successes
            ),
            "perception_validation_success_rate": (
                len(perception_validation_successes)
                / len(perception_validation_rows)
                if perception_validation_rows
                else None
            ),
            "mean_detection_rate": mean(detection_rates),
            "mean_first_detection_latency_s": mean(detection_latencies),
            "mean_x_error_m": mean(x_errors),
            "mean_y_error_m": mean(y_errors),
            "mean_z_error_m": mean(z_errors),
            "mean_place_error_m": mean(placement_errors),
            "mean_elapsed_s": mean(elapsed),
        }

    return {
        "total": len(rows),
        "successes": len(successful),
        "failures": len(rows) - len(successful),
        "success_rate": len(successful) / len(rows) if rows else 0.0,
        "success_rate_ci": wilson_interval(len(successful), len(rows)),
        "infrastructure_failures": len(infrastructure_failures),
        "valid_start_trials": len(valid_start_rows),
        "valid_start_successes": len(valid_start_successes),
        "valid_start_success_rate": (
            len(valid_start_successes) / len(valid_start_rows)
            if valid_start_rows
            else 0.0
        ),
        "valid_start_success_rate_ci": wilson_interval(
            len(valid_start_successes), len(valid_start_rows)
        ),
        "mean_lift_delta_m": mean(lift_values),
        "std_lift_delta_m": stddev(lift_values),
        "mean_place_error_m": mean(error_values),
        "std_place_error_m": stddev(error_values),
        "mean_final_tilt_deg": mean(tilt_values),
        "std_final_tilt_deg": stddev(tilt_values),
        "mean_elapsed_s": mean(elapsed_values),
        "mean_valid_start_elapsed_s": mean(valid_start_elapsed_values),
        "direct_path_diagnostics": len(direct_path_rows),
        "blocked_direct_paths": len(blocked_direct_paths),
        "mean_ompl_planning_time_s": mean(planning_time_values),
        "mean_ompl_joint_path_length_rad": mean(path_length_values),
        "cpp_telemetry_trials": len(cpp_telemetry_rows),
        "cpp_telemetry_coverage": (
            len(cpp_telemetry_rows) / len(successful)
            if successful
            else None
        ),
        "mean_cpp_trajectory_segment_count": mean(cpp_segment_counts),
        "mean_cpp_joint_path_length_rad": mean(cpp_path_lengths),
        "mean_cpp_max_joint_step_rad": mean(cpp_max_steps),
        "mean_cpp_integrated_squared_acceleration": mean(
            cpp_smoothness_costs
        ),
        "mean_cpp_min_normalized_joint_limit_margin": mean(
            cpp_joint_limit_margins
        ),
        "worst_cpp_normalized_joint_limit_margin": (
            min(cpp_joint_limit_margins)
            if cpp_joint_limit_margins
            else None
        ),
        "initial_x_range": (
            [min(initial_x_values), max(initial_x_values)] if initial_x_values else None
        ),
        "initial_y_range": (
            [min(initial_y_values), max(initial_y_values)] if initial_y_values else None
        ),
        "failure_reasons": failures,
        "failure_categories": failure_categories,
        "planner_results": planner_results,
        "scenario_results": scenario_results,
        "perception_results": perception_results,
    }


def render_markdown(summary: Dict[str, Any]) -> str:
    def metric(value: float | None, precision: int = 4) -> str:
        return "N/A" if value is None else f"{value:.{precision}f}"

    def interval(value: tuple[float, float] | None) -> str:
        if value is None:
            return "N/A"
        return f"[{value[0]:.1%}, {value[1]:.1%}]"

    lines = [
        "# Gazebo Physical Pick Benchmark",
        "",
        f"- Trials: {summary['total']}",
        f"- Physical successes: {summary['successes']}",
        f"- Physical failures: {summary['failures']}",
        f"- Raw physical success rate: {summary['success_rate']:.1%}",
        f"- Raw success rate 95% CI: {interval(summary['success_rate_ci'])}",
        f"- Infrastructure startup failures: {summary['infrastructure_failures']}",
        f"- Valid-start task attempts: {summary['valid_start_trials']}",
        f"- Valid-start physical successes: {summary['valid_start_successes']}",
        f"- Valid-start physical success rate: "
        f"{summary['valid_start_success_rate']:.1%}",
        "- Valid-start success rate 95% CI: "
        f"{interval(summary['valid_start_success_rate_ci'])}",
        f"- Mean lift delta: {metric(summary['mean_lift_delta_m'])} m",
        f"- Lift delta stddev: {metric(summary['std_lift_delta_m'])} m",
        f"- Mean placement error: {metric(summary['mean_place_error_m'])} m",
        f"- Placement error stddev: {metric(summary['std_place_error_m'])} m",
        f"- Mean final tilt: {metric(summary['mean_final_tilt_deg'], 2)} deg",
        f"- Final tilt stddev: {metric(summary['std_final_tilt_deg'], 2)} deg",
    ]
    if summary["initial_x_range"] is not None and summary["initial_y_range"] is not None:
        lines.extend(
            [
                "- Measured target X range: "
                f"[{summary['initial_x_range'][0]:.3f}, {summary['initial_x_range'][1]:.3f}] m",
                "- Measured target Y range: "
                f"[{summary['initial_y_range'][0]:.3f}, {summary['initial_y_range'][1]:.3f}] m",
            ]
        )
    if summary["direct_path_diagnostics"]:
        lines.extend(
            [
                "- Collision-aware direct paths blocked: "
                f"{summary['blocked_direct_paths']}/"
                f"{summary['direct_path_diagnostics']}",
                "- Mean OMPL planning time: "
                f"{metric(summary['mean_ompl_planning_time_s'], 4)} s",
                "- Mean OMPL joint path length: "
                f"{metric(summary['mean_ompl_joint_path_length_rad'], 4)} rad",
            ]
        )
    if summary["cpp_telemetry_trials"]:
        lines.extend(
            [
                "- C++ trajectory telemetry coverage: "
                f"{summary['cpp_telemetry_trials']}/{summary['successes']} "
                f"successful trials ({summary['cpp_telemetry_coverage']:.1%})",
                "- Mean C++ observed trajectory segments: "
                f"{metric(summary['mean_cpp_trajectory_segment_count'], 2)}",
                "- Mean C++ observed joint path length: "
                f"{metric(summary['mean_cpp_joint_path_length_rad'])} rad",
                "- Mean C++ maximum joint step: "
                f"{metric(summary['mean_cpp_max_joint_step_rad'])} rad",
                "- Mean C++ integrated squared acceleration: "
                f"{metric(summary['mean_cpp_integrated_squared_acceleration'])}",
                "- Mean / worst C++ normalized joint-limit margin: "
                f"{metric(summary['mean_cpp_min_normalized_joint_limit_margin'])} / "
                f"{metric(summary['worst_cpp_normalized_joint_limit_margin'])}",
            ]
        )
    lines.extend(
        [
            f"- Raw mean trial time: {metric(summary['mean_elapsed_s'], 2)} s",
            "- Valid-start mean trial time: "
            f"{metric(summary['mean_valid_start_elapsed_s'], 2)} s",
            "",
            "A trial passes only when the pipeline succeeds and Gazebo measurements verify "
            "lift height, final placement error, and upright orientation.",
            "",
            "Raw success includes infrastructure startup failures. Valid-start success "
            "excludes only trials where controller spawners failed before the task began.",
        ]
    )
    if summary["failure_categories"]:
        lines.extend(["", "## Failure Categories", ""])
        for category, count in sorted(summary["failure_categories"].items()):
            lines.append(f"- {category}: {count}")
    if summary["failure_reasons"]:
        lines.extend(["", "## Failure Reasons", ""])
        for reason, count in sorted(summary["failure_reasons"].items()):
            lines.append(f"- {reason}: {count}")
    if summary["planner_results"]:
        lines.extend(
            [
                "",
                "## Planner Comparison",
                "",
                "| Planner | Raw success | Valid-start success | Infra failures | "
                "OMPL time (mean +/- std) | Joint path (mean +/- std) | "
                "C++ telemetry | C++ smoothness / max step | "
                "C++ mean / worst limit margin | "
                "Valid-start trial time (mean +/- std) | "
                "Place error (mean +/- std) |",
                "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | "
                "---: | ---: | ---: |",
            ]
        )
        for planner, result in summary["planner_results"].items():
            lines.append(
                f"| {planner} | {result['successes']}/{result['total']} "
                f"({result['success_rate']:.1%}) | "
                f"{result['valid_start_successes']}/{result['valid_start_trials']} "
                f"({result['valid_start_success_rate']:.1%}, "
                f"95% CI {interval(result['valid_start_success_rate_ci'])}) | "
                f"{result['infrastructure_failures']} | "
                f"{metric(result['mean_planning_time_s'])} +/- "
                f"{metric(result['std_planning_time_s'])} s | "
                f"{metric(result['mean_joint_path_length_rad'])} +/- "
                f"{metric(result['std_joint_path_length_rad'])} rad | "
                f"{result['cpp_telemetry_trials']}/{result['successes']} | "
                f"{metric(result['mean_cpp_integrated_squared_acceleration'])} / "
                f"{metric(result['mean_cpp_max_joint_step_rad'])} rad | "
                f"{metric(result['mean_cpp_min_normalized_joint_limit_margin'])} / "
                f"{metric(result['worst_cpp_normalized_joint_limit_margin'])} | "
                f"{metric(result['mean_valid_start_elapsed_s'], 2)} +/- "
                f"{metric(result['std_valid_start_elapsed_s'], 2)} s | "
                f"{metric(result['mean_place_error_m'])} +/- "
                f"{metric(result['std_place_error_m'])} m |"
            )
    if len(summary["perception_results"]) > 1:
        lines.extend(
            [
                "",
                "## Perception Input Comparison",
                "",
                "| Input | Physical success | Pose validation | Pose error | XYZ bias | "
                "Detection rate | First detection | Trial time | Place error |",
                "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
            ]
        )
        for mode, result in summary["perception_results"].items():
            validation_rate = result["perception_validation_success_rate"]
            validation_text = (
                "N/A"
                if validation_rate is None
                else (
                    f"{result['perception_validation_successes']}/"
                    f"{result['perception_validation_trials']} "
                    f"({validation_rate:.1%})"
                )
            )
            lines.append(
                f"| {mode} | {result['successes']}/{result['total']} "
                f"({result['raw_success_rate']:.1%}) | "
                f"{validation_text} | "
                f"{metric(result['mean_perception_error_m'])} m | "
                f"({metric(result['mean_x_error_m'])}, "
                f"{metric(result['mean_y_error_m'])}, "
                f"{metric(result['mean_z_error_m'])}) m | "
                f"{metric(result['mean_detection_rate'], 3)} | "
                f"{metric(result['mean_first_detection_latency_s'], 2)} s | "
                f"{metric(result['mean_elapsed_s'], 2)} s | "
                f"{metric(result['mean_place_error_m'])} m |"
            )
    if summary["scenario_results"]:
        lines.extend(
            [
                "",
                "## Scenario Breakdown",
                "",
                "| Input | Planner | Scenario | Raw success | Valid-start success |",
                "| --- | --- | --- | ---: | ---: |",
            ]
        )
        for result in summary["scenario_results"]:
            lines.append(
                f"| {result['perception_mode']} | {result['planner_id']} | "
                f"{result['scenario_name']} | "
                f"{result['successes']}/{result['total']} "
                f"({result['raw_success_rate']:.1%}) | "
                f"{result['successes']}/{result['valid_start_trials']} "
                f"({result['valid_start_success_rate']:.1%}) |"
            )
    return "\n".join(lines) + "\n"


def planner_display_name(planner_id: str) -> str:
    names = {
        "RRTConnectkConfigDefault": "RRTConnect",
        "PRMkConfigDefault": "PRM",
        "RRTstarkConfigDefault": "RRTstar",
    }
    return names.get(planner_id, planner_id)


def render_planner_comparison_svg(summary: Dict[str, Any]) -> str:
    """Render a dependency-free planner comparison chart."""
    planner_results = summary.get("planner_results", {})
    planners = sorted(planner_results)
    if not planners:
        raise ValueError("planner comparison requires at least one planner")

    width = 1200
    colors = {
        "RRTConnectkConfigDefault": "#0f766e",
        "PRMkConfigDefault": "#d97706",
        "RRTstarkConfigDefault": "#be123c",
    }
    panels = [
        (
            "Valid-start physical success rate",
            "Controller startup failures excluded",
            "valid_start_success_rate",
            "percent",
        ),
        (
            "Mean OMPL planning time",
            "Log scale highlights sampling cost",
            "mean_planning_time_s",
            "log_seconds",
        ),
        (
            "Mean joint-space path length",
            "Accumulated OMPL path across the task",
            "mean_joint_path_length_rad",
            "radians",
        ),
        (
            "Mean end-to-end trial time",
            "Valid starts: world, planning, execution and validation",
            "mean_valid_start_elapsed_s",
            "seconds",
        ),
    ]
    has_cpp_telemetry = any(
        int(planner_results[planner].get("cpp_telemetry_trials") or 0) > 0
        for planner in planners
    )
    if has_cpp_telemetry:
        panels.extend(
            [
                (
                    "Mean C++ smoothness cost",
                    "Integrated squared acceleration across observed plans",
                    "mean_cpp_integrated_squared_acceleration",
                    "scalar",
                ),
                (
                    "Worst C++ joint-limit margin",
                    "Minimum normalized margin in successful trials",
                    "worst_cpp_normalized_joint_limit_margin",
                    "margin",
                ),
            ]
        )
    panel_rows = math.ceil(len(panels) / 2)
    height = 760 + max(0, panel_rows - 2) * 316
    values_by_key = {
        key: [
            float(planner_results[planner].get(key) or 0.0)
            for planner in planners
        ]
        for _, _, key, _ in panels
    }

    def bar_ratio(value: float, key: str, scale: str) -> float:
        if scale == "percent":
            return max(0.0, min(1.0, value))
        if scale == "log_seconds":
            lower = -2.0
            upper = max(
                2.0,
                math.ceil(
                    math.log10(
                        max(max(values_by_key[key]), 0.01)
                    )
                ),
            )
            return max(
                0.0,
                min(1.0, (math.log10(max(value, 0.01)) - lower) / (upper - lower)),
            )
        upper = max(max(values_by_key[key]) * 1.15, 1.0)
        return max(0.0, min(1.0, value / upper))

    def value_label(value: float, scale: str) -> str:
        if scale == "percent":
            return f"{value:.0%}"
        if scale == "log_seconds":
            return f"{value:.4f} s"
        if scale == "radians":
            return f"{value:.3f} rad"
        if scale in {"scalar", "margin"}:
            return f"{value:.3f}"
        return f"{value:.1f} s"

    svg = [
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" '
            f'height="{height}" viewBox="0 0 {width} {height}" role="img" '
            'aria-labelledby="title desc">'
        ),
        "<title id=\"title\">Gazebo obstacle benchmark planner comparison</title>",
        (
            "<desc id=\"desc\">Physical success rate, OMPL planning time, "
            "joint path length and end-to-end time by planner.</desc>"
        ),
        f'<rect width="{width}" height="{height}" fill="#f8fafc"/>',
        (
            '<text x="48" y="52" font-family="Arial, sans-serif" '
            'font-size="28" font-weight="700" fill="#111827">'
            "Gazebo Obstacle Benchmark</text>"
        ),
        (
            '<text x="48" y="82" font-family="Arial, sans-serif" '
            'font-size="15" fill="#475569">'
            f"Raw {summary['successes']}/{summary['total']} "
            f"({summary['success_rate']:.1%}); valid-start "
            f"{summary['valid_start_successes']}/{summary['valid_start_trials']} "
            f"({summary['valid_start_success_rate']:.1%}); "
            f"{summary['blocked_direct_paths']}/"
            f"{summary['direct_path_diagnostics']} collision-aware direct paths blocked"
            "</text>"
        ),
    ]

    panel_positions = [
        (40 + (index % 2) * 570, 112 + (index // 2) * 316)
        for index in range(len(panels))
    ]
    for (title, subtitle, key, scale), (panel_x, panel_y) in zip(
        panels,
        panel_positions,
    ):
        panel_width = 550
        panel_height = 284
        bar_x = panel_x + 150
        bar_width = 292
        svg.extend(
            [
                (
                    f'<rect x="{panel_x}" y="{panel_y}" width="{panel_width}" '
                    f'height="{panel_height}" rx="8" fill="#ffffff" '
                    'stroke="#cbd5e1"/>'
                ),
                (
                    f'<text x="{panel_x + 24}" y="{panel_y + 36}" '
                    'font-family="Arial, sans-serif" font-size="18" '
                    f'font-weight="700" fill="#111827">{html.escape(title)}</text>'
                ),
                (
                    f'<text x="{panel_x + 24}" y="{panel_y + 60}" '
                    'font-family="Arial, sans-serif" font-size="12" '
                    f'fill="#64748b">{html.escape(subtitle)}</text>'
                ),
            ]
        )
        for index, planner in enumerate(planners):
            result = planner_results[planner]
            value = float(result.get(key) or 0.0)
            row_y = panel_y + 99 + index * 58
            filled_width = bar_width * bar_ratio(value, key, scale)
            color = colors.get(planner, "#2563eb")
            svg.extend(
                [
                    (
                        f'<text x="{panel_x + 24}" y="{row_y + 17}" '
                        'font-family="Arial, sans-serif" font-size="13" '
                        f'font-weight="600" fill="#334155">'
                        f"{html.escape(planner_display_name(planner))}</text>"
                    ),
                    (
                        f'<rect x="{bar_x}" y="{row_y}" width="{bar_width}" '
                        'height="22" rx="4" fill="#e2e8f0"/>'
                    ),
                    (
                        f'<rect x="{bar_x}" y="{row_y}" '
                        f'width="{filled_width:.2f}" height="22" rx="4" '
                        f'fill="{color}"/>'
                    ),
                    (
                        f'<text x="{panel_x + 532}" y="{row_y + 17}" '
                        'text-anchor="end" font-family="Arial, sans-serif" '
                        f'font-size="13" fill="#0f172a">'
                        f"{html.escape(value_label(value, scale))}</text>"
                    ),
                ]
            )

    svg.extend(
        [
            (
                f'<text x="48" y="{height - 18}" font-family="Arial, sans-serif" '
                'font-size="12" fill="#64748b">'
                "Success requires collision-aware planning, physical lift, "
                "upright placement and return-home.</text>"
            ),
            "</svg>",
        ]
    )
    return "\n".join(svg) + "\n"


def render_perception_comparison_svg(summary: Dict[str, Any]) -> str:
    """Render a paired synthetic-pose versus ArUco-camera comparison."""
    results = summary.get("perception_results", {})
    modes = [mode for mode in ("synthetic", "aruco") if mode in results]
    modes.extend(sorted(set(results) - set(modes)))
    if len(modes) < 2:
        raise ValueError("perception comparison requires at least two modes")

    width = 1200
    height = 650
    colors = {"synthetic": "#0f766e", "aruco": "#d97706"}
    names = {"synthetic": "Synthetic pose", "aruco": "ArUco camera"}
    panels = [
        (
            "Valid-start physical success rate",
            "End-to-end planning, execution and physical validation",
            "valid_start_success_rate",
            "percent",
        ),
        (
            "Mean end-to-end trial time",
            "Fresh Gazebo world through final validation",
            "mean_elapsed_s",
            "seconds",
        ),
        (
            "Mean placement error",
            "Successful physical placements only",
            "mean_place_error_m",
            "meters",
        ),
        (
            "Mean perception pose error",
            "ArUco estimate against Gazebo ground truth",
            "mean_perception_error_m",
            "meters",
        ),
    ]

    def label(value: float | None, scale: str) -> str:
        if value is None:
            return "N/A"
        if scale == "percent":
            return f"{value:.0%}"
        if scale == "seconds":
            return f"{value:.1f} s"
        return f"{value * 1000.0:.1f} mm"

    svg = [
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" '
            f'height="{height}" viewBox="0 0 {width} {height}" role="img" '
            'aria-labelledby="title desc">'
        ),
        "<title id=\"title\">Synthetic pose versus ArUco camera benchmark</title>",
        (
            "<desc id=\"desc\">Physical success, task time, placement error and "
            "perception error for paired input modes.</desc>"
        ),
        '<rect width="1200" height="650" fill="#f8fafc"/>',
        (
            '<text x="48" y="52" font-family="Arial, sans-serif" '
            'font-size="28" font-weight="700" fill="#111827">'
            "Perception-Driven Pick Benchmark</text>"
        ),
        (
            '<text x="48" y="82" font-family="Arial, sans-serif" '
            'font-size="15" fill="#475569">'
            f"Paired modes across {summary['total']} fresh-world physical trials"
            "</text>"
        ),
    ]
    panel_positions = [(40, 112), (610, 112), (40, 382), (610, 382)]
    for (title, subtitle, key, scale), (panel_x, panel_y) in zip(
        panels, panel_positions
    ):
        values = [
            float(results[mode][key])
            for mode in modes
            if results[mode].get(key) is not None
        ]
        upper = 1.0 if scale == "percent" else max(values, default=1.0) * 1.15
        panel_width = 550
        panel_height = 238
        bar_x = panel_x + 170
        bar_width = 275
        svg.extend(
            [
                (
                    f'<rect x="{panel_x}" y="{panel_y}" width="{panel_width}" '
                    f'height="{panel_height}" rx="8" fill="#ffffff" '
                    'stroke="#cbd5e1"/>'
                ),
                (
                    f'<text x="{panel_x + 24}" y="{panel_y + 36}" '
                    'font-family="Arial, sans-serif" font-size="18" '
                    f'font-weight="700" fill="#111827">{html.escape(title)}</text>'
                ),
                (
                    f'<text x="{panel_x + 24}" y="{panel_y + 60}" '
                    'font-family="Arial, sans-serif" font-size="12" '
                    f'fill="#64748b">{html.escape(subtitle)}</text>'
                ),
            ]
        )
        for index, mode in enumerate(modes):
            raw_value = results[mode].get(key)
            value = float(raw_value) if raw_value is not None else None
            row_y = panel_y + 92 + index * 62
            filled_width = (
                0.0
                if value is None or upper <= 0.0
                else bar_width * max(0.0, min(1.0, value / upper))
            )
            svg.extend(
                [
                    (
                        f'<text x="{panel_x + 24}" y="{row_y + 17}" '
                        'font-family="Arial, sans-serif" font-size="13" '
                        f'font-weight="600" fill="#334155">'
                        f"{html.escape(names.get(mode, mode))}</text>"
                    ),
                    (
                        f'<rect x="{bar_x}" y="{row_y}" width="{bar_width}" '
                        'height="22" rx="4" fill="#e2e8f0"/>'
                    ),
                    (
                        f'<rect x="{bar_x}" y="{row_y}" '
                        f'width="{filled_width:.2f}" height="22" rx="4" '
                        f'fill="{colors.get(mode, "#2563eb")}"/>'
                    ),
                    (
                        f'<text x="{panel_x + 532}" y="{row_y + 17}" '
                        'text-anchor="end" font-family="Arial, sans-serif" '
                        f'font-size="13" fill="#0f172a">'
                        f"{html.escape(label(value, scale))}</text>"
                    ),
                ]
            )
    svg.extend(
        [
            (
                '<text x="48" y="638" font-family="Arial, sans-serif" '
                'font-size="12" fill="#64748b">'
                "ArUco trials use RGB image detection and tf2 conversion; "
                "synthetic trials inject the target pose directly.</text>"
            ),
            "</svg>",
        ]
    )
    return "\n".join(svg) + "\n"


def write_svg_report(summary: Dict[str, Any], svg_path: str | Path) -> None:
    output_path = Path(svg_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    perception_results = summary.get("perception_results", {})
    renderer = (
        render_perception_comparison_svg
        if len(perception_results) > 1
        else render_planner_comparison_svg
    )
    output_path.write_text(
        renderer(summary),
        encoding="utf-8",
    )


def read_report_rows(csv_path: str | Path) -> List[Dict[str, Any]]:
    """Load checkpoint rows while restoring booleans and empty values."""
    with Path(csv_path).open(newline="", encoding="utf-8") as csv_file:
        rows = list(csv.DictReader(csv_file))
    for row in rows:
        for key, value in list(row.items()):
            if value == "":
                row[key] = None
        blocked = row.get("direct_path_blocked")
        if isinstance(blocked, str):
            normalized = blocked.lower()
            row["direct_path_blocked"] = (
                True
                if normalized == "true"
                else False
                if normalized == "false"
                else None
            )
        for count_key in (
            "cpp_trajectory_message_count",
            "cpp_trajectory_segment_count",
        ):
            count = row.get(count_key)
            if isinstance(count, str):
                try:
                    row[count_key] = int(count)
                except ValueError:
                    row[count_key] = None
        for metric_key in (
            "cpp_joint_path_length_rad",
            "cpp_max_joint_step_rad",
            "cpp_integrated_squared_acceleration",
            "cpp_min_normalized_joint_limit_margin",
        ):
            metric_value = row.get(metric_key)
            if isinstance(metric_value, str):
                try:
                    row[metric_key] = float(metric_value)
                except ValueError:
                    row[metric_key] = None
    return rows


def write_reports(
    rows: Iterable[Dict[str, Any]],
    csv_path: str | Path,
    markdown_path: str | Path,
) -> Dict[str, Any]:
    rows = list(rows)
    with Path(csv_path).open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    summary = summarize(rows)
    Path(markdown_path).write_text(render_markdown(summary), encoding="utf-8")
    return summary
