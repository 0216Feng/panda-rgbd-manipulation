#!/usr/bin/env python3
"""Run paired unloaded/loaded transfer trials with synchronized telemetry."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import re
import selectors
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, Optional


REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPO_ROOT / "src" / "panda_manipulation"
CPP_PACKAGE_ROOT = REPO_ROOT / "src" / "panda_manipulation_cpp"
PACKAGE_ROOTS = (PACKAGE_ROOT, CPP_PACKAGE_ROOT)
DEFAULT_WORLD = PACKAGE_ROOT / "worlds" / "panda_table.sdf"
sys.path.insert(0, str(PACKAGE_ROOT))

from panda_manipulation.physics_benchmark import write_trial_world  # noqa: E402
from panda_manipulation.transfer_diagnostics import (  # noqa: E402
    render_backend_comparison_markdown,
    render_comparison_markdown,
    render_physical_disturbance_safety_markdown,
    render_servo_watchdog_safety_markdown,
    selected_grasp_yaw_offset,
    summarize_backend_comparison,
    summarize_comparison,
    summarize_physical_disturbance_safety,
    summarize_servo_watchdog_safety,
)


SOURCE_SUFFIXES = {
    ".cfg", ".cmake", ".cpp", ".h", ".hpp", ".json", ".py", ".rviz", ".sdf",
    ".urdf", ".xacro", ".xml", ".yaml", ".yml",
}


def paired_source_manifest() -> Dict[str, object]:
    candidates = {Path(__file__).resolve()}
    for package_root in PACKAGE_ROOTS:
        if not package_root.is_dir():
            continue
        candidates.update(
            path.resolve() for path in package_root.rglob("*")
            if path.is_file()
            and (path.suffix.lower() in SOURCE_SUFFIXES
                 or path.name == "CMakeLists.txt")
            and "__pycache__" not in path.parts)
    files: Dict[str, str] = {}
    for path in candidates:
        try:
            relative = path.relative_to(REPO_ROOT).as_posix()
        except ValueError:
            relative = path.name
        files[relative] = hashlib.sha256(path.read_bytes()).hexdigest()
    combined = hashlib.sha256()
    for relative, digest in sorted(files.items()):
        combined.update(relative.encode("utf-8") + b"\0" + digest.encode("ascii") + b"\n")
    return {"schema_version": 1, "sha256": combined.hexdigest(), "files": files}


def guard_paired_checkpoint(args: argparse.Namespace, checkpoint: Path) -> None:
    excluded = {"output_dir", "resume_existing", "report_only_csv"}
    source = paired_source_manifest()
    config = {"schema_version": 1, "source_sha256": source["sha256"],
              "arguments": {key:value for key,value in vars(args).items()
                            if key not in excluded}}
    config = json.loads(json.dumps(config, allow_nan=False))
    config_path = checkpoint.parent / "experiment_config.json"
    source_path = checkpoint.parent / "source_manifest.json"
    if checkpoint.exists():
        if not args.resume_existing:
            raise ValueError("comparison CSV already exists; use a new --output-dir or --resume-existing")
        if not config_path.exists() or not source_path.exists():
            raise ValueError("resume metadata missing; preserve legacy results and use a new --output-dir")
        previous = json.loads(config_path.read_text(encoding="utf-8"))
        saved_source = json.loads(source_path.read_text(encoding="utf-8"))
        if saved_source.get("sha256") != previous.get("source_sha256"):
            raise ValueError("resume source manifest does not match its configuration")
        if previous != config:
            raise ValueError("resume configuration or source differs; use a new --output-dir")
        return
    checkpoint.parent.mkdir(parents=True, exist_ok=True)
    source_path.write_text(json.dumps(source, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    config_path.write_text(json.dumps(config, indent=2, sort_keys=True) + "\n", encoding="utf-8")


BAG_TOPICS = [
    "/clock",
    "/joint_states",
    "/panda_arm_controller/controller_state",
    "/panda_hand_controller/controller_state",
    "/gazebo/target_pose",
    "/gazebo/target_cube/contacts",
    "/pick_demo_state",
    "/pick_plan_result",
    "/physical_disturbance_event",
    "/trajectory_execution_event",
    "/transfer_diagnostics_summary",
    "/tf",
    "/tf_static",
]

DIAGNOSTIC_TARGET_X = 0.52
DIAGNOSTIC_TARGET_Y = 0.0
DIAGNOSTIC_OBSTACLE_POSE = (0.42, 0.30, 0.09)
DIAGNOSTIC_OBSTACLE_SIZE = (0.12, 0.12, 0.18)

RESULT_FIELDS = [
    "trial",
    "pair",
    "mode",
    "planner_id",
    "grasp_yaw_offset_deg",
    "grasp_yaw_was_forced",
    "grasp_prevalidation_max_rounds",
    "pre_place_joint_replay_requested",
    "pre_place_joint_reference_positions_rad",
    "pre_place_joint_planned_positions_rad",
    "pre_place_joint_replay_max_plan_delta_rad",
    "grasp_height_offset_m",
    "gripper_closed_position_m",
    "payload_transfer_velocity_scaling_factor",
    "place_descent_velocity_scaling_factor",
    "place_descent_recovery_velocity_scaling_factor",
    "place_descent_stage_count",
    "place_descent_backend",
    "place_descent_execution_time_s",
    "servo_cartesian_fallback_used",
    "servo_cartesian_fallback_count",
    "servo_watchdog_check_count",
    "servo_fault_injection_type",
    "servo_fault_injection_stage",
    "servo_fault_injection_delay_s",
    "servo_fault_injection_triggered",
    "servo_fault_injection_observed_stage",
    "servo_watchdog_stop_latency_s",
    "servo_watchdog_stop_latency_limit_s",
    "servo_watchdog_safe_stop_passed",
    "physical_disturbance_enabled",
    "physical_disturbance_stage",
    "physical_disturbance_delay_s",
    "physical_disturbance_duration_s",
    "physical_disturbance_force_x_n",
    "physical_disturbance_force_y_n",
    "physical_disturbance_force_z_n",
    "physical_disturbance_applied",
    "physical_disturbance_cleared",
    "physical_disturbance_observed_stage",
    "physical_disturbance_stop_latency_s",
    "physical_disturbance_stop_latency_limit_s",
    "physical_disturbance_safe_stop_passed",
    "final_state",
    "failure_reason",
    "elapsed_s",
    "samples",
    "max_arm_position_error_rad",
    "rms_arm_position_error_rad",
    "max_transfer_arm_position_error_rad",
    "rms_transfer_arm_position_error_rad",
    "max_gripper_position_error_m",
    "max_payload_relative_drift_m",
    "max_grasp_probe_relative_drift_m",
    "payload_bilateral_contact_ratio",
    "physical_lift_delta_m",
    "physical_place_error_m",
    "physical_final_tilt_deg",
    "first_payload_drift_stage",
    "first_payload_drift_elapsed_s",
    "pre_descent_arm_positions_rad",
    "telemetry_csv",
    "telemetry_summary",
    "rosbag_path",
    "log_path",
]


def isolated_environment(trial: int) -> Dict[str, str]:
    """Create per-trial ROS and Gazebo discovery namespaces."""
    environment = os.environ.copy()
    try:
        base_domain = int(environment.get("ROS_DOMAIN_ID", "0") or 0)
    except ValueError:
        base_domain = 0
    environment["ROS_DOMAIN_ID"] = str((base_domain + os.getpid() + trial) % 101)
    environment["GZ_PARTITION"] = f"payload_diagnostics_{os.getpid()}_{trial}"
    return environment


def stop_process_group(process: Optional[subprocess.Popen[Any]], wait_s: float = 15.0) -> None:
    """Stop a process group and escalate only when graceful shutdown stalls."""
    if process is None or process.poll() is not None:
        return
    for sig, timeout in ((signal.SIGINT, wait_s), (signal.SIGTERM, 5.0)):
        try:
            os.killpg(process.pid, sig)
            process.wait(timeout=timeout)
            return
        except (ProcessLookupError, subprocess.TimeoutExpired):
            continue
    os.killpg(process.pid, signal.SIGKILL)
    process.wait(timeout=5.0)


def prepare_world(output_path: Path, mode: str) -> None:
    """Create a transfer-isolation world without the unmodeled RGB-D obstacle."""
    write_trial_world(
        DEFAULT_WORLD,
        output_path,
        DIAGNOSTIC_TARGET_X,
        DIAGNOSTIC_TARGET_Y,
        DIAGNOSTIC_OBSTACLE_POSE,
        DIAGNOSTIC_OBSTACLE_SIZE,
    )
    tree = ET.parse(output_path)
    dynamic_obstacle = tree.getroot().find(
        ".//model[@name='dynamic_obstacle']"
    )
    if dynamic_obstacle is None:
        raise RuntimeError("dynamic_obstacle is missing from the diagnostic world")
    dynamic_pose = dynamic_obstacle.find("pose")
    if dynamic_pose is None:
        raise RuntimeError("dynamic_obstacle pose is missing from the diagnostic world")
    dynamic_pose.text = "2.500000 2.500000 0.870000 0 0 0"
    if mode == "loaded":
        tree.write(output_path, encoding="utf-8", xml_declaration=True)
        return
    target = tree.getroot().find(".//model[@name='target_cube']")
    if target is None:
        raise RuntimeError("target_cube is missing from the diagnostic world")
    pose = target.find("pose")
    if pose is None:
        raise RuntimeError("target_cube pose is missing from the diagnostic world")
    values = (pose.text or "").split()
    if len(values) != 6:
        raise RuntimeError("target_cube pose must contain six values")
    values[2] = "-1.000000"
    pose.text = " ".join(values)
    static = target.find("static")
    if static is None:
        static = ET.SubElement(target, "static")
    static.text = "true"
    tree.write(output_path, encoding="utf-8", xml_declaration=True)


def parse_pipeline_result(line: str) -> Optional[Dict[str, Any]]:
    """Extract the terminal pick payload from launch output."""
    if "gazebo_pick_pipeline" not in line or "final_state" not in line:
        return None
    start = line.find("{")
    if start < 0:
        return None
    try:
        payload = json.loads(line[start:])
    except json.JSONDecodeError:
        return None
    return payload if payload.get("final_state") in {"SUCCESS", "FAILED"} else None


def parse_physical_result(line: str) -> Optional[Dict[str, Any]]:
    """Extract Gazebo physical validation when a real payload is present."""
    if "gazebo_pick_validator" not in line or "pipeline_state" not in line:
        return None
    start = line.find("{")
    if start < 0:
        return None
    try:
        payload = json.loads(line[start:])
    except json.JSONDecodeError:
        return None
    return payload if "lift_delta_m" in payload else None


def parse_disturbance_event(line: str) -> Optional[Dict[str, Any]]:
    """Extract one physical-disturbance lifecycle event from launch output."""
    if "physical_disturbance_injector" not in line or '"state"' not in line:
        return None
    start = line.find("{")
    if start < 0:
        return None
    try:
        payload = json.loads(line[start:])
    except json.JSONDecodeError:
        return None
    return payload if payload.get("state") in {"ARMED", "APPLIED", "CLEARED"} else None


def physical_disturbance_safe_stop_verdict(
    events: list[Dict[str, Any]],
    pipeline_result: Optional[Dict[str, Any]],
    expected_stage: int,
    stop_latency_limit_s: float,
) -> Dict[str, object]:
    """Evaluate a real Gazebo force disturbance without accepting task failure alone."""
    applied = next(
        (event for event in events if event.get("state") == "APPLIED"),
        None,
    )
    cleared = next(
        (event for event in events if event.get("state") == "CLEARED"),
        None,
    )
    applied_ns = applied.get("monotonic_ns") if applied is not None else None
    stop_ns = (
        pipeline_result.get("servo_watchdog_abort_stop_monotonic_ns")
        if pipeline_result is not None
        else None
    )
    latency_s = None
    if isinstance(applied_ns, (int, float)) and isinstance(stop_ns, (int, float)):
        latency_s = (float(stop_ns) - float(applied_ns)) * 1e-9
    reason = str(
        pipeline_result.get("failure_reason") or ""
        if pipeline_result is not None
        else ""
    ).lower()
    watchdog_reason = (
        "servo_place_descent watchdog" in reason
        and ("payload drift=" in reason or "bilateral contact lost" in reason)
    )
    observed_stage = (
        pipeline_result.get("servo_watchdog_abort_stage")
        if pipeline_result is not None
        else None
    )
    passed = (
        applied is not None
        and cleared is not None
        and applied.get("stage") == expected_stage
        and cleared.get("stage") == expected_stage
        and pipeline_result is not None
        and pipeline_result.get("final_state") == "FAILED"
        and observed_stage == expected_stage
        and watchdog_reason
        and isinstance(latency_s, (int, float))
        and 0.0 <= float(latency_s) <= stop_latency_limit_s
    )
    return {
        "applied": applied is not None,
        "cleared": cleared is not None,
        "observed_stage": observed_stage,
        "stop_latency_s": latency_s,
        "passed": passed,
    }


def place_descent_execution_time_s(
    pipeline_result: Optional[Dict[str, Any]],
) -> Optional[float]:
    """Return executed contact-zone descent time across Servo and Cartesian stages."""
    if pipeline_result is None:
        return None
    duration_s = 0.0
    found = False
    for step in pipeline_result.get("steps", []):
        if not str(step.get("step") or "").startswith(
            "servo_place_descent_stage_"
        ):
            continue
        match = re.search(r"elapsed=([0-9]+(?:\.[0-9]+)?)s", str(step.get("reason")))
        if match is not None:
            duration_s += float(match.group(1))
            found = True
    for metrics in pipeline_result.get("cartesian_trajectory_metrics", []):
        if not str(metrics.get("step") or "").startswith(
            "cartesian_place_descent_stage_"
        ):
            continue
        values = [metrics.get("duration_s")]
        try:
            duration_s += float(values[0])
        except (TypeError, ValueError):
            continue
        found = True
    return duration_s if found else None


def run_trial(
    trial: int,
    pair: int,
    mode: str,
    planner_id: str,
    timeout_s: float,
    output_dir: Path,
    record_bag: bool,
    grasp_height_offset_m: float,
    gripper_closed_position_m: float,
    payload_transfer_velocity_scaling_factor: float,
    place_descent_velocity_scaling_factor: float,
    place_descent_recovery_velocity_scaling_factor: float,
    place_descent_stage_count: int,
    place_descent_backend: str,
    forced_grasp_yaw_offset_deg: Optional[float],
    grasp_prevalidation_max_rounds: int,
    pre_place_joint_replay_positions: Optional[list[float]],
    servo_fault_injection_type: str = "none",
    servo_fault_injection_stage: int = 2,
    servo_fault_injection_delay_s: float = 0.5,
    servo_watchdog_stop_latency_limit_s: float = 0.15,
    enable_physical_disturbance: bool = False,
    physical_disturbance_stage: int = 3,
    physical_disturbance_delay_s: float = 0.5,
    physical_disturbance_duration_s: float = 0.10,
    physical_disturbance_force_x_n: float = 0.0,
    physical_disturbance_force_y_n: float = 2.0,
    physical_disturbance_force_z_n: float = 0.0,
    physical_disturbance_stop_latency_limit_s: float = 0.2,
) -> Dict[str, object]:
    """Run one isolated fixed-scenario transfer and return flattened diagnostics."""
    fault_suffix = (
        ""
        if servo_fault_injection_type == "none"
        else (
            f"_{servo_fault_injection_type}"
            f"_stage_{servo_fault_injection_stage}"
        )
    )
    disturbance_suffix = (
        f"_physical_disturbance_stage_{physical_disturbance_stage}"
        if enable_physical_disturbance
        else ""
    )
    trial_name = (
        f"pair_{pair:02d}_{mode}_{place_descent_backend}"
        f"{fault_suffix}{disturbance_suffix}"
    )
    # Reserve a unique directory even when a case is retried after interruption.
    attempts_dir = output_dir / "attempts"
    attempts_dir.mkdir(parents=True, exist_ok=True)
    artifact_dir = Path(
        tempfile.mkdtemp(prefix=f"trial_{trial:04d}_", dir=attempts_dir)
    )
    telemetry_dir = artifact_dir / "telemetry"
    world_dir = artifact_dir / "worlds"
    bag_dir = artifact_dir / "bags" / trial_name
    log_dir = artifact_dir / "logs"
    for directory in (telemetry_dir, world_dir, log_dir):
        directory.mkdir(parents=True, exist_ok=True)
    telemetry_csv = telemetry_dir / f"{trial_name}.csv"
    telemetry_summary = telemetry_dir / f"{trial_name}.json"
    log_path = log_dir / f"{trial_name}.log"
    world_path = world_dir / f"{trial_name}.sdf"
    prepare_world(world_path, mode)
    scratch_dir = Path(tempfile.mkdtemp(prefix=f"panda_payload_{trial_name}_"))
    runtime_telemetry_csv = scratch_dir / telemetry_csv.name
    runtime_telemetry_summary = scratch_dir / telemetry_summary.name
    runtime_log_path = scratch_dir / log_path.name

    obstacle = DIAGNOSTIC_OBSTACLE_POSE
    size = DIAGNOSTIC_OBSTACLE_SIZE
    physical = mode == "loaded"
    command = [
        "ros2",
        "launch",
        "panda_manipulation",
        "gazebo_pick.launch.py",
        "gui:=false",
        "rviz:=false",
        f"world:={world_path.resolve()}",
        f"target_x:={DIAGNOSTIC_TARGET_X:.6f}",
        f"target_y:={DIAGNOSTIC_TARGET_Y:.6f}",
        f"planner_ids_csv:={planner_id}",
        f"obstacle_x:={obstacle[0]:.6f}",
        f"obstacle_y:={obstacle[1]:.6f}",
        f"obstacle_z:={obstacle[2]:.6f}",
        f"obstacle_size_x:={size[0]:.6f}",
        f"obstacle_size_y:={size[1]:.6f}",
        f"obstacle_size_z:={size[2]:.6f}",
        "diagnose_direct_path:=false",
        "require_direct_path_blocked:=false",
        "enable_grasp_candidate_prevalidation:=true",
        (
            "grasp_candidate_prevalidation_max_rounds:="
            f"{grasp_prevalidation_max_rounds}"
        ),
        "ompl_velocity_scaling_factor:=0.05",
        (
            "payload_transfer_velocity_scaling_factor:="
            f"{payload_transfer_velocity_scaling_factor:.6f}"
        ),
        (
            "place_descent_velocity_scaling_factor:="
            f"{place_descent_velocity_scaling_factor:.6f}"
        ),
        (
            "place_descent_recovery_velocity_scaling_factor:="
            f"{place_descent_recovery_velocity_scaling_factor:.6f}"
        ),
        f"place_descent_stage_count:={place_descent_stage_count}",
        f"place_descent_backend:={place_descent_backend}",
        f"servo_fault_injection_type:={servo_fault_injection_type}",
        f"servo_fault_injection_stage:={servo_fault_injection_stage}",
        f"servo_fault_injection_delay_s:={servo_fault_injection_delay_s:.6f}",
        (
            "enable_physical_disturbance:="
            f"{'true' if enable_physical_disturbance else 'false'}"
        ),
        f"physical_disturbance_stage:={physical_disturbance_stage}",
        f"physical_disturbance_delay_s:={physical_disturbance_delay_s:.6f}",
        (
            "physical_disturbance_duration_s:="
            f"{physical_disturbance_duration_s:.6f}"
        ),
        (
            "physical_disturbance_force_x_n:="
            f"{physical_disturbance_force_x_n:.6f}"
        ),
        (
            "physical_disturbance_force_y_n:="
            f"{physical_disturbance_force_y_n:.6f}"
        ),
        (
            "physical_disturbance_force_z_n:="
            f"{physical_disturbance_force_z_n:.6f}"
        ),
        "grasp_preclose_max_target_drift_m:=0.005",
        "use_ompl_for_place:=true",
        "use_cartesian_pre_place_transfer:=false",
        "pre_grasp_orientation_tolerance:=0.10",
        f"grasp_height_offset:={grasp_height_offset_m:.6f}",
        f"gripper_closed_position:={gripper_closed_position_m:.6f}",
        # Keep controller/load isolation inside the previously validated
        # central workspace. Obstacle and boundary-pose robustness have their
        # own benchmark suites and would confound this paired experiment.
        "place_x:=0.45",
        "place_y:=-0.16",
        f"enable_physical_grasp_verification:={'true' if physical else 'false'}",
        f"enable_tactile_grasp_supervision:={'true' if physical else 'false'}",
        "enable_transfer_diagnostics:=true",
        f"transfer_diagnostics_csv_path:={runtime_telemetry_csv}",
        f"transfer_diagnostics_summary_path:={runtime_telemetry_summary}",
        f"transfer_diagnostics_label:={mode}",
    ]
    if forced_grasp_yaw_offset_deg is not None:
        command.extend(
            [
                "force_grasp_yaw_offset:=true",
                f"forced_grasp_yaw_offset_deg:={forced_grasp_yaw_offset_deg:.6f}",
            ]
        )
    if pre_place_joint_replay_positions is not None:
        command.append(
            "pre_place_joint_replay_positions_csv:="
            + ",".join(
                f"{position:.12f}"
                for position in pre_place_joint_replay_positions
            )
        )
    environment = isolated_environment(trial)
    started = time.monotonic()
    launch_process: Optional[subprocess.Popen[str]] = None
    bag_process: Optional[subprocess.Popen[Any]] = None
    pipeline_result = None
    physical_result = None
    disturbance_events: list[Dict[str, Any]] = []
    pipeline_finished_at = None

    try:
        launch_process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
            env=environment,
        )
        if record_bag:
            bag_dir.parent.mkdir(parents=True, exist_ok=True)
            bag_process = subprocess.Popen(
                [
                    "ros2",
                    "bag",
                    "record",
                    "--storage",
                    "mcap",
                    "--storage-preset-profile",
                    "zstd_fast",
                    "-o",
                    str(bag_dir.resolve()),
                    *BAG_TOPICS,
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
                env=environment,
            )
        selector = selectors.DefaultSelector()
        assert launch_process.stdout is not None
        selector.register(launch_process.stdout, selectors.EVENT_READ)
        with runtime_log_path.open("w", encoding="utf-8") as log_file:
            while time.monotonic() - started < timeout_s:
                for key, _ in selector.select(timeout=0.5):
                    line = key.fileobj.readline()
                    if not line:
                        continue
                    log_file.write(line)
                    log_file.flush()
                    parsed_pipeline = parse_pipeline_result(line)
                    if parsed_pipeline is not None:
                        pipeline_result = parsed_pipeline
                        pipeline_finished_at = time.monotonic()
                    parsed_physical = parse_physical_result(line)
                    if parsed_physical is not None:
                        physical_result = parsed_physical
                    parsed_disturbance = parse_disturbance_event(line)
                    if parsed_disturbance is not None:
                        disturbance_events.append(parsed_disturbance)
                if pipeline_result is not None and runtime_telemetry_summary.exists():
                    disturbance_cleared = (
                        not enable_physical_disturbance
                        or any(
                            event.get("state") == "CLEARED"
                            for event in disturbance_events
                        )
                    )
                    if not disturbance_cleared:
                        if (
                            pipeline_finished_at is None
                            or time.monotonic() - pipeline_finished_at <= 2.0
                        ):
                            continue
                    if not physical or physical_result is not None:
                        break
                    if (
                        pipeline_finished_at is not None
                        and time.monotonic() - pipeline_finished_at > 5.0
                    ):
                        break
                if launch_process.poll() is not None and pipeline_result is None:
                    break
    finally:
        stop_process_group(launch_process)
        stop_process_group(bag_process, wait_s=20.0)

    elapsed_s = time.monotonic() - started
    diagnostics: Dict[str, Any] = {}
    if runtime_telemetry_summary.exists():
        diagnostics = json.loads(
            runtime_telemetry_summary.read_text(encoding="utf-8")
        )
    for source, destination in (
        (runtime_telemetry_csv, telemetry_csv),
        (runtime_telemetry_summary, telemetry_summary),
        (runtime_log_path, log_path),
    ):
        if source.exists():
            shutil.copy2(source, destination)
    shutil.rmtree(scratch_dir, ignore_errors=True)
    if pipeline_result is None:
        final_state = "FAILED"
        failure_reason = (
            f"trial timed out after {timeout_s:.0f}s"
            if elapsed_s >= timeout_s
            else "pick pipeline exited without a terminal result"
        )
    else:
        final_state = str(pipeline_result.get("final_state"))
        failure_reason = pipeline_result.get("failure_reason")
    if physical and physical_result is not None:
        final_state = str(physical_result.get("final_state"))
        if final_state != "SUCCESS":
            failure_reason = physical_result.get("reason")
    crossing = diagnostics.get("first_transfer_drift_threshold_crossing") or {}
    configured_fault = str(servo_fault_injection_type).strip().lower()
    fault_triggered = bool(
        pipeline_result.get("servo_fault_injection_triggered", False)
        if pipeline_result is not None
        else False
    )
    stop_latency = (
        pipeline_result.get("servo_watchdog_stop_latency_s")
        if pipeline_result is not None
        else None
    )
    pipeline_failure_reason = str(
        pipeline_result.get("failure_reason") or ""
        if pipeline_result is not None
        else ""
    ).lower()
    expected_reason = {
        "contact_loss": "bilateral contact lost",
        "payload_drift": "payload drift=",
    }.get(configured_fault)
    safety_stop_passed = None
    if expected_reason is not None:
        safety_stop_passed = (
            fault_triggered
            and pipeline_result is not None
            and pipeline_result.get("final_state") == "FAILED"
            and expected_reason in pipeline_failure_reason
            and isinstance(stop_latency, (int, float))
            and float(stop_latency) <= servo_watchdog_stop_latency_limit_s
        )
    disturbance_verdict = physical_disturbance_safe_stop_verdict(
        disturbance_events,
        pipeline_result,
        physical_disturbance_stage,
        physical_disturbance_stop_latency_limit_s,
    )
    return {
        "trial": trial,
        "pair": pair,
        "mode": mode,
        "planner_id": planner_id,
        "grasp_yaw_offset_deg": selected_grasp_yaw_offset(pipeline_result),
        "grasp_yaw_was_forced": forced_grasp_yaw_offset_deg is not None,
        "grasp_prevalidation_max_rounds": grasp_prevalidation_max_rounds,
        "pre_place_joint_replay_requested": (
            pre_place_joint_replay_positions is not None
        ),
        "pre_place_joint_reference_positions_rad": json.dumps(
            pre_place_joint_replay_positions
        ),
        "pre_place_joint_planned_positions_rad": json.dumps(
            pipeline_result.get("pre_place_joint_target_positions")
            if pipeline_result is not None
            else None
        ),
        "pre_place_joint_replay_max_plan_delta_rad": (
            pipeline_result.get("pre_place_joint_replay_max_plan_delta_rad")
            if pipeline_result is not None
            else None
        ),
        "grasp_height_offset_m": grasp_height_offset_m,
        "gripper_closed_position_m": gripper_closed_position_m,
        "payload_transfer_velocity_scaling_factor": (
            payload_transfer_velocity_scaling_factor
        ),
        "place_descent_velocity_scaling_factor": (
            place_descent_velocity_scaling_factor
        ),
        "place_descent_recovery_velocity_scaling_factor": (
            place_descent_recovery_velocity_scaling_factor
        ),
        "place_descent_stage_count": place_descent_stage_count,
        "place_descent_backend": place_descent_backend,
        "place_descent_execution_time_s": place_descent_execution_time_s(
            pipeline_result
        ),
        "servo_cartesian_fallback_used": bool(
            pipeline_result.get("servo_cartesian_fallback_used", False)
            if pipeline_result is not None
            else False
        ),
        "servo_cartesian_fallback_count": (
            pipeline_result.get("servo_cartesian_fallback_count", 0)
            if pipeline_result is not None
            else 0
        ),
        "servo_watchdog_check_count": (
            pipeline_result.get("servo_watchdog_check_count", 0)
            if pipeline_result is not None
            else 0
        ),
        "servo_fault_injection_type": configured_fault,
        "servo_fault_injection_stage": servo_fault_injection_stage,
        "servo_fault_injection_delay_s": servo_fault_injection_delay_s,
        "servo_fault_injection_triggered": fault_triggered,
        "servo_fault_injection_observed_stage": (
            pipeline_result.get("servo_fault_injection_observed_stage")
            if pipeline_result is not None
            else None
        ),
        "servo_watchdog_stop_latency_s": stop_latency,
        "servo_watchdog_stop_latency_limit_s": (
            servo_watchdog_stop_latency_limit_s
        ),
        "servo_watchdog_safe_stop_passed": safety_stop_passed,
        "physical_disturbance_enabled": enable_physical_disturbance,
        "physical_disturbance_stage": physical_disturbance_stage,
        "physical_disturbance_delay_s": physical_disturbance_delay_s,
        "physical_disturbance_duration_s": physical_disturbance_duration_s,
        "physical_disturbance_force_x_n": physical_disturbance_force_x_n,
        "physical_disturbance_force_y_n": physical_disturbance_force_y_n,
        "physical_disturbance_force_z_n": physical_disturbance_force_z_n,
        "physical_disturbance_applied": disturbance_verdict["applied"],
        "physical_disturbance_cleared": disturbance_verdict["cleared"],
        "physical_disturbance_observed_stage": (
            disturbance_verdict["observed_stage"]
        ),
        "physical_disturbance_stop_latency_s": (
            disturbance_verdict["stop_latency_s"]
        ),
        "physical_disturbance_stop_latency_limit_s": (
            physical_disturbance_stop_latency_limit_s
        ),
        "physical_disturbance_safe_stop_passed": (
            disturbance_verdict["passed"] if enable_physical_disturbance else None
        ),
        "final_state": final_state,
        "failure_reason": failure_reason,
        "elapsed_s": elapsed_s,
        "samples": diagnostics.get("samples"),
        "max_arm_position_error_rad": diagnostics.get(
            "max_arm_position_error_rad"
        ),
        "rms_arm_position_error_rad": diagnostics.get(
            "rms_arm_position_error_rad"
        ),
        "max_transfer_arm_position_error_rad": diagnostics.get(
            "max_transfer_arm_position_error_rad"
        ),
        "rms_transfer_arm_position_error_rad": diagnostics.get(
            "rms_transfer_arm_position_error_rad"
        ),
        "max_gripper_position_error_m": diagnostics.get(
            "max_gripper_position_error_m"
        ),
        "max_payload_relative_drift_m": diagnostics.get(
            "max_transfer_relative_drift_m"
        ),
        "max_grasp_probe_relative_drift_m": diagnostics.get(
            "max_grasp_probe_relative_drift_m"
        ),
        "payload_bilateral_contact_ratio": diagnostics.get(
            "payload_bilateral_contact_ratio"
        ),
        "physical_lift_delta_m": (
            physical_result.get("lift_delta_m")
            if physical_result is not None
            else None
        ),
        "physical_place_error_m": (
            physical_result.get("place_error_m")
            if physical_result is not None
            else None
        ),
        "physical_final_tilt_deg": (
            physical_result.get("final_tilt_deg")
            if physical_result is not None
            else None
        ),
        "first_payload_drift_stage": crossing.get("stage"),
        "first_payload_drift_elapsed_s": crossing.get("elapsed_s"),
        "pre_descent_arm_positions_rad": json.dumps(
            diagnostics.get("pre_descent_arm_positions_rad")
        ),
        "telemetry_csv": str(telemetry_csv),
        "telemetry_summary": str(telemetry_summary),
        "rosbag_path": str(bag_dir) if record_bag else None,
        "log_path": str(log_path),
    }


def write_outputs(rows: list[Dict[str, object]], output_dir: Path) -> Dict[str, object]:
    """Write raw paired rows plus JSON and Markdown aggregate reports."""
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "payload_transfer_comparison.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=RESULT_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    summary = summarize_comparison(rows)
    (output_dir / "payload_transfer_comparison.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8"
    )
    (output_dir / "payload_transfer_comparison.md").write_text(
        render_comparison_markdown(summary), encoding="utf-8"
    )
    if len(
        {
            str(row.get("place_descent_backend"))
            for row in rows
            if row.get("place_descent_backend") is not None
        }
    ) > 1:
        backend_summary = summarize_backend_comparison(rows)
        (output_dir / "place_descent_backend_comparison.json").write_text(
            json.dumps(backend_summary, indent=2, sort_keys=True),
            encoding="utf-8",
        )
        (output_dir / "place_descent_backend_comparison.md").write_text(
            render_backend_comparison_markdown(backend_summary),
            encoding="utf-8",
        )
    if any(
        row.get("servo_fault_injection_type") in {
            "contact_loss",
            "payload_drift",
        }
        for row in rows
    ):
        safety_summary = summarize_servo_watchdog_safety(rows)
        (output_dir / "servo_watchdog_safety.json").write_text(
            json.dumps(safety_summary, indent=2, sort_keys=True),
            encoding="utf-8",
        )
        (output_dir / "servo_watchdog_safety.md").write_text(
            render_servo_watchdog_safety_markdown(safety_summary),
            encoding="utf-8",
        )
    if any(row.get("physical_disturbance_enabled") is True for row in rows):
        physical_safety = summarize_physical_disturbance_safety(rows)
        (output_dir / "physical_disturbance_safety.json").write_text(
            json.dumps(physical_safety, indent=2, sort_keys=True),
            encoding="utf-8",
        )
        (output_dir / "physical_disturbance_safety.md").write_text(
            render_physical_disturbance_safety_markdown(physical_safety),
            encoding="utf-8",
        )
    return summary


def load_output_rows(csv_path: Path) -> list[Dict[str, object]]:
    """Load previously saved trial rows for report-only regeneration."""
    rows: list[Dict[str, object]] = []
    with csv_path.open(newline="", encoding="utf-8") as source:
        for raw_row in csv.DictReader(source):
            row: Dict[str, object] = {}
            for field, value in raw_row.items():
                row[field] = decode_csv_value(value)
            rows.append(row)
    return rows


def decode_csv_value(value: str) -> object:
    """Decode JSON cells plus csv.DictWriter's Python booleans."""
    if value == "":
        return None
    normalized = value.strip().lower()
    if normalized == "true":
        return True
    if normalized == "false":
        return False
    if normalized == "none":
        return None
    try:
        return json.loads(value)
    except (json.JSONDecodeError, TypeError):
        return value


def parse_joint_positions_arg(value: str) -> list[float]:
    """Parse one finite seven-joint configuration from a CLI argument."""
    try:
        positions = [float(item.strip()) for item in value.split(",")]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            "pre-place joint positions must be comma-separated numbers"
        ) from exc
    if len(positions) != 7:
        raise argparse.ArgumentTypeError(
            "pre-place joint replay requires exactly seven positions"
        )
    if not all(math.isfinite(position) for position in positions):
        raise argparse.ArgumentTypeError(
            "pre-place joint positions must all be finite"
        )
    return positions


def servo_fault_cases(
    stage_suite: bool,
    suite_types: list[str] | tuple[str, ...],
    stage_count: int,
    single_type: str,
    single_stage: int,
) -> tuple[tuple[str, int], ...]:
    """Build the deterministic fault matrix or one configured fault case."""
    if not stage_suite:
        return ((str(single_type), int(single_stage)),)
    return tuple(
        (str(fault_type), stage)
        for fault_type in suite_types
        for stage in range(1, int(stage_count) + 1)
    )


def physical_disturbance_stages(
    stage_suite: bool,
    stage_count: int,
    single_stage: int,
) -> tuple[int, ...]:
    """Build either one physical-disturbance stage or the full descent matrix."""
    if not stage_suite:
        return (int(single_stage),)
    return tuple(range(1, int(stage_count) + 1))


def diagnostic_case_key(row: Dict[str, object]) -> tuple[object, ...]:
    """Return the stable identity of one matrix row for interruption-safe resume."""
    return (
        int(row.get("pair") or 0),
        str(row.get("mode") or ""),
        str(row.get("place_descent_backend") or ""),
        str(row.get("servo_fault_injection_type") or "none"),
        int(row.get("servo_fault_injection_stage") or 0),
        row.get("physical_disturbance_enabled") is True,
        int(row.get("physical_disturbance_stage") or 0),
        float(row.get("physical_disturbance_force_x_n") or 0.0),
        float(row.get("physical_disturbance_force_y_n") or 0.0),
        float(row.get("physical_disturbance_force_z_n") or 0.0),
        float(row.get("physical_disturbance_duration_s") or 0.0),
        float(row.get("physical_disturbance_delay_s") or 0.0),
    )


def decoded_joint_positions(value: object) -> Optional[list[float]]:
    """Decode one finite seven-joint cell from an in-memory or CSV row."""
    if isinstance(value, str):
        try:
            value = json.loads(value)
        except json.JSONDecodeError:
            return None
    if not isinstance(value, list) or len(value) != 7:
        return None
    if not all(
        isinstance(position, (int, float)) and math.isfinite(float(position))
        for position in value
    ):
        return None
    return [float(position) for position in value]


def physical_disturbance_forces(
    bidirectional: bool,
    force_x_n: float,
    force_y_n: float,
    force_z_n: float,
) -> tuple[tuple[float, float, float], ...]:
    """Return one wrench direction or a deterministic opposite-direction pair."""
    force = (float(force_x_n), float(force_y_n), float(force_z_n))
    if not bidirectional:
        return (force,)
    opposite = tuple(-component for component in force)
    return (force, opposite)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs-per-mode", type=int, default=5)
    parser.add_argument(
        "--modes", nargs="+", choices=("unloaded", "loaded"), default=("unloaded", "loaded")
    )
    parser.add_argument("--planner-id", default="RRTConnectkConfigDefault")
    parser.add_argument("--grasp-height-offset", type=float, default=0.10)
    parser.add_argument("--gripper-closed-position", type=float, default=0.022)
    parser.add_argument(
        "--payload-transfer-velocity-scaling-factor",
        type=float,
        default=0.03,
    )
    parser.add_argument(
        "--place-descent-velocity-scaling-factor",
        type=float,
        default=0.01,
    )
    parser.add_argument(
        "--place-descent-recovery-velocity-scaling-factor",
        type=float,
        default=0.02,
    )
    parser.add_argument("--place-descent-stage-count", type=int, default=6)
    parser.add_argument(
        "--place-descent-backend",
        choices=("cartesian", "servo"),
        default="cartesian",
    )
    parser.add_argument(
        "--compare-place-descent-backends",
        action="store_true",
        help=(
            "Run matched loaded Cartesian then Servo trials for each pair. The "
            "Servo trial replays the Cartesian grasp yaw and pre-place joints."
        ),
    )
    parser.add_argument(
        "--servo-fault-injection-type",
        choices=("none", "contact_loss", "payload_drift"),
        default="none",
    )
    parser.add_argument(
        "--servo-fault-stage-suite",
        action="store_true",
        help=(
            "Run contact_loss and payload_drift once at every configured "
            "descent stage. --runs-per-mode controls repetitions per case."
        ),
    )
    parser.add_argument(
        "--servo-fault-suite-types",
        nargs="+",
        choices=("contact_loss", "payload_drift"),
        default=("contact_loss", "payload_drift"),
    )
    parser.add_argument("--servo-fault-injection-stage", type=int, default=2)
    parser.add_argument("--servo-fault-injection-delay-s", type=float, default=0.5)
    parser.add_argument(
        "--servo-watchdog-stop-latency-limit-s",
        type=float,
        default=0.15,
    )
    parser.add_argument(
        "--physical-disturbance",
        action="store_true",
        help=(
            "Apply a bounded Gazebo force to the loaded target during one "
            "Servo descent stage and evaluate the real watchdog response."
        ),
    )
    parser.add_argument(
        "--physical-disturbance-stage-suite",
        action="store_true",
        help="Apply the configured real Gazebo wrench once at every Servo stage.",
    )
    parser.add_argument(
        "--physical-disturbance-bidirectional",
        action="store_true",
        help="Run both the configured Gazebo force vector and its opposite.",
    )
    parser.add_argument("--physical-disturbance-stage", type=int, default=3)
    parser.add_argument("--physical-disturbance-delay-s", type=float, default=0.5)
    parser.add_argument(
        "--physical-disturbance-duration-s",
        type=float,
        default=0.10,
    )
    parser.add_argument("--physical-disturbance-force-x-n", type=float, default=0.0)
    parser.add_argument("--physical-disturbance-force-y-n", type=float, default=2.0)
    parser.add_argument("--physical-disturbance-force-z-n", type=float, default=0.0)
    parser.add_argument(
        "--physical-disturbance-stop-latency-limit-s",
        type=float,
        default=0.2,
    )
    parser.add_argument("--grasp-prevalidation-max-rounds", type=int, default=4)
    parser.add_argument(
        "--forced-grasp-yaw-offset-deg",
        type=float,
        default=None,
        help=(
            "Force one grasp yaw for every paired run instead of selecting the "
            "unloaded yaw dynamically. Useful for repeatable causal diagnostics."
        ),
    )
    parser.add_argument(
        "--pre-place-joint-replay-positions",
        type=parse_joint_positions_arg,
        default=None,
        metavar="J1,J2,J3,J4,J5,J6,J7",
        help=(
            "Replay one measured seven-joint pre-place configuration in every "
            "trial. This makes Servo singularity and contact-zone regressions "
            "reproducible instead of depending on OMPL sampling."
        ),
    )
    parser.add_argument("--timeout-s", type=float, default=420.0)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("artifacts/runs/payload_transfer_diagnostics"),
    )
    parser.add_argument("--no-rosbag", action="store_true")
    parser.add_argument(
        "--resume-existing",
        action="store_true",
        help=(
            "Continue an interrupted matrix from payload_transfer_comparison.csv "
            "inside the output directory."
        ),
    )
    parser.add_argument(
        "--report-only-csv",
        type=Path,
        nargs="+",
        default=None,
        help=(
            "Regenerate reports from one or more existing comparison CSVs "
            "without starting ROS or Gazebo. Multiple inputs are merged into "
            "--output-dir."
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.report_only_csv is not None:
        csv_paths = [path.resolve() for path in args.report_only_csv]
        missing = [path for path in csv_paths if not path.is_file()]
        if missing:
            raise SystemExit(f"comparison CSV does not exist: {missing[0]}")
        rows = [
            row
            for csv_path in csv_paths
            for row in load_output_rows(csv_path)
        ]
        report_dir = (
            csv_paths[0].parent
            if len(csv_paths) == 1
            else args.output_dir.resolve()
        )
        summary = write_outputs(rows, report_dir)
        print(
            f"Regenerated report for {summary['total_trials']} trials: "
            f"{report_dir}",
            flush=True,
        )
        return 0
    if args.runs_per_mode < 1:
        raise SystemExit("--runs-per-mode must be at least 1")
    if not 0.05 <= args.grasp_height_offset <= 0.15:
        raise SystemExit("--grasp-height-offset must be between 0.05 and 0.15 m")
    if not 0.0 <= args.gripper_closed_position <= 0.04:
        raise SystemExit("--gripper-closed-position must be between 0.0 and 0.04 m")
    if not 0.005 <= args.payload_transfer_velocity_scaling_factor <= 0.10:
        raise SystemExit(
            "--payload-transfer-velocity-scaling-factor must be between 0.005 and 0.10"
        )
    if not 0.005 <= args.place_descent_velocity_scaling_factor <= 0.05:
        raise SystemExit(
            "--place-descent-velocity-scaling-factor must be between 0.005 and 0.05"
        )
    if not (
        args.place_descent_velocity_scaling_factor
        <= args.place_descent_recovery_velocity_scaling_factor
        <= 0.10
    ):
        raise SystemExit(
            "--place-descent-recovery-velocity-scaling-factor must be between "
            "the nominal descent factor and 0.10"
        )
    if not 1 <= args.place_descent_stage_count <= 12:
        raise SystemExit("--place-descent-stage-count must be between 1 and 12")
    if not 1 <= args.grasp_prevalidation_max_rounds <= 10:
        raise SystemExit(
            "--grasp-prevalidation-max-rounds must be between 1 and 10"
        )
    if not 1 <= args.servo_fault_injection_stage <= args.place_descent_stage_count:
        raise SystemExit(
            "--servo-fault-injection-stage must be inside the configured descent"
        )
    if not 0.0 <= args.servo_fault_injection_delay_s <= 10.0:
        raise SystemExit(
            "--servo-fault-injection-delay-s must be between 0 and 10 seconds"
        )
    if not 0.02 <= args.servo_watchdog_stop_latency_limit_s <= 1.0:
        raise SystemExit(
            "--servo-watchdog-stop-latency-limit-s must be between 0.02 and 1.0"
        )
    if not 1 <= args.physical_disturbance_stage <= args.place_descent_stage_count:
        raise SystemExit(
            "--physical-disturbance-stage must be inside the configured descent"
        )
    if not 0.0 <= args.physical_disturbance_delay_s <= 10.0:
        raise SystemExit(
            "--physical-disturbance-delay-s must be between 0 and 10 seconds"
        )
    if not 0.01 <= args.physical_disturbance_duration_s <= 2.0:
        raise SystemExit(
            "--physical-disturbance-duration-s must be between 0.01 and 2 seconds"
        )
    disturbance_force_norm = math.sqrt(
        args.physical_disturbance_force_x_n**2
        + args.physical_disturbance_force_y_n**2
        + args.physical_disturbance_force_z_n**2
    )
    if not 0.0 < disturbance_force_norm <= 20.0:
        raise SystemExit(
            "physical disturbance force magnitude must be above 0 and at most 20 N"
        )
    if not 0.05 <= args.physical_disturbance_stop_latency_limit_s <= 5.0:
        raise SystemExit(
            "--physical-disturbance-stop-latency-limit-s must be between 0.05 and 5.0"
        )
    if (
        args.servo_fault_injection_type != "none"
        and args.place_descent_backend != "servo"
        and not args.compare_place_descent_backends
    ):
        raise SystemExit("Servo fault injection requires the Servo descent backend")
    if (
        args.forced_grasp_yaw_offset_deg is not None
        and not -180.0 <= args.forced_grasp_yaw_offset_deg <= 180.0
    ):
        raise SystemExit(
            "--forced-grasp-yaw-offset-deg must be between -180 and 180"
        )
    if args.compare_place_descent_backends and tuple(args.modes) != ("loaded",):
        raise SystemExit(
            "--compare-place-descent-backends requires --modes loaded"
        )
    if (
        args.compare_place_descent_backends
        and args.servo_fault_injection_type != "none"
    ):
        raise SystemExit(
            "fault injection and backend comparison must run as separate suites"
        )
    if args.servo_fault_stage_suite:
        if tuple(args.modes) != ("loaded",):
            raise SystemExit("--servo-fault-stage-suite requires --modes loaded")
        if args.place_descent_backend != "servo":
            raise SystemExit(
                "--servo-fault-stage-suite requires --place-descent-backend servo"
            )
        if args.compare_place_descent_backends:
            raise SystemExit(
                "fault stage suite and backend comparison must run separately"
            )
    if args.physical_disturbance:
        if tuple(args.modes) != ("loaded",):
            raise SystemExit("--physical-disturbance requires --modes loaded")
        if args.place_descent_backend != "servo":
            raise SystemExit(
                "--physical-disturbance requires --place-descent-backend servo"
            )
        if args.compare_place_descent_backends:
            raise SystemExit(
                "physical disturbance and backend comparison must run separately"
            )
        if args.servo_fault_stage_suite or args.servo_fault_injection_type != "none":
            raise SystemExit(
                "physical and synthetic disturbance suites must run separately"
            )
    if args.physical_disturbance_stage_suite and not args.physical_disturbance:
        raise SystemExit(
            "--physical-disturbance-stage-suite requires --physical-disturbance"
        )
    if args.physical_disturbance_bidirectional and not args.physical_disturbance:
        raise SystemExit(
            "--physical-disturbance-bidirectional requires --physical-disturbance"
        )
    place_descent_backends = (
        ("cartesian", "servo")
        if args.compare_place_descent_backends
        else (args.place_descent_backend,)
    )
    fault_cases = servo_fault_cases(
        args.servo_fault_stage_suite,
        args.servo_fault_suite_types,
        args.place_descent_stage_count,
        args.servo_fault_injection_type,
        args.servo_fault_injection_stage,
    )
    disturbance_stages = physical_disturbance_stages(
        args.physical_disturbance_stage_suite,
        args.place_descent_stage_count,
        args.physical_disturbance_stage,
    )
    disturbance_forces = physical_disturbance_forces(
        args.physical_disturbance_bidirectional,
        args.physical_disturbance_force_x_n,
        args.physical_disturbance_force_y_n,
        args.physical_disturbance_force_z_n,
    )
    total = (
        args.runs_per_mode
        * len(args.modes)
        * len(place_descent_backends)
        * len(fault_cases)
        * len(disturbance_stages)
        * len(disturbance_forces)
    )
    rows: list[Dict[str, object]] = []
    existing_csv = args.output_dir / "payload_transfer_comparison.csv"
    try:
        guard_paired_checkpoint(args, existing_csv)
    except (ValueError, OSError) as exc:
        raise SystemExit(str(exc)) from exc
    if args.resume_existing and existing_csv.exists():
        rows = load_output_rows(existing_csv)
        print(
            f"Resuming from {len(rows)} completed trial(s) in {existing_csv}.",
            flush=True,
        )
    pair_yaw_offsets: Dict[int, float] = {}
    pair_pre_place_joint_targets: Dict[int, list[float]] = {}
    for existing_row in rows:
        existing_pair = int(existing_row.get("pair") or 0)
        existing_yaw = existing_row.get("grasp_yaw_offset_deg")
        if (
            existing_pair > 0
            and existing_pair not in pair_yaw_offsets
            and isinstance(existing_yaw, (int, float))
            and math.isfinite(float(existing_yaw))
        ):
            pair_yaw_offsets[existing_pair] = float(existing_yaw)
        existing_positions = decoded_joint_positions(
            existing_row.get("pre_place_joint_planned_positions_rad")
        )
        if (
            existing_pair > 0
            and existing_pair not in pair_pre_place_joint_targets
            and existing_positions is not None
        ):
            pair_pre_place_joint_targets[existing_pair] = existing_positions
    completed_case_keys = {diagnostic_case_key(row) for row in rows}
    trial = len(rows)
    for pair in range(1, args.runs_per_mode + 1):
        for mode in args.modes:
            for place_descent_backend in place_descent_backends:
                for fault_type, fault_stage in fault_cases:
                    for disturbance_stage in disturbance_stages:
                        for disturbance_force in disturbance_forces:
                            case_key = (
                                pair,
                                mode,
                                place_descent_backend,
                                fault_type,
                                fault_stage,
                                args.physical_disturbance,
                                disturbance_stage,
                                *disturbance_force,
                                args.physical_disturbance_duration_s,
                                args.physical_disturbance_delay_s,
                            )
                            if case_key in completed_case_keys:
                                continue
                            trial += 1
                            print(
                                f"[{trial}/{total}] pair={pair}, mode={mode}, "
                                f"backend={place_descent_backend}, "
                                f"fault={fault_type}, fault_stage={fault_stage}, "
                                f"physical_disturbance="
                                f"{'on' if args.physical_disturbance else 'off'}, "
                                f"disturbance_stage={disturbance_stage}, "
                                f"force={disturbance_force}N, "
                                f"planner={args.planner_id}...",
                                flush=True,
                            )
                            row = run_trial(
                                trial,
                                pair,
                                mode,
                                args.planner_id,
                                args.timeout_s,
                                args.output_dir,
                                not args.no_rosbag,
                                args.grasp_height_offset,
                                args.gripper_closed_position,
                                args.payload_transfer_velocity_scaling_factor,
                                args.place_descent_velocity_scaling_factor,
                                args.place_descent_recovery_velocity_scaling_factor,
                                args.place_descent_stage_count,
                                place_descent_backend,
                                (
                                    args.forced_grasp_yaw_offset_deg
                                    if args.forced_grasp_yaw_offset_deg is not None
                                    else pair_yaw_offsets.get(pair)
                                ),
                                args.grasp_prevalidation_max_rounds,
                                (
                                    args.pre_place_joint_replay_positions
                                    if args.pre_place_joint_replay_positions
                                    is not None
                                    else pair_pre_place_joint_targets.get(pair)
                                ),
                                fault_type,
                                fault_stage,
                                args.servo_fault_injection_delay_s,
                                args.servo_watchdog_stop_latency_limit_s,
                                args.physical_disturbance,
                                disturbance_stage,
                                args.physical_disturbance_delay_s,
                                args.physical_disturbance_duration_s,
                                disturbance_force[0],
                                disturbance_force[1],
                                disturbance_force[2],
                                args.physical_disturbance_stop_latency_limit_s,
                            )
                            rows.append(row)
                            completed_case_keys.add(case_key)
                            selected_yaw = row.get("grasp_yaw_offset_deg")
                            if (
                                pair not in pair_yaw_offsets
                                and isinstance(selected_yaw, (int, float))
                                and math.isfinite(float(selected_yaw))
                            ):
                                pair_yaw_offsets[pair] = float(selected_yaw)
                            planned_positions = decoded_joint_positions(
                                row.get("pre_place_joint_planned_positions_rad")
                            )
                            if (
                                pair not in pair_pre_place_joint_targets
                                and planned_positions is not None
                            ):
                                pair_pre_place_joint_targets[pair] = planned_positions
                            verdict = None
                            if args.physical_disturbance:
                                verdict = (
                                    "SAFE_STOP_PASS"
                                    if row.get(
                                        "physical_disturbance_safe_stop_passed"
                                    )
                                    is True
                                    else "SAFE_STOP_FAIL"
                                )
                            reason = (
                                f": {row['failure_reason']}"
                                if row.get("failure_reason")
                                and verdict != "SAFE_STOP_PASS"
                                else ""
                            )
                            print(
                                f"[{trial}/{total}] {verdict or row['final_state']}"
                                f"{reason} ({row['elapsed_s']:.1f}s)",
                                flush=True,
                            )
                            write_outputs(rows, args.output_dir)
    summary = write_outputs(rows, args.output_dir)
    if args.physical_disturbance:
        report_name = "physical_disturbance_safety.md"
    elif args.servo_fault_stage_suite or args.servo_fault_injection_type != "none":
        report_name = "servo_watchdog_safety.md"
    elif args.compare_place_descent_backends:
        report_name = "place_descent_backend_comparison.md"
    else:
        report_name = "payload_transfer_comparison.md"
    print(
        f"Completed {summary['total_trials']} diagnostic trials. "
        f"Report: {args.output_dir / report_name}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
