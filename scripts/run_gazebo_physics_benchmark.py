"""Run isolated Gazebo pick trials and write physical-validation reports."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import random
import selectors
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Any, Dict, Optional

REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPO_ROOT / "src" / "panda_manipulation"
DEFAULT_WORLD_PATH = PACKAGE_ROOT / "worlds" / "panda_table.sdf"
sys.path.insert(0, str(PACKAGE_ROOT))

from panda_manipulation.physics_benchmark import (  # noqa: E402
    CHALLENGE_SCENARIOS,
    ChallengeScenario,
    pipeline_process_exit_reason,
    read_report_rows,
    reclassify_infrastructure_failures,
    validation_to_row,
    write_reports,
    write_svg_report,
    write_trial_world,
)


PLANNER_SUITE = [
    "RRTConnectkConfigDefault",
    "PRMkConfigDefault",
    "RRTstarkConfigDefault",
]


def isolated_trial_environment(
    trial: int,
    process_id: Optional[int] = None,
) -> Dict[str, str]:
    """Isolate repeated ROS and Gazebo launches from stale discovery state."""
    environment = os.environ.copy()
    try:
        base_domain_id = int(environment.get("ROS_DOMAIN_ID", "0") or 0)
    except ValueError:
        base_domain_id = 0
    owner_pid = os.getpid() if process_id is None else process_id
    environment["ROS_DOMAIN_ID"] = str(
        (base_domain_id + owner_pid + trial) % 101
    )
    environment["GZ_PARTITION"] = f"panda_benchmark_{owner_pid}_{trial}"
    return environment


def stop_process_group(process: subprocess.Popen[str]) -> None:
    if process.poll() is not None:
        return
    for sig, wait_s in ((signal.SIGINT, 15), (signal.SIGTERM, 5)):
        try:
            os.killpg(process.pid, sig)
            process.wait(timeout=wait_s)
            return
        except (ProcessLookupError, subprocess.TimeoutExpired):
            continue
    os.killpg(process.pid, signal.SIGKILL)
    process.wait(timeout=5)


def stop_gazebo_world(world_path: Optional[Path]) -> None:
    """Stop a detached gz server that still owns this trial's world file."""
    if world_path is None:
        return
    world_signature = str(world_path.resolve())
    listing = subprocess.run(
        ["ps", "-eo", "pid=,args="],
        check=False,
        capture_output=True,
        text=True,
    ).stdout
    pids = []
    for line in listing.splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        pid_text, _, command = stripped.partition(" ")
        if (
            world_signature in command
            and ("gz sim" in command or "ruby " in command)
        ):
            try:
                pids.append(int(pid_text))
            except ValueError:
                continue
    if not pids:
        return
    for sig, wait_s in ((signal.SIGTERM, 2.0), (signal.SIGKILL, 0.0)):
        for pid in pids:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        if wait_s:
            time.sleep(wait_s)


def parse_validation(line: str) -> Optional[Dict[str, Any]]:
    if "gazebo_pick_validator" not in line or "pipeline_state" not in line:
        return None
    json_start = line.find("{")
    if json_start < 0:
        return None
    try:
        payload = json.loads(line[json_start:])
    except json.JSONDecodeError:
        return None
    return payload if "lift_delta_m" in payload else None


def parse_pipeline_status(line: str) -> Optional[Dict[str, Any]]:
    if "gazebo_pick_pipeline" not in line or "final_state" not in line:
        return None
    json_start = line.find("{")
    if json_start < 0:
        return None
    try:
        payload = json.loads(line[json_start:])
    except json.JSONDecodeError:
        return None
    return payload if "final_state" in payload else None


def parse_perception_validation(line: str) -> Optional[Dict[str, Any]]:
    if "perception_validator" not in line or "mean_error_m" not in line:
        return None
    json_start = line.find("{")
    if json_start < 0:
        return None
    try:
        payload = json.loads(line[json_start:])
    except json.JSONDecodeError:
        return None
    return payload if "samples" in payload else None


def merge_perception_validation(
    physical_validation: Dict[str, Any],
    perception_validation: Optional[Dict[str, Any]],
    perception_mode: str,
) -> Dict[str, Any]:
    merged = dict(physical_validation)
    merged["perception_mode"] = perception_mode
    if perception_mode not in ("aruco", "rgbd"):
        return merged
    if perception_validation is None:
        merged["perception_validation_state"] = "NOT_RECEIVED"
        if merged.get("final_state") == "SUCCESS":
            merged.update(
                {
                    "final_state": "FAILED",
                    "reason": "perception validation result was not received",
                }
            )
        return merged
    merged.update(
        {
            "perception_validation_state": perception_validation.get(
                "final_state"
            ),
            "perception_samples": perception_validation.get("samples"),
            "perception_mean_error_m": perception_validation.get(
                "mean_error_m"
            ),
            "perception_max_error_m": perception_validation.get(
                "max_error_m"
            ),
            "perception_std_error_m": perception_validation.get(
                "std_error_m"
            ),
            "perception_mean_x_error_m": perception_validation.get(
                "mean_x_error_m"
            ),
            "perception_mean_y_error_m": perception_validation.get(
                "mean_y_error_m"
            ),
            "perception_mean_z_error_m": perception_validation.get(
                "mean_z_error_m"
            ),
            "perception_first_detection_latency_s": perception_validation.get(
                "first_detection_latency_s"
            ),
            "perception_validation_duration_s": perception_validation.get(
                "validation_duration_s"
            ),
            "perception_detection_rate": perception_validation.get(
                "detection_rate"
            ),
        }
    )
    if perception_validation.get("final_state") != "SUCCESS":
        merged["final_state"] = "FAILED"
        merged["reason"] = (
            "perception validation failed: "
            f"{perception_validation.get('reason') or 'unknown failure'}"
        )
    return merged


def run_trial(
    trial: int,
    timeout_s: float,
    log_dir: Path,
    target_x: float = 0.52,
    target_y: float = 0.0,
    world_path: Optional[Path] = None,
    planner_id: Optional[str] = None,
    scenario: Optional[ChallengeScenario] = None,
    show_rviz: bool = False,
    show_gazebo_gui: bool = False,
    hold_after_result_s: float = 0.0,
    perception_mode: str = "synthetic",
    transfer_diagnostics: bool = False,
    pre_place_height_step: float = 0.0,
    cube_symmetric_placement: bool = False,
    place_endpoint_precheck: bool = False,
    loaded_path_precheck: bool = False,
    grasp_height_offset: Optional[float] = None,
    grasp_pitch_offset_deg: float = 0.0,
) -> Dict[str, Any]:
    effective_world_path = (world_path or DEFAULT_WORLD_PATH).resolve()
    trial_environment = isolated_trial_environment(trial)
    command = [
        "ros2",
        "launch",
        "panda_manipulation",
        "gazebo_pick.launch.py",
        f"gui:={'true' if show_gazebo_gui else 'false'}",
        f"rviz:={'true' if show_rviz else 'false'}",
        f"target_x:={target_x:.6f}",
        f"target_y:={target_y:.6f}",
        f"use_camera_perception:={'true' if perception_mode == 'aruco' else 'false'}",
        f"use_rgbd_target_perception:={'true' if perception_mode == 'rgbd' else 'false'}",
        f"world:={effective_world_path}",
        f"cube_symmetric_placement:={'true' if cube_symmetric_placement else 'false'}",
        f"enable_place_endpoint_precheck:={'true' if place_endpoint_precheck or loaded_path_precheck else 'false'}",
        f"enable_loaded_path_precheck:={'true' if loaded_path_precheck else 'false'}",
        f"grasp_pitch_offset_deg:={grasp_pitch_offset_deg}",
    ]
    if perception_mode == "rgbd":
        command.extend(
            [
                "grasp_preclose_max_target_drift_m:=0.002",
                # The deeper pad overlap matches the established physical
                # benchmark; the strict pre-close drift gate catches pushes.
                "grasp_height_offset:=0.105",
                "gripper_closed_position:=0.022",
                # Keep the markerless baseline inside the conservative
                # controller's repeatable workspace. Obstacle challenges use
                # the farther -0.30 m placement target below.
                "place_y:=-0.080",
                # Transfer only to a high, safe pre-place waypoint, then descend
                # through collision-checked Cartesian stages.
                "use_ompl_for_place:=true",
                "use_cartesian_pre_place_transfer:=true",
            ]
        )
    if planner_id is not None:
        command.append(f"planner_ids_csv:={planner_id}")
    if grasp_height_offset is not None:
        command.append(f"grasp_height_offset:={grasp_height_offset}")
    if scenario is not None:
        obstacle_pose = scenario.obstacle_pose_xyz
        obstacle_size = scenario.obstacle_size_xyz
        command.extend(
            [
                f"obstacle_x:={obstacle_pose[0]:.6f}",
                f"obstacle_y:={obstacle_pose[1]:.6f}",
                f"obstacle_z:={obstacle_pose[2]:.6f}",
                f"obstacle_size_x:={obstacle_size[0]:.6f}",
                f"obstacle_size_y:={obstacle_size[1]:.6f}",
                f"obstacle_size_z:={obstacle_size[2]:.6f}",
                "diagnose_direct_path:=true",
                "require_direct_path_blocked:=true",
                f"static_environment_world:={effective_world_path}",
                f"pre_place_candidate_height_step_m:={pre_place_height_step}",
                "enable_grasp_candidate_prevalidation:=true",
                "ompl_velocity_scaling_factor:=0.05",
                "payload_transfer_velocity_scaling_factor:=0.03",
                "place_descent_velocity_scaling_factor:=0.01",
                "place_descent_recovery_velocity_scaling_factor:=0.02",
                "place_descent_stage_count:=6",
                "grasp_preclose_max_target_drift_m:=0.005",
                "use_ompl_for_place:=true",
                "use_cartesian_pre_place_transfer:=false",
                "pre_grasp_orientation_tolerance:=0.10",
                # Keep the challenge placement goal outside every obstacle's
                # footprint so the benchmark measures planning, not an invalid goal.
                "place_x:=0.50",
                "place_y:=-0.30",
            ]
        )
    if (place_endpoint_precheck or loaded_path_precheck) and scenario is None:
        command.append("enable_grasp_candidate_prevalidation:=true")
    if transfer_diagnostics:
        telemetry_dir = Path(tempfile.mkdtemp(
            prefix=f"trial_{trial:03d}_telemetry_", dir=log_dir.resolve()
        ))
        command.extend([
            "enable_transfer_diagnostics:=true",
            "diagnose_rejected_descent:=true",
            f"transfer_diagnostics_csv_path:={telemetry_dir / 'samples.csv'}",
            f"transfer_diagnostics_summary_path:={telemetry_dir / 'summary.json'}",
            f"transfer_diagnostics_label:=benchmark_trial_{trial:03d}",
        ])
        print(f"Telemetry directory: {telemetry_dir}", flush=True)
    started = time.monotonic()
    log_path = log_dir / f"trial_{trial:03d}.log"
    stop_gazebo_world(effective_world_path)
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        start_new_session=True,
        env=trial_environment,
    )
    physical_validation = None
    perception_validation = None
    validation_elapsed_s = None
    latest_pipeline_status = None
    selector = selectors.DefaultSelector()
    assert process.stdout is not None
    selector.register(process.stdout, selectors.EVENT_READ)

    try:
        with log_path.open("w", encoding="utf-8") as log_file:
            while time.monotonic() - started < timeout_s:
                events = selector.select(timeout=0.5)
                for key, _ in events:
                    line = key.fileobj.readline()
                    if not line:
                        continue
                    log_file.write(line)
                    log_file.flush()
                    parsed_physical = parse_validation(line)
                    if parsed_physical is not None:
                        physical_validation = parsed_physical
                    parsed_perception = parse_perception_validation(line)
                    if parsed_perception is not None:
                        perception_validation = parsed_perception
                    if physical_validation is not None and (
                        perception_mode not in ("aruco", "rgbd")
                        or perception_validation is not None
                    ):
                        validation_elapsed_s = time.monotonic() - started
                        break
                    pipeline_status = parse_pipeline_status(line)
                    if pipeline_status is not None:
                        latest_pipeline_status = pipeline_status
                    process_exit_reason = pipeline_process_exit_reason(line)
                    if process_exit_reason is not None:
                        physical_validation = {
                            "final_state": "FAILED",
                            "pipeline_state": None,
                            "reason": process_exit_reason,
                        }
                        validation_elapsed_s = time.monotonic() - started
                        break
                result_complete = physical_validation is not None and (
                    perception_mode not in ("aruco", "rgbd")
                    or perception_validation is not None
                    or physical_validation.get("final_state") != "SUCCESS"
                )
                if result_complete or process.poll() is not None:
                    break
            if physical_validation is not None and hold_after_result_s > 0.0:
                time.sleep(hold_after_result_s)
    finally:
        selector.close()
        stop_process_group(process)
        stop_gazebo_world(effective_world_path)

    elapsed_s = (
        validation_elapsed_s
        if validation_elapsed_s is not None
        else time.monotonic() - started
    )
    if physical_validation is None:
        if elapsed_s >= timeout_s:
            waiting_reason = str(
                (latest_pipeline_status or {}).get("failure_reason") or ""
            )
            if "controller action" in waiting_reason and "not available" in waiting_reason:
                reason = f"infrastructure startup timeout: {waiting_reason}"
            elif waiting_reason:
                reason = (
                    f"trial timed out after {timeout_s:.0f}s; "
                    f"last pipeline state: {waiting_reason}"
                )
            else:
                reason = f"trial timed out after {timeout_s:.0f}s"
        else:
            reason = f"launch exited before validation (code={process.returncode})"
        physical_validation = {
            "final_state": "FAILED",
            "pipeline_state": None,
            "reason": reason,
        }
    validation = merge_perception_validation(
        physical_validation,
        perception_validation,
        perception_mode,
    )
    row = validation_to_row(trial, validation, elapsed_s)
    row["planner_id"] = planner_id or "fallback_chain"
    if scenario is not None:
        row.update(
            {
                "scenario_name": scenario.name,
                "obstacle_x": scenario.obstacle_pose_xyz[0],
                "obstacle_y": scenario.obstacle_pose_xyz[1],
                "obstacle_z": scenario.obstacle_pose_xyz[2],
                "obstacle_size_x": scenario.obstacle_size_xyz[0],
                "obstacle_size_y": scenario.obstacle_size_xyz[1],
                "obstacle_size_z": scenario.obstacle_size_xyz[2],
            }
        )
    return row


SOURCE_SUFFIXES = {
    ".cfg", ".cpp", ".h", ".hpp", ".json", ".py", ".rviz", ".sdf",
    ".urdf", ".xacro", ".xml", ".yaml", ".yml",
}


def benchmark_source_manifest():
    """Fingerprint executable benchmark inputs without recording machine paths."""
    candidates = {Path(__file__).resolve()}
    if PACKAGE_ROOT.is_dir():
        candidates.update(
            path.resolve()
            for path in PACKAGE_ROOT.rglob("*")
            if path.is_file()
            and path.suffix.lower() in SOURCE_SUFFIXES
            and "__pycache__" not in path.parts
        )
    entries = {}
    combined = hashlib.sha256()
    for path in sorted(candidates, key=lambda value: value.as_posix()):
        try:
            relative = path.relative_to(REPO_ROOT).as_posix()
        except ValueError:
            relative = path.name
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        entries[relative] = digest
    for relative, digest in sorted(entries.items()):
        combined.update(relative.encode("utf-8") + b"\0" + digest.encode("ascii") + b"\n")
    return {"schema_version": 1, "sha256": combined.hexdigest(), "files": entries}


def guard_checkpoint_config(args):
    """Bind a checkpoint to its invocation before reading or overwriting rows."""
    excluded = {"csv", "markdown", "svg", "log_dir", "resume"}
    source = benchmark_source_manifest()
    config = {"schema_version": 2, "source_sha256": source["sha256"], "arguments": {
        key: value for key, value in vars(args).items() if key not in excluded
    }}
    checkpoint = Path(args.csv)
    manifest = checkpoint.with_suffix(checkpoint.suffix + ".config.json")
    source_path = checkpoint.with_suffix(checkpoint.suffix + ".source.json")
    if checkpoint.exists():
        if not args.resume:
            raise ValueError("CSV already exists; use a new output path or --resume")
        if not manifest.exists():
            raise ValueError("resume configuration is missing; keep legacy results and use a new output path")
        if not source_path.exists():
            raise ValueError("resume source manifest is missing; keep legacy results and use a new output path")
        previous = json.loads(manifest.read_text(encoding="utf-8"))
        saved_source = json.loads(source_path.read_text(encoding="utf-8"))
        if saved_source.get("sha256") != previous.get("source_sha256"):
            raise ValueError("resume source manifest does not match its configuration")
        if previous != config:
            raise ValueError("resume configuration differs; use a new output path")
    else:
        manifest.parent.mkdir(parents=True, exist_ok=True)
        source_path.write_text(
            json.dumps(source, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        manifest.write_text(json.dumps(config, indent=2, sort_keys=True, allow_nan=False) + "\n",
                            encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Benchmark Gazebo physical pick-and-place in fresh worlds."
    )
    parser.add_argument(
        "--trials",
        type=int,
        default=5,
        help="number of scenarios per planner (planner suite multiplies this by 3)",
    )
    parser.add_argument("--timeout-s", type=float, default=120.0)
    parser.add_argument("--cooldown-s", type=float, default=3.0)
    parser.add_argument(
        "--rviz",
        action="store_true",
        help="show RViz during each trial",
    )
    parser.add_argument(
        "--gazebo-gui",
        action="store_true",
        help="show Gazebo Sim during each trial",
    )
    parser.add_argument(
        "--hold-after-result-s",
        type=float,
        default=0.0,
        help="keep visualization processes alive for this many seconds after validation",
    )
    parser.add_argument(
        "--csv",
        default="artifacts/runs/gazebo_physics_benchmark/results.csv",
    )
    parser.add_argument(
        "--markdown",
        default="artifacts/runs/gazebo_physics_benchmark/report.md",
    )
    parser.add_argument(
        "--svg",
        default=None,
        help="planner comparison chart path (defaults to the Markdown path with .svg)",
    )
    parser.add_argument(
        "--log-dir",
        default="artifacts/runs/gazebo_physics_benchmark/logs",
    )
    parser.add_argument(
        "--resume",
        action="store_true",
        help="continue a compatible matrix from the existing CSV checkpoint",
    )
    parser.add_argument(
        "--transfer-diagnostics",
        action="store_true",
        help="record joint/contact telemetry in a unique per-trial log subdirectory",
    )
    scene_group = parser.add_mutually_exclusive_group()
    parser.add_argument("--cube-symmetric-placement", action="store_true",
                        help="allow cube-equivalent placement yaw (not for oriented parts)")
    parser.add_argument("--place-endpoint-precheck", action="store_true",
                        help="screen nominal loaded placement endpoints before any grasp motion")
    parser.add_argument("--loaded-path-precheck", action="store_true",
                        help="screen lift, loaded transfer, descent, released-object retreat and home")
    parser.add_argument("--grasp-height-offset", type=float, default=None,
                        help="explicit experimental palm-to-object-center distance in meters")
    parser.add_argument("--grasp-pitch-offset-deg", type=float, default=0.0,
                        help="experimental closing-axis pitch; requires --loaded-path-precheck")
    parser.add_argument("--pre-place-height-step", type=float, default=0.0,
                        help="experimental static challenge height diversity in meters (default disabled)")
    scene_group.add_argument("--randomize-target", action="store_true")
    scene_group.add_argument(
        "--challenge-obstacles",
        action="store_true",
        help="run deterministic layouts whose direct Cartesian path must be blocked",
    )
    parser.add_argument(
        "--challenge-scenario",
        choices=[scenario.name for scenario in CHALLENGE_SCENARIOS],
        help="repeat one challenge layout (requires --challenge-obstacles)",
    )
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--target-x-min", type=float, default=0.48)
    parser.add_argument("--target-x-max", type=float, default=0.56)
    parser.add_argument("--target-y-min", type=float, default=-0.06)
    parser.add_argument("--target-y-max", type=float, default=0.06)
    perception_group = parser.add_mutually_exclusive_group()
    perception_group.add_argument(
        "--camera-perception",
        action="store_true",
        help="drive each pick from the Gazebo RGB camera and ArUco estimate",
    )
    perception_group.add_argument(
        "--rgbd-perception",
        action="store_true",
        help="drive each pick from markerless RGB segmentation fused with depth points",
    )
    perception_group.add_argument(
        "--perception-suite",
        action="store_true",
        help="run synthetic, ArUco and markerless RGB-D perception trials",
    )
    planner_group = parser.add_mutually_exclusive_group()
    planner_group.add_argument("--planner-id", choices=PLANNER_SUITE)
    planner_group.add_argument("--planner-suite", action="store_true")
    args = parser.parse_args()

    if args.trials < 1:
        parser.error("--trials must be at least 1")
    if not math.isfinite(args.grasp_pitch_offset_deg) or abs(args.grasp_pitch_offset_deg) > 20:
        parser.error("--grasp-pitch-offset-deg must be finite and within +/-20 degrees")
    if args.grasp_pitch_offset_deg and not args.loaded_path_precheck:
        parser.error("--grasp-pitch-offset-deg requires --loaded-path-precheck")
    if args.grasp_height_offset is not None and (
        not math.isfinite(args.grasp_height_offset) or args.grasp_height_offset <= 0
    ):
        parser.error("--grasp-height-offset must be finite and positive")
    if args.hold_after_result_s < 0.0:
        parser.error("--hold-after-result-s must not be negative")
    if shutil.which("ros2") is None:
        parser.error("ros2 was not found; source ROS2 and the workspace first")
    if args.target_x_min > args.target_x_max:
        parser.error("--target-x-min must not exceed --target-x-max")
    if args.target_y_min > args.target_y_max:
        parser.error("--target-y-min must not exceed --target-y-max")
    if not math.isfinite(args.pre_place_height_step) or args.pre_place_height_step < 0:
        parser.error("--pre-place-height-step must be finite and nonnegative")
    if args.pre_place_height_step and not args.challenge_obstacles:
        parser.error("--pre-place-height-step requires --challenge-obstacles")
    if args.challenge_scenario and not args.challenge_obstacles:
        parser.error("--challenge-scenario requires --challenge-obstacles")

    try:
        guard_checkpoint_config(args)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    Path(args.csv).parent.mkdir(parents=True, exist_ok=True)
    Path(args.markdown).parent.mkdir(parents=True, exist_ok=True)
    if args.svg:
        Path(args.svg).parent.mkdir(parents=True, exist_ok=True)
    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    source_world = PACKAGE_ROOT / "worlds" / "panda_table.sdf"
    rng = random.Random(args.seed)
    if args.challenge_obstacles:
        challenge_pool = (
            [
                scenario
                for scenario in CHALLENGE_SCENARIOS
                if scenario.name == args.challenge_scenario
            ]
            if args.challenge_scenario
            else CHALLENGE_SCENARIOS
        )
        scenarios = [
            challenge_pool[index % len(challenge_pool)]
            for index in range(args.trials)
        ]
    else:
        scenarios = [
            ChallengeScenario(
                f"{'random' if args.randomize_target else 'baseline'}_{index + 1:03d}",
                rng.uniform(args.target_x_min, args.target_x_max)
                if args.randomize_target
                else 0.52,
                rng.uniform(args.target_y_min, args.target_y_max)
                if args.randomize_target
                else 0.0,
                (0.42, 0.30, 0.09),
                (0.12, 0.12, 0.18),
            )
            for index in range(args.trials)
        ]
    planners = PLANNER_SUITE if args.planner_suite else [args.planner_id]
    perception_modes = (
        ["synthetic", "aruco", "rgbd"]
        if args.perception_suite
        else ["rgbd"]
        if args.rgbd_perception
        else ["aruco"]
        if args.camera_perception
        else ["synthetic"]
    )
    total_runs = len(perception_modes) * len(planners) * len(scenarios)
    rows = (
        reclassify_infrastructure_failures(
            read_report_rows(args.csv),
            args.log_dir,
        )
        if args.resume and Path(args.csv).exists()
        else []
    )
    if len(rows) > total_runs:
        parser.error(
            f"resume CSV contains {len(rows)} rows, but this matrix has "
            f"only {total_runs} trials"
        )
    for completed_index, row in enumerate(rows):
        mode_block_size = len(planners) * len(scenarios)
        mode_index = completed_index // mode_block_size
        index_in_mode = completed_index % mode_block_size
        planner_index = index_in_mode // len(scenarios)
        scenario_index = completed_index % len(scenarios)
        expected_mode = perception_modes[mode_index]
        expected_planner = planners[planner_index] or "fallback_chain"
        expected_scenario = scenarios[scenario_index].name
        if (
            str(row.get("perception_mode") or "synthetic") != expected_mode
            or row.get("planner_id") != expected_planner
            or row.get("scenario_name") != expected_scenario
        ):
            parser.error(
                "resume CSV does not match the requested matrix at trial "
                f"{completed_index + 1}: expected perception={expected_mode}, "
                f"planner={expected_planner}, "
                f"scenario={expected_scenario}"
            )
    print(
        f"Benchmark matrix: {len(scenarios)} scenario(s) x "
        f"{len(planners)} planner(s) x {len(perception_modes)} perception "
        f"mode(s) = {total_runs} trial(s).",
        flush=True,
    )
    if rows:
        print(
            f"Resuming from {args.csv}: {len(rows)}/{total_runs} trial(s) "
            "already completed.",
            flush=True,
        )
    run_index = 0
    for perception_mode in perception_modes:
        for planner_id in planners:
            planner_label = planner_id or "fallback_chain"
            planner_slug = planner_label.replace("ConfigDefault", "").replace("k", "_")
            for scenario_index, scenario in enumerate(scenarios, start=1):
                run_index += 1
                if run_index <= len(rows):
                    continue
                target_x = scenario.target_x
                target_y = scenario.target_y
                world_path = None
                if args.randomize_target or args.challenge_obstacles:
                    world_path = (
                        log_dir
                        / "worlds"
                        / perception_mode
                        / f"{planner_slug}_scenario_{scenario_index:03d}.sdf"
                    )
                    write_trial_world(
                        source_world,
                        world_path,
                        target_x,
                        target_y,
                        scenario.obstacle_pose_xyz
                        if args.challenge_obstacles
                        else None,
                        scenario.obstacle_size_xyz
                        if args.challenge_obstacles
                        else None,
                    )
                print(
                    f"[{run_index}/{total_runs}] perception={perception_mode}, "
                    f"planner={planner_label}, "
                    f"scenario={scenario_index}:{scenario.name}, "
                    f"target=({target_x:.3f}, {target_y:.3f})...",
                    flush=True,
                )
                row = run_trial(
                    run_index,
                    args.timeout_s,
                    log_dir,
                    target_x,
                    target_y,
                    world_path,
                    planner_id,
                    scenario if args.challenge_obstacles else None,
                    args.rviz,
                    args.gazebo_gui,
                    args.hold_after_result_s,
                    perception_mode,
                    args.transfer_diagnostics,
                    args.pre_place_height_step,
                    args.cube_symmetric_placement,
                    args.place_endpoint_precheck,
                    args.loaded_path_precheck,
                    args.grasp_height_offset,
                    args.grasp_pitch_offset_deg,
                )
                row["scenario_index"] = scenario_index
                row["scenario_name"] = scenario.name
                row["perception_mode"] = perception_mode
                rows.append(row)
                print(
                    f"[{run_index}/{total_runs}] {row['final_state']}: "
                    f"{row['reason']} ({row['elapsed_s']:.1f}s)",
                    flush=True,
                )
                partial_summary = write_reports(rows, args.csv, args.markdown)
                svg_path = args.svg or str(
                    Path(args.markdown).with_suffix(".svg")
                )
                write_svg_report(partial_summary, svg_path)
                if run_index < total_runs:
                    time.sleep(args.cooldown_s)

    summary = write_reports(rows, args.csv, args.markdown)
    svg_path = args.svg or str(Path(args.markdown).with_suffix(".svg"))
    write_svg_report(summary, svg_path)
    print(
        f"Completed {summary['total']} trials: {summary['successes']} physical "
        f"successes ({summary['success_rate']:.1%})."
    )
    print(f"Reports: {args.csv}, {args.markdown}, {svg_path}; logs: {log_dir}")
    return 0 if summary["failures"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
