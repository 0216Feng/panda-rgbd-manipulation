"""Run one isolated RGB-D online-replanning physical pick trial."""

from __future__ import annotations

import argparse
import json
import os
import selectors
import signal
import subprocess
import time
from pathlib import Path
from statistics import fmean
from typing import Any, Dict, Optional


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


def stop_gazebo_world(world_signature: str) -> None:
    """Stop detached Gazebo servers that still own this project's world."""
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
        if world_signature in command and ("gz sim" in command or "ruby " in command):
            try:
                pids.append(int(pid_text))
            except ValueError:
                continue
    for sig, wait_s in ((signal.SIGTERM, 2.0), (signal.SIGKILL, 0.0)):
        for pid in pids:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        if wait_s and pids:
            time.sleep(wait_s)


def json_payload(line: str) -> Optional[Dict[str, Any]]:
    start = line.find("{")
    if start < 0:
        return None
    try:
        return json.loads(line[start:])
    except json.JSONDecodeError:
        return None


def mean_event_metric(events: list[Dict[str, Any]], key: str) -> Optional[float]:
    values = [float(event[key]) for event in events if event.get(key) is not None]
    return fmean(values) if values else None


def result_reason(
    success: bool,
    pipeline: Optional[Dict[str, Any]],
    physical: Optional[Dict[str, Any]],
    perception: Optional[Dict[str, Any]],
    safety_events: list[Dict[str, Any]],
) -> str:
    if success:
        return "online cancellation, replanning and physical pick verified"
    if pipeline is None:
        return "pick pipeline result was not received"
    if pipeline.get("final_state") != "SUCCESS":
        return str(
            pipeline.get("failure_reason")
            or pipeline.get("reason")
            or "pick pipeline failed"
        )
    if physical is None:
        return "Gazebo physical validation result was not received"
    if physical.get("final_state") != "SUCCESS":
        return str(
            physical.get("failure_reason")
            or physical.get("reason")
            or "Gazebo physical validation failed"
        )
    if perception is None:
        return "dynamic obstacle perception validation result was not received"
    if perception.get("final_state") != "SUCCESS":
        return str(
            perception.get("failure_reason")
            or perception.get("reason")
            or "dynamic obstacle perception validation failed"
        )
    if not safety_events:
        return "trajectory safety monitor did not publish a hazard event"
    return "required dynamic-replanning evidence was not observed"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout-s", type=float, default=180.0)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--rviz", action="store_true")
    parser.add_argument("--planner-id", default="RRTConnectkConfigDefault")
    parser.add_argument("--scenario-name", default="nominal_crossing")
    parser.add_argument("--obstacle-start-y", type=float, default=-0.55)
    parser.add_argument("--obstacle-end-y", type=float, default=0.55)
    parser.add_argument("--obstacle-speed-mps", type=float, default=0.14)
    parser.add_argument("--obstacle-trigger-delay-s", type=float, default=0.2)
    parser.add_argument("--perception-required-samples", type=int, default=12)
    parser.add_argument("--perception-minimum-span-m", type=float, default=0.12)
    parser.add_argument("--perception-skip-initial-samples", type=int, default=5)
    parser.add_argument("--enable-motion-prediction", action="store_true")
    parser.add_argument("--prediction-horizon-s", type=float, default=0.65)
    parser.add_argument("--log", default="dynamic_replanning_smoke.log")
    args = parser.parse_args()
    world_signature = "panda_manipulation/worlds/panda_table.sdf"
    stop_gazebo_world(world_signature)
    command = [
        "ros2",
        "launch",
        "panda_manipulation",
        "dynamic_replanning_demo.launch.py",
        f"gui:={'true' if args.gui else 'false'}",
        f"rviz:={'true' if args.rviz else 'false'}",
        f"planner_id:={args.planner_id}",
        f"obstacle_start_y:={args.obstacle_start_y}",
        f"obstacle_end_y:={args.obstacle_end_y}",
        f"obstacle_speed_mps:={args.obstacle_speed_mps}",
        f"obstacle_trigger_delay_s:={args.obstacle_trigger_delay_s}",
        f"perception_required_samples:={args.perception_required_samples}",
        f"perception_minimum_span_m:={args.perception_minimum_span_m}",
        f"perception_skip_initial_samples:={args.perception_skip_initial_samples}",
        f"enable_motion_prediction:={'true' if args.enable_motion_prediction else 'false'}",
        f"prediction_horizon_s:={args.prediction_horizon_s}",
    ]
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        start_new_session=True,
    )
    pipeline = None
    physical = None
    perception = None
    safety_events = []
    selector = selectors.DefaultSelector()
    assert process.stdout is not None
    selector.register(process.stdout, selectors.EVENT_READ)
    started = time.monotonic()
    log_path = Path(args.log)
    try:
        with log_path.open("w", encoding="utf-8") as log_file:
            while time.monotonic() - started < args.timeout_s:
                events = selector.select(timeout=0.5)
                for key, _ in events:
                    line = key.fileobj.readline()
                    if not line:
                        continue
                    log_file.write(line)
                    log_file.flush()
                    payload = json_payload(line)
                    if payload is None:
                        continue
                    if "trajectory_safety_monitor" in line and payload.get("event"):
                        safety_events.append(payload)
                    elif "gazebo_pick_pipeline" in line and "dynamic_replan_count" in payload:
                        pipeline = payload
                    elif "gazebo_pick_validator" in line and "lift_delta_m" in payload:
                        physical = payload
                    elif (
                        "dynamic_obstacle_perception_validator" in line
                        and "mean_error_m" in payload
                    ):
                        perception = payload
                if physical is not None and perception is not None:
                    break
                if (
                    pipeline is not None
                    and pipeline.get("final_state") == "FAILED"
                    and physical is not None
                ):
                    break
                if process.poll() is not None:
                    break
    finally:
        selector.close()
        stop_process_group(process)
        stop_gazebo_world(world_signature)
    elapsed_s = time.monotonic() - started
    success = (
        pipeline is not None
        and pipeline.get("final_state") == "SUCCESS"
        and int(pipeline.get("dynamic_replan_count", 0)) >= 1
        and physical is not None
        and physical.get("final_state") == "SUCCESS"
        and perception is not None
        and perception.get("final_state") == "SUCCESS"
        and bool(safety_events)
    )
    dynamic_events = list((pipeline or {}).get("dynamic_replan_events", []))
    output = {
        "final_state": "SUCCESS" if success else "FAILED",
        "planner_id": args.planner_id,
        "scenario_name": args.scenario_name,
        "obstacle_start_y": args.obstacle_start_y,
        "obstacle_end_y": args.obstacle_end_y,
        "obstacle_speed_mps": args.obstacle_speed_mps,
        "obstacle_trigger_delay_s": args.obstacle_trigger_delay_s,
        "motion_prediction_enabled": args.enable_motion_prediction,
        "prediction_horizon_s": args.prediction_horizon_s,
        "perception_required_samples": args.perception_required_samples,
        "perception_skip_initial_samples": args.perception_skip_initial_samples,
        "elapsed_s": elapsed_s,
        "dynamic_replan_count": (pipeline or {}).get("dynamic_replan_count", 0),
        "safety_events": len(safety_events),
        "pipeline_state": (pipeline or {}).get("final_state"),
        "physical_state": (physical or {}).get("final_state"),
        "perception_state": (perception or {}).get("final_state"),
        "perception_mean_error_m": (perception or {}).get("mean_error_m"),
        "perception_max_error_m": (perception or {}).get("max_error_m"),
        "safety_check_latency_s": mean_event_metric(safety_events, "check_latency_s"),
        "collision_lead_time_s": mean_event_metric(
            safety_events,
            "collision_lead_time_s",
        ),
        "cancel_ack_latency_s": mean_event_metric(dynamic_events, "cancel_ack_latency_s"),
        "replan_trigger_latency_s": mean_event_metric(
            dynamic_events,
            "replan_trigger_latency_s",
        ),
        "ompl_planning_time_s": (pipeline or {}).get("ompl_planning_time_s"),
        "ompl_joint_path_length_rad": (pipeline or {}).get(
            "ompl_joint_path_length_rad"
        ),
        "lift_delta_m": (physical or {}).get("lift_delta_m"),
        "place_error_m": (physical or {}).get("place_error_m"),
        "reason": result_reason(
            success,
            pipeline,
            physical,
            perception,
            safety_events,
        ),
    }
    print(json.dumps(output, indent=2))
    print(f"Log: {log_path}")
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
