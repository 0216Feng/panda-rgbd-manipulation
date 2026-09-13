#!/usr/bin/env python3
"""Verify installed ROS topic wiring with synthetic poses, never physical motion."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


REQUIRED_STAGES = (
    "DETECT_OBJECT", "PLAN_TO_PRE_GRASP", "APPROACH", "CLOSE_GRIPPER",
    "LIFT", "PLAN_TO_PLACE", "OPEN_GRIPPER", "RETURN_HOME", "SUCCESS",
)


def valid_task_report(report: object) -> bool:
    return (
        isinstance(report, dict)
        and report.get("final_state") == "SUCCESS"
        and report.get("history") == list(REQUIRED_STAGES)
        and report.get("failure_reason") is None
        and not report.get("failures")
    )


def stop_launch(process: subprocess.Popen) -> None:
    for sig, grace_s in ((signal.SIGINT, 10), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            process.wait(timeout=grace_s)
            return
        try:
            process.wait(timeout=grace_s)
            return
        except subprocess.TimeoutExpired:
            continue
    raise RuntimeError("ROS smoke launch did not stop")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout-s", type=float, default=45.0)
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/runs/ros_graph_smoke"))
    args = parser.parse_args()
    if not math.isfinite(args.timeout_s) or args.timeout_s <= 0:
        parser.error("--timeout-s must be finite and positive")

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String

    args.output_dir.mkdir(parents=True, exist_ok=True)
    attempt = Path(tempfile.mkdtemp(prefix="attempt_", dir=args.output_dir))
    domain = 200 + os.getpid() % 30
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = rclpy.create_node("ci_graph_observer", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    observed = {"pose": False, "candidates": False, "task": None}

    def on_pose(message):
        point = message.pose.position
        observed["pose"] = bool(message.header.frame_id) and all(
            math.isfinite(value) for value in (point.x, point.y, point.z)
        )

    def on_candidates(message):
        try:
            data = json.loads(message.data)
            observed["candidates"] = (
                isinstance(data, dict) and data.get("selected_candidate_id") is not None
            )
        except (ValueError, TypeError):
            observed["candidates"] = False

    def on_task(message):
        try:
            observed["task"] = json.loads(message.data)
        except (ValueError, TypeError):
            observed["task"] = None

    subscriptions = [
        node.create_subscription(PoseStamped, "/detected_object_pose", on_pose, 10),
        node.create_subscription(String, "/grasp_candidates", on_candidates, 10),
        node.create_subscription(String, "/task_state", on_task, 10),
    ]
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = str(domain)
    environment["ROS_LOG_DIR"] = str(attempt.resolve() / "ros_logs")
    process = None
    success = False
    reason = "timed out waiting for pose, candidates and a complete task history"
    started = time.monotonic()
    try:
        with (attempt / "launch.log").open("w", encoding="utf-8") as log:
            process = subprocess.Popen(
                ["ros2", "launch", "panda_manipulation", "demo.launch.py",
                 "use_synthetic_pose:=true", "dry_run:=true"],
                stdout=log, stderr=subprocess.STDOUT, env=environment,
                start_new_session=True,
            )
            while time.monotonic() - started < args.timeout_s:
                executor.spin_once(timeout_sec=0.1)
                if process.poll() is not None:
                    reason = f"launch exited before graph validation: {process.returncode}"
                    break
                if all((observed["pose"], observed["candidates"], valid_task_report(observed["task"]))):
                    success = True
                    reason = "installed ROS topic chain verified with synthetic pose and dry_run"
                    break
    except Exception as exc:
        success = False
        reason = f"{type(exc).__name__}: {exc}"
    finally:
        try:
            if process is not None:
                stop_launch(process)
        finally:
            executor.shutdown()
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown(context=context)
    result = {
        "final_state": "SUCCESS" if success else "FAILED",
        "scope": "synthetic_pose_ros_wiring_only",
        "physical_execution_verified": False,
        "ros_domain_id": domain,
        "elapsed_s": time.monotonic() - started,
        "observed": observed,
        "reason": reason,
    }
    (attempt / "result.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result), flush=True)
    print(f"Artifacts: {attempt}", flush=True)
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
