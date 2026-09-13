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
    from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String
    from trajectory_msgs.msg import JointTrajectoryPoint

    args.output_dir.mkdir(parents=True, exist_ok=True)
    attempt = Path(tempfile.mkdtemp(prefix="attempt_", dir=args.output_dir))
    domain = 200 + os.getpid() % 30
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = rclpy.create_node("ci_graph_observer", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    observed = {
        "pose": False,
        "candidates": False,
        "task": None,
        "cpp_trajectory_metrics": None,
    }

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

    def on_cpp_trajectory_metrics(message):
        try:
            data = json.loads(message.data)
            aggregate = data.get("aggregate", {})
            metrics = data.get("metrics", [])
            expected_length = math.sqrt(0.1 ** 2 + 0.2 ** 2)
            valid = (
                data.get("schema_version") == 1
                and data.get("source_topic") == "/display_planned_path"
                and data.get("trajectory_count") == 1
                and len(metrics) == 1
                and metrics[0].get("point_count") == 2
                and metrics[0].get("invalid_position_segment_count") == 0
                and metrics[0].get("nonpositive_duration_segment_count") == 0
                and math.isclose(
                    float(aggregate.get("joint_path_length_rad", -1.0)),
                    expected_length,
                    abs_tol=1e-6,
                )
                and math.isclose(
                    float(aggregate.get("max_joint_step_rad", -1.0)),
                    0.2,
                    abs_tol=1e-6,
                )
                and metrics[0].get("min_normalized_joint_limit_margin") is not None
            )
            observed["cpp_trajectory_metrics"] = data if valid else False
        except (ValueError, TypeError):
            observed["cpp_trajectory_metrics"] = False

    subscriptions = [
        node.create_subscription(PoseStamped, "/detected_object_pose", on_pose, 10),
        node.create_subscription(String, "/grasp_candidates", on_candidates, 10),
        node.create_subscription(String, "/task_state", on_task, 10),
        node.create_subscription(
            String,
            "/trajectory_quality_metrics",
            on_cpp_trajectory_metrics,
            10,
        ),
    ]
    trajectory_publisher = node.create_publisher(
        DisplayTrajectory,
        "/display_planned_path",
        10,
    )

    synthetic_trajectory = DisplayTrajectory()
    robot_trajectory = RobotTrajectory()
    robot_trajectory.joint_trajectory.joint_names = [
        "panda_joint1",
        "panda_joint2",
    ]
    start_point = JointTrajectoryPoint()
    start_point.positions = [0.0, 0.0]
    start_point.time_from_start.sec = 0
    end_point = JointTrajectoryPoint()
    end_point.positions = [0.1, -0.2]
    end_point.time_from_start.sec = 1
    robot_trajectory.joint_trajectory.points = [start_point, end_point]
    synthetic_trajectory.trajectory = [robot_trajectory]
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = str(domain)
    environment["ROS_LOG_DIR"] = str(attempt.resolve() / "ros_logs")
    process = None
    success = False
    reason = (
        "timed out waiting for pose, candidates, task history and C++ trajectory metrics"
    )
    started = time.monotonic()
    last_trajectory_publish = 0.0
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
                now = time.monotonic()
                if (
                    observed["cpp_trajectory_metrics"] is None
                    and now - last_trajectory_publish >= 0.5
                ):
                    trajectory_publisher.publish(synthetic_trajectory)
                    last_trajectory_publish = now
                if process.poll() is not None:
                    reason = f"launch exited before graph validation: {process.returncode}"
                    break
                if all(
                    (
                        observed["pose"],
                        observed["candidates"],
                        valid_task_report(observed["task"]),
                        isinstance(observed["cpp_trajectory_metrics"], dict),
                    )
                ):
                    success = True
                    reason = (
                        "installed ROS topic chain and C++17 trajectory metrics verified "
                        "with synthetic pose and dry_run"
                    )
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
            node.destroy_publisher(trajectory_publisher)
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
