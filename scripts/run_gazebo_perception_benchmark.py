#!/usr/bin/env python3
"""Benchmark ArUco pose accuracy across fresh randomized Gazebo worlds."""

from __future__ import annotations

import argparse
import csv
import json
import os
import random
import selectors
import shutil
import signal
import statistics
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPO_ROOT / "src" / "panda_manipulation"
sys.path.insert(0, str(PACKAGE_ROOT))

from panda_manipulation.physics_benchmark import write_trial_world  # noqa: E402


CSV_FIELDS = [
    "trial",
    "target_x",
    "target_y",
    "marker_size_m",
    "final_state",
    "samples",
    "mean_error_m",
    "max_error_m",
    "std_error_m",
    "mean_x_error_m",
    "mean_y_error_m",
    "mean_z_error_m",
    "detection_rate",
    "first_detection_latency_s",
    "elapsed_s",
    "reason",
]


def stop_process_group(process: subprocess.Popen[str]) -> None:
    if process.poll() is not None:
        return
    for sig, wait_s in ((signal.SIGINT, 10), (signal.SIGTERM, 3)):
        try:
            os.killpg(process.pid, sig)
            process.wait(timeout=wait_s)
            return
        except (ProcessLookupError, subprocess.TimeoutExpired):
            continue
    os.killpg(process.pid, signal.SIGKILL)
    process.wait(timeout=3)


def stop_gazebo_world(world_path: Path) -> None:
    world_signature = str(world_path.resolve())
    listing = subprocess.run(
        ["ps", "-eo", "pid=,args="],
        check=False,
        capture_output=True,
        text=True,
    ).stdout
    pids = []
    for line in listing.splitlines():
        pid_text, _, command = line.strip().partition(" ")
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


def parse_validation(line: str) -> dict[str, Any] | None:
    if "aruco_perception_validator" not in line or "mean_error_m" not in line:
        return None
    json_start = line.find("{")
    if json_start < 0:
        return None
    try:
        payload = json.loads(line[json_start:])
    except json.JSONDecodeError:
        return None
    return payload if "samples" in payload else None


def run_trial(
    trial: int,
    world_path: Path,
    marker_size_m: float,
    object_center_offset_z_m: float,
    required_samples: int,
    timeout_s: float,
    log_dir: Path,
) -> dict[str, Any]:
    command = [
        "ros2",
        "launch",
        "panda_manipulation",
        "gazebo_perception.launch.py",
        "gui:=false",
        f"world:={world_path.resolve()}",
        f"marker_size_m:={marker_size_m:.8f}",
        f"object_center_offset_z_m:={object_center_offset_z_m:.8f}",
        f"required_samples:={required_samples}",
    ]
    started = time.monotonic()
    stop_gazebo_world(world_path)
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        start_new_session=True,
    )
    selector = selectors.DefaultSelector()
    assert process.stdout is not None
    selector.register(process.stdout, selectors.EVENT_READ)
    payload = None
    log_path = log_dir / f"trial_{trial:03d}.log"
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
                    payload = parse_validation(line)
                    if payload is not None:
                        break
                    if (
                        "aruco_pose_estimator" in line
                        and "process has died" in line
                    ):
                        payload = {
                            "final_state": "FAILED",
                            "reason": "ArUco pose estimator process exited",
                        }
                        break
                if payload is not None or process.poll() is not None:
                    break
    finally:
        selector.close()
        stop_process_group(process)
        stop_gazebo_world(world_path)
    elapsed_s = time.monotonic() - started
    if payload is None:
        payload = {
            "final_state": "FAILED",
            "reason": (
                f"perception trial timed out after {timeout_s:.0f}s"
                if elapsed_s >= timeout_s
                else f"launch exited before validation (code={process.returncode})"
            ),
        }
    return {
        "trial": trial,
        "marker_size_m": marker_size_m,
        "final_state": payload.get("final_state", "FAILED"),
        "samples": payload.get("samples"),
        "mean_error_m": payload.get("mean_error_m"),
        "max_error_m": payload.get("max_error_m"),
        "std_error_m": payload.get("std_error_m"),
        "mean_x_error_m": payload.get("mean_x_error_m"),
        "mean_y_error_m": payload.get("mean_y_error_m"),
        "mean_z_error_m": payload.get("mean_z_error_m"),
        "detection_rate": payload.get("detection_rate"),
        "first_detection_latency_s": payload.get(
            "first_detection_latency_s"
        ),
        "elapsed_s": elapsed_s,
        "reason": payload.get("reason", "unknown failure"),
    }


def mean(rows: list[dict[str, Any]], key: str) -> float | None:
    values = [
        float(row[key])
        for row in rows
        if row.get(key) is not None
    ]
    return statistics.fmean(values) if values else None


def render_markdown(rows: list[dict[str, Any]]) -> str:
    successful = [row for row in rows if row["final_state"] == "SUCCESS"]
    measured = [row for row in rows if row.get("mean_error_m") is not None]

    def metric(key: str, scale: float = 1.0, suffix: str = "") -> str:
        value = mean(measured, key)
        return "N/A" if value is None else f"{value * scale:.3f}{suffix}"

    lines = [
        "# Gazebo ArUco Perception Benchmark",
        "",
        f"- Trials: {len(rows)}",
        f"- Successful validations: {len(successful)}",
        f"- Validation success rate: "
        f"{len(successful) / len(rows) if rows else 0.0:.1%}",
        f"- Mean 3D pose error: {metric('mean_error_m', 1000.0, ' mm')}",
        f"- Mean X bias: {metric('mean_x_error_m', 1000.0, ' mm')}",
        f"- Mean Y bias: {metric('mean_y_error_m', 1000.0, ' mm')}",
        f"- Mean Z bias: {metric('mean_z_error_m', 1000.0, ' mm')}",
        f"- Mean detection rate: {metric('detection_rate')}",
        f"- Mean first detection latency: "
        f"{metric('first_detection_latency_s', 1.0, ' s')}",
        "",
        "| Trial | Target X | Target Y | State | 3D error | X bias | Y bias | "
        "Z bias | Detection |",
        "| ---: | ---: | ---: | --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in rows:
        def cell(key: str, scale: float = 1.0) -> str:
            value = row.get(key)
            return "N/A" if value is None else f"{float(value) * scale:.3f}"

        lines.append(
            f"| {row['trial']} | {float(row['target_x']):.3f} | "
            f"{float(row['target_y']):.3f} | {row['final_state']} | "
            f"{cell('mean_error_m', 1000.0)} mm | "
            f"{cell('mean_x_error_m', 1000.0)} mm | "
            f"{cell('mean_y_error_m', 1000.0)} mm | "
            f"{cell('mean_z_error_m', 1000.0)} mm | "
            f"{cell('detection_rate')} |"
        )
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Benchmark Gazebo ArUco accuracy without moving the robot."
    )
    parser.add_argument("--trials", type=int, default=5)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--target-x-min", type=float, default=0.48)
    parser.add_argument("--target-x-max", type=float, default=0.56)
    parser.add_argument("--target-y-min", type=float, default=-0.06)
    parser.add_argument("--target-y-max", type=float, default=0.06)
    parser.add_argument("--marker-size-m", type=float, default=0.045)
    parser.add_argument("--object-center-offset-z-m", type=float, default=-0.04)
    parser.add_argument("--required-samples", type=int, default=10)
    parser.add_argument("--timeout-s", type=float, default=45.0)
    parser.add_argument("--cooldown-s", type=float, default=2.0)
    parser.add_argument("--csv", default="gazebo_perception_benchmark.csv")
    parser.add_argument("--markdown", default="gazebo_perception_benchmark.md")
    parser.add_argument("--log-dir", default="gazebo_perception_benchmark_logs")
    args = parser.parse_args()
    if args.trials < 1:
        parser.error("--trials must be at least 1")
    if args.marker_size_m <= 0.0:
        parser.error("--marker-size-m must be positive")
    if not shutil.which("ros2"):
        parser.error("ros2 was not found; source ROS2 and the workspace first")

    rng = random.Random(args.seed)
    log_dir = Path(args.log_dir)
    world_dir = log_dir / "worlds"
    log_dir.mkdir(parents=True, exist_ok=True)
    source_world = PACKAGE_ROOT / "worlds" / "panda_table.sdf"
    rows = []
    for trial in range(1, args.trials + 1):
        target_x = rng.uniform(args.target_x_min, args.target_x_max)
        target_y = rng.uniform(args.target_y_min, args.target_y_max)
        world_path = world_dir / f"trial_{trial:03d}.sdf"
        write_trial_world(
            source_world,
            world_path,
            target_x,
            target_y,
        )
        print(
            f"[{trial}/{args.trials}] target=({target_x:.3f}, "
            f"{target_y:.3f}), marker={args.marker_size_m:.5f}m...",
            flush=True,
        )
        row = run_trial(
            trial,
            world_path,
            args.marker_size_m,
            args.object_center_offset_z_m,
            args.required_samples,
            args.timeout_s,
            log_dir,
        )
        row["target_x"] = target_x
        row["target_y"] = target_y
        rows.append(row)
        print(
            f"[{trial}/{args.trials}] {row['final_state']}: {row['reason']} "
            f"({row['elapsed_s']:.1f}s)",
            flush=True,
        )
        if trial < args.trials:
            time.sleep(args.cooldown_s)

    with Path(args.csv).open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    Path(args.markdown).write_text(render_markdown(rows), encoding="utf-8")
    successes = sum(row["final_state"] == "SUCCESS" for row in rows)
    print(
        f"Completed {len(rows)} trials: {successes} successful perception "
        f"validations ({successes / len(rows):.1%})."
    )
    print(f"Reports: {args.csv}, {args.markdown}; logs: {args.log_dir}")
    return 0 if successes == len(rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
