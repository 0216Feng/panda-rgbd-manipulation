"""Run one isolated RGB-D dynamic-obstacle perception validation."""

from __future__ import annotations

import argparse
import json
import os
import selectors
import signal
import subprocess
import time
from pathlib import Path
from typing import Any, Dict, Optional


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


def parse_result(line: str) -> Optional[Dict[str, Any]]:
    if "dynamic_obstacle_perception_validator" not in line or "mean_error_m" not in line:
        return None
    start = line.find("{")
    if start < 0:
        return None
    try:
        return json.loads(line[start:])
    except json.JSONDecodeError:
        return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout-s", type=float, default=75.0)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--moving", action="store_true")
    parser.add_argument("--log", default="dynamic_obstacle_perception_smoke.log")
    args = parser.parse_args()
    command = [
        "ros2",
        "launch",
        "panda_manipulation",
        "dynamic_obstacle_perception.launch.py",
        f"gui:={'true' if args.gui else 'false'}",
        f"move_obstacle:={'true' if args.moving else 'false'}",
        f"required_samples:={40 if args.moving else 20}",
        f"minimum_tracking_span_m:={0.30 if args.moving else 0.0}",
    ]
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        start_new_session=True,
    )
    result = None
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
                    parsed = parse_result(line)
                    if parsed is not None:
                        result = parsed
                        break
                if result is not None or process.poll() is not None:
                    break
    finally:
        selector.close()
        stop_process_group(process)
    if result is None:
        print(f"FAILED: no validation result within {args.timeout_s:.0f}s; log={log_path}")
        return 1
    print(json.dumps(result, indent=2))
    print(f"Log: {log_path}")
    return 0 if result.get("final_state") == "SUCCESS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
