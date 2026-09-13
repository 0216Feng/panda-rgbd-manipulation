#!/usr/bin/env python3
"""Generate deterministic GitHub portfolio charts from public evidence files."""

from __future__ import annotations

import argparse
import csv
import html
import json
import math
import statistics
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BASELINES = ROOT / "artifacts" / "baselines"
ASSETS = ROOT / "docs" / "assets"


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


def floats(rows: list[dict[str, str]], key: str) -> list[float]:
    values = []
    for row in rows:
        value = row.get(key, "").strip()
        if value:
            values.append(float(value))
    return values


def successes(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    return [row for row in rows if row.get("final_state") == "SUCCESS"]


def wilson_interval(success_count: int, total: int, z: float = 1.95996398454) -> tuple[float, float]:
    if total <= 0:
        return 0.0, 0.0
    p = success_count / total
    denominator = 1.0 + z * z / total
    center = (p + z * z / (2.0 * total)) / denominator
    margin = z * math.sqrt((p * (1.0 - p) + z * z / (4.0 * total)) / total) / denominator
    return max(0.0, center - margin), min(1.0, center + margin)


def mean_or_zero(values: list[float]) -> float:
    return statistics.fmean(values) if values else 0.0


def load_metrics() -> dict:
    release_path = BASELINES / "v1.0.0"
    release_summary = json.loads(
        (release_path / "release_summary.json").read_text(encoding="utf-8")
    )
    fixed = read_csv(release_path / "fixed_rgbd" / "results.csv")
    random = read_csv(release_path / "random_rgbd" / "results.csv")
    static = read_csv(release_path / "static_obstacles" / "results.csv")
    planner_obstacle = read_csv(BASELINES / "gazebo_obstacle_90_trials.csv")
    paired_path = BASELINES / "v1_candidate_20260910" / "payload_transfer_paired5"
    paired = read_csv(paired_path / "results.csv")
    paired_summary = json.loads((paired_path / "summary.json").read_text(encoding="utf-8"))

    cohorts = []
    for label, rows in (
        ("Fixed RGB-D", fixed),
        ("Random RGB-D", random),
        ("Static obstacles", static),
    ):
        success_count = len(successes(rows))
        low, high = wilson_interval(success_count, len(rows))
        cohorts.append(
            {
                "label": label,
                "successes": success_count,
                "trials": len(rows),
                "rate": success_count / len(rows),
                "ci_low": low,
                "ci_high": high,
                "scope": "v1.0",
            }
        )
    safe_stop = next(
        cohort
        for cohort in release_summary["cohorts"]
        if cohort["name"] == "gazebo_wrench_safe_stop"
    )
    cohorts.append(
        {
            "label": "Wrench safe stop",
            "successes": safe_stop["safe_stops"],
            "trials": safe_stop["trials"],
            "rate": safe_stop["safe_stop_rate"],
            "ci_low": safe_stop["safe_stop_rate_ci"][0],
            "ci_high": safe_stop["safe_stop_rate_ci"][1],
            "scope": "v1.0",
        }
    )

    planners = []
    for planner_id in sorted({row["planner_id"] for row in planner_obstacle}):
        rows = [row for row in planner_obstacle if row["planner_id"] == planner_id]
        ok = successes(rows)
        short_name = planner_id.replace("kConfigDefault", "")
        planners.append(
            {
                "name": short_name,
                "successes": len(ok),
                "trials": len(rows),
                "rate": len(ok) / len(rows),
                "planning_time_s": mean_or_zero(floats(rows, "ompl_planning_time_s")),
                "path_length_rad": mean_or_zero(floats(rows, "ompl_joint_path_length_rad")),
            }
        )

    quality = []
    for label, rows in (
        ("Fixed RGB-D", fixed),
        ("Random RGB-D", random),
        ("Static obstacles", static),
    ):
        ok = successes(rows)
        quality.append(
            {
                "label": label,
                "placement_mm": mean_or_zero(floats(ok, "place_error_m")) * 1000.0,
                "perception_mm": mean_or_zero(floats(ok, "perception_mean_error_m")) * 1000.0,
            }
        )

    loaded = [row for row in paired if row.get("mode") == "loaded"]
    unloaded = [row for row in paired if row.get("mode") == "unloaded"]
    return {
        "release": {
            "profile": release_summary["profile"],
            "passed": release_summary["passed"],
            "source_commit": (release_path / "commit.txt").read_text(encoding="utf-8").strip(),
            "safe_stop_latency_ms": safe_stop["max_stop_latency_s"] * 1000.0,
            "safe_stop_limit_ms": safe_stop["stop_latency_limit_s"] * 1000.0,
        },
        "cohorts": cohorts,
        "planners": planners,
        "quality": quality,
        "payload": {
            "matched_pairs": paired_summary["matched_state_pair_count"],
            "pair_count": paired_summary["complete_pair_count"],
            "max_state_delta_rad": paired_summary["max_pre_descent_joint_delta_rad"],
            "mean_state_delta_rad": paired_summary["mean_pre_descent_joint_delta_rad"],
            "loaded_mean_drift_m": paired_summary["modes"]["loaded"]["mean_max_payload_relative_drift_m"],
            "loaded_mean_max_error_rad": mean_or_zero(floats(loaded, "max_transfer_arm_position_error_rad")),
            "unloaded_mean_max_error_rad": mean_or_zero(floats(unloaded, "max_transfer_arm_position_error_rad")),
            "loaded_p95_error_rad": paired_summary["modes"]["loaded"]["p95_max_transfer_arm_position_error_rad"],
        },
    }


def text(x: float, y: float, value: str, size: int = 22, color: str = "#263238", weight: int = 400, anchor: str = "start") -> str:
    return (
        f'<text x="{x:.1f}" y="{y:.1f}" font-family="Arial, Helvetica, sans-serif" '
        f'font-size="{size}" font-weight="{weight}" fill="{color}" text-anchor="{anchor}">'
        f'{html.escape(value)}</text>'
    )


def rect(x: float, y: float, width: float, height: float, fill: str, radius: float = 4, stroke: str = "none") -> str:
    return (
        f'<rect x="{x:.1f}" y="{y:.1f}" width="{width:.1f}" height="{height:.1f}" '
        f'rx="{radius:.1f}" fill="{fill}" stroke="{stroke}"/>'
    )


def line(x1: float, y1: float, x2: float, y2: float, stroke: str, width: float = 2) -> str:
    return f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="{stroke}" stroke-width="{width:.1f}"/>'


def render_overview(metrics: dict) -> str:
    width, height = 1600, 980
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img" aria-labelledby="title desc">',
        '<title id="title">Panda manipulation validation overview</title>',
        '<desc id="desc">Final v1.0 success rates and quality metrics, plus supplemental planner and payload studies from public benchmark files.</desc>',
        rect(0, 0, width, height, "#f7f8fa", 0),
        text(70, 70, "Panda Manipulation Validation", 38, "#17212b", 700),
        text(70, 108, "Final v1.0 physical acceptance plus clearly separated supplemental studies", 20, "#52616b"),
    ]

    panel_fill = "#ffffff"
    parts += [
        rect(55, 145, 730, 365, panel_fill, 8, "#dfe4e8"),
        text(85, 190, "End-to-end success rate", 27, "#17212b", 700),
        text(85, 222, "Wilson 95% interval shown by whiskers", 17, "#60717c"),
    ]
    bar_x, bar_width = 300, 420
    colors = ["#167d6d", "#2878b5", "#d08a24", "#6f52a3"]
    for index, cohort in enumerate(metrics["cohorts"]):
        y = 270 + index * 58
        parts.append(text(85, y + 21, cohort["label"], 19, "#263238", 600))
        parts.append(rect(bar_x, y, bar_width, 28, "#e8edf0", 3))
        parts.append(rect(bar_x, y, bar_width * cohort["rate"], 28, colors[index], 3))
        low_x = bar_x + bar_width * cohort["ci_low"]
        high_x = bar_x + bar_width * cohort["ci_high"]
        parts += [
            line(low_x, y + 14, high_x, y + 14, "#17212b", 2),
            line(low_x, y + 7, low_x, y + 21, "#17212b", 2),
            line(high_x, y + 7, high_x, y + 21, "#17212b", 2),
            text(740, y + 21, f'{cohort["successes"]}/{cohort["trials"]}', 19, colors[index], 700, "end"),
        ]

    parts += [
        rect(815, 145, 730, 365, panel_fill, 8, "#dfe4e8"),
        text(845, 190, "Supplemental planner comparison", 27, "#17212b", 700),
        text(845, 222, "Prior 90-trial obstacle study; 30 trials per planner", 17, "#60717c"),
    ]
    planner_colors = {"PRM": "#167d6d", "RRTConnect": "#2878b5", "RRTstar": "#d08a24"}
    for index, planner in enumerate(metrics["planners"]):
        y = 270 + index * 76
        color = planner_colors.get(planner["name"], "#6f52a3")
        parts += [
            text(845, y + 23, planner["name"], 20, "#263238", 650),
            rect(1015, y, 330, 30, "#e8edf0", 3),
            rect(1015, y, 330 * planner["rate"], 30, color, 3),
            text(1365, y + 23, f'{planner["successes"]}/{planner["trials"]}', 19, color, 700),
            text(1015, y + 56, f'OMPL {planner["planning_time_s"]:.3f} s   path {planner["path_length_rad"]:.2f} rad', 17, "#60717c"),
        ]

    parts += [
        rect(55, 540, 730, 330, panel_fill, 8, "#dfe4e8"),
        text(85, 585, "Outcome quality", 27, "#17212b", 700),
        text(85, 617, "Successful trials only; lower is better", 17, "#60717c"),
    ]
    for index, item in enumerate(metrics["quality"]):
        y = 665 + index * 62
        parts += [
            text(85, y + 21, item["label"], 19, "#263238", 600),
            text(355, y + 21, f'{item["placement_mm"]:.1f} mm placement', 20, "#2878b5", 700),
        ]
        perception = item["perception_mm"]
        perception_text = f'{perception:.1f} mm RGB-D error' if perception > 0 else "N/A perception"
        parts.append(text(590, y + 21, perception_text, 18, "#167d6d", 600))

    payload = metrics["payload"]
    parts += [
        rect(815, 540, 730, 330, panel_fill, 8, "#dfe4e8"),
        text(845, 585, "Supplemental payload transfer", 27, "#17212b", 700),
        text(845, 617, "Prior candidate study; five matched unloaded/loaded pairs", 17, "#60717c"),
        text(845, 690, f'{payload["matched_pairs"]}/{payload["pair_count"]}', 52, "#167d6d", 700),
        text(965, 684, "state-matched pairs", 20, "#263238", 600),
        text(845, 748, f'{payload["max_state_delta_rad"] * 1000:.2f} mrad', 31, "#2878b5", 700),
        text(1040, 745, "maximum start-state delta", 18, "#52616b"),
        text(845, 806, f'{payload["loaded_mean_drift_m"] * 1000:.1f} mm', 31, "#d08a24", 700),
        text(1040, 803, "mean maximum payload drift", 18, "#52616b"),
        text(70, 925, f'v1.0 cohorts share source {metrics["release"]["source_commit"][:7]}; planner and payload panels are supplemental. Gazebo truth assists monitoring and scoring.', 17, "#60717c"),
        text(1530, 952, "Generated from committed CSV/JSON evidence", 15, "#7b8991", 400, "end"),
        "</svg>",
    ]
    return "\n".join(parts) + "\n"


def render_payload(metrics: dict) -> str:
    payload = metrics["payload"]
    width, height = 1200, 640
    loaded_mean = payload["loaded_mean_max_error_rad"] * 1000.0
    unloaded_mean = payload["unloaded_mean_max_error_rad"] * 1000.0
    loaded_p95 = payload["loaded_p95_error_rad"] * 1000.0
    values = [unloaded_mean, loaded_mean, loaded_p95]
    maximum = max(values) * 1.15
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img" aria-labelledby="title desc">',
        '<title id="title">Matched payload transfer diagnostics</title>',
        '<desc id="desc">Comparison of unloaded and loaded arm tracking errors with payload drift and state matching.</desc>',
        rect(0, 0, width, height, "#f7f8fa", 0),
        text(60, 66, "Matched Payload Transfer Diagnostics", 34, "#17212b", 700),
        text(60, 102, "Supplemental candidate study, five unloaded/loaded pairs", 19, "#60717c"),
        rect(45, 135, 720, 435, "#ffffff", 8, "#dfe4e8"),
        text(75, 180, "Arm transfer tracking error", 25, "#17212b", 700),
        text(75, 210, "mrad; lower is better", 17, "#60717c"),
    ]
    labels = ["Unloaded mean max", "Loaded mean max", "Loaded P95 max"]
    colors = ["#2878b5", "#167d6d", "#d08a24"]
    for index, (label, value, color) in enumerate(zip(labels, values, colors)):
        y = 260 + index * 92
        parts += [
            text(75, y + 24, label, 19, "#263238", 600),
            rect(300, y, 390, 32, "#e8edf0", 3),
            rect(300, y, 390 * value / maximum, 32, color, 3),
            text(710, y + 24, f"{value:.2f}", 20, color, 700, "end"),
        ]
    parts += [
        rect(795, 135, 360, 435, "#ffffff", 8, "#dfe4e8"),
        text(825, 180, "Physical stability", 25, "#17212b", 700),
        text(825, 260, f'{payload["loaded_mean_drift_m"] * 1000:.1f} mm', 43, "#d08a24", 700),
        text(825, 292, "mean maximum payload drift", 17, "#60717c"),
        text(825, 375, f'{payload["max_state_delta_rad"] * 1000:.2f} mrad', 43, "#2878b5", 700),
        text(825, 407, "maximum pair start delta", 17, "#60717c"),
        text(825, 490, f'{payload["matched_pairs"]}/{payload["pair_count"]} pairs', 43, "#167d6d", 700),
        text(825, 522, "matched under 50 mrad gate", 17, "#60717c"),
        text(60, 615, "The loaded/unloaded relative ratio is omitted because unloaded RMS is near zero; absolute errors and drift are reported instead.", 16, "#60717c"),
        "</svg>",
    ]
    return "\n".join(parts) + "\n"


def outputs(metrics: dict) -> dict[Path, str]:
    public_metrics = json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    return {
        ASSETS / "benchmark_overview.svg": render_overview(metrics),
        ASSETS / "payload_diagnostics.svg": render_payload(metrics),
        ASSETS / "benchmark_metrics.json": public_metrics,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="fail if committed assets are stale")
    args = parser.parse_args()
    rendered = outputs(load_metrics())
    stale = []
    for path, content in rendered.items():
        if args.check:
            if not path.exists() or path.read_text(encoding="utf-8") != content:
                stale.append(path.relative_to(ROOT))
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content, encoding="utf-8", newline="\n")
            print(f"Wrote {path.relative_to(ROOT)}")
    if stale:
        for path in stale:
            print(f"Stale portfolio asset: {path}")
        return 1
    if args.check:
        print("Portfolio assets match public evidence.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
