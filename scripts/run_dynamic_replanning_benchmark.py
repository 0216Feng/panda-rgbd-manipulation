"""Run planner/scenario matrices for RGB-D online replanning."""

from __future__ import annotations

import argparse
import csv
import html
import json
import subprocess
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from statistics import fmean
from typing import Any, Dict, Iterable, List, Optional


PLANNERS = [
    "RRTConnectkConfigDefault",
    "PRMkConfigDefault",
    "RRTstarkConfigDefault",
]


@dataclass(frozen=True)
class DynamicScenario:
    name: str
    speed_mps: float
    trigger_delay_s: float
    start_y: float = -0.55
    end_y: float = 0.55
    perception_samples: int = 12
    perception_span_m: float = 0.12


SCENARIOS = [
    DynamicScenario("slow_crossing", speed_mps=0.10, trigger_delay_s=0.0),
    DynamicScenario(
        "fast_crossing",
        speed_mps=0.18,
        trigger_delay_s=0.1,
        perception_samples=8,
        perception_span_m=0.08,
    ),
    DynamicScenario(
        "delayed_crossing",
        speed_mps=0.18,
        trigger_delay_s=1.5,
        perception_samples=8,
        perception_span_m=0.08,
    ),
]


CSV_FIELDS = [
    "trial",
    "repeat",
    "planner_id",
    "scenario_name",
    "obstacle_speed_mps",
    "obstacle_trigger_delay_s",
    "motion_prediction_enabled",
    "prediction_horizon_s",
    "perception_required_samples",
    "perception_skip_initial_samples",
    "final_state",
    "elapsed_s",
    "dynamic_replan_count",
    "safety_events",
    "safety_check_latency_s",
    "collision_lead_time_s",
    "cancel_ack_latency_s",
    "replan_trigger_latency_s",
    "ompl_planning_time_s",
    "ompl_joint_path_length_rad",
    "pipeline_state",
    "physical_state",
    "perception_state",
    "perception_mean_error_m",
    "perception_max_error_m",
    "lift_delta_m",
    "place_error_m",
    "reason",
]


def parse_result(output: str) -> Dict[str, Any]:
    start = output.find("{")
    if start < 0:
        return {"final_state": "FAILED", "reason": "runner returned no JSON result"}
    try:
        payload, _ = json.JSONDecoder().raw_decode(output[start:])
    except json.JSONDecodeError as exc:
        return {"final_state": "FAILED", "reason": f"invalid runner JSON: {exc}"}
    return payload


def numeric_values(rows: Iterable[Dict[str, Any]], field: str) -> List[float]:
    return [
        float(row[field])
        for row in rows
        if row.get(field) not in (None, "")
    ]


def mean_value(rows: Iterable[Dict[str, Any]], field: str) -> Optional[float]:
    values = numeric_values(rows, field)
    return fmean(values) if values else None


def grouped_summary(rows: List[Dict[str, Any]], field: str) -> List[Dict[str, Any]]:
    summaries = []
    for value in dict.fromkeys(str(row[field]) for row in rows):
        group = [row for row in rows if str(row[field]) == value]
        successes = [row for row in group if row.get("final_state") == "SUCCESS"]
        task_successes = [
            row
            for row in group
            if row.get("pipeline_state") == "SUCCESS"
            and row.get("physical_state") == "SUCCESS"
        ]
        summaries.append(
            {
                "name": value,
                "trials": len(group),
                "successes": len(successes),
                "success_rate": len(successes) / len(group),
                "task_successes": len(task_successes),
                "task_success_rate": len(task_successes) / len(group),
                "mean_elapsed_s": mean_value(group, "elapsed_s"),
                "mean_check_latency_s": mean_value(group, "safety_check_latency_s"),
                "mean_collision_lead_s": mean_value(group, "collision_lead_time_s"),
                "mean_cancel_latency_s": mean_value(group, "cancel_ack_latency_s"),
                "mean_replan_latency_s": mean_value(group, "replan_trigger_latency_s"),
                "mean_planning_time_s": mean_value(group, "ompl_planning_time_s"),
                "mean_perception_error_m": mean_value(
                    successes,
                    "perception_mean_error_m",
                ),
                "mean_place_error_m": mean_value(task_successes, "place_error_m"),
            }
        )
    return summaries


def format_metric(value: Optional[float], digits: int = 4) -> str:
    return "N/A" if value is None else f"{value:.{digits}f}"


def markdown_table(title: str, summaries: List[Dict[str, Any]]) -> List[str]:
    lines = [
        f"## {title}",
        "",
        "| Name | Full evidence | Physical task | Check latency (s) | Collision lead (s) | Cancel ack (s) | Replan trigger (s) | OMPL time (s) | RGB-D error (m) | Place error (m) |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for item in summaries:
        lines.append(
            "| {name} | {successes}/{trials} ({rate:.1%}) | "
            "{task_successes}/{trials} ({task_rate:.1%}) | {check} | {lead} | {cancel} | "
            "{replan} | {planning} | {perception} | {place} |".format(
                name=item["name"],
                successes=item["successes"],
                trials=item["trials"],
                rate=item["success_rate"],
                task_successes=item["task_successes"],
                task_rate=item["task_success_rate"],
                check=format_metric(item["mean_check_latency_s"]),
                lead=format_metric(item["mean_collision_lead_s"]),
                cancel=format_metric(item["mean_cancel_latency_s"]),
                replan=format_metric(item["mean_replan_latency_s"]),
                planning=format_metric(item["mean_planning_time_s"]),
                perception=format_metric(item["mean_perception_error_m"]),
                place=format_metric(item["mean_place_error_m"]),
            )
        )
    return lines


def write_reports(
    rows: List[Dict[str, Any]],
    csv_path: Path,
    markdown_path: Path,
    svg_path: Path,
) -> None:
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows({field: row.get(field) for field in CSV_FIELDS} for row in rows)

    successes = [row for row in rows if row.get("final_state") == "SUCCESS"]
    task_successes = [
        row
        for row in rows
        if row.get("pipeline_state") == "SUCCESS"
        and row.get("physical_state") == "SUCCESS"
    ]
    task_failure_reasons = Counter(
        str(row.get("reason", "unknown"))
        for row in rows
        if row not in task_successes
    )
    evidence_gaps = Counter(
        str(row.get("reason", "unknown"))
        for row in task_successes
        if row.get("final_state") != "SUCCESS"
    )
    planner_summary = grouped_summary(rows, "planner_id")
    scenario_summary = grouped_summary(rows, "scenario_name")
    lines = [
        "# RGB-D Dynamic Replanning Matrix",
        "",
        f"- Trials: {len(rows)}",
        f"- Full-system successes: {len(successes)}",
        f"- Full-evidence success rate: {len(successes) / len(rows):.1%}",
        f"- Physically validated task successes: {len(task_successes)}",
        f"- Physical task success rate: {len(task_successes) / len(rows):.1%}",
        f"- Mean trial time: {format_metric(mean_value(rows, 'elapsed_s'))} s",
        f"- Mean safety-check latency: {format_metric(mean_value(rows, 'safety_check_latency_s'))} s",
        f"- Mean collision lead time: {format_metric(mean_value(rows, 'collision_lead_time_s'))} s",
        f"- Mean cancel acknowledgement latency: {format_metric(mean_value(rows, 'cancel_ack_latency_s'))} s",
        f"- Mean replan-trigger latency: {format_metric(mean_value(rows, 'replan_trigger_latency_s'))} s",
        f"- Mean RGB-D position error: {format_metric(mean_value(successes, 'perception_mean_error_m'))} m",
        f"- Mean placement error: {format_metric(mean_value(task_successes, 'place_error_m'))} m",
        "",
        "A trial passes only when RGB-D tracking, trajectory invalidation, online "
        "cancellation/replanning, physical pick-and-place, and return-home all succeed.",
        "",
        *markdown_table("By Planner", planner_summary),
        "",
        *markdown_table("By Dynamic Scenario", scenario_summary),
    ]
    if task_failure_reasons:
        lines.extend(["", "## Physical Task Failure Reasons", ""])
        lines.extend(
            f"- {reason}: {count}"
            for reason, count in sorted(task_failure_reasons.items())
        )
    if evidence_gaps:
        lines.extend(["", "## Validation Evidence Gaps", ""])
        lines.extend(
            f"- {reason}: {count}"
            for reason, count in sorted(evidence_gaps.items())
        )
    markdown_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    write_svg_report(planner_summary, scenario_summary, svg_path)


def write_svg_report(
    planner_summary: List[Dict[str, Any]],
    scenario_summary: List[Dict[str, Any]],
    path: Path,
) -> None:
    groups = [("Planner", planner_summary), ("Scenario", scenario_summary)]
    width = 1260
    height = 180 + 90 * sum(len(items) for _, items in groups)
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f7f8fa"/>',
        '<style>text{font-family:Arial,sans-serif;fill:#20242a}.title{font-size:26px;font-weight:700}.section{font-size:19px;font-weight:700}.label{font-size:14px}.value{font-size:13px;font-weight:700}</style>',
        '<text x="40" y="48" class="title">RGB-D Dynamic Replanning Matrix</text>',
        '<text x="40" y="76" class="label">Blue: full evidence. Green: physically validated task success.</text>',
    ]
    y = 120
    for section, items in groups:
        parts.append(f'<text x="40" y="{y}" class="section">By {section}</text>')
        y += 28
        for item in items:
            name = html.escape(str(item["name"]))
            rate = float(item["success_rate"])
            task_rate = float(item["task_success_rate"])
            bar_width = 620 * rate
            task_bar_width = 620 * task_rate
            parts.extend(
                [
                    f'<text x="40" y="{y + 18}" class="label">{name}</text>',
                    f'<rect x="270" y="{y}" width="620" height="24" rx="3" fill="#dfe3e8"/>',
                    f'<rect x="270" y="{y}" width="{task_bar_width:.1f}" height="24" rx="3" fill="#64a878"/>',
                    f'<rect x="270" y="{y}" width="{bar_width:.1f}" height="24" rx="3" fill="#2878b5"/>',
                    f'<text x="910" y="{y + 18}" class="value">{item["successes"]}/{item["trials"]} full; {item["task_successes"]}/{item["trials"]} physical</text>',
                ]
            )
            y += 54
        y += 28
    parts.append("</svg>")
    path.write_text("\n".join(parts), encoding="utf-8")


def scenario_by_name(name: str) -> DynamicScenario:
    return next(scenario for scenario in SCENARIOS if scenario.name == name)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--trials",
        type=int,
        default=3,
        help="repetitions for every selected planner/scenario pair",
    )
    parser.add_argument("--timeout-s", type=float, default=180.0)
    parser.add_argument("--planner-id", default=PLANNERS[0], choices=PLANNERS)
    parser.add_argument("--planner-suite", action="store_true")
    parser.add_argument(
        "--scenario",
        default="fast_crossing",
        choices=[scenario.name for scenario in SCENARIOS],
    )
    parser.add_argument("--scenario-suite", action="store_true")
    parser.add_argument("--enable-motion-prediction", action="store_true")
    parser.add_argument("--prediction-horizon-s", type=float, default=0.65)
    parser.add_argument("--predictive-perception-required-samples", type=int, default=4)
    parser.add_argument("--predictive-perception-skip-initial-samples", type=int, default=3)
    parser.add_argument("--csv", default="dynamic_replanning_matrix.csv")
    parser.add_argument("--markdown", default="dynamic_replanning_matrix.md")
    parser.add_argument("--svg", default="dynamic_replanning_matrix.svg")
    parser.add_argument("--log-dir", default="dynamic_replanning_matrix_logs")
    parser.add_argument(
        "--report-only-csv",
        help="regenerate Markdown/SVG from an existing per-trial CSV without running Gazebo",
    )
    args = parser.parse_args()
    if args.report_only_csv:
        with Path(args.report_only_csv).open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        if not rows:
            parser.error("--report-only-csv contains no trial rows")
        write_reports(rows, Path(args.csv), Path(args.markdown), Path(args.svg))
        print(
            f"Regenerated reports for {len(rows)} trials: "
            f"{args.csv}, {args.markdown}, {args.svg}."
        )
        return 0
    if args.trials < 1:
        parser.error("--trials must be at least 1")

    planners = PLANNERS if args.planner_suite else [args.planner_id]
    scenarios = SCENARIOS if args.scenario_suite else [scenario_by_name(args.scenario)]
    total = args.trials * len(planners) * len(scenarios)
    print(
        f"Benchmark matrix: {len(planners)} planner(s) x {len(scenarios)} "
        f"scenario(s) x {args.trials} repeat(s) = {total} trial(s)."
    )
    script = Path(__file__).with_name("run_dynamic_replanning_smoke.py")
    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    rows: List[Dict[str, Any]] = []
    run_index = 0
    for planner in planners:
        for scenario in scenarios:
            for repeat in range(1, args.trials + 1):
                run_index += 1
                print(
                    f"[{run_index}/{total}] planner={planner}, "
                    f"scenario={scenario.name}, repeat={repeat}...",
                    flush=True,
                )
                log_name = (
                    f"{planner.replace('kConfigDefault', '').lower()}_"
                    f"{scenario.name}_{repeat:02d}.log"
                )
                command = [
                    sys.executable,
                    str(script),
                    "--timeout-s",
                    str(args.timeout_s),
                    "--planner-id",
                    planner,
                    "--scenario-name",
                    scenario.name,
                    "--obstacle-start-y",
                    str(scenario.start_y),
                    "--obstacle-end-y",
                    str(scenario.end_y),
                    "--obstacle-speed-mps",
                    str(scenario.speed_mps),
                    "--obstacle-trigger-delay-s",
                    str(scenario.trigger_delay_s),
                    "--perception-required-samples",
                    str(
                        args.predictive_perception_required_samples
                        if args.enable_motion_prediction
                        else scenario.perception_samples
                    ),
                    "--perception-minimum-span-m",
                    str(scenario.perception_span_m),
                    "--perception-skip-initial-samples",
                    str(
                        args.predictive_perception_skip_initial_samples
                        if args.enable_motion_prediction
                        else 5
                    ),
                    "--log",
                    str(log_dir / log_name),
                ]
                if args.enable_motion_prediction:
                    command.extend(
                        [
                            "--enable-motion-prediction",
                            "--prediction-horizon-s",
                            str(args.prediction_horizon_s),
                        ]
                    )
                try:
                    completed = subprocess.run(
                        command,
                        check=False,
                        capture_output=True,
                        text=True,
                        timeout=args.timeout_s + 40.0,
                    )
                    row = parse_result(completed.stdout)
                except subprocess.TimeoutExpired:
                    row = {
                        "final_state": "FAILED",
                        "reason": "outer benchmark runner timed out",
                        "elapsed_s": args.timeout_s + 40.0,
                    }
                row.update(
                    {
                        "trial": run_index,
                        "repeat": repeat,
                        "planner_id": planner,
                        "scenario_name": scenario.name,
                        "obstacle_speed_mps": scenario.speed_mps,
                        "obstacle_trigger_delay_s": scenario.trigger_delay_s,
                        "motion_prediction_enabled": args.enable_motion_prediction,
                        "prediction_horizon_s": args.prediction_horizon_s,
                        "perception_required_samples": (
                            args.predictive_perception_required_samples
                            if args.enable_motion_prediction
                            else scenario.perception_samples
                        ),
                        "perception_skip_initial_samples": (
                            args.predictive_perception_skip_initial_samples
                            if args.enable_motion_prediction
                            else 5
                        ),
                    }
                )
                rows.append(row)
                print(
                    f"[{run_index}/{total}] {row.get('final_state', 'FAILED')}: "
                    f"{row.get('reason', 'unknown')} "
                    f"({float(row.get('elapsed_s', 0.0)):.1f}s)",
                    flush=True,
                )

    write_reports(rows, Path(args.csv), Path(args.markdown), Path(args.svg))
    successes = sum(row.get("final_state") == "SUCCESS" for row in rows)
    print(
        f"Completed {len(rows)} trials: {successes} full-system successes "
        f"({successes / len(rows):.1%})."
    )
    print(f"Reports: {args.csv}, {args.markdown}, {args.svg}; logs: {args.log_dir}")
    return 0 if successes == len(rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
