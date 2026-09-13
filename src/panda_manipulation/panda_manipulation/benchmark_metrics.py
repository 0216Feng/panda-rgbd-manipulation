"""Planner benchmark data model and reports."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from statistics import mean
from typing import Dict, Iterable, List


@dataclass(frozen=True)
class PlannerRun:
    planner_id: str
    scenario_id: str
    success: bool
    planning_time_s: float
    trajectory_length_m: float
    execution_time_s: float
    failure_reason: str = ""

    def as_row(self) -> Dict[str, object]:
        return {
            "planner_id": self.planner_id,
            "scenario_id": self.scenario_id,
            "success": self.success,
            "planning_time_s": round(self.planning_time_s, 4),
            "trajectory_length_m": round(self.trajectory_length_m, 4),
            "execution_time_s": round(self.execution_time_s, 4),
            "failure_reason": self.failure_reason,
        }


def aggregate_by_planner(runs: Iterable[PlannerRun]) -> Dict[str, Dict[str, float]]:
    grouped: Dict[str, List[PlannerRun]] = {}
    for run in runs:
        grouped.setdefault(run.planner_id, []).append(run)

    summary: Dict[str, Dict[str, float]] = {}
    for planner_id, planner_runs in grouped.items():
        successful = [run for run in planner_runs if run.success]
        summary[planner_id] = {
            "trials": float(len(planner_runs)),
            "success_rate": len(successful) / len(planner_runs) if planner_runs else 0.0,
            "avg_planning_time_s": mean([run.planning_time_s for run in successful]) if successful else 0.0,
            "avg_trajectory_length_m": mean([run.trajectory_length_m for run in successful]) if successful else 0.0,
            "avg_execution_time_s": mean([run.execution_time_s for run in successful]) if successful else 0.0,
        }
    return summary


def write_csv(path: str | Path, runs: Iterable[PlannerRun]) -> None:
    rows = [run.as_row() for run in runs]
    fieldnames = [
        "planner_id",
        "scenario_id",
        "success",
        "planning_time_s",
        "trajectory_length_m",
        "execution_time_s",
        "failure_reason",
    ]
    with Path(path).open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def render_markdown(summary: Dict[str, Dict[str, float]]) -> str:
    lines = [
        "# Planner Benchmark Report",
        "",
        "| Planner | Trials | Success Rate | Avg Planning Time (s) | Avg Trajectory Length (m) | Avg Execution Time (s) |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for planner_id, metrics in sorted(summary.items()):
        lines.append(
            "| {planner} | {trials:.0f} | {success:.1%} | {planning:.3f} | {length:.3f} | {execution:.3f} |".format(
                planner=planner_id,
                trials=metrics["trials"],
                success=metrics["success_rate"],
                planning=metrics["avg_planning_time_s"],
                length=metrics["avg_trajectory_length_m"],
                execution=metrics["avg_execution_time_s"],
            )
        )
    return "\n".join(lines) + "\n"
