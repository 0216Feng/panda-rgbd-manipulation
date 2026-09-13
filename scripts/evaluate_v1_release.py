#!/usr/bin/env python3
"""Evaluate one frozen-source v1.0 physical release evidence bundle."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any


EXPECTED_OBSTACLE_SCENARIOS = {
    "center_tall_barrier",
    "positive_y_barrier",
    "negative_y_barrier",
}


def load_csv(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        raise ValueError(f"missing CSV: {path}")
    with path.open(newline="", encoding="utf-8") as source:
        return list(csv.DictReader(source))


def load_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise ValueError(f"missing JSON: {path}")
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected JSON object: {path}")
    return payload


def as_bool(value: object) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def finite_column(rows: list[dict[str, str]], name: str) -> list[float]:
    values: list[float] = []
    for row in rows:
        try:
            value = float(row.get(name, ""))
        except (TypeError, ValueError):
            continue
        if math.isfinite(value):
            values.append(value)
    return values


def wilson_interval(successes: int, total: int) -> tuple[float, float] | None:
    if total <= 0:
        return None
    z = 1.959963984540054
    rate = successes / total
    denominator = 1.0 + z * z / total
    center = (rate + z * z / (2.0 * total)) / denominator
    margin = z * math.sqrt(
        rate * (1.0 - rate) / total + z * z / (4.0 * total * total)
    ) / denominator
    return max(0.0, center - margin), min(1.0, center + margin)


def physics_source_manifest(csv_path: Path) -> dict[str, Any]:
    manifest = load_json(Path(f"{csv_path}.source.json"))
    config = load_json(Path(f"{csv_path}.config.json"))
    if config.get("source_sha256") != manifest.get("sha256"):
        raise ValueError(f"source/config fingerprint mismatch beside {csv_path}")
    return manifest


def safety_source_manifest(safety_json: Path) -> dict[str, Any]:
    directory = safety_json.parent
    manifest = load_json(directory / "source_manifest.json")
    config = load_json(directory / "experiment_config.json")
    if config.get("source_sha256") != manifest.get("sha256"):
        raise ValueError(f"source/config fingerprint mismatch in {directory}")
    return manifest


def package_files(manifest: dict[str, Any]) -> dict[str, str]:
    files = manifest.get("files")
    if not isinstance(files, dict):
        raise ValueError("source manifest has no files mapping")
    selected = {
        str(name): str(digest)
        for name, digest in files.items()
        if str(name).startswith("src/panda_manipulation/")
    }
    if not selected:
        raise ValueError("source manifest has no panda_manipulation files")
    return selected


def cohort_result(
    name: str,
    rows: list[dict[str, str]],
    expected_trials: int,
    minimum_rate: float,
) -> dict[str, Any]:
    successes = sum(row.get("final_state") == "SUCCESS" for row in rows)
    total = len(rows)
    rate = successes / total if total else 0.0
    reasons = [
        row.get("reason") or "unspecified failure"
        for row in rows
        if row.get("final_state") != "SUCCESS"
    ]
    perception_errors = finite_column(rows, "perception_mean_error_m")
    placement_errors = finite_column(
        [row for row in rows if row.get("final_state") == "SUCCESS"],
        "place_error_m",
    )
    final_tilts = finite_column(
        [row for row in rows if row.get("final_state") == "SUCCESS"],
        "final_tilt_deg",
    )
    mean_perception_error = (
        sum(perception_errors) / len(perception_errors)
        if perception_errors
        else None
    )
    mean_placement_error = (
        sum(placement_errors) / len(placement_errors)
        if placement_errors
        else None
    )
    checks = {
        "trial_count": total == expected_trials,
        "rgbd_perception": all(row.get("perception_mode") == "rgbd" for row in rows),
        "perception_validation": all(
            row.get("perception_validation_state") == "SUCCESS" for row in rows
        ),
        "perception_error": (
            len(perception_errors) == total
            and mean_perception_error is not None
            and mean_perception_error <= 0.010
        ),
        "placement_error": (
            len(placement_errors) == successes
            and mean_placement_error is not None
            and mean_placement_error <= 0.050
        ),
        "upright_orientation": (
            len(final_tilts) == successes
            and bool(final_tilts)
            and max(final_tilts) <= 15.0
        ),
        "minimum_success_rate": rate >= minimum_rate,
    }
    return {
        "name": name,
        "trials": total,
        "expected_trials": expected_trials,
        "successes": successes,
        "success_rate": rate,
        "success_rate_ci": wilson_interval(successes, total),
        "minimum_success_rate": minimum_rate,
        "mean_perception_error_m": mean_perception_error,
        "mean_placement_error_m": mean_placement_error,
        "max_final_tilt_deg": max(final_tilts, default=None),
        "checks": checks,
        "failure_reasons": reasons,
        "passed": all(checks.values()),
    }


def evaluate(args: argparse.Namespace) -> dict[str, Any]:
    fixed_rows = load_csv(args.fixed_csv)
    random_rows = load_csv(args.random_csv)
    obstacle_rows = load_csv(args.obstacle_csv)
    safety = load_json(args.safe_stop_json)

    fixed = cohort_result(
        "fixed_rgbd", fixed_rows, args.fixed_trials, args.fixed_minimum_rate
    )
    random = cohort_result(
        "random_rgbd", random_rows, args.random_trials, args.random_minimum_rate
    )
    obstacle = cohort_result(
        "static_obstacles",
        obstacle_rows,
        args.obstacle_trials,
        args.obstacle_minimum_rate,
    )
    obstacle_scenarios = {row.get("scenario_name") for row in obstacle_rows}
    obstacle["checks"].update(
        {
            "representative_scenarios": obstacle_scenarios
            == EXPECTED_OBSTACLE_SCENARIOS,
            "direct_paths_blocked": all(
                as_bool(row.get("direct_path_blocked")) for row in obstacle_rows
            ),
        }
    )
    obstacle["scenarios"] = sorted(str(item) for item in obstacle_scenarios)
    obstacle["passed"] = all(obstacle["checks"].values())

    safe_total = int(safety.get("total_trials") or 0)
    safe_stops = int(safety.get("safe_stops") or 0)
    safe_checks = {
        "trial_count": safe_total == args.safe_stop_trials,
        "wrench_applied": int(safety.get("disturbances_applied") or 0) == safe_total,
        "wrench_cleared": int(safety.get("disturbances_cleared") or 0) == safe_total,
        "watchdog_safe_stop": safe_stops == safe_total,
        "latency_limit": safety.get("all_meet_strictest_stop_latency_limit") is True,
    }
    safe_stop = {
        "name": "gazebo_wrench_safe_stop",
        "trials": safe_total,
        "expected_trials": args.safe_stop_trials,
        "safe_stops": safe_stops,
        "safe_stop_rate": safe_stops / safe_total if safe_total else 0.0,
        "safe_stop_rate_ci": wilson_interval(safe_stops, safe_total),
        "mean_stop_latency_s": safety.get("mean_stop_latency_s"),
        "max_stop_latency_s": safety.get("max_stop_latency_s"),
        "stop_latency_limit_s": safety.get("stop_latency_limit_s"),
        "checks": safe_checks,
        "failure_reasons": safety.get("failed_verdict_reasons", []),
        "passed": all(safe_checks.values()),
    }

    manifests = [
        physics_source_manifest(args.fixed_csv),
        physics_source_manifest(args.random_csv),
        physics_source_manifest(args.obstacle_csv),
        safety_source_manifest(args.safe_stop_json),
    ]
    reference_package_files = package_files(manifests[0])
    source_consistent = all(
        package_files(manifest) == reference_package_files
        for manifest in manifests[1:]
    )
    source = {
        "package_files": len(reference_package_files),
        "consistent_across_all_cohorts": source_consistent,
        "cohort_sha256": [manifest.get("sha256") for manifest in manifests],
    }
    cohorts = [fixed, random, obstacle, safe_stop]
    return {
        "schema_version": 1,
        "profile": args.profile,
        "passed": source_consistent and all(cohort["passed"] for cohort in cohorts),
        "source": source,
        "cohorts": cohorts,
    }


def percentage(value: object) -> str:
    return f"{100.0 * float(value):.1f}%"


def metric(value: object, scale: float, suffix: str) -> str:
    if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        return "n/a"
    return f"{scale * float(value):.2f} {suffix}"


def render_markdown(result: dict[str, Any]) -> str:
    lines = [
        "# v1.0 Release Validation",
        "",
        f"- Profile: `{result['profile']}`",
        f"- Final verdict: `{'PASS' if result['passed'] else 'FAIL'}`",
        "- One source snapshot across all physical cohorts: "
        + ("yes" if result["source"]["consistent_across_all_cohorts"] else "no"),
        "",
        "## Cohorts",
        "",
        "| Cohort | Result | Required | Verdict |",
        "| --- | ---: | ---: | --- |",
    ]
    for cohort in result["cohorts"]:
        if cohort["name"] == "gazebo_wrench_safe_stop":
            achieved = f"{cohort['safe_stops']}/{cohort['trials']} safe stops"
            required = f"{cohort['expected_trials']}/{cohort['expected_trials']}"
        else:
            achieved = (
                f"{cohort['successes']}/{cohort['trials']} "
                f"({percentage(cohort['success_rate'])})"
            )
            required = (
                f"{cohort['expected_trials']} trials, "
                f">={percentage(cohort['minimum_success_rate'])}"
            )
        lines.append(
            f"| {cohort['name']} | {achieved} | {required} | "
            f"{'PASS' if cohort['passed'] else 'FAIL'} |"
        )
    lines.extend(
        [
            "",
            "## Quality Metrics",
            "",
            "| Cohort | Mean perception error | Mean placement error | Max tilt |",
            "| --- | ---: | ---: | ---: |",
        ]
    )
    for cohort in result["cohorts"]:
        if cohort["name"] == "gazebo_wrench_safe_stop":
            lines.append(
                f"| {cohort['name']} | n/a | n/a | "
                f"stop {metric(cohort['max_stop_latency_s'], 1000.0, 'ms')} |"
            )
            continue
        lines.append(
            f"| {cohort['name']} | "
            f"{metric(cohort['mean_perception_error_m'], 1000.0, 'mm')} | "
            f"{metric(cohort['mean_placement_error_m'], 1000.0, 'mm')} | "
            f"{metric(cohort['max_final_tilt_deg'], 1.0, 'deg')} |"
        )
    lines.extend(["", "## Failed Checks", ""])
    failed = []
    for cohort in result["cohorts"]:
        for check, passed in cohort["checks"].items():
            if not passed:
                failed.append(f"- `{cohort['name']}.{check}`")
        failed.extend(
            f"- `{cohort['name']}`: {reason}"
            for reason in cohort.get("failure_reasons", [])
        )
    if not result["source"]["consistent_across_all_cohorts"]:
        failed.append("- `source.consistent_across_all_cohorts`")
    lines.extend(failed or ["- None."])
    lines.append("")
    return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fixed-csv", type=Path, required=True)
    parser.add_argument("--random-csv", type=Path, required=True)
    parser.add_argument("--obstacle-csv", type=Path, required=True)
    parser.add_argument("--safe-stop-json", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-markdown", type=Path, required=True)
    parser.add_argument("--profile", choices=("smoke", "full"), default="full")
    parser.add_argument("--fixed-trials", type=int, default=10)
    parser.add_argument("--random-trials", type=int, default=20)
    parser.add_argument("--obstacle-trials", type=int, default=3)
    parser.add_argument("--safe-stop-trials", type=int, default=1)
    parser.add_argument("--fixed-minimum-rate", type=float, default=1.0)
    parser.add_argument("--random-minimum-rate", type=float, default=0.85)
    parser.add_argument("--obstacle-minimum-rate", type=float, default=1.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        result = evaluate(args)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        raise SystemExit(str(exc)) from exc
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_markdown.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    args.output_markdown.write_text(render_markdown(result), encoding="utf-8")
    print(
        f"v1.0 {args.profile} release verdict: "
        f"{'PASS' if result['passed'] else 'FAIL'}"
    )
    print(f"Report: {args.output_markdown}")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
