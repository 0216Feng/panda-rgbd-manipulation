import argparse
import csv
import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "evaluate_v1_release.py"
SPEC = importlib.util.spec_from_file_location("evaluate_v1_release", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def write_physics_cohort(
    directory: Path,
    scenarios: list[str],
    states: list[str],
    *,
    source_digest: str = "package-digest",
    blocked: bool = True,
) -> Path:
    directory.mkdir(parents=True)
    csv_path = directory / "results.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as destination:
        writer = csv.DictWriter(
            destination,
            fieldnames=(
                "scenario_name",
                "perception_mode",
                "perception_validation_state",
                "perception_mean_error_m",
                "place_error_m",
                "final_tilt_deg",
                "final_state",
                "direct_path_blocked",
                "reason",
            ),
        )
        writer.writeheader()
        for scenario, state in zip(scenarios, states):
            writer.writerow(
                {
                    "scenario_name": scenario,
                    "perception_mode": "rgbd",
                    "perception_validation_state": "SUCCESS",
                    "perception_mean_error_m": "0.004",
                    "place_error_m": "0.020",
                    "final_tilt_deg": "1.0",
                    "final_state": state,
                    "direct_path_blocked": blocked,
                    "reason": "verified" if state == "SUCCESS" else "failed",
                }
            )
    manifest = {
        "sha256": f"cohort-{directory.name}",
        "files": {"src/panda_manipulation/node.py": source_digest},
    }
    Path(f"{csv_path}.source.json").write_text(json.dumps(manifest))
    Path(f"{csv_path}.config.json").write_text(
        json.dumps({"source_sha256": manifest["sha256"]})
    )
    return csv_path


def write_safe_stop(directory: Path, source_digest: str = "package-digest") -> Path:
    directory.mkdir(parents=True)
    report = directory / "physical_disturbance_safety.json"
    report.write_text(
        json.dumps(
            {
                "total_trials": 1,
                "disturbances_applied": 1,
                "disturbances_cleared": 1,
                "safe_stops": 1,
                "all_meet_strictest_stop_latency_limit": True,
                "mean_stop_latency_s": 0.1,
                "max_stop_latency_s": 0.1,
                "stop_latency_limit_s": 0.2,
                "failed_verdict_reasons": [],
            }
        )
    )
    manifest = {
        "sha256": "safe-stop-cohort",
        "files": {"src/panda_manipulation/node.py": source_digest},
    }
    (directory / "source_manifest.json").write_text(json.dumps(manifest))
    (directory / "experiment_config.json").write_text(
        json.dumps({"source_sha256": manifest["sha256"]})
    )
    return report


def release_arguments(tmp_path: Path) -> argparse.Namespace:
    fixed = write_physics_cohort(
        tmp_path / "fixed", ["baseline"] * 10, ["SUCCESS"] * 10
    )
    random = write_physics_cohort(
        tmp_path / "random",
        [f"random_{index:03d}" for index in range(20)],
        ["SUCCESS"] * 17 + ["FAILED"] * 3,
    )
    obstacles = write_physics_cohort(
        tmp_path / "obstacles",
        sorted(MODULE.EXPECTED_OBSTACLE_SCENARIOS),
        ["SUCCESS"] * 3,
    )
    safe_stop = write_safe_stop(tmp_path / "safe_stop")
    return argparse.Namespace(
        fixed_csv=fixed,
        random_csv=random,
        obstacle_csv=obstacles,
        safe_stop_json=safe_stop,
        output_json=tmp_path / "summary.json",
        output_markdown=tmp_path / "summary.md",
        profile="full",
        fixed_trials=10,
        random_trials=20,
        obstacle_trials=3,
        safe_stop_trials=1,
        fixed_minimum_rate=1.0,
        random_minimum_rate=0.85,
        obstacle_minimum_rate=1.0,
    )


def test_v1_release_accepts_complete_thresholded_evidence(tmp_path):
    result = MODULE.evaluate(release_arguments(tmp_path))
    assert result["passed"] is True
    assert result["source"]["consistent_across_all_cohorts"] is True
    assert result["cohorts"][1]["success_rate"] == pytest.approx(0.85)
    assert "Final verdict: `PASS`" in MODULE.render_markdown(result)


def test_v1_release_rejects_unblocked_obstacle_or_mixed_source(tmp_path):
    args = release_arguments(tmp_path)
    obstacle_rows = MODULE.load_csv(args.obstacle_csv)
    obstacle_rows[0]["direct_path_blocked"] = "False"
    with args.obstacle_csv.open("w", newline="", encoding="utf-8") as destination:
        writer = csv.DictWriter(destination, fieldnames=obstacle_rows[0].keys())
        writer.writeheader()
        writer.writerows(obstacle_rows)
    safe_manifest = json.loads(
        (args.safe_stop_json.parent / "source_manifest.json").read_text()
    )
    safe_manifest["files"]["src/panda_manipulation/node.py"] = "different"
    (args.safe_stop_json.parent / "source_manifest.json").write_text(
        json.dumps(safe_manifest)
    )
    result = MODULE.evaluate(args)
    assert result["passed"] is False
    assert result["source"]["consistent_across_all_cohorts"] is False
    assert result["cohorts"][2]["checks"]["direct_paths_blocked"] is False
