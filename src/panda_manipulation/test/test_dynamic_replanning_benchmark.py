import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "run_dynamic_replanning_benchmark.py"
SPEC = importlib.util.spec_from_file_location("dynamic_replanning_benchmark", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_parse_result_reads_pretty_json_before_log_line():
    payload = MODULE.parse_result(
        '{\n  "final_state": "SUCCESS",\n  "elapsed_s": 12.5\n}\nLog: trial.log\n'
    )
    assert payload["final_state"] == "SUCCESS"
    assert payload["elapsed_s"] == 12.5


def test_grouped_summary_computes_success_rate_and_mean():
    rows = [
        {
            "planner_id": "RRTConnect",
            "final_state": "SUCCESS",
            "pipeline_state": "SUCCESS",
            "physical_state": "SUCCESS",
            "elapsed_s": 10.0,
            "place_error_m": 0.002,
        },
        {
            "planner_id": "RRTConnect",
            "final_state": "FAILED",
            "elapsed_s": 20.0,
        },
    ]
    summary = MODULE.grouped_summary(rows, "planner_id")[0]
    assert summary["successes"] == 1
    assert summary["success_rate"] == 0.5
    assert summary["task_success_rate"] == 0.5
    assert summary["mean_elapsed_s"] == 15.0
    assert summary["mean_place_error_m"] == 0.002


def test_write_reports_creates_all_artifacts(tmp_path):
    row = {
        "trial": 1,
        "repeat": 1,
        "planner_id": "RRTConnect",
        "scenario_name": "fast_crossing",
        "final_state": "SUCCESS",
        "elapsed_s": 10.0,
        "safety_check_latency_s": 0.01,
        "collision_lead_time_s": 1.25,
        "cancel_ack_latency_s": 0.02,
        "replan_trigger_latency_s": 0.8,
        "perception_mean_error_m": 0.01,
        "place_error_m": 0.003,
    }
    csv_path = tmp_path / "report.csv"
    markdown_path = tmp_path / "report.md"
    svg_path = tmp_path / "report.svg"
    MODULE.write_reports([row], csv_path, markdown_path, svg_path)
    assert "RRTConnect" in csv_path.read_text(encoding="utf-8")
    markdown = markdown_path.read_text(encoding="utf-8")
    assert "By Planner" in markdown
    assert "Mean collision lead time: 1.2500 s" in markdown
    assert "<svg" in svg_path.read_text(encoding="utf-8")


def test_fast_scenario_uses_pre_latch_perception_window():
    scenario = next(
        scenario for scenario in MODULE.SCENARIOS if scenario.name == "fast_crossing"
    )
    assert scenario.perception_samples == 8
    assert scenario.perception_span_m == 0.08


def test_delayed_scenario_uses_pre_latch_perception_window():
    scenario = next(
        scenario for scenario in MODULE.SCENARIOS if scenario.name == "delayed_crossing"
    )
    assert scenario.perception_samples == 8
    assert scenario.perception_span_m == 0.08


def test_grouped_summary_separates_evidence_gap_from_physical_success():
    rows = [
        {
            "planner_id": "PRM",
            "final_state": "FAILED",
            "pipeline_state": "SUCCESS",
            "physical_state": "SUCCESS",
            "reason": "perception result missing",
        }
    ]
    summary = MODULE.grouped_summary(rows, "planner_id")[0]
    assert summary["successes"] == 0
    assert summary["task_successes"] == 1
