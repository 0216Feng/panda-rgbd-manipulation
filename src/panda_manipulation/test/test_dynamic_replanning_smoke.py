import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "run_dynamic_replanning_smoke.py"
SPEC = importlib.util.spec_from_file_location("dynamic_replanning_smoke", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_result_reason_reports_missing_perception_result():
    reason = MODULE.result_reason(
        False,
        {"final_state": "SUCCESS"},
        {"final_state": "SUCCESS"},
        None,
        [{"event": "trajectory_hazard"}],
    )
    assert reason == "dynamic obstacle perception validation result was not received"


def test_result_reason_preserves_pipeline_failure():
    reason = MODULE.result_reason(
        False,
        {"final_state": "FAILED", "failure_reason": "planning failed"},
        None,
        None,
        [],
    )
    assert reason == "planning failed"
