import argparse
import importlib.util
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "run_payload_transfer_diagnostics.py"
SPEC = importlib.util.spec_from_file_location("payload_transfer_runner", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_parse_joint_positions_arg_accepts_seven_finite_values():
    assert MODULE.parse_joint_positions_arg("0, 1, -2, 3.5, 4, 5, 6") == [
        0.0,
        1.0,
        -2.0,
        3.5,
        4.0,
        5.0,
        6.0,
    ]


def test_parse_joint_positions_arg_rejects_wrong_length():
    try:
        MODULE.parse_joint_positions_arg("0,1,2")
    except argparse.ArgumentTypeError as exc:
        assert "exactly seven" in str(exc)
    else:
        raise AssertionError("expected argparse.ArgumentTypeError")


def test_parse_joint_positions_arg_rejects_non_finite_values():
    try:
        MODULE.parse_joint_positions_arg("0,1,2,3,4,5,nan")
    except argparse.ArgumentTypeError as exc:
        assert "finite" in str(exc)
    else:
        raise AssertionError("expected argparse.ArgumentTypeError")


def test_place_descent_execution_time_combines_servo_and_cartesian_fallback():
    result = {
        "steps": [
            {
                "step": "servo_place_descent_stage_1_of_6",
                "reason": "endpoint_error=0.001m, elapsed=2.25s, status=2",
            }
        ],
        "cartesian_trajectory_metrics": [
            {
                "step": "cartesian_place_descent_stage_1_of_6",
                "duration_s": 3.5,
            },
            {"step": "cartesian_retreat", "duration_s": 7.0},
        ],
    }
    assert MODULE.place_descent_execution_time_s(result) == 5.75


def test_decode_csv_value_handles_python_booleans_and_json():
    assert MODULE.decode_csv_value("True") is True
    assert MODULE.decode_csv_value("False") is False
    assert MODULE.decode_csv_value("None") is None
    assert MODULE.decode_csv_value("0.1004") == 0.1004
    assert MODULE.decode_csv_value('[1, 2, 3]') == [1, 2, 3]


def test_servo_fault_cases_builds_type_by_stage_matrix():
    assert MODULE.servo_fault_cases(
        True,
        ("contact_loss", "payload_drift"),
        3,
        "none",
        2,
    ) == (
        ("contact_loss", 1),
        ("contact_loss", 2),
        ("contact_loss", 3),
        ("payload_drift", 1),
        ("payload_drift", 2),
        ("payload_drift", 3),
    )
    assert MODULE.servo_fault_cases(
        False,
        ("contact_loss",),
        6,
        "payload_drift",
        4,
    ) == (("payload_drift", 4),)


def test_parse_disturbance_event_extracts_logged_lifecycle_json():
    line = (
        "[physical_disturbance_injector-12] [INFO] "
        '{"state": "APPLIED", "stage": 3, "monotonic_ns": 1200000000}'
    )
    assert MODULE.parse_disturbance_event(line) == {
        "state": "APPLIED",
        "stage": 3,
        "monotonic_ns": 1200000000,
    }
    assert MODULE.parse_disturbance_event("unrelated output") is None


def test_physical_disturbance_verdict_requires_real_lifecycle_and_fast_stop():
    events = [
        {"state": "APPLIED", "stage": 3, "monotonic_ns": 1_000_000_000},
        {"state": "CLEARED", "stage": 3, "monotonic_ns": 1_100_000_000},
    ]
    result = {
        "final_state": "FAILED",
        "failure_reason": (
            "servo_place_descent watchdog: payload drift=0.020m exceeds 0.015m"
        ),
        "servo_watchdog_abort_stage": 3,
        "servo_watchdog_abort_stop_monotonic_ns": 1_080_000_000,
    }
    verdict = MODULE.physical_disturbance_safe_stop_verdict(
        events, result, expected_stage=3, stop_latency_limit_s=0.15
    )
    assert verdict["passed"] is True
    assert abs(verdict["stop_latency_s"] - 0.08) < 1e-9

    assert MODULE.physical_disturbance_safe_stop_verdict(
        events[:1], result, expected_stage=3, stop_latency_limit_s=0.15
    )["passed"] is False
    assert MODULE.physical_disturbance_safe_stop_verdict(
        events, result, expected_stage=2, stop_latency_limit_s=0.15
    )["passed"] is False
    assert MODULE.physical_disturbance_safe_stop_verdict(
        events, result, expected_stage=3, stop_latency_limit_s=0.05
    )["passed"] is False


def test_physical_disturbance_stages_builds_single_or_full_matrix():
    assert MODULE.physical_disturbance_stages(False, 6, 3) == (3,)
    assert MODULE.physical_disturbance_stages(True, 4, 2) == (1, 2, 3, 4)


def test_diagnostic_case_key_and_joint_cell_support_csv_resume():
    row = {
        "pair": "2",
        "mode": "loaded",
        "place_descent_backend": "servo",
        "servo_fault_injection_type": "none",
        "servo_fault_injection_stage": "2",
        "physical_disturbance_enabled": True,
        "physical_disturbance_stage": "4",
        "physical_disturbance_force_x_n": "0",
        "physical_disturbance_force_y_n": "-16",
        "physical_disturbance_force_z_n": "0",
        "physical_disturbance_duration_s": "0.1",
        "physical_disturbance_delay_s": "0.5",
    }
    assert MODULE.diagnostic_case_key(row) == (
        2,
        "loaded",
        "servo",
        "none",
        2,
        True,
        4,
        0.0,
        -16.0,
        0.0,
        0.1,
        0.5,
    )
    assert MODULE.decoded_joint_positions("[0, 1, 2, 3, 4, 5, 6]") == [
        0.0,
        1.0,
        2.0,
        3.0,
        4.0,
        5.0,
        6.0,
    ]
    assert MODULE.decoded_joint_positions("[0, 1]") is None


def test_paired_checkpoint_binds_arguments_and_source(tmp_path, monkeypatch):
    source = {"schema_version":1, "sha256":"a"*64, "files":{"runner.py":"b"*64}}
    monkeypatch.setattr(MODULE, "paired_source_manifest", lambda:source)
    checkpoint = tmp_path / "payload_transfer_comparison.csv"
    args = argparse.Namespace(output_dir=tmp_path, resume_existing=False,
                              report_only_csv=None, runs_per_mode=1,
                              modes=["unloaded", "loaded"])
    MODULE.guard_paired_checkpoint(args, checkpoint)
    assert (tmp_path / "experiment_config.json").exists()
    assert (tmp_path / "source_manifest.json").exists()
    checkpoint.write_text("evidence")
    with pytest.raises(ValueError, match="already exists"):
        MODULE.guard_paired_checkpoint(args, checkpoint)
    args.resume_existing = True
    MODULE.guard_paired_checkpoint(args, checkpoint)
    args.runs_per_mode = 2
    with pytest.raises(ValueError, match="differs"):
        MODULE.guard_paired_checkpoint(args, checkpoint)
    assert checkpoint.read_text() == "evidence"


def test_physical_disturbance_forces_builds_opposite_direction_pair():
    assert MODULE.physical_disturbance_forces(False, 0.0, 4.0, 1.0) == (
        (0.0, 4.0, 1.0),
    )
    assert MODULE.physical_disturbance_forces(True, 0.0, 4.0, 1.0) == (
        (0.0, 4.0, 1.0),
        (-0.0, -4.0, -1.0),
    )


@pytest.mark.parametrize("second_force", [-16.0, 16.0])
def test_trial_attempts_never_reuse_artifact_paths(tmp_path, monkeypatch, second_force):
    world_paths = []

    class StopBeforeLaunch(Exception):
        pass

    def capture_world(path, mode):
        world_paths.append(path)
        path.write_text("preserved evidence", encoding="utf-8")
        raise StopBeforeLaunch

    monkeypatch.setattr(MODULE, "prepare_world", capture_world)
    options = dict(
        trial=13,
        pair=2,
        mode="loaded",
        planner_id="RRTConnectkConfigDefault",
        timeout_s=180.0,
        output_dir=tmp_path,
        record_bag=False,
        grasp_height_offset_m=0.105,
        gripper_closed_position_m=0.022,
        payload_transfer_velocity_scaling_factor=0.01,
        place_descent_velocity_scaling_factor=0.03,
        place_descent_recovery_velocity_scaling_factor=0.03,
        place_descent_stage_count=6,
        place_descent_backend="servo",
        forced_grasp_yaw_offset_deg=0.0,
        grasp_prevalidation_max_rounds=1,
        pre_place_joint_replay_positions=None,
        enable_physical_disturbance=True,
        physical_disturbance_stage=1,
    )
    for force in (16.0, second_force):
        with pytest.raises(StopBeforeLaunch):
            MODULE.run_trial(**options, physical_disturbance_force_y_n=force)
    assert world_paths[0] != world_paths[1]
    for path in world_paths:
        assert path.read_text(encoding="utf-8") == "preserved evidence"
        assert path.is_relative_to(tmp_path)
        attempt = path.parent.parent
        assert (attempt / "telemetry").is_dir()
        assert (attempt / "logs").is_dir()
