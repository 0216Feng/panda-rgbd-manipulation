"""Verify diagnostic launch wiring without starting ROS or Gazebo."""

import importlib.util
import argparse
from pathlib import Path

import pytest


def test_checkpoint_config_rejects_changed_parameters(runner, tmp_path):
    args = argparse.Namespace(csv=str(tmp_path / "results.csv"), resume=False,
                              grasp_pitch_offset_deg=-15, loaded_path_precheck=True)
    runner.guard_checkpoint_config(args)
    source = tmp_path / "results.csv.source.json"
    assert source.exists()
    source_data = runner.json.loads(source.read_text())
    assert len(source_data["sha256"]) == 64
    assert all(not Path(name).is_absolute() for name in source_data["files"])
    checkpoint = Path(args.csv)
    checkpoint.write_text("original results")
    with pytest.raises(ValueError, match="already exists"):
        runner.guard_checkpoint_config(args)
    args.resume = True
    runner.guard_checkpoint_config(args)
    args.grasp_pitch_offset_deg = -20
    with pytest.raises(ValueError, match="differs"):
        runner.guard_checkpoint_config(args)
    assert checkpoint.read_text() == "original results"


def test_legacy_checkpoint_requires_new_output(runner, tmp_path):
    checkpoint = tmp_path / "old.csv"
    checkpoint.write_text("historical results")
    args = argparse.Namespace(csv=str(checkpoint), resume=True)
    with pytest.raises(ValueError, match="missing"):
        runner.guard_checkpoint_config(args)
    assert checkpoint.read_text() == "historical results"


def test_resume_rejects_source_change(runner, monkeypatch, tmp_path):
    fingerprint = {"schema_version":1, "sha256":"a" * 64, "files":{"a.py":"b" * 64}}
    monkeypatch.setattr(runner, "benchmark_source_manifest", lambda:fingerprint)
    args = argparse.Namespace(csv=str(tmp_path / "results.csv"), resume=False, trials=1)
    runner.guard_checkpoint_config(args)
    Path(args.csv).write_text("results")
    args.resume = True
    fingerprint = {"schema_version":1, "sha256":"c" * 64, "files":{"a.py":"d" * 64}}
    with pytest.raises(ValueError, match="differs"):
        runner.guard_checkpoint_config(args)


@pytest.fixture
def runner(monkeypatch):
    script = Path(__file__).resolve().parents[3] / "scripts/run_gazebo_physics_benchmark.py"
    spec = importlib.util.spec_from_file_location("physics_runner_telemetry_test", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "stop_gazebo_world", lambda _: None)
    return module


def capture_launch(runner, monkeypatch, log_dir, enabled, **options):
    captured = []

    class LaunchCaptured(Exception):
        pass

    def capture(command, **kwargs):
        captured.extend(command)
        raise LaunchCaptured()

    monkeypatch.setattr(runner.subprocess, "Popen", capture)
    with pytest.raises(LaunchCaptured):
        runner.run_trial(1, 30.0, log_dir, 0.55, 0.0,
                         transfer_diagnostics=enabled, **options)
    return captured


def test_diagnostics_disabled_by_default(runner, monkeypatch, tmp_path):
    command = capture_launch(runner, monkeypatch, tmp_path, False)
    assert not any("transfer_diagnostics" in arg for arg in command)
    assert list(tmp_path.iterdir()) == []


def test_diagnostics_repeated_trial_has_unique_paths(runner, monkeypatch, tmp_path):
    commands = [capture_launch(runner, monkeypatch, tmp_path, True) for _ in range(2)]
    paths = []
    for command in commands:
        assert "enable_transfer_diagnostics:=true" in command
        csv = next(arg.split(":=", 1)[1] for arg in command
                   if arg.startswith("transfer_diagnostics_csv_path:="))
        summary = next(arg.split(":=", 1)[1] for arg in command
                       if arg.startswith("transfer_diagnostics_summary_path:="))
        assert Path(csv).parent.is_dir()
        assert Path(csv).parent == Path(summary).parent
        assert Path(csv).parent.parent == tmp_path.resolve()
        paths.append(csv)
    assert paths[0] != paths[1]


def test_loaded_path_switch_and_grasp_offset_are_forwarded(runner, monkeypatch, tmp_path):
    command = capture_launch(runner, monkeypatch, tmp_path, False,
                             loaded_path_precheck=True, grasp_height_offset=0.110,
                             grasp_pitch_offset_deg=10.0,
                             perception_mode="rgbd")
    effective = dict(arg.split(":=", 1) for arg in command if ":=" in arg)
    assert effective["enable_loaded_path_precheck"] == "true"
    assert effective["enable_place_endpoint_precheck"] == "true"
    assert effective["enable_grasp_candidate_prevalidation"] == "true"
    assert float(effective["grasp_height_offset"]) == 0.110
    assert float(effective["grasp_pitch_offset_deg"]) == 10.0


@pytest.mark.parametrize("arguments", [
    ["--grasp-pitch-offset-deg", "10"],
    ["--grasp-pitch-offset-deg", "21", "--loaded-path-precheck"],
    ["--grasp-pitch-offset-deg", "nan", "--loaded-path-precheck"],
])
def test_pitch_requires_bounded_angle_and_complete_precheck(runner, monkeypatch, arguments):
    monkeypatch.setattr(runner.sys, "argv", ["benchmark"] + arguments)
    with pytest.raises(SystemExit) as exc:
        runner.main()
    assert exc.value.code == 2


@pytest.mark.parametrize("value", ["0", "-0.1", "nan", "inf"])
def test_invalid_grasp_offset_rejected_before_launch(runner, monkeypatch, value):
    monkeypatch.setattr(runner.sys, "argv", ["benchmark", "--grasp-height-offset", value])
    with pytest.raises(SystemExit) as exc:
        runner.main()
    assert exc.value.code == 2
