import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[3]
SETUP = ROOT / "scripts" / "setup_wsl_ros2_jazzy.sh"
SMOKE = ROOT / "scripts" / "check_ros_graph_smoke.py"
RELEASE = ROOT / "scripts" / "run_release_candidate_validation.sh"
V1_RELEASE = ROOT / "scripts" / "run_v1_release_validation.sh"
SAFE_STOP_VIDEO = ROOT / "scripts" / "run_safe_stop_video.sh"
SPEC = importlib.util.spec_from_file_location("ci_graph_smoke", SMOKE)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_graph_smoke_requires_complete_success_history():
    report = {"final_state": "SUCCESS", "history": list(MODULE.REQUIRED_STAGES)}
    assert MODULE.valid_task_report(report)
    assert not MODULE.valid_task_report({**report, "history": ["SUCCESS"]})
    assert not MODULE.valid_task_report({**report, "failure_reason": "error"})
    assert not MODULE.valid_task_report({**report, "failures": {"APPROACH": 1}})
    assert not MODULE.valid_task_report({**report, "final_state": "FAILED"})
    assert not MODULE.valid_task_report(None)


@pytest.mark.parametrize("value", ["0", "-1", "nan", "inf"])
def test_graph_smoke_rejects_unbounded_timeout(value):
    import sys
    result = subprocess.run(
        [sys.executable, str(SMOKE), "--timeout-s", value],
        capture_output=True, text=True, timeout=5,
    )
    assert result.returncode == 2
    assert "finite and positive" in result.stderr


@pytest.fixture
def setup_environment(tmp_path):
    if os.name != "posix" or not Path("/opt/ros/jazzy/setup.bash").is_file():
        pytest.skip("shell integration tests require the supported ROS2 Jazzy environment")
    checkout = tmp_path / "checkout with spaces"
    (checkout / "scripts").mkdir(parents=True)
    (checkout / "src" / "panda_manipulation").mkdir(parents=True)
    (checkout / "src" / "panda_manipulation" / "package.xml").write_text("<package/>")
    shutil.copy2(SETUP, checkout / "scripts" / SETUP.name)
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    commands = {
        "sudo": 'printf "sudo %s\\n" "$*" >> "$CALL_LOG"\n',
        "lsb_release": 'printf "noble\\n"\n',
        "sleep": 'exit 0\n',
        "colcon": 'printf "colcon %s\\n" "$*" >> "$CALL_LOG"\n',
        "rosdep": (
            'printf "rosdep %s\\n" "$*" >> "$CALL_LOG"\n'
            'if [[ "$1" == "${FAIL_ROSDEP_STEP:-none}" ]]; then exit 9; fi\n'
        ),
    }
    for name, body in commands.items():
        path = bin_dir / name
        path.write_text("#!/usr/bin/env bash\n" + body, encoding="utf-8")
        path.chmod(0o755)
    home = tmp_path / "home"
    home.mkdir()
    (home / ".bashrc").write_text("# untouched\n")
    environment = os.environ.copy()
    environment.pop("REPO_SOURCE_DIR", None)
    environment.update(
        HOME=str(home), PATH=f"{bin_dir}:{environment['PATH']}",
        CALL_LOG=str(tmp_path / "calls.log"), ROS_DISTRO_EXPECTED="jazzy",
        WORKSPACE_DIR=str(tmp_path / "external workspace"),
    )
    return checkout, environment


def run_setup(checkout, environment):
    return subprocess.run(
        ["bash", str(checkout / "scripts" / SETUP.name)],
        cwd=checkout.parent, env=environment, capture_output=True, text=True, timeout=20,
    )


@pytest.mark.parametrize("in_place", [False, True])
def test_setup_resolves_checkout_independently_of_cwd(setup_environment, in_place):
    checkout, environment = setup_environment
    if in_place:
        environment["WORKSPACE_DIR"] = str(checkout)
    result = run_setup(checkout, environment)
    assert result.returncode == 0, result.stderr
    project_link = Path(environment["WORKSPACE_DIR"]) / "src" / "panda_manipulation_project"
    if in_place:
        assert not project_link.exists()
    else:
        assert project_link.resolve() == checkout.resolve()
    assert Path(environment["HOME"], ".bashrc").read_text() == "# untouched\n"
    assert "colcon build" in Path(environment["CALL_LOG"]).read_text()


@pytest.mark.parametrize("step", ["update", "install"])
def test_dependency_failure_never_reaches_build(setup_environment, step):
    checkout, environment = setup_environment
    environment["FAIL_ROSDEP_STEP"] = step
    result = run_setup(checkout, environment)
    assert result.returncode != 0
    assert "Setup complete" not in result.stdout
    calls = Path(environment["CALL_LOG"]).read_text().splitlines()
    assert not any(call.startswith("colcon ") for call in calls)


def test_setup_rejects_an_existing_different_checkout(setup_environment):
    checkout, environment = setup_environment
    link = Path(environment["WORKSPACE_DIR"], "src", "panda_manipulation_project")
    link.parent.mkdir(parents=True)
    other = checkout.parent / "other"
    other.mkdir()
    link.symlink_to(other)
    result = run_setup(checkout, environment)
    assert result.returncode != 0
    assert "not replacing" in result.stderr
    assert link.resolve() == other.resolve()


def test_setup_rejects_recursive_workspace_link(setup_environment):
    checkout, environment = setup_environment
    environment["WORKSPACE_DIR"] = str(checkout / "nested workspace")
    result = run_setup(checkout, environment)
    assert result.returncode != 0
    assert "nested inside" in result.stderr
    assert not Path(environment["WORKSPACE_DIR"], "src", "panda_manipulation_project").exists()


def test_container_and_ci_keep_software_checks_distinct_from_physics():
    dockerfile = (ROOT / "Dockerfile").read_text()
    assert "|| true" not in dockerfile
    assert "COPY . /workspace\n" in dockerfile
    assert "ros-jazzy-gz-ros2-control" in dockerfile
    assert '--skip-keys "ament_python moveit_servo"' in dockerfile
    ignore = (ROOT / ".dockerignore").read_text().splitlines()
    assert {"**/build", "**/install", "**/log", ".git", "artifacts/*"} <= set(ignore)
    assert {
        "!artifacts/README.md",
        "!artifacts/README.zh-CN.md",
        "!artifacts/baselines/",
        "!artifacts/baselines/**",
        "artifacts/baselines/**/*_logs",
        "artifacts/baselines/**/*.log",
        "artifacts/baselines/**/worlds",
    } <= set(ignore)
    workflow = yaml.load((ROOT / ".github/workflows/ros2-ci.yml").read_text(), Loader=yaml.BaseLoader)
    assert set(workflow["on"]) == {"push", "pull_request", "workflow_dispatch"}
    assert workflow["permissions"] == {"contents": "read"}
    steps = workflow["jobs"]["software"]["steps"]
    commands = "\n".join(step.get("run", "") for step in steps)
    assert "docker build" in commands
    assert "SKIP_PHYSICAL=1" in commands
    assert "check_ros_graph_smoke.py" in commands
    container = json.loads((ROOT / ".devcontainer/devcontainer.json").read_text())
    assert "--privileged" not in container["runArgs"]
    assert "${localWorkspaceFolder}" in container["workspaceMount"]


def test_release_validation_uses_reproducible_obstacle_profile_and_unique_output():
    script = RELEASE.read_text(encoding="utf-8")
    assert "release_candidate_${RUN_ID}" in script
    assert 'mkdir -p "${OUTPUT_DIR}/rgbd"' in script
    assert '"${OUTPUT_DIR}/obstacle/results.csv"' in script
    for argument in (
        "--challenge-scenario center_tall_barrier",
        "--rgbd-perception",
        "--grasp-height-offset 0.115",
        "--grasp-pitch-offset-deg -20",
        "--loaded-path-precheck",
        "--transfer-diagnostics",
    ):
        assert argument in script
    assert "--csv release_rgbd_physical.csv" not in script
    assert "--csv release_obstacle_physical.csv" not in script


def test_v1_release_runner_enforces_frozen_full_profile_and_all_gates():
    script = V1_RELEASE.read_text(encoding="utf-8")
    assert 'PROFILE="${V1_PROFILE:-full}"' in script
    assert 'FIXED_TRIALS="${FIXED_TRIALS:-10}"' in script
    assert 'RANDOM_TRIALS="${RANDOM_TRIALS:-20}"' in script
    assert "git rev-parse --verify HEAD" in script
    assert "git status --porcelain --untracked-files=normal" in script
    for required in (
        "fixed_rgbd/results.csv",
        "random_rgbd/results.csv",
        "static_obstacles/results.csv",
        "safe_stop/physical_disturbance_safety.json",
        "evaluate_v1_release.py",
    ):
        assert required in script


def test_safe_stop_video_uses_real_wrench_and_pipeline_result():
    script = SAFE_STOP_VIDEO.read_text(encoding="utf-8")
    assert "--physical-disturbance" in script
    assert "--physical-disturbance-force-y-n 16.0" in script
    assert "--physical-disturbance-stop-latency-limit-s 0.2" in script
    assert 'trial_partition="payload_diagnostics_${runner_pid}_1"' in script
    assert '--result-topic "/pick_plan_result"' in script
    assert "all_meet_strictest_stop_latency_limit" in script
    assert "cmd.exe /d /c where ffmpeg" in script
    assert 'wslpath "${windows_ffmpeg}"' in script
    assert "for packages_dir in" in script
    assert "find \"${packages_dir}\" -maxdepth 4" in script
    assert 'POSTER="${OUTPUT_DIR}/safe_stop_poster.png"' in script
    assert 'VIDEO_METADATA="${OUTPUT_DIR}/video_metadata.json"' in script
    assert "hashlib.sha256(video.read_bytes()).hexdigest()" in script


def test_public_safe_stop_video_matches_strict_source_bound_evidence():
    evidence = (
        ROOT
        / "artifacts"
        / "baselines"
        / "v1_candidate_20260913"
        / "safe_stop_video"
    )
    video = ROOT / "docs" / "assets" / "demo" / "gazebo_safe_stop.mp4"
    poster = ROOT / "docs" / "assets" / "demo" / "gazebo_safe_stop_poster.png"
    metadata = json.loads((evidence / "video_metadata.json").read_text())
    safety = json.loads(
        (evidence / "physical_disturbance_safety.json").read_text()
    )
    assert video.stat().st_size > 100_000
    assert poster.stat().st_size > 50_000
    assert hashlib.sha256(video.read_bytes()).hexdigest() == metadata["sha256"]
    assert metadata["codec"] == "h264"
    assert safety["total_trials"] == safety["safe_stops"] == 1
    assert safety["disturbances_applied"] == safety["disturbances_cleared"] == 1
    assert safety["all_meet_strictest_stop_latency_limit"] is True
    assert safety["max_stop_latency_s"] <= safety["stop_latency_limit_s"]


def test_public_docs_have_language_switches_and_pairs():
    pairs = (
        (ROOT / "README.md", ROOT / "README.zh-CN.md"),
        (ROOT / "docs" / "README.md", ROOT / "docs" / "README.zh-CN.md"),
        (ROOT / "artifacts" / "README.md", ROOT / "artifacts" / "README.zh-CN.md"),
        (ROOT / "docs" / "architecture.md", ROOT / "docs" / "architecture.zh-CN.md"),
        (ROOT / "docs" / "reproduction.md", ROOT / "docs" / "reproduction.zh-CN.md"),
        (ROOT / "docs" / "demo_script.md", ROOT / "docs" / "demo_script.zh-CN.md"),
        (
            ROOT / "docs" / "github_release_readiness.md",
            ROOT / "docs" / "github_release_readiness.zh-CN.md",
        ),
        (ROOT / "docs" / "roadmap.md", ROOT / "docs" / "roadmap.zh-CN.md"),
        (
            ROOT / "docs" / "assets" / "demo" / "README.md",
            ROOT / "docs" / "assets" / "demo" / "README.zh-CN.md",
        ),
        (
            ROOT / "artifacts" / "baselines" / "v1.0.0" / "README.md",
            ROOT / "artifacts" / "baselines" / "v1.0.0" / "README.zh-CN.md",
        ),
    )
    for english, chinese in pairs:
        assert english.is_file(), f"missing English document: {english.relative_to(ROOT)}"
        assert chinese.is_file(), f"missing Chinese document: {chinese.relative_to(ROOT)}"
        for document in (english, chinese):
            text = document.read_text(encoding="utf-8")
            assert "English" in text, f"missing English switch in {document.relative_to(ROOT)}"
            assert "简体中文" in text, f"missing Chinese switch in {document.relative_to(ROOT)}"
            assert f"]({english.name})" in text
            assert f"]({chinese.name})" in text
