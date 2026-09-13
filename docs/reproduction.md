[![English](https://img.shields.io/badge/lang-English-2563eb.svg)](reproduction.md) [![简体中文](https://img.shields.io/badge/lang-简体中文-d73a49.svg)](reproduction.zh-CN.md)

# Reproduction and Software CI

## Supported environment

- Ubuntu 24.04 with ROS2 Jazzy installed, including WSL2 Ubuntu.
- All commands below run from the full repository root, not a forwarding-script directory.
- The installation script uses sudo for apt and initial rosdep configuration. It never changes shell startup files.
- Hardware control is not supported by this reproduction procedure. Physical results refer to Gazebo simulation.

## Native setup

```bash
WORKSPACE_DIR="$PWD" bash scripts/setup_wsl_ros2_jazzy.sh
source /opt/ros/jazzy/setup.bash
source install/setup.bash
SKIP_PHYSICAL=1 bash scripts/run_release_candidate_validation.sh
python3 scripts/check_ros_graph_smoke.py
bash scripts/run_rgbd_markerless_pick.sh
```

The first command installs dependencies and builds in the checkout. Dependency download or installation failures stop setup; fix the reported error before retrying. No successful build is claimed after a failed rosdep update.

For an external workspace, omit `WORKSPACE_DIR` to retain the existing `$HOME/robot_ws` default, or set it explicitly. The source checkout is detected from the script location; `REPO_SOURCE_DIR` remains available as an explicit override. Source that workspace's `install/setup.bash` and run repository scripts from the checkout. Existing links to other checkouts are rejected without replacement. Do not create an external workspace inside the source checkout.

## Docker and devcontainer

```bash
docker build -t panda-manipulation:dev .
docker run --rm -it panda-manipulation:dev bash
```

Inside the image, `/workspace` is the project root. Run the software validation command and source `/workspace/install/setup.bash` after building. The image installs the dependencies exercised by headless CI and does not silently ignore installation failures. `ament_python` is supplied by the ROS base image. The optional `moveit_servo` runtime is excluded from container rosdep resolution because some public Jazzy apt snapshots do not publish `ros-jazzy-moveit-servo`; Servo demonstrations remain supported and tested in the maintained native WSL2 environment. `.dockerignore` excludes Git metadata, existing builds, local secrets, experiment artifacts and ROS bags from the build context.

The devcontainer binds the current checkout at `/workspace`, so edits target the visible source rather than an old image copy. It no longer requests privileged mode. The default container setup is for headless software work; GUI forwarding is not configured by these commands.

## CI scope

`.github/workflows/ros2-ci.yml` builds this same Dockerfile on an Ubuntu 24.04 GitHub runner, then runs:

1. `colcon build`, the complete package test suite, and `colcon test-result`.
2. An installed ROS graph smoke that observes `/detected_object_pose`, `/grasp_candidates` and `/task_state` in an isolated ROS domain.
3. Test XML and log collection even when a preceding check fails.

The ROS graph smoke uses `use_synthetic_pose:=true` and `dry_run:=true`. Its JSON explicitly reports `physical_execution_verified: false`. A pass proves installed entrypoints and the synthetic topic chain work; it does not prove perception accuracy, motion planning, Gazebo contact or physical grasp success. Each attempt keeps a separate log directory, and timeout returns a nonzero status.

The hosted workflow does not launch MoveIt Servo. Use the native setup for Servo, Gazebo GUI and physical simulation workflows until a supported Servo binary is available in the public Jazzy apt snapshot used by the container build.

The CI workflow is prepared locally. A GitHub-hosted run and a fresh Docker image build must still be observed before claiming clean-environment validation; merely parsing this YAML or passing tests in an existing WSL installation is insufficient.

## Physical release gate

Physical smoke and release experiments remain separate from CI. Follow [roadmap.md](roadmap.md) for the fixed/random RGB-D runs, obstacle cases, physical outcome checks and evidence requirements. Do not use synthetic graph results in the physical success-rate table.

Validate the release infrastructure without making a reliability claim:

```bash
V1_PROFILE=smoke bash scripts/run_v1_release_validation.sh
```

The full command requires an existing commit and a clean working tree before
starting the expensive physical matrix:

```bash
bash scripts/run_v1_release_validation.sh
```

It runs fixed 10, seeded-random 20, three representative static obstacles, and
one real Gazebo wrench safe-stop trial. `evaluate_v1_release.py` rejects missing
samples, mixed package source fingerprints, unblocked obstacle paths, or an
incomplete safe-stop lifecycle, then writes one JSON/Markdown release verdict.

## References

- [Docker build contexts and .dockerignore](https://docs.docker.com/build/concepts/context/)
- [OSRF Jazzy desktop-full image definition](https://github.com/osrf/docker_images/tree/master/ros/jazzy/ubuntu/noble/desktop-full)
- [GitHub checkout action](https://github.com/actions/checkout)
- [GitHub artifact upload action](https://github.com/actions/upload-artifact)
