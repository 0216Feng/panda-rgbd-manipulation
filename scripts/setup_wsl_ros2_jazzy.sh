#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO_EXPECTED="${ROS_DISTRO_EXPECTED:-jazzy}"
SCRIPT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/robot_ws}"
REPO_SOURCE_DIR="${REPO_SOURCE_DIR:-$SCRIPT_ROOT}"

if [[ "${ROS_DISTRO_EXPECTED}" != "jazzy" ]]; then
  echo "ERROR: this dependency installer supports ROS2 Jazzy only." >&2
  exit 1
fi
if [[ ! -f "${REPO_SOURCE_DIR}/src/panda_manipulation/package.xml" ]]; then
  echo "ERROR: REPO_SOURCE_DIR must point to the full project checkout." >&2
  exit 1
fi
REPO_SOURCE_DIR="$(cd "${REPO_SOURCE_DIR}" && pwd -P)"

if ! command -v lsb_release >/dev/null 2>&1; then
  sudo apt-get update
  sudo apt-get install -y lsb-release
fi

UBUNTU_CODENAME="$(lsb_release -cs)"
if [[ "$UBUNTU_CODENAME" != "noble" ]]; then
  echo "ERROR: ROS2 Jazzy targets Ubuntu 24.04 (noble). Current Ubuntu codename: $UBUNTU_CODENAME" >&2
  exit 1
fi

if [[ ! -f "/opt/ros/${ROS_DISTRO_EXPECTED}/setup.bash" ]]; then
  echo "ERROR: /opt/ros/${ROS_DISTRO_EXPECTED}/setup.bash not found. Install ROS2 Jazzy first." >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-opencv \
  python3-pip \
  python3-rosdep \
  python3-vcstool \
  ros-jazzy-cv-bridge \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-moveit \
  ros-jazzy-moveit-resources-panda-description \
  ros-jazzy-moveit-resources-panda-moveit-config \
  ros-jazzy-ros-gz \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-tf-transformations

if [[ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi

rosdep_update_ok=0
for attempt in 1 2 3; do
  if rosdep update --rosdistro "${ROS_DISTRO_EXPECTED}"; then
    rosdep_update_ok=1
    break
  fi
  echo "rosdep update attempt ${attempt} failed; retrying in 5 seconds..." >&2
  sleep 5
done

if [[ "${rosdep_update_ok}" != "1" ]]; then
  echo "ERROR: rosdep update failed; dependency installation is incomplete." >&2
  exit 1
fi

mkdir -p "${WORKSPACE_DIR}/src"

WORKSPACE_DIR="$(cd "${WORKSPACE_DIR}" && pwd -P)"
if [[ "${WORKSPACE_DIR}" == "${REPO_SOURCE_DIR}/"* ]]; then
  echo "ERROR: an external workspace cannot be nested inside the source checkout." >&2
  exit 1
fi
if [[ "${WORKSPACE_DIR}" != "${REPO_SOURCE_DIR}" ]]; then
  project_link="${WORKSPACE_DIR}/src/panda_manipulation_project"
  if [[ -e "${project_link}" || -L "${project_link}" ]]; then
    if [[ "$(readlink -f "${project_link}")" != "${REPO_SOURCE_DIR}" ]]; then
      echo "ERROR: ${project_link} already points to another checkout; not replacing it." >&2
      exit 1
    fi
  else
    ln -s "${REPO_SOURCE_DIR}" "${project_link}"
  fi
fi

set +u
source "/opt/ros/${ROS_DISTRO_EXPECTED}/setup.bash"
set -u
cd "${WORKSPACE_DIR}"
rosdep install --from-paths src --ignore-src --rosdistro "${ROS_DISTRO_EXPECTED}" -y
colcon build --symlink-install

echo
echo "Setup complete."
echo "Shell startup files were not modified. Source the environment in each new terminal."
echo "Next commands:"
printf '  source %q\n' "/opt/ros/${ROS_DISTRO_EXPECTED}/setup.bash"
printf '  source %q\n' "${WORKSPACE_DIR}/install/setup.bash"
echo "  ros2 launch panda_manipulation demo.launch.py use_synthetic_pose:=true dry_run:=true"
echo "  ros2 launch panda_manipulation moveit_scene.launch.py start_pipeline:=true use_synthetic_pose:=true dry_run:=true"
