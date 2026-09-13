#!/usr/bin/env bash

set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
exec ros2 launch panda_manipulation rgbd_markerless_pick.launch.py "$@"
