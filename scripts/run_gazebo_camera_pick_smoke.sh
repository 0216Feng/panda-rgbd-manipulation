#!/usr/bin/env bash

set -o pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
log_path="${1:-/tmp/panda_camera_pick.log}"
timeout_s="${CAMERA_PICK_TIMEOUT_S:-180}"
launch_pid=""

cleanup() {
  if [[ -n "${launch_pid}" ]] && kill -0 "${launch_pid}" 2>/dev/null; then
    kill -INT "${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

cd "${repo_root}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

timeout --signal=INT "${timeout_s}s" \
  ros2 launch panda_manipulation gazebo_pick.launch.py \
  use_camera_perception:=true \
  gui:=false \
  rviz:=false \
  >"${log_path}" 2>&1 &
launch_pid=$!

sleep 15
echo "PICK_VALIDATION"
timeout "${timeout_s}s" \
  ros2 topic echo /gazebo_pick_validation --once --full-length || true
echo "PICK_RESULT"
timeout 5s ros2 topic echo /pick_plan_result --once --full-length || true
echo "LOG_SUMMARY"
grep -E "aruco|final_state|FAILED|ERROR|process has died" "${log_path}" \
  | tail -n 80 \
  || true
