#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RGBD_TRIALS="${RGBD_TRIALS:-1}"
OBSTACLE_TRIALS="${OBSTACLE_TRIALS:-1}"
RUN_ID="${RELEASE_RUN_ID:-$(date -u +%Y%m%dT%H%M%SZ)}"
OUTPUT_DIR="${RELEASE_OUTPUT_DIR:-${ROOT_DIR}/artifacts/runs/release_candidate_${RUN_ID}}"

source /opt/ros/jazzy/setup.bash
cd "${ROOT_DIR}"
mkdir -p "${OUTPUT_DIR}/rgbd" "${OUTPUT_DIR}/obstacle"

colcon build --symlink-install --packages-select \
  panda_manipulation panda_manipulation_cpp
source install/setup.bash
set -u
colcon test --packages-select \
  panda_manipulation panda_manipulation_cpp \
  --event-handlers console_direct+
colcon test-result --verbose

if [[ "${SKIP_PHYSICAL:-0}" == "1" ]]; then
  echo "Release-candidate software checks passed; physical checks skipped."
  exit 0
fi

python3 scripts/run_gazebo_physics_benchmark.py \
  --trials "${RGBD_TRIALS}" \
  --rgbd-perception \
  --timeout-s 300 \
  --csv "${OUTPUT_DIR}/rgbd/results.csv" \
  --markdown "${OUTPUT_DIR}/rgbd/report.md" \
  --log-dir "${OUTPUT_DIR}/rgbd/logs"

python3 scripts/run_gazebo_physics_benchmark.py \
  --trials "${OBSTACLE_TRIALS}" \
  --challenge-obstacles \
  --challenge-scenario center_tall_barrier \
  --rgbd-perception \
  --planner-id RRTConnectkConfigDefault \
  --grasp-height-offset 0.115 \
  --grasp-pitch-offset-deg -20 \
  --loaded-path-precheck \
  --transfer-diagnostics \
  --timeout-s 420 \
  --csv "${OUTPUT_DIR}/obstacle/results.csv" \
  --markdown "${OUTPUT_DIR}/obstacle/report.md" \
  --log-dir "${OUTPUT_DIR}/obstacle/logs"

echo "Release-candidate software and physical checks passed."
echo "Reports: ${OUTPUT_DIR}"
