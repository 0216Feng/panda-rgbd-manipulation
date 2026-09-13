#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT_DIR="${1:-${ROOT_DIR}/artifacts/runs/gazebo_camera_video}"
RAW_VIDEO="${OUTPUT_DIR}/gazebo_rgb_camera_raw.mp4"

cd "${ROOT_DIR}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
mkdir -p "${OUTPUT_DIR}"

base_domain_id="${ROS_DOMAIN_ID:-0}"
if ! [[ "${base_domain_id}" =~ ^[0-9]+$ ]]; then
  base_domain_id=0
fi

python3 scripts/run_gazebo_physics_benchmark.py \
  --trials 1 \
  --challenge-obstacles \
  --challenge-scenario center_tall_barrier \
  --rgbd-perception \
  --planner-id RRTConnectkConfigDefault \
  --grasp-height-offset 0.115 \
  --grasp-pitch-offset-deg -20 \
  --loaded-path-precheck \
  --transfer-diagnostics \
  --timeout-s 360 \
  --csv "${OUTPUT_DIR}/results.csv" \
  --markdown "${OUTPUT_DIR}/report.md" \
  --svg "${OUTPUT_DIR}/report.svg" \
  --log-dir "${OUTPUT_DIR}/logs" &
benchmark_pid=$!

# The benchmark isolates trial 1 with this exact domain formula.
trial_domain_id=$(( (base_domain_id + benchmark_pid + 1) % 101 ))
trial_partition="panda_benchmark_${benchmark_pid}_1"
echo "Recording trial ROS_DOMAIN_ID=${trial_domain_id} to ${RAW_VIDEO}"

GZ_PARTITION="${trial_partition}" ROS_DOMAIN_ID="${trial_domain_id}" \
  ros2 run ros_gz_bridge parameter_bridge \
  "/showcase_camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image" \
  >"${OUTPUT_DIR}/showcase_bridge.log" 2>&1 &
bridge_pid=$!

ROS_DOMAIN_ID="${trial_domain_id}" python3 scripts/record_gazebo_camera.py \
  --output "${RAW_VIDEO}" \
  --image-topic "/showcase_camera/image_raw" \
  --timeout-s 360 \
  --post-result-s 3 &
recorder_pid=$!

benchmark_status=0
wait "${benchmark_pid}" || benchmark_status=$?

# A failed trial can end before publishing validation. SIGINT still finalizes MP4.
kill -INT "${recorder_pid}" 2>/dev/null || true
recorder_status=0
wait "${recorder_pid}" || recorder_status=$?
kill -TERM "${bridge_pid}" 2>/dev/null || true
wait "${bridge_pid}" 2>/dev/null || true

if [[ ${benchmark_status} -ne 0 ]]; then
  echo "Gazebo benchmark failed with status ${benchmark_status}." >&2
  exit "${benchmark_status}"
fi
if [[ ${recorder_status} -ne 0 ]]; then
  echo "Camera recorder failed with status ${recorder_status}." >&2
  exit "${recorder_status}"
fi

echo "Gazebo camera video ready: ${RAW_VIDEO}"
