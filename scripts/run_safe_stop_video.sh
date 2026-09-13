#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT_DIR="${1:-${ROOT_DIR}/artifacts/runs/safe_stop_video}"
RAW_VIDEO="${OUTPUT_DIR}/safe_stop_raw.mp4"
PUBLIC_VIDEO="${OUTPUT_DIR}/safe_stop.mp4"
POSTER="${OUTPUT_DIR}/safe_stop_poster.png"
VIDEO_METADATA="${OUTPUT_DIR}/video_metadata.json"

cd "${ROOT_DIR}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
mkdir -p "${OUTPUT_DIR}"

base_domain_id="${ROS_DOMAIN_ID:-0}"
if ! [[ "${base_domain_id}" =~ ^[0-9]+$ ]]; then
  base_domain_id=0
fi

python3 scripts/run_payload_transfer_diagnostics.py \
  --runs-per-mode 1 \
  --modes loaded \
  --place-descent-backend servo \
  --physical-disturbance \
  --physical-disturbance-stage 3 \
  --physical-disturbance-delay-s 0.5 \
  --physical-disturbance-duration-s 0.10 \
  --physical-disturbance-force-y-n 16.0 \
  --physical-disturbance-stop-latency-limit-s 0.2 \
  --no-rosbag \
  --timeout-s 420 \
  --output-dir "${OUTPUT_DIR}" &
runner_pid=$!

# run_payload_transfer_diagnostics isolates trial 1 with this formula.
trial_domain_id=$(( (base_domain_id + runner_pid + 1) % 101 ))
trial_partition="payload_diagnostics_${runner_pid}_1"
echo "Recording safe-stop trial ROS_DOMAIN_ID=${trial_domain_id} to ${RAW_VIDEO}"

GZ_PARTITION="${trial_partition}" ROS_DOMAIN_ID="${trial_domain_id}" \
  ros2 run ros_gz_bridge parameter_bridge \
  "/showcase_camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image" \
  >"${OUTPUT_DIR}/showcase_bridge.log" 2>&1 &
bridge_pid=$!

ROS_DOMAIN_ID="${trial_domain_id}" python3 scripts/record_gazebo_camera.py \
  --output "${RAW_VIDEO}" \
  --image-topic "/showcase_camera/image_raw" \
  --result-topic "/pick_plan_result" \
  --timeout-s 420 \
  --post-result-s 3 &
recorder_pid=$!

runner_status=0
wait "${runner_pid}" || runner_status=$?

kill -INT "${recorder_pid}" 2>/dev/null || true
recorder_status=0
wait "${recorder_pid}" || recorder_status=$?
kill -TERM "${bridge_pid}" 2>/dev/null || true
wait "${bridge_pid}" 2>/dev/null || true

if [[ ${runner_status} -ne 0 ]]; then
  echo "Safe-stop runner failed with status ${runner_status}." >&2
  exit "${runner_status}"
fi
if [[ ${recorder_status} -ne 0 ]]; then
  echo "Camera recorder failed with status ${recorder_status}." >&2
  exit "${recorder_status}"
fi

python3 - "${OUTPUT_DIR}/physical_disturbance_safety.json" <<'PY'
import json
import sys
from pathlib import Path

report = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
passed = (
    report.get("total_trials") == 1
    and report.get("safe_stops") == 1
    and report.get("disturbances_applied") == 1
    and report.get("disturbances_cleared") == 1
    and report.get("all_meet_strictest_stop_latency_limit") is True
)
if not passed:
    raise SystemExit("safe-stop evidence did not meet the release gate")
PY

ffmpeg_bin="$(command -v ffmpeg || true)"
if [[ -z "${ffmpeg_bin}" ]] && command -v cmd.exe >/dev/null 2>&1; then
  windows_ffmpeg="$(cmd.exe /d /c where ffmpeg 2>/dev/null | tr -d '\r' | head -n 1)"
  if [[ -n "${windows_ffmpeg}" ]]; then
    ffmpeg_bin="$(wslpath "${windows_ffmpeg}")"
  fi
fi
if [[ -z "${ffmpeg_bin}" ]] && command -v cmd.exe >/dev/null 2>&1; then
  for packages_dir in \
    /mnt/c/Users/*/AppData/Local/Microsoft/WinGet/Packages
  do
    [[ -d "${packages_dir}" ]] || continue
    ffmpeg_bin="$(
      find "${packages_dir}" -maxdepth 4 -type f -iname ffmpeg.exe -print -quit
    )"
    [[ -z "${ffmpeg_bin}" ]] || break
  done
fi

if [[ -n "${ffmpeg_bin}" ]]; then
  "${ffmpeg_bin}" -y -loglevel warning -i "${RAW_VIDEO}" \
    -vf "setpts=0.5*PTS,scale=960:540:flags=lanczos" \
    -an -c:v libx264 -crf 22 -pix_fmt yuv420p -movflags +faststart \
    "${PUBLIC_VIDEO}"
  "${ffmpeg_bin}" -y -loglevel warning -ss 12 -i "${PUBLIC_VIDEO}" \
    -frames:v 1 -update 1 "${POSTER}"
  python3 - "${PUBLIC_VIDEO}" "${VIDEO_METADATA}" <<'PY'
import hashlib
import json
import sys
from pathlib import Path

import cv2

video = Path(sys.argv[1])
output = Path(sys.argv[2])
capture = cv2.VideoCapture(str(video))
fps = float(capture.get(cv2.CAP_PROP_FPS))
frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
metadata = {
    "codec": "h264",
    "duration_s": frames / fps if fps > 0 else None,
    "frame_rate_fps": fps,
    "frames": frames,
    "height": int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
    "playback_speed": 2.0,
    "sha256": hashlib.sha256(video.read_bytes()).hexdigest(),
    "width": int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
}
capture.release()
output.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n")
PY
  echo "Safe-stop H.264 video ready: ${PUBLIC_VIDEO}"
else
  echo "ffmpeg unavailable; raw video retained: ${RAW_VIDEO}" >&2
fi

echo "Safe-stop evidence: ${OUTPUT_DIR}/physical_disturbance_safety.md"
