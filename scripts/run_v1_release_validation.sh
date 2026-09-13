#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROFILE="${V1_PROFILE:-full}"
RUN_ID="${V1_RUN_ID:-$(date -u +%Y%m%dT%H%M%SZ)}"
OUTPUT_DIR="${V1_OUTPUT_DIR:-${ROOT_DIR}/artifacts/runs/v1_${PROFILE}_${RUN_ID}}"

case "${PROFILE}" in
  smoke)
    FIXED_TRIALS="${FIXED_TRIALS:-1}"
    RANDOM_TRIALS="${RANDOM_TRIALS:-1}"
    REQUIRE_COMMIT="${REQUIRE_COMMIT:-0}"
    ;;
  full)
    FIXED_TRIALS="${FIXED_TRIALS:-10}"
    RANDOM_TRIALS="${RANDOM_TRIALS:-20}"
    REQUIRE_COMMIT="${REQUIRE_COMMIT:-1}"
    ;;
  *)
    echo "V1_PROFILE must be smoke or full" >&2
    exit 2
    ;;
esac

OBSTACLE_TRIALS=3
SAFE_STOP_TRIALS=1

cd "${ROOT_DIR}"
source /opt/ros/jazzy/setup.bash

if [[ "${REQUIRE_COMMIT}" == "1" ]]; then
  git rev-parse --verify HEAD >/dev/null 2>&1 || {
    echo "Full v1.0 validation requires an existing commit." >&2
    exit 2
  }
  if [[ -n "$(git status --porcelain --untracked-files=normal)" ]]; then
    echo "Full v1.0 validation requires a clean working tree." >&2
    exit 2
  fi
fi

mkdir -p \
  "${OUTPUT_DIR}/software" \
  "${OUTPUT_DIR}/fixed_rgbd" \
  "${OUTPUT_DIR}/random_rgbd" \
  "${OUTPUT_DIR}/static_obstacles" \
  "${OUTPUT_DIR}/safe_stop"

printf '%s\n' "${PROFILE}" >"${OUTPUT_DIR}/profile.txt"
git rev-parse HEAD >"${OUTPUT_DIR}/commit.txt" 2>/dev/null || printf '%s\n' "UNCOMMITTED" >"${OUTPUT_DIR}/commit.txt"

SKIP_PHYSICAL=1 RELEASE_OUTPUT_DIR="${OUTPUT_DIR}/software" \
  bash scripts/run_release_candidate_validation.sh
source install/setup.bash
python3 scripts/check_ros_graph_smoke.py --timeout-s 45

run_physics() {
  local name="$1"
  shift
  set +e
  python3 scripts/run_gazebo_physics_benchmark.py "$@"
  local status=$?
  set -e
  printf '%s\n' "${status}" >"${OUTPUT_DIR}/${name}/runner_exit_code.txt"
}

run_physics fixed_rgbd \
  --trials "${FIXED_TRIALS}" \
  --rgbd-perception \
  --timeout-s 300 \
  --csv "${OUTPUT_DIR}/fixed_rgbd/results.csv" \
  --markdown "${OUTPUT_DIR}/fixed_rgbd/report.md" \
  --svg "${OUTPUT_DIR}/fixed_rgbd/report.svg" \
  --log-dir "${OUTPUT_DIR}/fixed_rgbd/logs"

run_physics random_rgbd \
  --trials "${RANDOM_TRIALS}" \
  --rgbd-perception \
  --randomize-target \
  --seed 42 \
  --timeout-s 300 \
  --csv "${OUTPUT_DIR}/random_rgbd/results.csv" \
  --markdown "${OUTPUT_DIR}/random_rgbd/report.md" \
  --svg "${OUTPUT_DIR}/random_rgbd/report.svg" \
  --log-dir "${OUTPUT_DIR}/random_rgbd/logs"

run_physics static_obstacles \
  --trials "${OBSTACLE_TRIALS}" \
  --challenge-obstacles \
  --rgbd-perception \
  --planner-id RRTConnectkConfigDefault \
  --grasp-height-offset 0.115 \
  --grasp-pitch-offset-deg -20 \
  --loaded-path-precheck \
  --transfer-diagnostics \
  --timeout-s 420 \
  --csv "${OUTPUT_DIR}/static_obstacles/results.csv" \
  --markdown "${OUTPUT_DIR}/static_obstacles/report.md" \
  --svg "${OUTPUT_DIR}/static_obstacles/report.svg" \
  --log-dir "${OUTPUT_DIR}/static_obstacles/logs"

python3 scripts/run_payload_transfer_diagnostics.py \
  --runs-per-mode "${SAFE_STOP_TRIALS}" \
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
  --output-dir "${OUTPUT_DIR}/safe_stop"

evaluator_args=(
  --profile "${PROFILE}"
  --fixed-csv "${OUTPUT_DIR}/fixed_rgbd/results.csv"
  --random-csv "${OUTPUT_DIR}/random_rgbd/results.csv"
  --obstacle-csv "${OUTPUT_DIR}/static_obstacles/results.csv"
  --safe-stop-json "${OUTPUT_DIR}/safe_stop/physical_disturbance_safety.json"
  --output-json "${OUTPUT_DIR}/release_summary.json"
  --output-markdown "${OUTPUT_DIR}/release_summary.md"
  --fixed-trials "${FIXED_TRIALS}"
  --random-trials "${RANDOM_TRIALS}"
  --obstacle-trials "${OBSTACLE_TRIALS}"
  --safe-stop-trials "${SAFE_STOP_TRIALS}"
)
if [[ "${PROFILE}" == "smoke" ]]; then
  evaluator_args+=(--random-minimum-rate 1.0)
fi
python3 scripts/evaluate_v1_release.py "${evaluator_args[@]}"

echo "v1.0 ${PROFILE} evidence bundle: ${OUTPUT_DIR}"
