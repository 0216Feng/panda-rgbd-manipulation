# Obstacle Transfer Telemetry Checkpoint

Date: 2026-09-05. Physical result: failed; no motion fix claimed.

The physics benchmark now accepts `--transfer-diagnostics` (off by default).
It enables the existing recorder and allocates a unique `trial_NNN_telemetry_*`
directory under the selected log directory. Each directory contains
`samples.csv` and, after recorder finalization, `summary.json`.
No planning, contact, motion or tolerance parameters were changed.

## Reproduction

```bash
python3 scripts/run_gazebo_physics_benchmark.py \
  --trials 1 --rgbd-perception --challenge-obstacles \
  --challenge-scenario center_tall_barrier \
  --planner-id RRTConnectkConfigDefault --transfer-diagnostics --timeout-s 300 \
  --csv artifacts/runs/release_obstacle_diagnostic_20260905/results.csv \
  --markdown artifacts/runs/release_obstacle_diagnostic_20260905/report.md \
  --log-dir artifacts/runs/release_obstacle_diagnostic_20260905/logs
```

Use a new run directory for later attempts to preserve benchmark logs/results;
only the telemetry subdirectory allocation is automatically unique.

## Result

- One completed trial, 0/1 physical success, 212.1 s wall elapsed.
- Recorder finalized with 2,951 samples and FAILED state.
- Terminal stage: `cartesian_place_descent_stage_3_of_6`, MoveIt -4.
- Start-state maximum error: 0.0000 rad.
- Controller reported repeated GOAL_TOLERANCE_VIOLATED after 5 s:
  joint index 4 error -0.011074 rad and joint index 1 errors
  0.061823 / 0.061875 rad, versus 0.010000 rad endpoint tolerance.
- At telemetry elapsed 142.2 s (simulation sampling clock), panda_joint2
  reference was 0.445266757 rad and feedback 0.383391502 rad;
  error was 0.061875255 rad. Bilateral contact was true and hand-relative
  payload drift was 0.002115192 m. Target world height was 0.860327339 m.
- All 505 samples labelled descent stage 3 reported contact names for the
  two fingers and target cube. This is not complete arm/environment contact
  coverage and cannot exclude another link being obstructed.

This differs from the preceding failed direct-place fallback, which exceeded
the 0.2 rad path tolerance. The current sample successfully reached staged
descent, but did not complete placement or return-home. Do not combine the two
runs into a claim of improvement; planning is stochastic and paths differed.

## Verification and Follow-up

Existing physics/diagnostic tests: 35 passed. New launch-wiring and repeated
trial output-isolation tests: 2 passed. The physical run confirms actual CSV
and summary production, beyond mocked launch tests. Full suite was not rerun.

Next isolate why joint feedback plateaus during descent: inspect arm/environment
contacts and actuator commands alongside the failing reference trajectory.
Do not loosen endpoint tolerance or label this a gripper failure without further
evidence. The release obstacle gate remains open.
