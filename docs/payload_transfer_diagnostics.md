# Payload Transfer Diagnostics

## Purpose

This diagnostic gate separates three failure families that previously looked
similar in Gazebo:

1. Arm controller tracking or execution timing failures.
2. Load-dependent grasp, friction, or wrist-pose failures.
3. Contact/collision-model failures where joints track normally but the object
   moves relative to the hand.

No planning, collision, placement, or rigid-payload tolerance is relaxed by
this instrumentation.

## Recorded Data

`transfer_diagnostics_recorder` samples at 20 Hz and aligns every row to the
latest `/pick_demo_state` stage. The CSV contains:

- arm controller reference, feedback, and position error for seven joints;
- gripper reference, feedback, error, output, and measured finger positions;
- Gazebo target pose and full `panda_hand` TF pose;
- left/right target contact state and collision names;
- payload phase, true hand-frame hand-object drift, and first 15 mm crossing;
- task terminal state and failure reason in the JSON summary.

Every paired runner trial also records a rosbag containing controller state,
joint state, TF, target pose, contact, task-stage, and terminal-result topics.

## Single Loaded Smoke

```bash
# Run from the repository root.
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select panda_manipulation
source install/setup.bash

python3 scripts/run_payload_transfer_diagnostics.py \
  --runs-per-mode 1 \
  --modes loaded \
  --output-dir artifacts/runs/payload_transfer_smoke
```

## Paired Gate

```bash
python3 scripts/run_payload_transfer_diagnostics.py \
  --runs-per-mode 5 \
  --modes unloaded loaded \
  --output-dir artifacts/runs/payload_transfer_diagnostics
```

The unloaded world keeps the same synthetic target, collision-aware planning
task, obstacle, goals, planner, and controller settings, but moves the physical
Gazebo cube below the workspace and disables physical/tactile gates. OMPL is
stochastic, so this is a matched-task comparison rather than byte-identical
trajectory replay. The first run selects a feasible grasp yaw and the second
run is forced to reuse it. The report also compares the measured seven-joint
state at the first place-descent stage. A pair is eligible for payload-effect
inference only when its maximum joint difference is at most 0.05 rad. The saved
bags retain the executed references for post-analysis; exact trajectory replay
is the next refinement when the state gate rejects a pair.

High-rate CSV and launch logs are written to WSL `/tmp` during each trial and
copied to `artifacts` after shutdown. This avoids adding OneDrive/DrvFS latency
to the Gazebo controller loop.

## Outputs

```text
artifacts/runs/payload_transfer_diagnostics/
  payload_transfer_comparison.csv
  payload_transfer_comparison.json
  payload_transfer_comparison.md
  telemetry/pair_XX_{unloaded,loaded}.csv
  telemetry/pair_XX_{unloaded,loaded}.json
  bags/pair_XX_{unloaded,loaded}/
  logs/pair_XX_{unloaded,loaded}.log
  worlds/pair_XX_{unloaded,loaded}.sdf
```

## Interpretation

- Unloaded tracking failures: investigate controller timing, trajectory
  execution, and simulation update rate before touching grasp parameters.
- Loaded-only tracking degradation: investigate force preload, friction,
  grasp depth, object inertia, and wrist orientation.
- Similar unloaded/loaded tracking with rising relative drift: investigate
  contact geometry, collision alignment, and contact-sensor semantics.

A one-run smoke only validates the data chain. Root-cause claims require the
five-pair report and retained failed trials.

## 2026-08-29 Checkpoint

The diagnostic chain now converts the measured world-frame object pose through
the full world-to-hand transform, including wrist rotation. A translation-only
subtraction is retained only as a descriptive stage value and is not used as the
payload-slip gate.

Placement descent is split into six measured-state stages. Before the first
stage, the complete descent line is translated to the measured pre-place hand
position. This prevents a sub-centimeter global-planner endpoint residual from
being corrected laterally in the contact zone. Each stage requires:

- a bounded horizontal hand path;
- recent bilateral target contact;
- cumulative rigid-payload drift below 15 mm in the fixed pre-place hand frame.

The `artifacts/current/payload_transfer/payload_transfer_six_stage_20260829_v6`
loaded run reached stage 4/6. Across
the payload phase it retained 99.86% bilateral contact and 6.6 mm maximum true
hand-frame drift. Stages 1-3 passed the payload gates. Stage 4 stopped safely
after the arm controller reached 0.200 rad maximum tracking error and remained
32.8 mm from the requested task-space endpoint. A single-factor recovery-speed
comparison at 0.06 was not consistently better than 0.03 and is not adopted.

This is useful failure isolation, not a release success. The next controlled
experiment compares unloaded and loaded execution at the same descent states,
then evaluates controller gains/time parameterization and MoveIt Servo for the
short contact-zone descent. Collision checking, bilateral contact, 15 mm payload
drift, and final Gazebo placement gates remain unchanged.

### Paired-control checkpoint

The paired runner now supports unloaded trials without pretending that a
physical object exists: object-pose reanchoring and payload-motion checks are
replaced by the nominal candidate transform, while arm tracking, planning,
collision checking, and execution remain active. Candidate yaw is frozen only
after a successful prevalidation selection, and the loaded run reuses that yaw.
Prevalidation resampling is configurable and defaults to four rounds in the
diagnostic runner.

`artifacts/runs/payload_transfer_paired_smoke_v7_20260829` proved that matching
the target pose and grasp yaw is not sufficient. Both runs selected 0 degrees,
but the first place-descent joint states differed by 0.987 rad because OMPL/IK
selected different branches. The report correctly rejected the pair, so the
observed 4.64x tracking-error ratio is not attributed to payload. Both runs also
showed bounded controller goal misses before abort: repeated unloaded errors
were 0.0101-0.0106 rad against a 0.0100 rad goal tolerance; the loaded run later
reached 0.0574 rad. This establishes two next experiments in order:

1. Replay a captured reference joint trajectory so unloaded and loaded runs
   enter the contact descent from the same branch and state.
2. With state-matched pairs only, sweep controller goal time/tolerance and
   trajectory timing as single factors before introducing MoveIt Servo.

### 2026-08-30 controller and trajectory checkpoint

Exact pre-place joint replay now produced a state-matched unloaded/loaded pair:
the planned replay differed by 0.0047 rad and the measured descent entry differed
by 0.0053 rad. Both trials still stopped in the contact descent, while the loaded
to unloaded tracking RMS ratio was only 0.89x. An isolated 10 s low-speed arm
trajectory then completed with less than 0.00001 rad tracking error at all sampled
points. These results rule out a general controller-tracking defect and do not
support payload mass as the primary failure cause.

The pipeline now records Cartesian trajectory quality for every executable path:
point count, duration, minimum segment time, maximum joint step, reported and
implied velocity, cumulative joint path, endpoint displacement, path tortuosity,
and terminal velocity/acceleration. The first instrumented loaded run showed
smooth, zero-terminal-velocity trajectories but exposed two experiment
confounders:

- the RGB-D demonstration obstacle was physically beside the diagnostic place
  point but was not part of this controller-isolation scene;
- after that obstacle was removed, the fifth short descent stage entered a
  near-singular branch and expanded to 884 points and 88.2 s.

Contact-zone Cartesian paths are now rejected before execution when a stage
exceeds 15 s or 0.5 rad cumulative joint travel. These limits are above the
validated central-workspace stage envelope (3.5-5.1 s and 0.09-0.30 rad) while
detecting the 88.2 s singular branch immediately.

The paired diagnostic world therefore moves the RGB-D obstacle out of the cell,
uses the standard nonblocking scene obstacle, and uses the already validated
central place point `(0.45, -0.16)`. This does not weaken the obstacle benchmark:
dynamic/static obstacle avoidance and boundary placement remain separate gates.
The paired test is intentionally limited to controller/load causality.

`artifacts/runs/payload_transfer_isolated_loaded_smoke_20260830` completed the
full physical pick, six-stage descent, release, retreat, and return-home chain.
Maximum transfer tracking error was 0.00008 rad. Healthy descent stages contained
37-91 points, lasted 3.5-9.0 s, accumulated 0.09-0.30 rad, and had path
tortuosity near 1.00.

The first paired smoke used an explicit 180 s per-trial override. Its unloaded
run succeeded; the loaded run completed descent, release, and retreat but was
terminated while executing return-home, so it is deliberately excluded from the
matched-pair metric. Production paired runs should use the runner's 420 s default.

### Five-pair isolated result

`artifacts/runs/payload_transfer_isolated_paired_5x_20260830` completed all ten
trials: five unloaded and five physically loaded. Both modes achieved 5/5 task
success. All five pairs passed the pre-descent state gate; their maximum joint
difference ranged from 0.00357 to 0.00484 rad.

The loaded runs had 0.00369 rad mean maximum transfer error, 0.00016 rad median
maximum error, 0.01095 rad P95 maximum error, 0.00066 rad mean RMS error, and
4.3 mm mean maximum rigid-payload drift. The unloaded mean RMS error was only
0.0000098 rad, so the resulting 67.39x loaded/unloaded ratio is numerically
sensitive to a near-zero denominator. The engineering conclusion is therefore
based on the absolute loaded error, P95, 5/5 physical success, bounded payload
drift, and matched start states rather than the multiplier alone.

Because five successes still yield a broad 95% Wilson interval of 56.6%-100%,
this result closes the controller/load root-cause experiment but is not promoted
as a final reliability claim. The next control experiment is a MoveIt Servo
contact-zone descent with the same collision, bilateral-contact, trajectory, and
15 mm rigid-payload safety gates.

Saved reports can be regenerated without starting ROS or Gazebo:

```bash
python3 scripts/run_payload_transfer_diagnostics.py \
  --report-only-csv \
  artifacts/runs/payload_transfer_isolated_paired_5x_20260830/payload_transfer_comparison.csv
```

### MoveIt Servo contact-zone checkpoint

The placement descent now has a feature-flagged MoveIt Servo backend. It keeps
the Cartesian backend as the default and preserves the same collision,
bilateral-contact, 15 mm rigid-payload drift, release, and final Gazebo pose
gates. Servo publishes position commands at 50 Hz and executes the same six
measured-state descent stages. A 100 ms watchdog samples Servo status, target
contact, and payload drift throughout each stage.

`artifacts/runs/payload_transfer_servo_fallback_3x_20260830` completed 3/3
physically loaded Servo trials. The placement-feedback errors were 15.1, 5.2,
and 9.5 mm, with 140, 135, and 119 watchdog checks. No fallback was required in
those sampled configurations. Mean end-to-end diagnostic time was 115.5 s;
this includes fresh Gazebo startup, grasp candidate prevalidation, execution,
validation, and teardown, so backend-only latency still needs a matched report.

Servo collision deceleration is accepted only on the final stage with no more
than 12 mm remaining, and only after the normal physical endpoint gates pass.
A Servo singularity emergency stop (status code 2) pauses Servo and permits one
collision-checked Cartesian replan from the measured robot state. Collision
halts and all other emergency states remain terminal failures.

The singularity recovery was verified with an exact replay of the previously
failing pre-place configuration:

```bash
python3 scripts/run_payload_transfer_diagnostics.py \
  --runs-per-mode 1 --modes loaded \
  --place-descent-backend servo \
  --forced-grasp-yaw-offset-deg 0 \
  --pre-place-joint-replay-positions \
  '0.8301007563,0.1441671143,-1.1505281953,-2.4909924025,0.3211115718,2.6087075187,0.9662585553' \
  --no-rosbag \
  --output-dir artifacts/runs/payload_transfer_servo_singularity_replay_20260830
```

The replay matched the requested endpoint within 0.00483 rad, reproduced Servo
status `0 -> 2`, paused Servo, and completed the remaining descent through the
Cartesian fallback. Gazebo measured 121.5 mm lift, 6.1 mm placement error, and
0.000014 degree final tilt. This is recovery-path evidence, not a reliability
claim. The next experiment compares Cartesian and Servo backends on matched
loaded scenarios with backend-specific stage timing and fallback statistics.

### Matched Cartesian/Servo backend comparison

`artifacts/runs/place_descent_backend_3x_20260830` contains three physically
loaded Cartesian/Servo pairs. For each pair, the Cartesian run selects the
grasp yaw and pre-place joint endpoint; the Servo run replays both. All six
trials succeeded, all three pairs passed the state gate, and the maximum
pre-descent joint delta was 0.00477 rad.

Across the three matched pairs, Cartesian descent took 26.81 s on average and
Servo took 15.18 s. The paired mean difference was 11.63 s and the mean
Cartesian/Servo speedup was 1.76x. Servo's mean placement error was 10.7 mm
versus 8.2 mm for Cartesian, and mean rigid-payload drift was 11.1 mm versus
9.1 mm. Transfer tracking RMS increased by only 0.000004 rad. All physical
errors remained within the unchanged 50 mm placement and 15 mm payload-drift
gates, and no Servo fallback was required in this sample.

This is a useful backend checkpoint, not a reliability claim: three successes
per backend still produce a 43.9%-100% Wilson interval. The next reliability run
should use at least ten matched pairs, followed by a deliberately injected
contact/singularity hazard suite that measures watchdog stop latency and false
acceptance, not only nominal task success.

### Servo watchdog safety injection checkpoint

The Servo runner now supports explicit `contact_loss` and `payload_drift`
sensor-path injections. The default is `none`, so production and nominal
benchmark behavior is unchanged. A configured injection is accepted as a
safety-test pass only when all of the following are true:

- the fault triggers at the requested descent stage and delay;
- the task terminates with the matching watchdog reason;
- a zero Servo command is published before asynchronous pause handling;
- trigger-to-zero-command latency is at most 150 ms.

The completed stage matrix is consolidated in
`artifacts/runs/servo_watchdog_stage_matrix_20260901`. Contact loss and a
synthetic 30 mm payload drift were each injected once in all six descent
stages after a 0.5 s delay. All 12 expected-failure trials stopped for the
matching reason and passed the 150 ms gate. Contact-loss latency had a 60.6 ms
mean and 80.5 ms maximum; payload-drift latency had a 63.9 ms mean and 100.5 ms
maximum. Replayed pre-place joint states remained within 4.98 mrad of the
reference. The task-level `FAILED` state is intentional;
`servo_watchdog_safe_stop_passed` is the safety acceptance field.

The consolidated report also includes the three no-injection Servo rows from
the matched backend checkpoint. Those nominal physical trials completed 3/3
with zero watchdog aborts. This is a small baseline, not proof of a zero false
positive rate: its success Wilson interval is still 43.9%-100.0%, and any
future nominal watchdog abort must be checked against physical contact and
payload telemetry before it is classified as false.

Multiple saved CSVs can be consolidated without restarting ROS or Gazebo:

```bash
python3 scripts/run_payload_transfer_diagnostics.py \
  --report-only-csv \
  artifacts/runs/servo_watchdog_contact_loss_stage_matrix_v2_20260831/payload_transfer_comparison.csv \
  artifacts/runs/servo_watchdog_payload_drift_stage_matrix_20260901/payload_transfer_comparison.csv \
  artifacts/runs/place_descent_backend_3x_20260830/payload_transfer_comparison.csv \
  --output-dir artifacts/runs/servo_watchdog_stage_matrix_20260901
```

This validates the software sensor/watchdog/stop path, not physical collision
impulse response. The next safety step is repeated nominal and injected runs
per stage plus a separate Gazebo physical disturbance test, so synthetic fault
coverage is not confused with real contact dynamics.

### Gazebo physical wrench disturbance checkpoint

The world now loads Gazebo's `ApplyLinkWrench` system. A dedicated ROS 2 node
waits for a configured `servo_place_descent_stage_N_of_6` execution event,
publishes a bounded persistent `EntityWrench` to the dynamic `target_cube`, and
clears it after the configured duration. The runner records explicit `ARMED`,
`APPLIED`, and `CLEARED` events. Cross-process stop latency uses Linux monotonic
time rather than subtracting Gazebo simulation time from a system clock.

A physical safety verdict passes only when the wrench is both applied and
cleared, the measured payload/contact watchdog stops the same configured
stage, a zero Servo command is published, and the apply-to-zero latency meets
the configured gate. A generic pipeline failure is not accepted.

`artifacts/runs/servo_physical_disturbance_bidirectional_matrix_20260903`
combines positive- and negative-Y `16 N`, `100 ms` wrench trials at all six
descent stages. All 12 produced measured payload slip and passed the physical
safe-stop verdict. Mean, P95, and maximum stop latency were 105.5, 137.9, and
154.6 ms. Every lifecycle event and observed stage matched, and every measured
latency met the strictest 200 ms gate. The aggregate 12/12 result has a
75.8%-100.0% Wilson 95% interval. This is a bidirectional stage-coverage
checkpoint, not a final reliability claim.

```bash
python3 scripts/run_payload_transfer_diagnostics.py \
  --runs-per-mode 1 --modes loaded \
  --place-descent-backend servo \
  --physical-disturbance --physical-disturbance-stage-suite \
  --physical-disturbance-bidirectional \
  --physical-disturbance-delay-s 0.5 \
  --physical-disturbance-duration-s 0.10 \
  --physical-disturbance-force-y-n 16.0 \
  --physical-disturbance-stop-latency-limit-s 0.2 \
  --no-rosbag --resume-existing \
  --output-dir artifacts/runs/servo_physical_disturbance_bidirectional_matrix
```

`--physical-disturbance-bidirectional` evaluates the configured force and its
opposite in the same matrix. `--resume-existing` reloads the existing CSV,
restores the selected grasp yaw and pre-place joint replay target, and skips
completed matrix cells using a key that includes the force vector and timing.
The next physical experiment adds within-direction repeats and lower-impulse
boundary cases. Real hardware certification, force-torque sensing, and
safety-rated stopping remain out of scope.
# Current-Source Matched Pair Checkpoint: 2026-09-10

The release-candidate runner completed five state-matched unloaded/loaded pairs,
ten task runs in total, from one guarded source/configuration fingerprint:

- Source SHA256: `862ef86cae466061f65fad6c64546f0d86ca6766475712bb19470d7fca7e988e`.
- Task completion: unloaded 5/5 and loaded 5/5.
- Matched pre-descent states: 5/5 pairs under the 0.05 rad gate.
- Pair start-state delta: 0.00473 rad mean and 0.00499 rad maximum.
- Loaded maximum transfer tracking error: 0.00211 rad mean, 0.00801 rad P95.
- Loaded RMS transfer tracking error: 0.000034 rad mean.
- Loaded maximum payload-relative drift: 0.0056 m mean.
- Mean task time: 125.8 s unloaded and 149.6 s loaded.

Both modes report a 100% point estimate with a wide 56.6%-100% Wilson 95%
interval because each mode contains only five trials. The loaded/unloaded RMS
ratio is 3.53x, but unloaded RMS is below 0.0001 rad; the ratio is numerically
sensitive and is not used as a headline claim. Absolute error, drift, task
success, state matching, and sample size remain the primary interpretation.

The compact public evidence is stored in
`artifacts/baselines/v1_candidate_20260910/payload_transfer_paired5/`. Full
rosbags, telemetry, and launch logs remain local under `artifacts/runs/` and are
excluded from Git because they contain machine-specific paths and are not needed
to recompute the published summary.
