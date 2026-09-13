# Dynamic Replanning Benchmark Results

## Experiment

The formal baseline contains 27 isolated Gazebo worlds:

- 3 planners: RRTConnect, PRM, and RRTstar
- 3 moving-obstacle scenarios: slow, fast, and delayed crossing
- 3 repeats for every planner/scenario cell

Every strict success requires RGB-D tracking, PlanningScene update, remaining
trajectory invalidation, online stop/cancel, replanning from measured joints,
physical grasp and lift, upright placement, and return-home.

## Baseline Results

| Metric | Result |
|---|---:|
| Physical task success | 23/27 (85.2%) |
| Full-evidence success | 19/27 (70.4%) |
| Mean safety-check latency | 6.3 ms |
| Mean cancel acknowledgement | 40.0 ms |
| Mean replan-trigger latency | 837.6 ms |
| Mean RGB-D position error | 17.9 mm |
| Mean placement error | 2.9 mm |

| Planner | Full evidence | Physical task | Mean OMPL time |
|---|---:|---:|---:|
| RRTConnect | 6/9 | 8/9 | 0.0696 s |
| PRM | 6/9 | 7/9 | 0.1644 s |
| RRTstar | 7/9 | 8/9 | 31.1199 s |

| Scenario | Full evidence | Physical task |
|---|---:|---:|
| slow_crossing | 9/9 | 9/9 |
| fast_crossing | 5/9 | 5/9 |
| delayed_crossing | 5/9 | 9/9 |

## Failure Analysis

Four trials were genuine task failures: two Cartesian approaches stopped below
the 0.98 completion threshold, one PRM approach-recovery plan failed, and one
RRTstar pre-grasp alignment stopped below its bounded 0.90 recovery threshold.
The rejected paths were not executed.

Four delayed trials completed online replanning and physical placement but did
not receive the perception validator's terminal result before point-cloud
tracking latched on the safety event. They remain strict failures in the
immutable baseline. A follow-up changed only the validation sample window from
12 samples / 0.12 m to 8 samples / 0.08 m, matching the existing fast scenario.
The independent 9-trial delayed regression then produced perception results in
9/9 trials. Its 6/9 task success reflects three real approach-planning failures.

## Reproduction

```bash
python3 scripts/run_dynamic_replanning_benchmark.py \
  --trials 3 --planner-suite --scenario-suite --timeout-s 240 \
  --csv dynamic_replanning_matrix.csv \
  --markdown dynamic_replanning_matrix.md \
  --svg dynamic_replanning_matrix.svg \
  --log-dir dynamic_replanning_matrix_logs
```

Reports can be regenerated without rerunning Gazebo:

```bash
python3 scripts/run_dynamic_replanning_benchmark.py \
  --report-only-csv dynamic_replanning_matrix.csv \
  --csv dynamic_replanning_matrix.csv \
  --markdown dynamic_replanning_matrix.md \
  --svg dynamic_replanning_matrix.svg
```

## Bounded Recovery Experiment

The fast-crossing failures were investigated without weakening collision
checking or the 0.98 completion threshold for contact motions. The pipeline now
supports:

- multiple OMPL intermediate approach candidates at 25%, 50%, 75%, and 90%;
- collision-checked partial execution followed by replanning from measured
  joints for pre-grasp alignment, contact approach, and grasp probe;
- bounded Cartesian timing recomputation instead of executing duplicate or
  non-increasing trajectory timestamps;
- recovery-only orientation tolerances that progress from 0.04 to 0.12 rad;
- one same-planner resample for recovery OMPL requests returning no solution.

The first 3-planner smoke test passed 3/3. The independent 9-trial fast-crossing
matrix produced 5/9 strict successes, equal to the immutable baseline rather
than a statistically reliable improvement. Its failures remained planning-side:
one Cartesian approach and three recovery OMPL requests. No failed trial was
accepted as a collision-free physical success.

After adding progressive orientation tolerance and bounded recovery resampling,
an independent RRTstar 3-trial regression reached 2/3. The remaining failure
completed recovery OMPL planning but reached only 0.143 of the final Cartesian
approach. This points to the next structural improvement: generate and validate
equivalent grasp orientations instead of repeatedly sampling one end-effector
orientation.

Artifacts:

- `dynamic_fast_recovery_final.csv`, `.md`, `.svg`
- `dynamic_fast_recovery_final_logs/`
- `dynamic_rrtstar_recovery_retest.csv`, `.md`, `.svg`
- `dynamic_rrtstar_recovery_retest_logs/`

## Equivalent-Grasp Recovery

The next iteration addressed IK-branch and tight-contact failures structurally,
while retaining collision checking and the 0.98 Cartesian threshold:

- generate cube-equivalent grasp candidates at +90, -90, and 180 degrees;
- keep target attachment transforms synchronized with the active candidate;
- use RRTConnect as a bounded recovery planner while preserving the benchmarked
  planner for the primary motion;
- split recovery into elevated staging and a strict OMPL pre-grasp pose before
  the collision-checked contact approach;
- clamp only simulator drift within 2 mrad of Panda joint limits, leaving real
  limit violations for MoveIt to reject;
- accept target recentering only inside a 0.10 m recovery envelope.

An independent 9-trial `fast_crossing` matrix produced 7/9 full-system
successes (77.8%): RRTConnect 2/3, PRM 3/3, and RRTstar 2/3. This improves the
immutable fast-scenario baseline from 5/9 (55.6%) by 22.2 percentage points.
Both remaining failures were planning-side no-solution outcomes; neither path
was executed or counted as a physical success.

A final one-trial-per-planner gate produced 2/3: PRM and RRTstar completed
online cancellation, replanning, physical lift, upright placement, and
return-home; RRTConnect exhausted all bounded equivalent-orientation candidates
at the final pre-grasp OMPL stage. Together with the 9-trial matrix, this shows
a useful but stochastic improvement rather than a defensible 100% result.

A temporary experiment that resumed RGB-D tracking two seconds after the first
safety event was reverted. The scenario controller intentionally halts the
obstacle at the detected hazard pose; resuming tracking after the arm occluded
the camera introduced roughly 4 cm of apparent obstacle drift and repeatedly
spent the dynamic-replan budget. Keeping the validated hazard pose latched is
the safer behavior for this benchmark.

Verification: 110 package tests pass with zero failures.

## Velocity-Aware Predictive Occupancy

The RGB-D obstacle tracker now estimates a bounded, low-pass-filtered Cartesian
velocity and can project an axis-aligned swept volume into MoveIt's
PlanningScene. The default dynamic demo uses a 0.65 s horizon, ignores motion
below 0.03 m/s, caps estimated speed at 0.50 m/s, and caps projected
displacement at 0.20 m. The trajectory monitor continues to call
`/check_state_validity`; no collision checks or contact thresholds are relaxed.

Safety events now include `collision_lead_time_s`, measured along the active
trajectory from the joint state nearest the robot's current measured state to
the first invalid sampled state. Obstacle status also records measured
velocity, speed, prediction displacement, predicted center, and predicted box
dimensions. These values flow into the per-trial CSV and grouped Markdown
report.

The first independent `fast_crossing` RRTConnect regression passed 3/3 complete
trials. Mean collision lead time was 3.67 s, mean safety-check latency was
13.0 ms, mean cancellation acknowledgement was 55.7 ms, mean replan-trigger
latency was 850.5 ms, mean RGB-D position error was 15.3 mm, and mean physical
placement error was 3.2 mm. A separate one-trial-per-planner gate passed 3/3
for RRTConnect, PRM, and RRTstar. Across these two gates the predictive mode
therefore passed 6/6 full-system trials.

Predictive triggering shortens the available pre-latch perception interval.
Its evidence profile transparently skips three startup-transient samples and
requires four measured samples spanning at least 0.08 m; the existing mean and
maximum pose-error thresholds are unchanged. Before that profile was added,
three physical tasks passed 3/3 but were correctly reported as evidence gaps
because the legacy eight-sample validator never emitted its terminal result.

Artifacts:

- `dynamic_predictive_rrtconnect_3_v2.csv`, `.md`, `.svg`
- `dynamic_predictive_rrtconnect_3_v2_logs/`
- `dynamic_predictive_planner_smoke.csv`, `.md`, `.svg`
- `dynamic_predictive_planner_smoke_logs/`

Verification: 114 package tests pass with zero failures.

Artifacts:

- `dynamic_hybrid_recovery_matrix_v2.csv`, `.md`, `.svg`
- `dynamic_hybrid_recovery_matrix_v2_logs/`
- `dynamic_recovery_final_smoke_logs/`

## Global Grasp-Candidate Prevalidation

An experimental selector evaluates the original, +90, -90, and 180 degree
cube-equivalent grasps without moving the robot. Each candidate receives a
plan-only OMPL check, an exact collision-checked Cartesian alignment/contact
check, and a score based on Cartesian completion, normalized joint-limit
margin, joint path length, and planning time. The target collision object is
removed only during the contact-path checks and restored before execution;
all environmental obstacles remain active. Candidate metrics and rejection
reasons are included in `/pick_plan_result`.

The selector worked as intended in an isolated fast-crossing trial: after the
sensed trajectory cancellation it evaluated 4/4 candidates, selected -90
degrees, completed the physical task, and used zero execution-time equivalent
orientation switches. However, the feature did not improve the independent
one-trial-per-planner gates consistently:

- prevalidation before obstacle motion: 2/3 strict successes;
- strict post-cancellation prevalidation: 0/3 because the stopped obstacle
  blocked every direct contact path;
- post-cancellation prevalidation with bounded fallback and a shared
  RRTConnect recovery planner: 1/3 strict successes.

The last result is below the existing 7/9 equivalent-grasp recovery matrix, so
candidate prevalidation remains opt-in and is not used for resume success-rate
claims. This preserves the validated default while retaining the module as a
diagnostic and future optimization surface. Collision checking and the 0.98
contact-path threshold were never weakened.

Artifacts:

- `dynamic_candidate_prevalidation_smoke_v2.csv`, `.md`, `.svg`
- `dynamic_candidate_prevalidation_smoke_v3.csv`, `.md`, `.svg`
- `dynamic_candidate_prevalidation_final_smoke.csv`, `.md`, `.svg`
- corresponding `*_logs/` directories

Verification: 114 package tests pass with zero failures.
