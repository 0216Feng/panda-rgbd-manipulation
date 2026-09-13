# Reverse Branch Integration Status

This is development evidence, not v1.0 release acceptance.

## Retained experiments

All three runs use the original center_tall_barrier scene, RGB-D perception,
RRTConnect, -20 degree closing-axis pitch, 0.115 m grasp offset and full loaded
lookahead. Static collision geometry and physical acceptance limits are unchanged.

| Run directory under artifacts/runs | Result | Evidence |
| --- | --- | --- |
| release_reverse_probe_20260905 | 0/1 physical successes | Two reverse approach branches found; no execution authorized by the probe |
| release_reverse_connection_20260905 | 0/1 physical successes | Forward approach fraction 1.0 for two branches; nominal lift/transfer passed, descent rejected |
| release_reverse_place_20260905 | 0/1 physical successes | Nominal chain passed, physical execution stopped at descent stage 5/6: 60.921 s exceeds 15 s limit |

These runs belong to successive source revisions and must not be pooled as one
candidate-version success-rate estimate. The last run has no final physical
lift/placement metrics in its CSV; do not infer them from the planning outcome.

## Current change

Loaded lookahead now reuses joint_trajectory_quality_metrics and
place_descent_trajectory_quality_error from physical execution for every descent
stage. Reports include trajectory_quality. The 15 s threshold is unchanged.
Full regression: 303 tests passed in 8.25 s. This change has not yet undergone a
new physical trial.

## Remaining integration

### Update: measured runtime branch integrated

The runtime pre-place planner now invokes reverse placement IK using the
post-grasp compensated place pose and updated object-in-hand transform, then
plans a joint target and retains the existing forward descent validation.
Explicit paired joint replay targets keep their previous precedence.

`release_measured_reverse_place_20260905`: 1/1 physical success, 165.575 s,
placement error 0.003106 m, final tilt 0.000046 degrees. Retreat and return_home
both executed successfully. Full regression: 306 passed in 8.35 s; build passed.
The reported maximum lift delta is 0.601841 m across the task, not the commanded
initial lift distance. It should be reviewed with the full transfer path.
This is a single trial, not release stability evidence; repeated trials and
latency improvement remain necessary.

### Previous integration gap (now addressed above)

The runtime pre-place planner still generates pose-target candidates after
measured payload compensation. The new reverse placement branch is currently
used by nominal lookahead only. Next, solve the reverse branch using the measured
payload transform and current loaded state, connect with a joint goal, and retain
all forward validation, collision, timing, transfer-length and physical outcome
checks. Do not reuse nominal pre-place joint values after payload drift.

At this earlier revision the benchmark sidecar bound CLI arguments but not source
or installed dependency hashes. Those earlier runs therefore remain historical
evidence and must not be resumed after source changes.

## 2026-09-10 bounded multi-branch result

`release_multibranch_repeat3_20260910` used one unchanged source revision and
the same center_tall_barrier configuration for all three fresh worlds. Physical
result: 3/3, mean placement error 0.0105 m, mean trial time 167.88 s, and mean
reported OMPL joint path length 9.623 rad. Collision-aware direct paths were
blocked in 3/3. The Wilson 95% interval is [43.9%, 100.0%], so this is a smoke
regression rather than a release-scale reliability estimate.

The reverse probe now returns up to three bounded valid branches. Initial
pre-grasp connection tries the remaining branches when one plan-only connection
fails. Nominal loaded lookahead selects the branch closest to its current state.
The 4.5 rad transfer limit and all contact-zone limits are unchanged. Full
regression at this revision: 307 tests passed; build passed.

## 2026-09-10 measured branch ranking

The measured post-grasp reverse placement probe now collects all three bounded
seed results and ranks valid branches by maximum joint delta from the measured
loaded state. Repeated pre-place attempts rotate through ranked alternatives.
Default, Servo and explicit paired-joint replay paths are unchanged.

`release_ranked_branch_smoke_20260910`: 1/1 physical success, placement error
0.013671 m, final tilt 0.000108 degrees, 158.557 s, and 7.758 rad reported OMPL
joint path. The selected branch had a 1.918 rad maximum joint delta. Against the
preceding three-run mean, this one sample has about 19% less reported OMPL joint
path and 5.6% less elapsed time; it does not establish a stable performance gain.

This run is the first one here with automatic source binding: schema 2 config,
aggregate source SHA-256
`022da1f2292655937bef7882018b87496b2e33818f56bdfee55f5f4169acfe7e`,
and 100 relative-path file hashes in `results.csv.source.json`. Resume now rejects
changed arguments, changed source, missing manifests and inconsistent manifests.
Full regression at this revision: 309 tests passed; build passed.
