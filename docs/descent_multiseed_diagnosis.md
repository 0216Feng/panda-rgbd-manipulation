# Descent Multi-seed IK Diagnosis

2026-09-05. Physical trial: 0/1, 125.0 s; release gate remains open.

The read-only endpoint diagnostic now compares pre-place, home, positive-elbow
and negative-elbow seeds. The latter two modify bounded initial joint values,
not the live robot. Every IK solution is collision checked with attached payload
preserved. The log records seed label, solved joint names/positions, validity,
contact pairs and `executed=false`.

Eight unit cases passed: four seed variants times IK success/failure. No full
software suite was rerun in this iteration. Runtime artifacts:
`artifacts/runs/release_descent_multiseed_20260905/`.

| Seed | Diagnostic results | IK solutions | Collision-free endpoints |
| --- | --- | --- | --- |
| pre_place | 3 | 3 | 0 |
| home | 3 | 3 | 0 |
| elbow_positive | 3 | 3 | 0 |
| elbow_negative | 3 | 3 | 0 |

All 12 solutions failed validity, with panda_link5/panda_link6 contacting
static_auxiliary_obstacle. The joint configurations differ, so the diagnostic
did explore more than one returned solution. These bounded samples are not a
proof that every IK branch is infeasible. The physical pipeline still rejected
direct-place fallback; no diagnostic solution was executed.

Next examine allowed cube-equivalent placement orientations, not only seeds
for the same end-effector orientation. A different placement orientation must
respect the object's accepted symmetry and compensated grasp transform, then
pass loaded path and descent checks. Arbitrary workpiece yaw changes or removal
of the obstacle are not acceptable substitutes.
