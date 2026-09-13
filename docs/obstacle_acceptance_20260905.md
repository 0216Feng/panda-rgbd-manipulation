# RGB-D Obstacle Release Checkpoint

Date: 2026-09-05. Status: **failed, release obstacle gate remains open**.

## Scope

One fresh-world trial, RGB-D perception, RRTConnect only, central tall barrier.
This is not the earlier three-planner obstacle benchmark and is not pooled with
the fixed10/random20 workspace acceptance batches.

```bash
python3 scripts/run_gazebo_physics_benchmark.py \
  --trials 1 --rgbd-perception --challenge-obstacles \
  --challenge-scenario center_tall_barrier \
  --planner-id RRTConnectkConfigDefault --timeout-s 300 \
  --csv artifacts/runs/release_rgbd_obstacle_center_20260905/results.csv \
  --markdown artifacts/runs/release_rgbd_obstacle_center_20260905/report.md \
  --log-dir artifacts/runs/release_rgbd_obstacle_center_20260905/logs
```

The challenge overrides the RGB-D profile: placement is (0.50, -0.30) m,
pre-close drift limit is 0.005 m, pre-grasp orientation tolerance is 0.10 rad,
and Cartesian pre-place transfer is disabled. These are existing challenge
settings, not changes made to pass this trial.

Post-run SHA256 verification of all 105 files in the fixed10 source manifest
found zero differences. This checks listed source/configuration files, not the
entire operating system or a committed candidate revision.

## Observations

- Physical result: 0/1; elapsed 104.9 s. Process completed with exit code 1.
- Collision-checked direct path fraction: 0.318; unchecked: 1.000;
  `direct_path_blocked=true`. This supports the planner's obstacle test, not
  a claim that all executed motion had zero physical contact.
- Pre-grasp, approach, bilateral target contact, grasp probe and lift passed.
  Probe lift measured 0.0294 m.
- Pre-place candidates were rejected by planning validation or transfer-length
  limits (one candidate: 4.606 rad versus 4.500 rad limit).
- Direct-place fallback passed retreat prevalidation, then failed execution.
- Terminal step: `ompl_to_place_fallback`, MoveIt `-4`, action status 6.
  Start-state maximum difference was 0.0000 rad.
- Controller log: joint index 4 position error -0.200218 rad versus path
  tolerance 0.200000 rad; `PATH_TOLERANCE_VIOLATED`.
- Placement, retreat and return-home were not verified in this trial.

Raw evidence remains in the ignored local run directory above. Relevant
controller events are at lines 936-940 of `logs/trial_001.log`; pipeline terminal
JSON is at line 944. Do not replace these files with a rerun.

## Next Diagnosis

The immediate failure is trajectory tracking during loaded transfer, not a
start-state mismatch. Physical obstruction, dynamic tracking and path geometry
are not distinguished by these logs. Capture measured/desired joint tracking
and contact telemetry on a separate diagnostic run before changing motion
generation. Keep collision, path-length and tracking thresholds intact. Any fix
requires a new versioned run and all three obstacle layouts, not just a repeat
of the successful fixed-workspace test.
