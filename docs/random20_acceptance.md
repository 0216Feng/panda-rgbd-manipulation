# Randomized RGB-D Acceptance: 2026-09-05

The planned 20-trial randomized batch is complete: 19 physical successes and one
execution failure. This satisfies the planned sample collection; it does not
complete all v1.0 release requirements.

| Metric | Result |
| --- | --- |
| Raw physical successes | 19/20 (95.0%) |
| Wilson 95% interval | 76.4%-99.1% |
| Infrastructure startup failures | 0 |
| Successful return-home steps | 19/20 |
| Mean lift on successful trials | 127.4 mm |
| Mean placement error on successful trials | 2.2 mm |
| Placement error standard deviation | 1.5 mm |
| Mean total trial time, all attempts | 112.10 s |

## Conditions

Seed 42, 20 independently initialized worlds, RGB-D perception, default planner
fallback chain, Cartesian descent, 300-second trial timeout. Requested target
ranges were x=[0.48,0.56] m and y=[-0.06,0.06] m; sampled ranges were
x=[0.482,0.557] m and y=[-0.059,0.057] m. Placement remained (0.45,-0.08) m.
This is one object appearance and a bounded workspace, not arbitrary-object
grasping or full-workspace coverage.

The 105-file manifest matches the fixed-10 batch and post-run checks found zero
source/configuration changes. Installed package versions were captured again.
There is still no candidate Git commit; the manifest is a file fingerprint,
not a substitute for final release versioning.

## Failure retained

Trial 15, target approximately (0.548,0.012) m, failed at cartesian_approach.
MoveIt returned CONTROL_FAILED (-4) after the controller reported
GOAL_TOLERANCE_VIOLATED. The start-state check passed. The final controller log
reports joint-index 1 and 4 errors of about 0.0862 and -0.1107 rad against
0.01 rad goal tolerance, after an additional five-second convergence window.

This establishes an execution endpoint tracking failure. The available log does
not distinguish contact obstruction from other tracking causes. No tolerance
was loosened, and no retry result replaces this row. The runner exited 1 because
it requires every trial to pass; this is expected for a complete 19/20 batch.

## Evidence

Public raw data and reports: [random batch](../artifacts/baselines/rgbd_release_20260905/random20/report.md).

`artifacts/runs/release_rgbd_random20_20260905/` contains source_manifest.csv,
installed_packages.tsv, results.csv, report.md, report.svg and logs/trial_*.log.
All 20 terminal pipeline records were checked: 19 SUCCESS records include a
successful return_home, and one FAILED record does not.

Fixed results remain separate in [fixed10_acceptance.md](fixed10_acceptance.md).
See [release_architecture.md](release_architecture.md) for the online
ground-truth monitoring/compensation dependencies and the distinction between
Cartesian release evidence and Servo-specific experiments.

Next release work: freeze and review the candidate contents, verify representative
obstacle/fault behavior on the candidate, finish clean-environment reproduction,
third-party notices and demonstration materials. Preserve this failed case for
targeted diagnosis without rewriting this acceptance batch.
