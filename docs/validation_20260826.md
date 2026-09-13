# Validation Checkpoint: 2026-08-26

## Status

Release candidate: NOT PASSED.

- Software: 161 tests passed after the transfer-diagnostics additions.
- Physical: one fresh center-obstacle trial failed after 338.3 s.
- Historical RGB-D 3/3 and obstacle-matrix successes are not repeatability
  evidence for this revised controller and supervision configuration.
- The full release script and fixed 10-trial RGB-D gate remain incomplete.

## Changes

- Candidate reports omit internal candidate payloads and RobotTrajectory
  objects at the JSON output boundary, including early rejection paths.
  Regression tests verify serialization and preservation of internal objects.
- Terminal payload-check failures stop execution-result handling immediately;
  a generic CONTROL_FAILED result must not overwrite the original diagnosis.
- Pre-close recentering and physical grasp recovery have separate bounded
  retry counters. Recentring no longer consumes every physical probe retry.
- OMPL and payload speed arguments are declared and wired to the node.
- A bounded placement-descent endpoint check uses measured task-space position,
  orientation, current joint-limit margin, recent bilateral contact, and rigid
  payload error. It does not declare whole-task success.
- Non-finite or invalid quaternions are rejected by orientation comparison.

The final terminal-state guard was tested in software after the physical run;
it has not yet been rerun in Gazebo. The trial did not exercise the all-candidates-
rejected JSON path; that path is covered by the new serialization regression.

## Trial Configuration

| Setting | Value |
| --- | --- |
| Scenario | center_tall_barrier |
| Perception | synthetic target pose, not an RGB-D accuracy test |
| Planner | RRTConnectkConfigDefault |
| Initial target | x=0.550 m, y=0.000 m |
| OMPL velocity/acceleration scaling | 0.05 |
| Payload velocity/acceleration scaling | 0.01 |
| Pre-close target drift limit | 0.005 m |
| Pre-close recenter retries | 2 |
| Physical grasp recovery retries | 2 |
| Payload descent rigid-error limit | 0.015 m |
| Descent endpoint position limit | 0.015 m |
| Descent endpoint orientation limit | 0.10 rad |
| Minimum normalized joint-limit margin | 0.01 |
| Trial wall timeout | 420 s |

The attempted 0.02 payload-speed trial exhibited payload loss. This is an
observation, not a controlled demonstration that speed alone caused the loss.
The challenge configuration was returned to 0.01; no controller or collision
tolerance was enlarged to make this trial pass.

## Evidence

Reports: `release_obstacle_20260826.csv`, `.md`, `.svg`.
Raw log: `release_obstacle_20260826_logs/trial_001.log`.

| Measurement | Observed result |
| --- | --- |
| Direct Cartesian fraction, collision checked | 0.3111 |
| Direct Cartesian fraction, unchecked diagnostic | 1.0000 |
| Physical grasp probes | 1.0 mm, 0.0 mm, then 23.9 mm lift |
| Pre-place descent candidate validation | fraction=1.000 |
| Rigid payload error after transfer | 32.5 mm, below transfer limit of 60 mm |
| Rigid payload error at first descent failure | 32.6 mm, above descent limit of 15 mm |
| Measured endpoint position error | 12.0 mm |
| Measured endpoint orientation error | 0.0214 rad |
| Current normalized joint-limit margin | 0.0784 |
| Recent bilateral target contact | present, approximately 0.024 s old |

The endpoint and tactile checks passed, but the rigid-payload check failed.
Recent bilateral contact is therefore not sufficient evidence of a stable
hand-to-object transform. The physical gate correctly refused continuation.
Joint residuals alone do not establish redundant-IK equivalence or identify
the cause of the tracking error.

Before the terminal-state fix, the raw log contained the specific rigid-error
failure followed by a generic execution failure. The generated report retains
that original run output; it has not been rewritten as a successful trial.

## Next Gate

Implementation update (2026-08-27): steps 1 and the matched-task form of step
2 are now instrumented in software. `transfer_diagnostics_recorder` writes a
20 Hz synchronized CSV and terminal JSON summary, and
`run_payload_transfer_diagnostics.py` records per-trial rosbag data plus paired
unloaded/loaded reports. The physical smoke and five-pair comparison are still
pending, so this checkpoint remains NOT PASSED. See
`docs/payload_transfer_diagnostics.md`.

1. Freeze the current thresholds and gains. Capture desired and measured arm
   joints, velocities, controller errors, gripper effort, hand TF, both contact
   streams, and object pose on one synchronized timebase during transfer.
2. Compare unloaded and loaded execution of a fixed, collision-checked path.
   Track the calibrated hand-to-object transform throughout motion, not only
   at waypoints, to locate the first persistent slip or tracking divergence.
3. Change one cause at a time, keeping all failed runs. Do not increase rigid
   payload or placement tolerances to conceal slip.
4. After a stable controlled regression, rerun release RGB-D and obstacle
   smokes, then 10 fixed RGB-D trials and the matched obstacle planner matrix.
5. Report physical success separately from planning success and from diagnostic
   recoveries. Headline reliability metrics require the larger fixed-configuration evidence.
