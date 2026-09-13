# Loaded Descent Endpoint Collision Diagnosis

2026-09-05. Physical trial failed (0/1, 102.4 s), but concrete collision
evidence now narrows the planning problem.

`diagnose_rejected_descent` is a default-off launch/node option, enabled by
the benchmark's `--transfer-diagnostics`. For rejected pre-place descent paths
it requests IK with collision avoidance disabled only for diagnosis, then sends
the solution to `/check_state_validity` with payload-preserving diff semantics.
The resulting solution is never sent to an execution action. The IK timeout is
1 s; callbacks are asynchronous and do not block normal rejection/retry logic.

Experiment: RGB-D central tall barrier, RRTConnect, no height or yaw experiment.
Artifacts: `artifacts/runs/release_descent_contacts_20260905/`.

Attempts 1 and 4 produced:

- IK error code 1 (solution found).
- Endpoint invalid.
- Contact pairs: panda_link5 / static_auxiliary_obstacle;
  panda_link6 / static_auxiliary_obstacle.
- `executed=false` for both diagnostic solutions.

These are MoveIt geometric collision checks of diagnostic endpoint solutions,
not Gazebo contact measurements during execution. They do not prove that every
IK branch is invalid or that every prior failure has this cause. Attempts that
fail other gates, such as path length, do not invoke this descent diagnostic.

Verification: existing full suite 233 passed and build succeeded; two additional
mocked callback tests passed for IK success/collision and IK failure. The live
run confirms service wiring and meaningful contact reports.

Next: search alternative endpoint IK branches and assess loaded transfer plus
descent before selecting the grasp. Keep the auxiliary obstacle, collision
checks, final placement and path-length limits intact. Merely increasing
planning time, tolerances or retry counts does not address these contacts.
