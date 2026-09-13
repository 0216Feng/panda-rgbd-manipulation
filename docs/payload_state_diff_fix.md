# Joint-only State Payload Preservation

2026-09-05. A state-construction defect is fixed and its semantics were verified
against the installed MoveIt service. Physical task regression remains pending.

`measured_robot_state`, `end_state_from_trajectory` and `fixed_home_state`
previously supplied joint values with `RobotState.is_diff=False` and an empty
attached-object list. They now set `is_diff=True`: these are joint overrides,
not declarations that the robot has no attached payload.

Full software tests: 233 passed. Colcon build completed with rc=0. Three new
tests execute the nested state-building methods with minimal message doubles.

## Runtime Evidence

```bash
python3 scripts/check_static_scene.py \
  --world src/panda_manipulation/worlds/panda_table.sdf \
  --check-payload-diff \
  --output-dir artifacts/runs/payload_state_semantics_20260905
```

The isolated process attaches a diagnostic collision box to the fixed base so
it deliberately overlaps the auxiliary obstacle. It requests state validity
with the same nonempty joint override and an empty attachment list, differing
only in `is_diff`. No task motion is commanded and no real payload is moved.

- `is_diff=True`: payload/obstacle contact remains present.
- `is_diff=False`: that contact disappears from the returned collision pairs.
- Both requests also report unrelated robot self contacts. Overall `valid`
  flags are therefore not used to establish the payload distinction.
- Successful probe: 8.55 s; `attempt_y6obokuo/result.json` in the directory above.

First attempt `attempt_7bir9iwz` is retained as failed evidence: its empty joint
list was not representative of the pipeline. MoveIt rejects a full state with
no joints before processing attachments. The corrected probe includes
`panda_joint1=0.0`, identical in both requests.

This agrees with the Jazzy
[robot-state conversion implementation](https://raw.githubusercontent.com/moveit/moveit2/jazzy/moveit_core/robot_state/src/conversions.cpp)
and the installed RobotState message definition. The defect can invalidate
loaded-state collision checking, but this probe does not prove it caused every
previous physical execution failure. Historical success rates are not evidence
for the corrected collision semantics. Next rerun physical checks and implement
downstream grasp-candidate validation using explicit hypothetical attachments.
