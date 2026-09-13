# MoveIt Scene Readback

2026-09-05. Scope: runtime scene geometry, no commanded task motion.

`scripts/check_static_scene.py` starts an isolated headless MoveIt/Gazebo
instance, queries `/get_planning_scene`, checks the auxiliary box, writes an
attempt-specific JSON/log bundle, then terminates its launch process group.

```bash
python3 scripts/check_static_scene.py \
  --world src/panda_manipulation/worlds/panda_table.sdf
```

Verified by the second attempt, in 8.43 s:

- Object ID: `static_auxiliary_obstacle`.
- Frame: `world`.
- Center: (0.37, -0.24, 0.87) m.
- Box dimensions: (0.12, 0.12, 0.30) m.
- Orientation: axis-aligned.
- Evidence: `artifacts/runs/static_scene_check/attempt_ozwbgx_p/result.json`.

The first attempt (`attempt_pmbg4_b7`) is retained as FAILED. It incorrectly
assumed that MoveIt returned a base-frame primitive pose. MoveIt returned a
world-frame object pose with zero primitive-local translation instead. The
verifier now composes those axis-aligned translations and accounts for the
0.72 m robot base elevation; it rejects non-identity rotations. The verifier
fix did not change robot or scene behavior.

This closes the previously missing runtime readback evidence for auxiliary
geometry. It does not prove collision-free execution or successful placement.
The latest physical obstacle trial still failed; see
[the modeling checkpoint](static_environment_fix_20260905.md).
