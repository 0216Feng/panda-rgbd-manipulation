# Static Challenge Environment Consistency

2026-09-05. Modeling correction implemented; physical acceptance still failed.

## Evidence and Change

The physical world contains a static model named `dynamic_obstacle`. The static
scene manager originally published only the table, target and primary obstacle.
The static challenge does not launch the dynamic obstacle tracker. Thus this
additional physical box was missing from that planning-scene publication.
This is a concrete modeling inconsistency, not proof that it caused every
previous tracking failure.

Static challenge benchmark launches now pass `static_environment_world` through
`gazebo_pick.launch.py` and `moveit_gazebo.launch.py`. The scene manager reads
the auxiliary box from that exact SDF and adds `static_auxiliary_obstacle`.
It uses the existing robot base world height 0.72 m to transform coordinates.
The standard world box is 0.12 x 0.12 x 0.30 m, centered at
(0.37, -0.24, 0.15) m in panda_link0.

The parameter defaults to empty outside static challenges. Dynamic perception
launches retain their existing behavior: no persistent static override of their
tracked geometry. This imports only the named auxiliary box, not a complete SDF
collision map. Unsupported rotations, pose attributes, non-static models and
non-box/multiple collision geometry fail explicitly. Missing auxiliary model
adds nothing. Physical geometry and controller tolerances are unchanged.

## Verification

- Full software suite: 224 passed; colcon build succeeded.
- Four new cases verify world-to-base translation, box size, missing model and
  rejection of rotated/nonfinite poses (parameterized test).
- RGB-D central static challenge, RRTConnect, telemetry enabled: 0/1, 120.1 s.
- Terminal failure: direct-place candidate validation, MoveIt 99999.
- Artifacts: `artifacts/runs/release_obstacle_scene_fix_20260905/`.
- A live CLI scene query did not return data before shutdown. Runtime geometry
  publication has not yet been independently confirmed by topic/service capture.

The failure moved to planning in this sample; stochastic planning prevents a
causal success claim. No failed run was replaced. The next gate is direct
runtime scene inspection followed by a feasible transfer/retreat candidate
under the complete intended static scene. Full robot/environment contact
instrumentation remains pending; it was not added in this change.
