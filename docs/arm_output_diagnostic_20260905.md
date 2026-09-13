# Arm Output Tracking Checkpoint

2026-09-05. Status: failure reproduced; root physical cause unresolved.

Added seven `arm_output_panda_jointN` columns to the existing diagnostic CSV.
The existing controller-state mapping fills them from the position output.
Missing output remains empty, not zero. No controller gains, trajectories or
safety tolerances changed. Full software suite: 220 passed in 5.96 s.

One RGB-D central-barrier trial with RRTConnect and `--transfer-diagnostics`
completed in 139.4 s, with physical result FAILED. Local evidence directory:
`artifacts/runs/release_obstacle_output_diagnostic_20260905/`.
Telemetry subdirectory: `logs/trial_001_telemetry_k4wi5kpj/`.
Use the command from [the preceding diagnostic](obstacle_tracking_diagnostic_20260905.md)
with this new output directory; preserve earlier failed runs.

The terminal step was `ompl_to_place_fallback`. Controller goal-time tolerance
expired after 5.003554 s. Joint indices 1, 2 and 3 had endpoint errors
0.036795, -0.018445 and -0.176059 rad respectively (limit 0.01 rad).

At telemetry elapsed 92.05 s, panda_joint4 had:

| Quantity | Value |
| --- | --- |
| Reference | -2.026573270 rad |
| Controller output | -2.026573270 rad |
| Feedback | -1.850513912 rad |
| Error | -0.176059357 rad |
| Bilateral target contact | true |
| Hand-relative payload drift | 0.001569895 m |

This shows the position command was present at the controller output while
measured motion fell short. It does not prove the hardware plugin applied it
correctly, or rule out physical obstruction. Current contacts cover the target,
not every robot/environment collision pair.

Separately, the preceding descent-failure snapshot was compared to the vendored
URDF: all seven feedback positions were inside hard limits with margins above
1 rad; panda_joint2 margin was about 1.45 rad. Hard joint-limit contact is not
supported for that snapshot.

Next evidence required: arm/environment contact data and plugin-side actuation
for the stalled configuration. Do not change safety thresholds or claim a fix
from the new observability alone. Obstacle release acceptance remains open.
