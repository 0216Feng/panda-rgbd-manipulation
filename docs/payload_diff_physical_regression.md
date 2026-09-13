# Payload Diff Physical Regression

2026-09-05. Result: 0/1, 99.3 s; obstacle release gate remains open.

The central-barrier RGB-D challenge was rerun after all three joint-only state
constructors were changed to `is_diff=True`. Height diversification and cube-yaw
symmetry were disabled. Planner: RRTConnect; transfer diagnostics enabled.

Evidence directory: `artifacts/runs/release_payload_diff_physical_20260905/`.
The previous semantics test and physical failure records remain unchanged.

- Selected grasp yaw: 0 degrees, 1/4 feasible candidates.
- Physical probe lift: 0.0296 m; grasp and lift completed.
- Pre-place descent fractions: 0.292, 0.320, 0.320, 0.250, all below 0.980.
- Direct-place fallback failed planning with MoveIt code 99999.
- No successful placement, retreat or home return is claimed.

The payload-preservation defect is independently verified and fixed, but this
physical run provides no evidence of improved task success. It shifts the next
diagnosis toward loaded descent feasibility, not controller gain tuning.

Before executing more grasps, downstream validation should use the approach
endpoint, closed-finger state and explicit hypothetical attached-object geometry;
validate lift, global transfer and descent; and reject the entire candidate when
any stage fails. The existing bounded-recovery fallback must not bypass that
rejection. This is required future work, not functionality implemented by this
regression. Runtime collision-contact diagnostics at the rejected descent target
are also needed to distinguish IK failure from payload/environment interference.
