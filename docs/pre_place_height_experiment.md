# Pre-place Height Experiment

2026-09-05. Outcome: no physical success; default remains disabled.

Four pre-place heights were tested within the existing four-attempt budget.
Offsets: 0, +0.04, -0.04, +0.08 m, bounded above final placement by at least
0.05 m. Orientation, final placement, collision checking, 4.5 rad transfer
length limit and 0.98 descent fraction requirement were retained.

RGB-D central-barrier trial with RRTConnect: 0/1, 110.4 s.
Evidence: `artifacts/runs/release_obstacle_height_candidates_20260905/`.

| Attempt | Base-frame height (m) | Rejection |
| --- | --- | --- |
| 1 | 0.2740 | path 6.237 rad > 4.500 |
| 2 | 0.3140 | path 5.631 rad > 4.500 |
| 3 | 0.2340 | path 5.834 rad > 4.500 |
| 4 | 0.3540 | descent fraction 0.610 < 0.980 |

Direct-place fallback also failed planning. No threshold was relaxed and no
failure record overwritten. This sample does not show improvement or establish
that all possible paths are infeasible.

The implementation is retained as an explicit optional experiment:
add `--pre-place-height-step 0.04` to a challenge benchmark command. Default 0
preserves the original height. The launch parameter is
`pre_place_candidate_height_step_m`; joint-replay diagnostics bypass variation.
The original run temporarily enabled 0.04 in the benchmark source; subsequent
source changes restored disabled default and exposed the equivalent CLI option.

Verification: full suite 229 passed before the final CLI change; 7 targeted
height/benchmark tests passed after it. Build passed. Physical release gate
remains open. Next investigate grasp-to-place configuration compatibility and
path geometry; do not increase the transfer limit just to admit these paths.
