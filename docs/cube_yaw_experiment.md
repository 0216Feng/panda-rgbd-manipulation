# Cube Placement Yaw Experiment

2026-09-05. Result: 0/1 physical success; optional policy remains disabled.

The existing cube-equivalent grasp rotation changed pre-grasp, grasp and lift
orientation but left placement unchanged. An opt-in `--cube-symmetric-placement`
benchmark flag now also rotates place/pre-place/retreat orientations for the
equivalent grasp candidate. Translation is unchanged. This is only appropriate
for a cube task whose acceptance allows yaw symmetry, not oriented components.
Default behavior remains unchanged. The launch parameter is
`cube_symmetric_placement`.

Full software suite: 230 passed; build passed. The new pure test checks opt-in
behavior, preservation of positions, matching waypoint rotations and no mutation
of the input candidate.

Physical run: RGB-D central tall barrier, RRTConnect, telemetry enabled,
`--cube-symmetric-placement`, no height variation. Elapsed 127.6 s.
Artifacts: `artifacts/runs/release_cube_yaw_20260905/`.

The selected grasp yaw was 90 degrees (only 1/4 feasible). Pre-place candidates
were rejected with path lengths 5.757 and 6.907 rad versus the retained 4.5 rad
limit, or descent fractions 0.000 and 0.174 versus the retained 0.980 minimum.
Direct-place fallback also failed. No improvement is established by this sample.

The broader limitation remains: grasp prevalidation covers approach, not the
complete loaded transfer and descent. Next work should evaluate downstream
feasibility before choosing and executing a grasp, with an attached-object
planning state and the same collision/length/descent checks. Do not admit the
rejected trajectories by relaxing limits. This experiment is not release
acceptance and does not replace preceding failed trials.
