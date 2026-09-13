# Fixed RGB-D Acceptance: 2026-09-05

Status: fixed-scene release gate passed. Randomized acceptance and the remaining
release deliverables are still pending.

## Results

Public raw data and reports: [fixed batch](../artifacts/baselines/rgbd_release_20260905/fixed10/report.md).

| Metric | Result |
| --- | --- |
| Cold-start trials | 10 |
| Physical successes | 10/10 |
| Infrastructure failures | 0 |
| Wilson 95% success interval | 72.2%-100.0% |
| Mean lift | 127.2 mm |
| Lift standard deviation | 0.6 mm |
| Mean placement error | 2.2 mm |
| Placement error standard deviation | 1.6 mm |
| Mean trial duration | 105.48 s |
| Duration standard deviation | 2.68 s |
| Return-home success in terminal logs | 10/10 |

The duration includes world startup, planning, execution and validation. A 10/10
sample is not evidence of a universal 100% success rate.

## Configuration and provenance

All trials used RGB-D perception, target `(0.520, 0.000) m`, placement
`(0.450, -0.080) m`, the default three-planner fallback chain and Cartesian
descent. Seed was 42; timeout was 300 seconds per trial. Each trial started a
fresh world. No runtime parameter or source changes were made during the batch.

Evidence directory: `artifacts/runs/release_rgbd_fixed10_20260905/`.

- `source_manifest.csv`: 105 source, script, configuration and container files
  hashed before the run. Post-run verification found zero changed files.
- `installed_packages.tsv`: installed Debian package versions at run time.
- `results.csv`, `report.md`, `report.svg`: full raw batch and statistics.
- `logs/trial_001.log` through `trial_010.log`: terminal pipeline and physical
  validation evidence. All ten terminal pipelines report successful return_home.

There is no candidate Git commit yet. The manifest identifies the listed inputs
but does not replace a committed, fully reproducible release environment.

## Interpretation and next gate

This batch satisfies the planned fixed RGB-D requirement of at least 9 successes
in 10 trials. It does not include obstacle-challenge or randomized target coverage.
The RGB-D estimator provides target localization; Gazebo truth still participates
in online physical checks and placement feedback, as documented in
[the release architecture](release_architecture.md).

Next run 20 randomized RGB-D trials in a new directory with the same source and
configuration, retaining every failure. Keep this batch separate from earlier
single-trial and three-trial smoke results. Update the final headline metrics only
after the random acceptance report is available.
