# RGB-D Release Evidence: 2026-09-05

These are unmodified copies of two completed physical benchmark batches. The
CSV includes every attempt, including the randomized approach execution failure.
The batches used identical fingerprints for the 105 recorded source/configuration
files. A candidate Git commit is still pending.

| Batch | Physical success | Wilson 95% interval | Mean successful placement error | Mean time, all trials |
| --- | --- | --- | --- | --- |
| [Fixed 10](fixed10/report.md) | 10/10 | 72.2%-100.0% | 2.2 mm | 105.48 s |
| [Random 20](random20/report.md) | 19/20 | 76.4%-99.1% | 2.2 mm | 112.10 s |

## Files

Each subdirectory contains:

- `results.csv`: all recorded trial rows, including failures.
- `report.md`: success counts, intervals, timing and failure categories.
- `report.svg`: the runner-generated summary chart.
- `source_manifest.csv`: SHA256 fingerprints with repository-relative paths.
- `installed_packages.tsv`: installed Debian package versions from the test host.

The package list describes an existing environment. It is not a dependency lock,
a minimal install recipe or proof of a clean Docker build. Windows separators in
the source manifest identify paths relative to the project root.

## Experimental scope

Both batches use markerless RGB-D, the default planner fallback chain and
Cartesian descent. Fixed target XY is (0.52,0.00) m. Random targets use seed 42
with x=[0.48,0.56] m and y=[-0.06,0.06] m; the placement target stays at
(0.45,-0.08) m. Success requires the pipeline and simulated physical validator to
pass. The successful terminal logs also contain successful return-home steps.

Placement and lift means use successful trials; the success-rate denominator and
mean total time include failed trials. Startup, planning, execution and validation
all contribute to total time. These batches do not isolate individual OMPL
planners, test Servo descent, or establish arbitrary-object grasping.

Trial 15 of the random batch fails during Cartesian approach execution, with
controller goal-tolerance violation after the convergence timeout. Its CSV row
is retained. The start-state check passed; the exact physical cause is unresolved.

## Provenance and remaining limits

The original directories are `artifacts/runs/release_rgbd_fixed10_20260905` and
`artifacts/runs/release_rgbd_random20_20260905`. They retain full launch logs
locally and are excluded from Git. Full logs are not included in this compact
public evidence bundle; therefore CSV consumers can recompute statistics but
cannot independently replay every logged event from this bundle alone.

See [fixed acceptance](../../../docs/fixed10_acceptance.md),
[random acceptance](../../../docs/random20_acceptance.md), and
[sensing boundaries](../../../docs/release_architecture.md). Gazebo ground truth
participates in online monitoring and compensation as well as scoring. These are
simulation results, with no hardware acceptance claim.
