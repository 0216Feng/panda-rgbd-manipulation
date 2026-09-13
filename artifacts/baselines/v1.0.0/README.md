[![English](https://img.shields.io/badge/lang-English-2563eb.svg)](README.md) [![简体中文](https://img.shields.io/badge/lang-简体中文-d73a49.svg)](README.zh-CN.md)

# v1.0 Acceptance Evidence

This directory is the curated public evidence for the final `v1.0` physical
simulation gate. The full matrix ran on 2026-09-13 from source commit
`2fa0440a00afeb29ff1987e88ca6d676d2df65ac` with profile `full`.

## Results

| Cohort | Result | Mean perception error | Mean placement error | Additional gate |
| --- | ---: | ---: | ---: | --- |
| Fixed RGB-D | 10/10 | 0.55 mm | 1.74 mm | 100% required |
| Seeded-random RGB-D | 20/20 | 1.29 mm | 2.35 mm | 85% required |
| Representative static obstacles | 3/3 | 2.49 mm | 4.02 mm | Direct paths blocked in 3/3 |
| Gazebo wrench safe stop | 1/1 | n/a | n/a | 171.21 ms under a 200 ms limit |

The final machine-readable verdict is `PASS`. The complete software suite also
passed 318/318 tests before the physical matrix was run.

## Method

- Each physical trial started a fresh Gazebo Harmonic world.
- Random target positions used the release runner's fixed seed and denominator.
- A separate validator measured target lift, placement error, final tilt, and
  direct-path blocking instead of trusting the pipeline success message alone.
- CSV files retain every trial. Configuration and source sidecars preserve the
  exact experiment settings and package SHA256 manifest.
- The safe-stop cohort injected a bounded `16 N`, `100 ms` Gazebo wrench during
  descent and required a zero command within `200 ms`.

Raw process logs and generated world files are intentionally omitted to keep the
repository compact. The public package contains the result CSVs, generated
reports, configuration, source fingerprints, and aggregate verdict needed to
audit the claims.

## Boundaries

This is Gazebo contact-physics evidence, not real-robot validation or a safety
certification. Gazebo truth assists independent scoring and the runtime safety
monitor; the project therefore does not claim a purely vision-only controller.

- [Aggregate report](release_summary.md)
- [Machine-readable verdict](release_summary.json)
- [Fixed RGB-D report](fixed_rgbd/report.md)
- [Random RGB-D report](random_rgbd/report.md)
- [Static-obstacle report](static_obstacles/report.md)
- [Safe-stop report](safe_stop/physical_disturbance_safety.md)
