[![English](https://img.shields.io/badge/lang-English-2563eb.svg)](github_release_readiness.md) [![简体中文](https://img.shields.io/badge/lang-简体中文-d73a49.svg)](github_release_readiness.zh-CN.md)

# GitHub Release Status

Updated: 2026-09-13. Target: public `v1.0.0` release.

## Decision

The release acceptance gate is complete and the final verdict is `PASS`.
Physical simulation evidence was generated from frozen source commit `2fa0440`
and is preserved under [`artifacts/baselines/v1.0.0`](../artifacts/baselines/v1.0.0/README.md).
The documentation and curated evidence are packaged in a follow-up release-only
commit; no package source changed after the accepted snapshot.

## Completed Gates

- Full software regression: 318/318 tests passed.
- Installed synthetic ROS graph smoke passed and explicitly records
  `physical_execution_verified: false`.
- Fixed RGB-D physical acceptance: 10/10, Wilson 95% CI 72.2-100%.
- Seeded-random RGB-D acceptance: 20/20, Wilson 95% CI 83.9-100%.
- Representative static obstacles: 3/3, with all three unchecked direct paths
  blocked and collision-aware transfer completed.
- Gazebo wrench safe stop: 1/1; a `16 N`, `100 ms` disturbance produced a zero
  command in `171.21 ms`, below the `200 ms` gate.
- Source fingerprints are consistent across all four physical cohorts.
- Hosted GitHub Actions run `34736305077` passed on source commit `2fa0440`:
  clean Docker build, deterministic assets, all tests, and installed graph smoke.
- Public documentation is paired in English and Simplified Chinese with language
  switches and automated pairing checks.
- Public evidence contains CSV/JSON/Markdown/SVG reports, configuration sidecars,
  and source manifests; raw logs and generated worlds remain local.
- README includes RViz and Gazebo stills, an end-to-end physical pick video, and
  a real Gazebo disturbance safe-stop video with source-bound metadata.
- MIT license, third-party attribution, Docker/devcontainer setup, contribution
  guide, and least-privilege CI workflow are present.

## Release Boundary

The tag packages the accepted physical evidence and public documentation. The
project is ready as a polished simulation portfolio release, but it is not
hardware-ready software or an industrial safety certification.

## Claim Rules

- Describe the result as Gazebo physical simulation validation.
- Keep final `v1.0` cohorts separate from supplemental historical experiments.
- Do not describe Gazebo truth-assisted monitoring and scoring as purely
  vision-only control.
- Preserve all attempted rows, fixed denominators, Wilson intervals, and
  source/configuration sidecars.
- Treat real-hardware deployment, calibration, latency characterization, and
  safety assessment as future work.
