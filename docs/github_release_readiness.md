[![English](https://img.shields.io/badge/lang-English-2563eb.svg)](github_release_readiness.md) [![简体中文](https://img.shields.io/badge/lang-简体中文-d73a49.svg)](github_release_readiness.zh-CN.md)

# GitHub Release Readiness

Updated: 2026-09-13. Target: public `v1.0` candidate repository.

## Current Decision

The public repository is now suitable as a `v1.0` release candidate. Hosted CI
has built the clean Docker image and passed the software and installed ROS graph
checks. It is not yet a final `v1.0` release because the frozen-commit full
physical acceptance matrix, final evidence curation, and release tag remain open.

## Completed For The Candidate

- ROS2 Jazzy package builds successfully in the maintained WSL2 environment.
- Full software regression passed: 318 tests.
- Installed synthetic ROS graph smoke passed in 3.49 s and observed target pose,
  grasp candidates, and the complete dry-run state history. Its result explicitly
  records `physical_execution_verified: false`.
- Current-source paired payload experiment completed 10/10 task runs across
  five unloaded/loaded pairs.
- All five pre-descent state pairs matched; maximum joint delta was 0.00499 rad
  against the 0.05 rad gate.
- Loaded runs averaged 5.6 mm maximum payload drift and completed 5/5.
- Source/configuration guard sidecars bind the paired results to SHA256
  `862ef86cae466061f65fad6c64546f0d86ca6766475712bb19470d7fca7e988e`.
- Historical fixed 10, random 20, and static-obstacle 90-trial evidence is
  retained with explicit source/version boundaries.
- README has a concise project overview, architecture, evidence table, quick
  start, benchmark entry points, limitations, and documentation map.
- README now includes three live RViz stills and an eight-frame animation from
  one current-source RGB-D obstacle run that independently passed lift,
  placement, upright-orientation, and direct-path-blocked validation.
- README now links a 38.2-second H.264 Gazebo opposite-side video from a separate
  successful current-source RGB-D obstacle run. The associated CSV, report,
  configuration, source fingerprint, video metadata, and SHA256 are retained
  in the public candidate evidence bundle.
- Deterministic SVG charts summarize cohort success, planner behavior, outcome
  quality, and matched payload diagnostics directly from committed CSV/JSON
  evidence; CI rejects stale generated assets.
- Root MIT license and Apache-2.0 third-party attribution are present.
- Build caches, rosbags, raw run directories, historical logs, generated worlds,
  and developer-only current artifacts are excluded from Git.
- Public source/docs scan found no obvious credentials. Machine-specific paths
  are excluded from the public evidence bundle.
- Dockerfile, devcontainer, contribution guide, and least-privilege GitHub
  Actions workflow are present.
- GitHub Actions run `34735251206` passed on commit `42ecae5`: the clean Docker
  image built successfully, deterministic portfolio assets matched their public
  evidence, all 318 tests passed, and the installed ROS graph smoke succeeded.
- The release runner now writes each audit to a unique timestamped directory
  and uses the maintained RGB-D obstacle profile. A fresh central-barrier
  physical acceptance trial passed after this update.
- Public-facing documentation is paired in English and Simplified Chinese,
  with language-switch badges and an automated pairing check.
- A single v1.0 release runner now enforces a frozen clean commit, executes the
  fixed/random/obstacle/safe-stop cohorts, verifies package source fingerprints,
  and emits one machine-readable release verdict. The physical smoke passed
  fixed `1/1`, seeded-random `1/1`, representative obstacles `3/3`, and real
  wrench safe-stop `1/1`; the full physical matrix remains pending.
- The injected safe-stop clip is complete: one `16 N`, `100 ms` Gazebo wrench
  produced a strict `SAFE_STOP_PASS` with `107.8 ms` zero-command latency. The
  H.264 clip, poster, source/config fingerprint, CSV, and JSON/Markdown safety
  verdict are retained in the public candidate bundle.

## Completed First Publication

1. Reviewed and committed the 234-file public candidate without local career
   documents, raw runs, build caches, credentials, or machine-specific paths.
2. Created and pushed the public `0216Feng/panda-rgbd-manipulation` repository.
3. Observed a successful hosted CI run before adding the workflow badge.

## Final v1.0 Gates After Upload

- Re-run fixed 10 and randomized 20 RGB-D physical acceptance from one frozen
  commit. Report all attempts and Wilson intervals.
- Run the three representative static obstacles and one safe-stop injection from
  that same commit.
- Re-run the safe-stop case from the final frozen commit so its source manifest
  matches the fixed/random/obstacle release cohorts; the public demonstration
  clip and strict verdict format are complete.
- Replace source-hash-only evidence references with the final commit SHA and tag
  the accepted revision `v1.0.0`.

## Claim Rules

- Call the first public revision a `v1.0 candidate`, not hardware-ready software.
- Keep current-source results separate from historical evidence.
- Never describe Gazebo truth-assisted monitoring and correction as a purely
  vision-only controller.
- Do not pool infrastructure-excluded and raw success rates without both
  denominators.
- Preserve failed CSV rows and configuration/source sidecars.
