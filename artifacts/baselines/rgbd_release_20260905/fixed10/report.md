# Gazebo Physical Pick Benchmark

- Trials: 10
- Physical successes: 10
- Physical failures: 0
- Raw physical success rate: 100.0%
- Raw success rate 95% CI: [72.2%, 100.0%]
- Infrastructure startup failures: 0
- Valid-start task attempts: 10
- Valid-start physical successes: 10
- Valid-start physical success rate: 100.0%
- Valid-start success rate 95% CI: [72.2%, 100.0%]
- Mean lift delta: 0.1272 m
- Lift delta stddev: 0.0006 m
- Mean placement error: 0.0022 m
- Placement error stddev: 0.0016 m
- Mean final tilt: 0.00 deg
- Final tilt stddev: 0.00 deg
- Measured target X range: [0.520, 0.520] m
- Measured target Y range: [0.000, 0.000] m
- Raw mean trial time: 105.48 s
- Valid-start mean trial time: 105.48 s

A trial passes only when the pipeline succeeds and Gazebo measurements verify lift height, final placement error, and upright orientation.

Raw success includes infrastructure startup failures. Valid-start success excludes only trials where controller spawners failed before the task began.

## Planner Comparison

| Planner | Raw success | Valid-start success | Infra failures | OMPL time (mean +/- std) | Joint path (mean +/- std) | Valid-start trial time (mean +/- std) | Place error (mean +/- std) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| fallback_chain | 10/10 (100.0%) | 10/10 (100.0%, 95% CI [72.2%, 100.0%]) | 0 | 0.0122 +/- 0.0047 s | 3.3189 +/- 1.1370 rad | 105.48 +/- 2.68 s | 0.0022 +/- 0.0016 m |

## Scenario Breakdown

| Input | Planner | Scenario | Raw success | Valid-start success |
| --- | --- | --- | ---: | ---: |
| rgbd | fallback_chain | baseline_001 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | baseline_002 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | baseline_003 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | baseline_004 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | baseline_005 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | baseline_006 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | baseline_007 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | baseline_008 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | baseline_009 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | baseline_010 | 1/1 (100.0%) | 1/1 (100.0%) |
