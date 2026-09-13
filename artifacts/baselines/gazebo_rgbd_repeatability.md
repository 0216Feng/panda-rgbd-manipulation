# Gazebo Physical Pick Benchmark

- Trials: 3
- Physical successes: 3
- Physical failures: 0
- Raw physical success rate: 100.0%
- Raw success rate 95% CI: [43.9%, 100.0%]
- Infrastructure startup failures: 0
- Valid-start task attempts: 3
- Valid-start physical successes: 3
- Valid-start physical success rate: 100.0%
- Valid-start success rate 95% CI: [43.9%, 100.0%]
- Mean lift delta: 0.1274 m
- Lift delta stddev: 0.0001 m
- Mean placement error: 0.0028 m
- Placement error stddev: 0.0027 m
- Mean final tilt: 0.00 deg
- Final tilt stddev: 0.00 deg
- Measured target X range: [0.520, 0.520] m
- Measured target Y range: [0.000, 0.000] m
- Raw mean trial time: 106.73 s
- Valid-start mean trial time: 106.73 s

A trial passes only when the pipeline succeeds and Gazebo measurements verify lift height, final placement error, and upright orientation.

Raw success includes infrastructure startup failures. Valid-start success excludes only trials where controller spawners failed before the task began.

## Planner Comparison

| Planner | Raw success | Valid-start success | Infra failures | OMPL time (mean +/- std) | Joint path (mean +/- std) | Valid-start trial time (mean +/- std) | Place error (mean +/- std) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| fallback_chain | 3/3 (100.0%) | 3/3 (100.0%, 95% CI [43.9%, 100.0%]) | 0 | 0.0083 +/- 0.0047 s | 4.4264 +/- 1.9265 rad | 106.73 +/- 9.04 s | 0.0028 +/- 0.0027 m |

## Scenario Breakdown

| Input | Planner | Scenario | Raw success | Valid-start success |
| --- | --- | --- | ---: | ---: |
| rgbd | fallback_chain | baseline_001 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | baseline_002 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | baseline_003 | 1/1 (100.0%) | 1/1 (100.0%) |
