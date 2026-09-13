# Gazebo Physical Pick Benchmark

- Trials: 20
- Physical successes: 20
- Physical failures: 0
- Raw physical success rate: 100.0%
- Raw success rate 95% CI: [83.9%, 100.0%]
- Infrastructure startup failures: 0
- Valid-start task attempts: 20
- Valid-start physical successes: 20
- Valid-start physical success rate: 100.0%
- Valid-start success rate 95% CI: [83.9%, 100.0%]
- Mean lift delta: 0.1274 m
- Lift delta stddev: 0.0006 m
- Mean placement error: 0.0023 m
- Placement error stddev: 0.0013 m
- Mean final tilt: 0.00 deg
- Final tilt stddev: 0.00 deg
- Measured target X range: [0.482, 0.557] m
- Measured target Y range: [-0.059, 0.057] m
- Raw mean trial time: 119.72 s
- Valid-start mean trial time: 119.72 s

A trial passes only when the pipeline succeeds and Gazebo measurements verify lift height, final placement error, and upright orientation.

Raw success includes infrastructure startup failures. Valid-start success excludes only trials where controller spawners failed before the task began.

## Planner Comparison

| Planner | Raw success | Valid-start success | Infra failures | OMPL time (mean +/- std) | Joint path (mean +/- std) | Valid-start trial time (mean +/- std) | Place error (mean +/- std) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| fallback_chain | 20/20 (100.0%) | 20/20 (100.0%, 95% CI [83.9%, 100.0%]) | 0 | 0.0116 +/- 0.0047 s | 3.8055 +/- 1.4395 rad | 119.72 +/- 16.50 s | 0.0023 +/- 0.0013 m |

## Scenario Breakdown

| Input | Planner | Scenario | Raw success | Valid-start success |
| --- | --- | --- | ---: | ---: |
| rgbd | fallback_chain | random_001 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | random_002 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | random_003 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | random_004 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | random_005 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | random_006 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | random_007 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | random_008 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | random_009 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | random_010 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | random_011 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | random_012 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | random_013 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | random_014 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | random_015 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | random_016 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | random_017 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | random_018 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | random_019 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | random_020 | 1/1 (100.0%) | 1/1 (100.0%) |
