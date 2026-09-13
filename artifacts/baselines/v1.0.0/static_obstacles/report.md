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
- Mean lift delta: 0.2465 m
- Lift delta stddev: 0.1749 m
- Mean placement error: 0.0040 m
- Placement error stddev: 0.0018 m
- Mean final tilt: 0.00 deg
- Final tilt stddev: 0.00 deg
- Measured target X range: [0.540, 0.550] m
- Measured target Y range: [-0.100, 0.100] m
- Collision-aware direct paths blocked: 3/3
- Mean OMPL planning time: 0.0444 s
- Mean OMPL joint path length: 4.9324 rad
- Raw mean trial time: 136.53 s
- Valid-start mean trial time: 136.53 s

A trial passes only when the pipeline succeeds and Gazebo measurements verify lift height, final placement error, and upright orientation.

Raw success includes infrastructure startup failures. Valid-start success excludes only trials where controller spawners failed before the task began.

## Planner Comparison

| Planner | Raw success | Valid-start success | Infra failures | OMPL time (mean +/- std) | Joint path (mean +/- std) | Valid-start trial time (mean +/- std) | Place error (mean +/- std) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| RRTConnectkConfigDefault | 3/3 (100.0%) | 3/3 (100.0%, 95% CI [43.9%, 100.0%]) | 0 | 0.0444 +/- 0.0102 s | 4.9324 +/- 2.0059 rad | 136.53 +/- 19.85 s | 0.0040 +/- 0.0018 m |

## Scenario Breakdown

| Input | Planner | Scenario | Raw success | Valid-start success |
| --- | --- | --- | ---: | ---: |
| rgbd | RRTConnectkConfigDefault | center_tall_barrier | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | RRTConnectkConfigDefault | negative_y_barrier | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | RRTConnectkConfigDefault | positive_y_barrier | 1/1 (100.0%) | 1/1 (100.0%) |
