# Gazebo Physical Pick Benchmark

- Trials: 1
- Physical successes: 1
- Physical failures: 0
- Raw physical success rate: 100.0%
- Raw success rate 95% CI: [20.7%, 100.0%]
- Infrastructure startup failures: 0
- Valid-start task attempts: 1
- Valid-start physical successes: 1
- Valid-start physical success rate: 100.0%
- Valid-start success rate 95% CI: [20.7%, 100.0%]
- Mean lift delta: 0.1258 m
- Lift delta stddev: 0.0000 m
- Mean placement error: 0.0041 m
- Placement error stddev: 0.0000 m
- Mean final tilt: 0.00 deg
- Final tilt stddev: 0.00 deg
- Measured target X range: [0.550, 0.550] m
- Measured target Y range: [0.000, 0.000] m
- Collision-aware direct paths blocked: 1/1
- Mean OMPL planning time: 0.0441 s
- Mean OMPL joint path length: 6.3205 rad
- Raw mean trial time: 173.52 s
- Valid-start mean trial time: 173.52 s

A trial passes only when the pipeline succeeds and Gazebo measurements verify lift height, final placement error, and upright orientation.

Raw success includes infrastructure startup failures. Valid-start success excludes only trials where controller spawners failed before the task began.

## Planner Comparison

| Planner | Raw success | Valid-start success | Infra failures | OMPL time (mean +/- std) | Joint path (mean +/- std) | Valid-start trial time (mean +/- std) | Place error (mean +/- std) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| RRTConnectkConfigDefault | 1/1 (100.0%) | 1/1 (100.0%, 95% CI [20.7%, 100.0%]) | 0 | 0.0441 +/- 0.0000 s | 6.3205 +/- 0.0000 rad | 173.52 +/- 0.00 s | 0.0041 +/- 0.0000 m |

## Scenario Breakdown

| Input | Planner | Scenario | Raw success | Valid-start success |
| --- | --- | --- | ---: | ---: |
| rgbd | RRTConnectkConfigDefault | center_tall_barrier | 1/1 (100.0%) | 1/1 (100.0%) |
