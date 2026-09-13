# Gazebo Physical Pick Benchmark

- Trials: 9
- Physical successes: 9
- Physical failures: 0
- Raw physical success rate: 100.0%
- Raw success rate 95% CI: [70.1%, 100.0%]
- Infrastructure startup failures: 0
- Valid-start task attempts: 9
- Valid-start physical successes: 9
- Valid-start physical success rate: 100.0%
- Valid-start success rate 95% CI: [70.1%, 100.0%]
- Mean lift delta: 0.3641 m
- Lift delta stddev: 0.1653 m
- Mean placement error: 0.0037 m
- Placement error stddev: 0.0017 m
- Mean final tilt: 0.00 deg
- Final tilt stddev: 0.00 deg
- Measured target X range: [0.540, 0.550] m
- Measured target Y range: [-0.100, 0.100] m
- Collision-aware direct paths blocked: 9/9
- Mean OMPL planning time: 5.3577 s
- Mean OMPL joint path length: 7.5233 rad
- Raw mean trial time: 82.47 s
- Valid-start mean trial time: 82.47 s

A trial passes only when the pipeline succeeds and Gazebo measurements verify lift height, final placement error, and upright orientation.

Raw success includes infrastructure startup failures. Valid-start success excludes only trials where controller spawners failed before the task began.

## Planner Comparison

| Planner | Raw success | Valid-start success | Infra failures | OMPL time (mean +/- std) | Joint path (mean +/- std) | Valid-start trial time (mean +/- std) | Place error (mean +/- std) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| PRMkConfigDefault | 3/3 (100.0%) | 3/3 (100.0%, 95% CI [43.9%, 100.0%]) | 0 | 0.0403 +/- 0.0035 s | 7.4024 +/- 1.0140 rad | 78.86 +/- 7.63 s | 0.0047 +/- 0.0027 m |
| RRTConnectkConfigDefault | 3/3 (100.0%) | 3/3 (100.0%, 95% CI [43.9%, 100.0%]) | 0 | 0.0280 +/- 0.0133 s | 7.4650 +/- 0.7538 rad | 66.47 +/- 8.34 s | 0.0031 +/- 0.0006 m |
| RRTstarkConfigDefault | 3/3 (100.0%) | 3/3 (100.0%, 95% CI [43.9%, 100.0%]) | 0 | 16.0048 +/- 0.0002 s | 7.7026 +/- 0.4383 rad | 102.09 +/- 13.85 s | 0.0034 +/- 0.0000 m |

## Scenario Breakdown

| Input | Planner | Scenario | Raw success | Valid-start success |
| --- | --- | --- | ---: | ---: |
| aruco | PRMkConfigDefault | center_tall_barrier | 1/1 (100.0%) | 1/1 (100.0%) |
| aruco | PRMkConfigDefault | negative_y_barrier | 1/1 (100.0%) | 1/1 (100.0%) |
| aruco | PRMkConfigDefault | positive_y_barrier | 1/1 (100.0%) | 1/1 (100.0%) |
| aruco | RRTConnectkConfigDefault | center_tall_barrier | 1/1 (100.0%) | 1/1 (100.0%) |
| aruco | RRTConnectkConfigDefault | negative_y_barrier | 1/1 (100.0%) | 1/1 (100.0%) |
| aruco | RRTConnectkConfigDefault | positive_y_barrier | 1/1 (100.0%) | 1/1 (100.0%) |
| aruco | RRTstarkConfigDefault | center_tall_barrier | 1/1 (100.0%) | 1/1 (100.0%) |
| aruco | RRTstarkConfigDefault | negative_y_barrier | 1/1 (100.0%) | 1/1 (100.0%) |
| aruco | RRTstarkConfigDefault | positive_y_barrier | 1/1 (100.0%) | 1/1 (100.0%) |
