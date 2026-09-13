# Gazebo Physical Pick Benchmark

- Trials: 20
- Physical successes: 19
- Physical failures: 1
- Raw physical success rate: 95.0%
- Raw success rate 95% CI: [76.4%, 99.1%]
- Infrastructure startup failures: 0
- Valid-start task attempts: 20
- Valid-start physical successes: 19
- Valid-start physical success rate: 95.0%
- Valid-start success rate 95% CI: [76.4%, 99.1%]
- Mean lift delta: 0.1274 m
- Lift delta stddev: 0.0006 m
- Mean placement error: 0.0022 m
- Placement error stddev: 0.0015 m
- Mean final tilt: 0.00 deg
- Final tilt stddev: 0.00 deg
- Measured target X range: [0.482, 0.557] m
- Measured target Y range: [-0.059, 0.057] m
- Raw mean trial time: 112.10 s
- Valid-start mean trial time: 112.10 s

A trial passes only when the pipeline succeeds and Gazebo measurements verify lift height, final placement error, and upright orientation.

Raw success includes infrastructure startup failures. Valid-start success excludes only trials where controller spawners failed before the task began.

## Failure Categories

- cartesian_motion: 1

## Failure Reasons

- pick pipeline failed: cartesian_approach: execution_status=6, moveit_error_code=-4, execution_start_check: max_delta=0.0000 rad at panda_joint1, trajectory_start=-1.2164, current=-1.2164, tolerance=0.0500: 1

## Planner Comparison

| Planner | Raw success | Valid-start success | Infra failures | OMPL time (mean +/- std) | Joint path (mean +/- std) | Valid-start trial time (mean +/- std) | Place error (mean +/- std) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| fallback_chain | 19/20 (95.0%) | 19/20 (95.0%, 95% CI [76.4%, 99.1%]) | 0 | 0.0123 +/- 0.0045 s | 3.9518 +/- 1.5640 rad | 112.10 +/- 11.19 s | 0.0022 +/- 0.0015 m |

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
| rgbd | fallback_chain | random_015 | 0/1 (0.0%) | 0/1 (0.0%) |
| rgbd | fallback_chain | random_016 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | random_017 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | random_018 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | random_019 | 1/1 (100.0%) | 1/1 (100.0%) |
| rgbd | fallback_chain | random_020 | 1/1 (100.0%) | 1/1 (100.0%) |
