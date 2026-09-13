# Gazebo Physical Pick Benchmark

- Trials: 90
- Physical successes: 81
- Physical failures: 9
- Raw physical success rate: 90.0%
- Raw success rate 95% CI: [82.1%, 94.6%]
- Infrastructure startup failures: 4
- Valid-start task attempts: 86
- Valid-start physical successes: 81
- Valid-start physical success rate: 94.2%
- Valid-start success rate 95% CI: [87.1%, 97.5%]
- Mean lift delta: 0.3107 m
- Lift delta stddev: 0.1679 m
- Mean placement error: 0.0042 m
- Placement error stddev: 0.0061 m
- Mean final tilt: 0.00 deg
- Final tilt stddev: 0.00 deg
- Measured target X range: [0.540, 0.550] m
- Measured target Y range: [-0.100, 0.100] m
- Collision-aware direct paths blocked: 86/86
- Mean OMPL planning time: 4.7722 s
- Mean OMPL joint path length: 8.2731 rad
- Raw mean trial time: 83.68 s
- Valid-start mean trial time: 79.14 s

A trial passes only when the pipeline succeeds and Gazebo measurements verify lift height, final placement error, and upright orientation.

Raw success includes infrastructure startup failures. Valid-start success excludes only trials where controller spawners failed before the task began.

## Failure Categories

- cartesian_motion: 2
- infrastructure_startup: 4
- physical_grasp: 2
- planner_constraint: 1

## Failure Reasons

- infrastructure startup failed: controller spawners exited before activation (joint_state_broadcaster, panda_arm_controller, panda_hand_controller): 4
- pick pipeline failed: cartesian_approach: fraction=0.442, required=0.980, moveit_error_code=1: 1
- pick pipeline failed: cartesian_place_descent: fraction=0.800, required=0.980, moveit_error_code=1: 1
- pick pipeline failed: direct-place candidate validation failed: joint_path_length=5.195rad exceeds transfer limit 4.500rad: 1
- pick pipeline failed: grasp_recenter: target moved 0.0632m outside recovery envelope: 1
- pick pipeline failed: physical_grasp_check failed after retries: object_lift_delta=0.0000m: 1

## Planner Comparison

| Planner | Raw success | Valid-start success | Infra failures | OMPL time (mean +/- std) | Joint path (mean +/- std) | Valid-start trial time (mean +/- std) | Place error (mean +/- std) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| PRMkConfigDefault | 29/30 (96.7%) | 29/30 (96.7%, 95% CI [83.3%, 99.4%]) | 0 | 0.0519 +/- 0.0241 s | 8.2688 +/- 1.2162 rad | 66.81 +/- 9.20 s | 0.0042 +/- 0.0059 m |
| RRTConnectkConfigDefault | 28/30 (93.3%) | 28/30 (93.3%, 95% CI [78.7%, 98.2%]) | 0 | 0.0335 +/- 0.0082 s | 8.4412 +/- 1.6087 rad | 68.38 +/- 9.51 s | 0.0046 +/- 0.0057 m |
| RRTstarkConfigDefault | 24/30 (80.0%) | 24/26 (92.3%, 95% CI [75.9%, 97.9%]) | 4 | 16.0043 +/- 0.0011 s | 8.0823 +/- 1.2268 rad | 105.78 +/- 25.12 s | 0.0039 +/- 0.0068 m |

## Scenario Breakdown

| Planner | Scenario | Raw success | Valid-start success |
| --- | --- | ---: | ---: |
| PRMkConfigDefault | center_tall_barrier | 10/10 (100.0%) | 10/10 (100.0%) |
| PRMkConfigDefault | negative_y_barrier | 9/10 (90.0%) | 9/10 (90.0%) |
| PRMkConfigDefault | positive_y_barrier | 10/10 (100.0%) | 10/10 (100.0%) |
| RRTConnectkConfigDefault | center_tall_barrier | 9/10 (90.0%) | 9/10 (90.0%) |
| RRTConnectkConfigDefault | negative_y_barrier | 9/10 (90.0%) | 9/10 (90.0%) |
| RRTConnectkConfigDefault | positive_y_barrier | 10/10 (100.0%) | 10/10 (100.0%) |
| RRTstarkConfigDefault | center_tall_barrier | 9/10 (90.0%) | 9/9 (100.0%) |
| RRTstarkConfigDefault | negative_y_barrier | 7/10 (70.0%) | 7/9 (77.8%) |
| RRTstarkConfigDefault | positive_y_barrier | 8/10 (80.0%) | 8/8 (100.0%) |
