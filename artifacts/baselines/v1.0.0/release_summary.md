# v1.0 Release Validation

- Profile: `full`
- Final verdict: `PASS`
- One source snapshot across all physical cohorts: yes

## Cohorts

| Cohort | Result | Required | Verdict |
| --- | ---: | ---: | --- |
| fixed_rgbd | 10/10 (100.0%) | 10 trials, >=100.0% | PASS |
| random_rgbd | 20/20 (100.0%) | 20 trials, >=85.0% | PASS |
| static_obstacles | 3/3 (100.0%) | 3 trials, >=100.0% | PASS |
| gazebo_wrench_safe_stop | 1/1 safe stops | 1/1 | PASS |

## Quality Metrics

| Cohort | Mean perception error | Mean placement error | Max tilt |
| --- | ---: | ---: | ---: |
| fixed_rgbd | 0.55 mm | 1.74 mm | 0.00 deg |
| random_rgbd | 1.29 mm | 2.35 mm | 0.00 deg |
| static_obstacles | 2.49 mm | 4.02 mm | 0.00 deg |
| gazebo_wrench_safe_stop | n/a | n/a | stop 171.21 ms |

## Failed Checks

- None.
