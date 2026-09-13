# Contributing

## Development environment

Use Ubuntu 24.04 and ROS2 Jazzy. Follow [the reproduction guide](docs/reproduction.md)
from the full repository root. The Docker and GitHub workflow require their own
successful runs before they can be claimed as validated environments.

## Change verification

Run the relevant focused tests first, then the software release checks:

```bash
SKIP_PHYSICAL=1 bash scripts/run_release_candidate_validation.sh
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/check_ros_graph_smoke.py
```

The graph smoke verifies synthetic ROS wiring. Changes to planning, contact,
grasping, release or recovery also require representative Gazebo trials. Describe
the physical scenario and measured result in the change description.

## Experimental evidence

- Record the source revision, parameters, seed and environment with each run.
- Use a new output directory for a new experiment; retain failures and interrupted runs.
- State denominators, confidence intervals and the code version for success rates.
- Distinguish synthetic tests, planning-only results and physical outcomes.
- Identify any Gazebo ground-truth input used by monitoring or control.
- Preserve collision, contact and placement gates when evaluating an improvement.

Keep generated build directories and raw experiment logs out of commits. Curated
reports must have traceable source data and a clear scope. Document any missing
evidence instead of inferring a result from an animation or a task-state label.

## Reporting a problem

Include the revision, OS/ROS versions, launch or benchmark command, relevant
parameters, failure stage and expected behavior. Include the smallest useful log
excerpt and an artifact reference; remove credentials and personal information.

## Review and release

Explain the observable behavior changed and how it was tested. Keep unrelated
refactoring separate. Follow [the release-readiness checklist](docs/github_release_readiness.md) for
release acceptance. Third-party models and code retain their upstream licensing;
verify notices before including new assets. Remote publication is a separate
release action after the local candidate and public contents have been reviewed.
