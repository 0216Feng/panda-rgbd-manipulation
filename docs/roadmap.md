[![English](https://img.shields.io/badge/lang-English-2563eb.svg)](roadmap.md) [![简体中文](https://img.shields.io/badge/lang-简体中文-d73a49.svg)](roadmap.zh-CN.md)

# Long-Term Roadmap

## Objective

Evolve the current ROS2, MoveIt2, and Gazebo portfolio from a reproducible
simulation prototype into a manipulation system that can be transferred to
real hardware and defended with quantitative engineering evidence.

The v1.0 public baseline is released. Work now proceeds through M2 in measured,
source-bound increments without weakening reproducibility, failure retention,
documentation, or the frozen v1.0 evidence boundary.

## Current Baseline

- ROS2 Jazzy package with modular perception, grasp generation, planning,
  execution, monitoring, and validation nodes.
- Franka Panda simulation with MoveIt2, OMPL, ros2_control, and synchronized
  two-finger contact grasping in Gazebo Harmonic.
- Marker-based and markerless RGB-D target localization with tf2 conversion.
- Static and dynamic obstacle handling, remaining-path state validation,
  cancellation, and measured-state replanning.
- Fresh-world benchmark runners with raw failures, source/config fingerprints,
  CSV/JSON/Markdown/SVG output, and independent Gazebo scoring.
- Bilingual public documentation, screenshots, animation, and Gazebo video.
- A separate C++17 package with a reusable trajectory-quality library, online
  observer, gtest coverage, ROS graph integration, and benchmark export.

The current system is simulation-only. Gazebo ground truth participates in
online payload monitoring and scoring, the main execution pipeline is Python,
and no hardware calibration or safety case has been completed.

M1 is complete at the frozen v1.0 boundary. The first M2 increment was verified
on 2026-09-13: both packages built, the current tree passed 329
colcon-reported tests, an installed graph emitted C++ metrics, and one RGB-D
fresh-world physical smoke retained those metrics in CSV/Markdown. This single
physical run is integration evidence, not a new reliability claim.

## Milestones

### M1: Trusted Physical Simulation Loop

Status: complete in v1.0. The immutable acceptance evidence remains tied to
source revision `2fa0440`.

- Freeze one candidate source revision.
- Run fixed 10 and seeded random 20 RGB-D physical acceptance batches.
- Retain all failed rows and report Wilson confidence intervals.
- Complete representative static-obstacle and safe-stop evidence.
- Keep planning success separate from independently validated physical success.

Acceptance: fixed 10/10, randomized success at least 85%, mean RGB-D position
error at most 10 mm, mean placement error at most 50 mm, and final tilt at most
15 degrees.

### M2: Algorithm Depth and C++ Mainline

Status: in progress. Metric collection has moved to a C++17 observer while task
management and MoveIt command ownership remain on the validated Python path.

- Migrate task management, MoveIt calls, and metrics to C++17 in measured steps.
- Add a MoveIt Task Constructor comparison pipeline.
- Compare sampling, optimization, and industrial motion methods such as OMPL,
  STOMP, and Pilz where supported.
- Score grasp candidates using manipulability, joint-limit margin, obstacle
  clearance, path length, and smoothness.
- Add Jacobian conditioning, singularity, and joint-jump diagnostics.

Next checkpoint: add manipulability and scene-clearance metrics to candidate
scoring, publish an ablation over fixed candidate sets, then evaluate a second
planning family without conflating physical reliability with planner quality.

Acceptance: at least 90 planning experiments, planner comparisons covering
success, time, length, smoothness, and minimum clearance, plus one ablation.

### M3: Production-Oriented Software Architecture

- Update PlanningScene or OctoMap from live RGB-D point clouds.
- Use MoveIt Servo for bounded short-horizon correction and obstacle response.
- Separate `simulation` and `real_robot` execution backends.
- Add lifecycle management, watchdogs, timeouts, emergency-stop integration,
  speed limits, and explicit recovery states.
- Formalize QoS, TF, synchronization, rosbag replay, parameter layering,
  clang-format, clang-tidy, gtest, pytest, and CI.

Acceptance: no more than eight core commands from clean environment to demo;
automated unit/integration/system checks; every failure has a stage, code, and
replayable log.

### M4: Real-Robot Migration

- Obtain supervised access through a laboratory, makerspace, internship, or
  rental before considering hardware purchase.
- Calibrate intrinsics, distortion, eye-to-hand or eye-in-hand transform, TCP,
  and robot base.
- Integrate one supported commercial six- or seven-axis arm.
- Measure joint tracking, end-effector error, latency, and repeatability.
- Enforce real workspace, velocity, acceleration, collision, and e-stop limits.

Acceptance: at least 20 real grasp trials, retained success/failure videos,
calibration report, error statistics, and simulation-to-hardware comparison.

### M5: Motion Control and Contact Tasks

- Implement one of impedance, admittance, or hybrid force/position control.
- Validate kinematics or dynamics with Pinocchio, KDL, or TRAC-IK.
- Add one contact-rich task such as insertion, button pressing, drawer motion,
  or surface following.
- Quantify peak force, tracking error, steady-state error, and stability.

### M6: One Advanced Specialization

Choose one: constrained/task-and-motion planning, 3D instance segmentation and
6D pose, learning-based grasp generation, or industrial integration with PLC,
Modbus, cycle-time, and recovery requirements.

## Evaluation Matrix

| Area | Scope | Minimum samples | Metrics |
| --- | --- | ---: | --- |
| Perception | fixed/random targets | 30+ | detection, XYZ error, latency |
| Planning | 3 planners x 30 scenes | 90+ | success, time, length, smoothness |
| Static obstacles | three barriers | 30+ | collision, clearance, detour rate |
| Dynamic obstacles | multiple speeds | 30+ | stop, replan, collision-free rate |
| Physical grasp | fixed/random targets | 30+ | dual contact, lift, drop rate |
| Placement | multiple destinations | 30+ | error, tilt, drag rate |
| Recovery | injected failures | 20+ | recovery, attribution, safe stop |

Every report should include sample size, success rate, 95% confidence interval,
mean, standard deviation, failure classes, and replayable log references.

## Delivery Rules

- Maintain README, architecture decisions, CSV/Markdown/SVG evidence, demo
  video, and bilingual technical documentation.
- Prefer stability, reproducibility, and explainability over adding buzzwords.
- Change one dominant failure mode at a time and validate on a small cohort
  before expanding the matrix.
- Never use one successful run as a headline reliability metric.
- Prioritize C++, hardware, calibration, safety, and recovery before YOLO, VLA,
  reinforcement learning, or dual-arm expansion.
- Every automated success must pass both software-state and physical-outcome
  validation.
