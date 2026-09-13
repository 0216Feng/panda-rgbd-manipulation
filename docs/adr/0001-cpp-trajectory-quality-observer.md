[![English](https://img.shields.io/badge/lang-English-2563eb.svg)](0001-cpp-trajectory-quality-observer.md) [![简体中文](https://img.shields.io/badge/lang-简体中文-d73a49.svg)](0001-cpp-trajectory-quality-observer.zh-CN.md)

# ADR 0001: C++ Trajectory-Quality Observer

- Status: Accepted
- Date: 2026-09-13

## Context

The validated pick pipeline is Python and already carries substantial physical
recovery logic. Replacing command ownership in one step would mix a language
migration with behavioral changes and make regressions hard to attribute. M2
still needs credible C++17 code, reusable algorithms, runtime integration, and
quantitative planner-quality evidence.

## Decision

Create `panda_manipulation_cpp` as a separate `ament_cmake` package. Its reusable
library computes trajectory duration, path length, endpoint displacement,
tortuosity, maximum joint step, implied/reported velocity and acceleration,
integrated squared acceleration, terminal motion, and normalized joint-limit
margin. `trajectory_metrics_node` observes `/display_planned_path` and publishes
versioned JSON on `/trajectory_quality_metrics`.

The node is read-only. It has no MoveIt client, controller action client,
cancellation authority, or execution state transition. The Python pipeline
retains recent messages and exports an aggregate through the independent Gazebo
validator into benchmark CSV and Markdown. Experiment fingerprints include
both ROS packages and `CMakeLists.txt`.

## Consequences

- C++ behavior is testable with gtest independently of ROS execution.
- Installed-graph and physical smoke tests verify the cross-language data path.
- Existing manipulation behavior remains stable during the first migration step.
- Metrics currently describe every displayed segment, including fallback and
  recovery plans; they are observation totals, not only the final executed path.
- Task management and MoveIt command ownership still require later C++ migration.
