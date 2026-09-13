[![English](https://img.shields.io/badge/lang-English-2563eb.svg)](0001-cpp-trajectory-quality-observer.md) [![简体中文](https://img.shields.io/badge/lang-简体中文-d73a49.svg)](0001-cpp-trajectory-quality-observer.zh-CN.md)

# ADR 0001：C++ 轨迹质量观察节点

- 状态：已接受
- 日期：2026-09-13

## 背景

已经验证的抓取流水线使用 Python，并包含较多物理恢复逻辑。一次性替换控制权限会把语言迁移与行为变化混在一起，使回归难以归因。M2 同时需要可信的 C++17 代码、可复用算法、运行时集成和可量化的规划质量证据。

## 决策

建立独立 `ament_cmake` 包 `panda_manipulation_cpp`。可复用库计算轨迹时长、路径长度、端点位移、绕行比、最大关节步长、隐含/报告速度与加速度、加速度平方积分、终端运动状态和归一化关节限位裕度。`trajectory_metrics_node` 观察 `/display_planned_path`，在 `/trajectory_quality_metrics` 发布带版本号的 JSON。

该节点为只读节点，不包含 MoveIt 客户端、控制器 Action 客户端、取消权限或执行状态转换。Python 流水线保留近期消息，经独立 Gazebo 验证器把汇总值写入 benchmark CSV 与 Markdown。实验源码指纹覆盖两个 ROS 包及 `CMakeLists.txt`。

## 影响

- C++ 算法可通过 gtest 独立于 ROS 执行验证。
- 已安装节点图与物理 smoke 会验证跨语言数据链路。
- 第一步迁移期间不改变现有抓取行为。
- 当前指标覆盖所有显示过的轨迹段，包括回退与恢复规划，因此它是观察总量，不等同于最终实际执行路径。
- 任务管理与 MoveIt 指令权限仍需在后续阶段迁移。
