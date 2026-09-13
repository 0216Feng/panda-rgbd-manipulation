# Execution Debug Notes

当前稳定基线是 collision-aware plan-only showcase。真实执行模式仍处于调试阶段，主要目标是定位 `/execute_trajectory` 返回 `moveit_error_code=-4` 时到底是 controller 问题，还是轨迹起点和当前 `/joint_states` 不匹配。

## Strict Diagnostic Run

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_ws/install/setup.bash
ros2 launch panda_manipulation moveit_scene.launch.py \
  start_pipeline:=true \
  start_pick_pipeline:=true \
  execute_trajectories:=true \
  visualize_only_execution:=false \
  fail_on_start_state_mismatch:=true \
  execution_start_tolerance:=0.05 \
  joint_state_wait_timeout_s:=5.0 \
  use_synthetic_pose:=true \
  dry_run:=true
```

查看结果：

```bash
ros2 topic echo /pick_plan_result --once --full-length
```

如果失败原因包含：

```text
execution_start_check: max_delta=...
```

说明 pipeline 已经比较了轨迹首点和当前 `/joint_states`。若 `max_delta` 大于 `execution_start_tolerance`，应先解决状态同步问题，而不是继续调 grasp 参数。

如果日志包含：

```text
waiting for /joint_states before execution planning
```

说明 pipeline 已收到抓取候选，但还没有收到 `/joint_states`。当前实现会等待 `joint_state_wait_timeout_s` 秒；若仍失败，才会在 `/pick_plan_result` 发布 `FAILED`。先检查 `ros2 topic echo /joint_states --once`。

如果第一段 `ompl_to_pre_grasp` 从当前状态规划失败，pipeline 会自动运行一次 fixed-home 对照规划，但不会执行这条对照轨迹。结果里可能出现：

```text
diagnostic_fixed_home_to_pre_grasp
current_vs_home: max_delta=...
```

判读方式：

- `current-state planning failed, but fixed-home diagnostic planning succeeded`: 目标和 grasp 参数基本可用，问题集中在当前机器人状态不适合作为执行起点。
- `current state and fixed-home diagnostic also failed`: 目标位姿、约束、碰撞场景或 planner 配置本身需要继续调整。
- `current_vs_home: max_delta` 很大：RViz/fake controller 当前关节状态偏离项目默认 home，先重启 demo 或让机器人回到已知姿态。

## Non-blocking Diagnostic Run

如果想让 pipeline 即使发现起点偏差也继续调用 `/execute_trajectory`，可以关闭严格失败：

```bash
ros2 launch panda_manipulation moveit_scene.launch.py \
  start_pipeline:=true \
  start_pick_pipeline:=true \
  execute_trajectories:=true \
  visualize_only_execution:=false \
  fail_on_start_state_mismatch:=false \
  execution_start_tolerance:=0.05 \
  use_synthetic_pose:=true \
  dry_run:=true
```

这时如果执行器仍返回 `moveit_error_code=-4`，`/pick_plan_result` 的失败原因会同时包含 MoveIt execution status 和最近一次 start-state check 的最大关节差。

## MoveGroup Execution Backend Probe

默认执行后端是：

```text
ompl_execution_backend:=execute_trajectory
```

如果起点检查显示 `max_delta=0` 但 `/execute_trajectory` 仍返回 `moveit_error_code=-4`，可以让 OMPL 段改用 MoveGroup action 自己的 plan+execute 路径：

```bash
ros2 launch panda_manipulation moveit_scene.launch.py \
  start_pipeline:=true \
  start_pick_pipeline:=true \
  execute_trajectories:=true \
  visualize_only_execution:=false \
  ompl_execution_backend:=move_group \
  fail_on_start_state_mismatch:=true \
  execution_start_tolerance:=0.05 \
  joint_state_wait_timeout_s:=5.0 \
  use_synthetic_pose:=true \
  dry_run:=true
```

判读方式：

- `ompl_to_pre_grasp` 显示 `move_group_executed`: MoveGroup 内部执行可用，问题更可能在直接调用 `/execute_trajectory` 的路径。
- `ompl_to_pre_grasp` 仍返回 `moveit_error_code=-4`: controller/execution manager 本身不可用或配置不匹配。
- OMPL 段成功但 Cartesian 段执行失败：下一步需要给 Cartesian 轨迹也改成兼容当前 controller 的执行路径，或接入 ros2_control/Gazebo。

Current validated execution result:

```text
ompl_to_pre_grasp: move_group_executed
cartesian_approach: fraction=1.000, executed
cartesian_lift: fraction=1.000, executed
cartesian_to_place: fraction=1.000, executed
final_state: SUCCESS
```

If `cartesian_approach` fails below the configured fraction, the pipeline can trigger:

```text
cartesian_approach_recovery_triggered
ompl_to_approach_recovery
cartesian_approach
```

This means the robot first uses OMPL/MoveGroup to reach a closer intermediate pre-grasp pose, then retries the final shorter Cartesian approach. It is a recovery strategy, not a lowered success threshold.

## Useful Checks

```bash
ros2 topic echo /joint_states --once
ros2 action list
ros2 action info /execute_trajectory
ros2 action info /move_action
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /pick_demo_state --full-length
```

## Current Interpretation

- `SUCCESS` in benchmark: MoveIt produced a valid plan-only path under the configured collision checks.
- `execute_trajectories:=true`: asks MoveIt to send the planned trajectory to the execution manager.
- `moveit_error_code=-4`: execution/control failure. The new diagnostics are meant to separate start-state mismatch from controller availability or fake-controller behavior.
