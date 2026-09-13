[![English](https://img.shields.io/badge/lang-English-2563eb.svg)](demo_script.md) [![简体中文](https://img.shields.io/badge/lang-简体中文-d73a49.svg)](demo_script.zh-CN.md)

# 演示与录制指南

## 90 秒主演示

完成一次构建后，启动无标记 RGB-D 物理抓取：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash scripts/run_rgbd_markerless_pick.sh
```

建议将 Gazebo 与 RViz 并排录制：

1. RViz 展示 RGB-D 点云、接受的目标点、融合位姿、PlanningScene 与规划轨迹。
2. Gazebo 展示红色目标方块、桌面、障碍物、Panda 与两根手指。
3. 机械臂移动到 pre-grasp，完成最后接近，闭合双指并实际抬升物体。
4. 转运路径绕开 PlanningScene 障碍，低速下降、直立释放、撤离并回原位。
5. 最后展示流水线与独立物理验证器的结构化 `SUCCESS` 记录。

```bash
ros2 topic echo /pick_plan_result --once --full-length
ros2 topic echo /gazebo_pick_validation --once --full-length
ros2 topic echo /rgbd_target_detection --once --full-length
```

仅录制第三人称 Gazebo 画面并同步保留 CSV/报告证据：

```bash
bash scripts/run_gazebo_camera_video.sh \
  artifacts/runs/gazebo_camera_video
```

该命令会启用默认关闭的 `/showcase_camera/image_raw`，并桥接到 benchmark 的隔离 ROS domain。感知仍使用独立顶置 RGB-D 相机，showcase camera 只用于记录画面。

## 静态避障证明

```bash
python3 scripts/run_gazebo_physics_benchmark.py \
  --trials 1 --challenge-obstacles \
  --planner-id RRTConnectkConfigDefault \
  --rviz --gazebo-gui --timeout-s 420 --hold-after-result-s 20 \
  --csv artifacts/runs/obstacle_demo/results.csv \
  --markdown artifacts/runs/obstacle_demo/report.md \
  --log-dir artifacts/runs/obstacle_demo/logs
```

演示时先说明未检查碰撞的 Cartesian 直线可完整生成，而碰撞感知版本会被阻断；随后展示 OMPL 替代路径和物理放置。报告中的 `direct_path_blocked` 才是可审计证据，RViz trail 本身不能证明无碰撞。

## 安全停止片段

使用维护中的真实 wrench 配置录制视频和机器可读判定：

```bash
bash scripts/run_safe_stop_video.sh artifacts/runs/safe_stop_video
```

命令通过对面视角 Gazebo camera 录制，在 Servo 下降第 3 阶段施加有界 `16 N`、`100 ms` wrench。只有 wrench 已施加并清除、对应 watchdog 停止任务、`200 ms` 内观察到零指令且视频包含有效帧时，脚本才成功。抓取任务本身预期以 `FAILED` 结束，这正是安全结果。

## 录制规则

- 每段视频开头和结尾保留命令与结果。
- 视频旁记录源码 commit SHA 与实验配置。
- benchmark 批次不能剪掉失败轮次。
- 明确说明 Gazebo 真值参与在线监测与评分。
- 不把 plan-only 动画或 attached object 移动称为物理抓取。
- 第三人称画面标注为 Gazebo 接触仿真，不标注为真机。
