[![English](https://img.shields.io/badge/lang-English-2563eb.svg)](README.md) [![简体中文](https://img.shields.io/badge/lang-简体中文-d73a49.svg)](README.zh-CN.md)

# 演示素材来源

三张截图采集自 2026-09-11 的一次 fresh-world Gazebo Harmonic 运行，并直接截取实时 RViz 窗口。该运行使用无标记 RGB-D 定位、`center_tall_barrier` 障碍场景、RRTConnect、物理双指控制、接触式载荷监测与独立 Gazebo 结果验证。

`obstacle_pick_sequence.gif` 是同一次运行的八帧摘要。三张 PNG 保留较高分辨率，可用于 README、技术汇报或设计评审。

`gazebo_obstacle_pick.mp4` 来自 2026-09-12 的另一次 fresh-world 运行，使用 Gazebo 世界中默认关闭的对面视角传感器。该视角避免高障碍物遮挡抓取点和指尖接触。公开 H.264 视频为 960x540、15 fps、38.2 秒，是完整 76.2 秒传感器流的 2 倍速版本，没有删除任何任务阶段或失败尝试。`gazebo_obstacle_pick_poster.png` 截取自抬升阶段。

这些素材是物理接触仿真证据，不是真机画面。真机演示需要另行完成机械臂获取、标定、驱动适配、工作区安全控制和急停流程。

`gazebo_safe_stop.mp4` 来自 2026-09-13 的第三次 fresh-world 运行，展示 Servo 下降期间施加真实 Gazebo wrench 后机械臂停止。13.8 秒 H.264 视频为 960x540、15 fps、2 倍速；`gazebo_safe_stop_poster.png` 展示带载停止状态。

## 运行结果

- 流水线与验证器：`SUCCESS`
- 抬升高度：`0.1258 m`
- 最终放置误差：`0.0041 m`
- 最终物体倾角：`0.00 deg`
- PlanningScene 确认直线路径受阻：是
- RGB-D 目标平均误差：10 个样本 `0.0011 m`
- OMPL 规划耗时：`0.0441 s`
- OMPL 关节路径长度：`6.3205 rad`

精简结果、配置和源码指纹保存在 `artifacts/baselines/v1_candidate_20260911/portfolio_capture/`。

视频对应的独立运行也通过验证：最大抬升 `0.4567 m`、放置误差 `0.0115 m`、最终倾角 `0.00 deg`、RGB-D 平均目标误差 `0.0011 m`，且 `1/1` 轮碰撞感知直线路径受阻。其 CSV、报告、配置、源码指纹和视频元数据位于 `artifacts/baselines/v1_candidate_20260912/gazebo_video/`。

安全停止运行在下降第 3 阶段施加 `[0, 16, 0] N`、持续 `100 ms`。wrench 已施加并清除，载荷/接触 watchdog 停止对应阶段，零指令延迟为 `107.8 ms`，低于 `200 ms` 门槛。任务终态按预期为 `FAILED`，独立安全判定为 `SAFE_STOP_PASS`；源码绑定证据位于 `artifacts/baselines/v1_candidate_20260913/safe_stop_video/`。

## 截图与 GIF 复现

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/run_gazebo_physics_benchmark.py \
  --trials 1 \
  --challenge-obstacles \
  --challenge-scenario center_tall_barrier \
  --rgbd-perception \
  --planner-id RRTConnectkConfigDefault \
  --grasp-height-offset 0.115 \
  --grasp-pitch-offset-deg -20 \
  --loaded-path-precheck \
  --transfer-diagnostics \
  --rviz --gazebo-gui \
  --timeout-s 360 \
  --hold-after-result-s 90 \
  --csv artifacts/runs/portfolio_capture/results.csv \
  --markdown artifacts/runs/portfolio_capture/report.md \
  --svg artifacts/runs/portfolio_capture/report.svg \
  --log-dir artifacts/runs/portfolio_capture/logs
```

截图是展示素材，机器可读 CSV/JSON 才是定量结论的事实来源。

## Gazebo 视频复现

录制器与 benchmark 共用隔离 ROS domain。只有下方命令会启用对面视角 camera 和专用 `ros_gz_bridge`，普通 benchmark 不承担额外渲染开销。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash scripts/run_gazebo_camera_video.sh \
  artifacts/runs/gazebo_camera_video
```

命令将原始 MP4、CSV、Markdown/SVG 报告、源码/配置 sidecar 和日志写入指定目录。公开 H.264 文件由以下命令生成：

```bash
ffmpeg -i gazebo_rgb_camera_raw.mp4 \
  -vf "setpts=0.5*PTS,scale=960:540:flags=lanczos,eq=contrast=1.03:saturation=1.08" \
  -an -c:v libx264 -crf 22 -pix_fmt yuv420p -movflags +faststart \
  gazebo_obstacle_pick.mp4
```

## 安全停止视频复现

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash scripts/run_safe_stop_video.sh \
  artifacts/runs/safe_stop_video
```

脚本既可使用 Linux 原生 FFmpeg，也会在 WSL 中发现已有 Windows FFmpeg。只有真实 wrench 生命周期、watchdog 阶段、零指令延迟和非空视频全部通过时才接受录制。
