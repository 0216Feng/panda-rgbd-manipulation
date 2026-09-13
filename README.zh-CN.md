[![English](https://img.shields.io/badge/lang-English-2563eb.svg)](README.md) [![简体中文](https://img.shields.io/badge/lang-简体中文-d73a49.svg)](README.zh-CN.md)

# Panda RGB-D 机械臂抓取系统

这是一个基于 ROS2 Jazzy、MoveIt2、ros2_control 与 Gazebo Harmonic 的可复现 Franka Panda 碰撞感知抓取项目。系统覆盖 RGB-D 目标定位、抓取位姿生成、OMPL 全局规划、接触区 Cartesian 运动、双指物理抓取、失败恢复和独立物理结果验证。

本仓库定位为机械臂运动规划岗位的仿真作品集，不代表已完成真机部署或工业安全认证。

## 项目亮点

- 基于 RGB 分割与深度聚类的无标记 RGB-D 目标定位。
- 使用 PlanningScene 表达碰撞几何，支持 RRTConnect、PRM 与 RRTstar。
- OMPL 与 Cartesian 混合执行，并基于实测关节状态重规划。
- 独立的机械臂控制器与双指同步 ros2_control 控制器。
- Gazebo 接触抓取、载荷漂移监测、直立放置、撤离和回原位验证。
- 静态/动态障碍实验、结构化失败原因、CSV/JSON/Markdown 报告、rosbag 与源码/配置指纹。
- Docker、devcontainer 和 GitHub Actions 软件检查。

## 实际运行效果

下图来自当前源码在 fresh-world Gazebo Harmonic 中运行的一次 RGB-D 障碍抓取。RViz 展示由接触仿真驱动的实时状态，不是预录动画或真机画面。

| 抓取前场景与目标估计 | 物理夹取与抬升 | 碰撞感知转运轨迹 |
| --- | --- | --- |
| ![RGB-D 抓取前场景](docs/assets/demo/rgbd_pregrasp.png) | ![抓取与抬升](docs/assets/demo/grasp_and_lift.png) | ![避障转运轨迹](docs/assets/demo/obstacle_transfer_trail.png) |

![障碍抓取过程](docs/assets/demo/obstacle_pick_sequence.gif)

该次采集通过独立物理验证：抬升 `0.126 m`、放置误差 `4.1 mm`、最终倾角 `0.00 deg`，并确认直线路径被障碍阻断。完整命令与来源见[演示素材说明](docs/assets/demo/README.zh-CN.md)。

[![Gazebo 物理抓取视频](docs/assets/demo/gazebo_obstacle_pick_poster.png)](docs/assets/demo/gazebo_obstacle_pick.mp4)

上方 38.2 秒视频采用障碍物对面视角，覆盖抓取前接近、物理抬升、避障转运、直立释放、撤离和回原位。验证器测得最大抬升 `0.457 m`、放置误差 `11.5 mm`、最终倾角 `0.00 deg`，并确认碰撞感知直线路径受阻。视频为完整 76.2 秒相机流的 2 倍速播放，没有删减任务阶段；它是 Gazebo 仿真画面，不是真机视频。

### 故障注入安全停止

[![Gazebo wrench 安全停止视频](docs/assets/demo/gazebo_safe_stop_poster.png)](docs/assets/demo/gazebo_safe_stop.mp4)

这段独立的 13.8 秒 H.264 视频展示一次预期内的安全失败：系统在 Servo 下降阶段受到 `16 N`、`100 ms` 的有界 Gazebo wrench 扰动，载荷/接触 watchdog 在 `107.8 ms` 内停止对应阶段并发布零指令，低于 `200 ms` 门槛。抓取流水线正确返回 `FAILED`，严格独立判定为 `SAFE_STOP_PASS`。证据见[源码绑定报告](artifacts/baselines/v1_candidate_20260913/safe_stop_video/README.md)。

## 系统架构

```mermaid
flowchart LR
    RGB[RGB 图像] --> Perception[RGB-D 目标估计]
    Depth[深度点云] --> Perception
    TF[tf2] --> Perception
    Perception --> Grasp[抓取候选]
    Grasp --> Pipeline[抓取流水线]
    Scene[PlanningScene] --> Planning[OMPL 与 Cartesian 规划]
    Pipeline --> Planning
    Planning --> Arm[机械臂控制器]
    Pipeline --> Hand[双指控制器]
    Arm --> Gazebo[Gazebo 接触动力学]
    Hand --> Gazebo
    Gazebo --> Monitor[接触与载荷监测]
    Monitor --> Pipeline
    Gazebo --> Validator[物理结果验证器]
    Pipeline --> Validator
    Validator --> Report[CSV / JSON / Markdown / rosbag]
```

详见[架构说明](docs/architecture.zh-CN.md)。

## 实验证据

不同源码版本和实验范围的结果严格分开，失败样本保留在分母中。

| 证据集 | 结果 | 能够证明的内容 |
| --- | ---: | --- |
| 当前源码匹配载荷转运 | 10/10 任务；5/5 配对 | 从匹配下降入口执行空载/负载转运的重复性 |
| 归档固定 RGB-D 发布批次 | 10/10 | 固定目标物理仿真验收 |
| 归档随机 RGB-D 发布批次 | 19/20 | 固定种子的工作区变化，保留 1 次执行失败 |
| 归档静态障碍矩阵 | 81/90 原始；81/86 有效启动 | 三类规划器、三类障碍，86/86 有效启动的直线路径均被阻断 |

当前源码配对矩阵每组包含一次空载与一次负载运行。5 组下降入口的最大关节差不超过 `0.00499 rad`，负载漂移平均为 `5.6 mm`，两种模式均完成 5/5。由于空载误差接近零，项目不把相对 RMS 倍率作为主指标。

- [当前源码配对载荷报告](artifacts/baselines/v1_candidate_20260910/payload_transfer_paired5/report.md)
- [归档 RGB-D 发布证据](artifacts/baselines/rgbd_release_20260905/README.md)
- [归档 90 轮障碍实验](artifacts/baselines/gazebo_obstacle_90_trials.md)
- [GitHub 发布检查](docs/github_release_readiness.zh-CN.md)

![验收结果总览](docs/assets/benchmark_overview.svg)

![匹配载荷诊断](docs/assets/payload_diagnostics.svg)

## 快速开始

支持 Ubuntu 24.04 或 WSL2 Ubuntu 24.04 + ROS2 Jazzy。请在仓库根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
WORKSPACE_DIR="$PWD" bash scripts/setup_wsl_ros2_jazzy.sh
source install/setup.bash
SKIP_PHYSICAL=1 bash scripts/run_release_candidate_validation.sh
bash scripts/run_rgbd_markerless_pick.sh
```

最后一条命令启动 RGB-D 抓取演示。WSL2 需要启用 WSLg 才能显示 RViz/Gazebo。安装脚本不会修改 shell 启动文件。

仅检查已安装 ROS 节点图：

```bash
python3 scripts/check_ros_graph_smoke.py --timeout-s 45
```

持续显示 RGB-D 点云：

```bash
bash scripts/run_rgbd_visualization.sh
```

执行一次 fresh-world 物理 benchmark：

```bash
mkdir -p artifacts/runs/rgbd_smoke
python3 scripts/run_gazebo_physics_benchmark.py \
  --trials 1 --rgbd-perception --timeout-s 300 \
  --csv artifacts/runs/rgbd_smoke/results.csv \
  --markdown artifacts/runs/rgbd_smoke/report.md \
  --log-dir artifacts/runs/rgbd_smoke/logs
```

原生环境、Docker、devcontainer 与 CI 步骤见[复现指南](docs/reproduction.zh-CN.md)。冻结干净 commit 后，可运行完整 `v1.0` 物理发布门禁：

```bash
bash scripts/run_v1_release_validation.sh
```

修改发布基础设施时先使用 `V1_PROFILE=smoke`。smoke 只验证 runner，不能作为 `v1.0` 可靠性结论。

## 演示与评测

推荐依次录制无标记 RGB-D 抓取、静态障碍与安全停止注入，完整脚本见[演示指南](docs/demo_script.zh-CN.md)。图表可通过以下命令更新并检查是否与数据一致：

```bash
python3 scripts/generate_portfolio_assets.py
python3 scripts/generate_portfolio_assets.py --check
bash scripts/run_gazebo_camera_video.sh
```

主要 benchmark 入口：

```bash
# 固定与随机 RGB-D 物理实验
python3 scripts/run_gazebo_physics_benchmark.py --trials 10 --rgbd-perception
python3 scripts/run_gazebo_physics_benchmark.py \
  --trials 20 --rgbd-perception --randomize-target --seed 42

# 静态障碍规划器/场景矩阵
python3 scripts/run_gazebo_physics_benchmark.py \
  --trials 3 --challenge-obstacles --planner-suite

# 动态 RGB-D 障碍取消与重规划
python3 scripts/run_dynamic_replanning_benchmark.py \
  --trials 3 --planner-suite --scenario-suite --timeout-s 240

# 匹配的空载/负载转运诊断
python3 scripts/run_payload_transfer_diagnostics.py \
  --runs-per-mode 5 --modes unloaded loaded \
  --output-dir artifacts/runs/payload_transfer_paired5
```

## 仓库结构

```text
.
|-- src/panda_manipulation/   # ROS2 节点、launch、配置、模型和测试
|-- scripts/                  # 安装、smoke test 与实验 runner
|-- artifacts/baselines/      # 可公开的精简数据、报告和配置证据
|-- docs/                     # 架构、验证和技术路线图
|-- Dockerfile
`-- .github/workflows/ros2-ci.yml
```

## 验证边界

当前维护者 ROS2 Jazzy 环境中共有 318 项软件测试通过。GitHub Actions 将构建 Docker 镜像、运行软件检查并验证已安装的合成 ROS 节点图；在首次推送并实际观察 hosted CI 前，不声明 CI 已通过。

Gazebo 目标真值与接触数据目前参与在线载荷监测、有界纠偏和最终评分。RGB-D 提供目标估计，但系统因此不是纯视觉控制器。目标检测采用颜色/聚类方法，并非任意物体 6D 位姿估计器。

当前限制：

- 仅完成仿真，无真机驱动、手眼标定或硬件安全论证。
- 主执行流水线为 Python，C++ 实时边界仍在后续计划中。
- 接触效果依赖 Gazebo 物理参数、摩擦和控制器调参。
- MoveIt Servo 为可选实验下降后端；当前发布证据使用 Cartesian path。

真机迁移、C++ 执行、标定和力控规划见[长期路线图](docs/roadmap.zh-CN.md)。

## 文档

- [中文文档索引](docs/README.zh-CN.md)
- [架构说明](docs/architecture.zh-CN.md)
- [复现与 CI](docs/reproduction.zh-CN.md)
- [演示与录制](docs/demo_script.zh-CN.md)
- [发布检查](docs/github_release_readiness.zh-CN.md)
- [长期路线图](docs/roadmap.zh-CN.md)

## 许可证

项目原创代码与文档采用 MIT License。基于 MoveIt Resources 修改的 Panda 模型仍遵循 Apache-2.0，详见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) 与 [LICENSES/Apache-2.0.txt](LICENSES/Apache-2.0.txt)。
