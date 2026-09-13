[![English](https://img.shields.io/badge/lang-English-2563eb.svg)](reproduction.md) [![简体中文](https://img.shields.io/badge/lang-简体中文-d73a49.svg)](reproduction.zh-CN.md)

# 复现与软件 CI

## 支持环境

- Ubuntu 24.04 + ROS2 Jazzy，包括 WSL2 Ubuntu。
- 所有命令均从完整仓库根目录运行，不要在仅包含转发脚本的目录运行。
- 安装脚本仅在 apt 与首次 rosdep 配置时使用 sudo，不修改 shell 启动文件。
- 本流程不支持真机控制，文中的“物理结果”均指 Gazebo 接触动力学仿真。

## 原生安装

```bash
WORKSPACE_DIR="$PWD" bash scripts/setup_wsl_ros2_jazzy.sh
source /opt/ros/jazzy/setup.bash
source install/setup.bash
SKIP_PHYSICAL=1 bash scripts/run_release_candidate_validation.sh
python3 scripts/check_ros_graph_smoke.py
bash scripts/run_rgbd_markerless_pick.sh
```

首条命令安装依赖并在当前 checkout 中构建。依赖下载或安装失败会立即停止，修复错误后再重试；rosdep 失败后不会伪装成构建成功。

如需外部 workspace，可省略 `WORKSPACE_DIR` 使用 `$HOME/robot_ws`，或显式指定路径。源码位置由脚本自身确定，`REPO_SOURCE_DIR` 可用于覆盖。脚本不会替换指向其他 checkout 的既有链接，也不允许把外部 workspace 建在源码目录内部。

## Docker 与 devcontainer

```bash
docker build -t panda-manipulation:dev .
docker run --rm -it panda-manipulation:dev bash
```

镜像中的 `/workspace` 是项目根目录。构建后运行软件验证，并 source `/workspace/install/setup.bash`。`.dockerignore` 排除了 Git 元数据、已有构建、密钥、实验产物和 rosbag。devcontainer 将当前 checkout 映射到 `/workspace`，默认用于无界面软件开发，不包含 GUI 转发设置。

## CI 范围

`.github/workflows/ros2-ci.yml` 在 Ubuntu 24.04 runner 上构建同一个 Dockerfile，并依次执行：

1. `colcon build`、完整包测试与 `colcon test-result`。
2. 在隔离 ROS domain 中检查 `/detected_object_pose`、`/grasp_candidates` 与 `/task_state`。
3. 即使前一步失败，也收集测试 XML 与日志。

节点图 smoke 使用 `use_synthetic_pose:=true` 与 `dry_run:=true`，JSON 明确写入 `physical_execution_verified: false`。通过仅证明安装后的入口和合成 topic 链路可用，不证明视觉精度、规划、Gazebo 接触或物理抓取。

在首次推送并实际观察 GitHub-hosted workflow、以及完成全新 Docker 镜像构建之前，不应声明干净环境 CI 已通过。

## 物理发布门槛

物理 smoke 和发布实验与软件 CI 分开。固定/随机 RGB-D、障碍场景与物理结果要求见[长期路线图](roadmap.zh-CN.md)。合成 ROS 图结果不得计入物理成功率。

先验证发布基础设施，但不形成可靠性结论：

```bash
V1_PROFILE=smoke bash scripts/run_v1_release_validation.sh
```

正式命令会在昂贵的物理矩阵开始前，强制要求已有 commit 且工作树干净：

```bash
bash scripts/run_v1_release_validation.sh
```

它执行固定 10 次、固定种子随机 20 次、三类代表性静态障碍与一次真实 Gazebo wrench 安全停止。`evaluate_v1_release.py` 会拒绝样本缺失、机械臂源码指纹混用、障碍直线路径未阻断或安全停止生命周期不完整，并输出统一 JSON/Markdown 判定。

## 参考资料

- [Docker build context 与 .dockerignore](https://docs.docker.com/build/concepts/context/)
- [OSRF Jazzy desktop-full 镜像定义](https://github.com/osrf/docker_images/tree/master/ros/jazzy/ubuntu/noble/desktop-full)
- [GitHub checkout action](https://github.com/actions/checkout)
- [GitHub artifact upload action](https://github.com/actions/upload-artifact)
