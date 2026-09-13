[![English](https://img.shields.io/badge/lang-English-2563eb.svg)](github_release_readiness.md) [![简体中文](https://img.shields.io/badge/lang-简体中文-d73a49.svg)](github_release_readiness.zh-CN.md)

# GitHub 发布检查

更新日期：2026-09-13。目标：公开的 `v1.0` 候选仓库。

## 当前结论

公开仓库现在已达到 `v1.0` release candidate 标准。Hosted CI 已完成全新 Docker 镜像构建，并通过软件与安装后 ROS graph 检查。它还不是最终 `v1.0`：冻结 commit 的完整物理验收矩阵、最终证据整理和 release tag 仍待完成。

## 已完成

- ROS2 Jazzy 包可在维护者 WSL2 环境中构建。
- 318 项软件回归测试全部通过。
- 安装后合成 ROS graph smoke 通过，并观察到目标位姿、抓取候选和完整 dry-run 状态历史；结果明确记录 `physical_execution_verified: false`。
- 当前源码完成 5 组空载/负载配对，共 10/10 任务成功；下降入口最大关节差 `0.00499 rad`。
- 负载模式平均最大载荷漂移 `5.6 mm`，完成 5/5。
- 配置与源码 sidecar 记录 SHA256 `862ef86cae466061f65fad6c64546f0d86ca6766475712bb19470d7fca7e988e`。
- 固定 10、随机 20、静态障碍 90 轮历史证据均保留明确的源码版本边界。
- README 包含项目概述、架构、证据表、快速开始、benchmark 入口、限制与文档索引。
- README 包含一次当前源码 RGB-D 障碍运行的三张 RViz 截图和八帧动画；该运行通过抬升、放置、倾角和直线路径阻断验证。
- README 链接一次独立成功运行的 38.2 秒 H.264 Gazebo 对面视角视频，并保留 CSV、报告、配置、源码指纹、视频元数据和 SHA256。
- SVG 图表由提交的 CSV/JSON 证据确定性生成，CI 会拒绝过期图表。
- MIT 主许可证与 Apache-2.0 第三方归属完整。
- 构建缓存、rosbag、原始运行目录、历史日志和生成世界已排除出 Git。
- 公开源码/文档未发现明显凭证，公开证据中不包含本机绝对路径。
- Dockerfile、devcontainer、贡献指南和最小权限 GitHub Actions workflow 已提供。
- GitHub Actions run `34735251206` 已在 commit `42ecae5` 通过：全新 Docker 镜像构建成功，确定性作品集图表与公开证据一致，318 项测试及安装后 ROS graph smoke 全部通过。
- 发布检查脚本使用唯一时间戳目录和稳定 RGB-D 障碍配置；更新后 fresh central-barrier 物理验收通过。
- 主要公开文档提供英文/简体中文配对、页首语言切换按钮与自动完整性测试。
- 新增统一 `v1.0` 发布 runner：强制冻结且干净的 commit，执行固定/随机/障碍/安全停止批次，验证包源码指纹并生成机器可读总判定。物理 smoke 已通过固定 `1/1`、固定种子随机 `1/1`、代表性障碍 `3/3` 和真实 wrench 安全停止 `1/1`；正式物理矩阵仍待运行。
- 故障注入安全停止视频已完成：一次 `16 N`、`100 ms` Gazebo wrench 得到严格 `SAFE_STOP_PASS`，零指令延迟 `107.8 ms`。H.264、poster、源码/配置指纹、CSV 与 JSON/Markdown 判定均已进入公开候选证据包。

## 首次发布已完成

1. 已审核并提交 234 个公开候选文件，未包含本地简历材料、原始 runs、构建缓存、凭证或本机路径。
2. 已创建并推送公开仓库 `0216Feng/panda-rgbd-manipulation`。
3. 已在 hosted CI 实际通过后添加 workflow badge。

## 上传后的最终 v1.0 门槛

- 从同一冻结 commit 重新运行固定 10 和随机 20 次 RGB-D 物理验收，保留全部尝试与 Wilson 区间。
- 从同一 commit 运行三类代表性静态障碍和一次安全停止注入。
- 从最终冻结 commit 重新运行安全停止，使其源码 manifest 与固定/随机/障碍发布 cohort 一致；公开演示视频与严格判定格式已经完成。
- 用最终 commit SHA 替换仅源码 hash 的引用，并标记 `v1.0.0`。

## 对外表述规则

- 首个公开版本称为 `v1.0 candidate`，不称为 hardware-ready 软件。
- 当前源码结果与历史证据分开表述。
- 不把 Gazebo 真值辅助监测与纠偏称为纯视觉控制器。
- 不在缺少两个分母时合并基础设施排除后成功率与原始成功率。
- 保留失败 CSV 行和配置/源码 sidecar。
