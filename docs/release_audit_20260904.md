# v1.0 发布前审计：2026-09-04

补充更新：2026-09-05。

> 本文是历史审计记录。2026-09-11 起的权威候选状态、310 项软件回归和
> 当前源码配对实验证据见 [GitHub Release Readiness](github_release_readiness.md)。

## 当前结论

发布检查已建立，但当前不是已验收的 v1.0。近期按 [发布检查](github_release_readiness.zh-CN.md) 收尾，不将历史专项成功等同于当前完整系统成功。

## 已完成

- 新增发布检查，并从 README 与文档索引链接。
- 只读检查 WSL：未发现仍在运行的载荷诊断、物理 benchmark 或 Gazebo 进程。
- 确认双向扰动产物路径碰撞，先用两项回归测试复现，再修复。
- 每次 `run_trial` 通过 `tempfile.mkdtemp` 分配独立 `attempts/trial_NNNN_*` 目录；世界、遥测、日志和 rosbag 全部位于该目录。CSV 保留实际路径，重复方向、相同 case 重试和中断续跑均不会复用原有产物目录。
- 定向运行器与诊断统计测试：33 项通过。回归测试覆盖正反方向和相同方向重复尝试，均在启动 ROS / Gazebo 之前验证路径隔离。
- 执行 `SKIP_PHYSICAL=1 bash scripts/run_release_candidate_validation.sh`：构建成功，全量 204 项测试通过，0 错误、0 失败、0 跳过。该命令明确跳过物理验收。
- 随后完成两次 headless Gazebo 物理 smoke：第 3 下降阶段，正负 Y 各 16 N、持续 100 ms、阶段延迟 500 ms，2/2 `SAFE_STOP_PASS`。施力到零命令延迟分别为 104.900 ms、99.518 ms，均低于 200 ms。两份日志的 APPLIED 事件分别保留正确的正负力值，遥测和 summary 文件均存在且位于不同 attempt 目录。
- 本轮物理结果位于 `artifacts/runs/release_artifact_isolation_smoke_20260904/`。这是停止链路 smoke，不是正常抓取成功率；Wilson 95% 区间为 34.2%-100.0%，没有扩展为全阶段可靠性结论。

## 历史实验完整性

`artifacts/runs/servo_physical_disturbance_bidirectional_matrix_20260903/payload_transfer_comparison.csv` 当前包含 16 条已写入结果，不是完整的 36 次矩阵。

- trial 13 / 14：pair 2、stage 1、正负 Y 方向共用同一日志和遥测路径。
- trial 15 / 16：pair 2、stage 2、正负 Y 方向共用同一日志和遥测路径。
- CSV 行仍保留，但两组方向的原始产物无法仅凭这些共享路径分别追溯；不将这四行作为新增的完整发布证据。
- 本次不删除、不重写历史 CSV，也不声称已经恢复被覆盖内容。
- 后续从修复后的运行器生成新目录和新报告；重跑结果不得冒充历史原始记录。旧版基线必须明确保留代码时期、配置和样本边界。

## 待解决的发布缺口

### 本轮复现改进

- 安装脚本从自身位置解析完整仓库，支持带空格的路径；保留默认外部 workspace，也支持 `WORKSPACE_DIR="$PWD"` 原地构建。错误源码、已有其他源码链接和嵌套 workspace 会被拒绝，不替换用户已有链接。
- rosdep update 重试耗尽或 install 失败后立即中止，不继续构建或输出 Setup complete；不再修改 `.bashrc`。
- Dockerfile 移除 `|| true`，补齐安装工具和 Gazebo 控制依赖，使用与本地相同的仓库布局；`.dockerignore` 排除构建缓存、Git 元数据、实验结果和 ROS bags。
- devcontainer 明确绑定当前 checkout，移除 privileged 模式；本轮没有配置容器 GUI 转发。
- 新增 `.github/workflows/ros2-ci.yml`：构建同一 Dockerfile、运行软件验收和隔离 ROS graph smoke，失败时保留诊断产物。工作流仅有 contents:read 权限，不保留 checkout 凭据。
- 新增 12 项复现测试全部通过，包括模拟依赖失败、路径和链接保护、配置检查及无效超时拒绝。模拟安装测试不实际执行 apt / rosdep / colcon。
- 复现改动后的完整构建与测试已通过：216 项，0 错误、0 失败、0 跳过；结果位于 `build/panda_manipulation/pytest.xml`。前文 204 项是改动前的检查点。
- 实际 ROS graph smoke 成功，耗时约 2.95 s，收到目标位姿、候选和完整 SUCCESS 历史；它明确使用 synthetic pose / dry_run，不是物理抓取结果。初次 smoke 暴露隔离 Context / executor 不匹配，修复后成功，失败日志保留。
- 极短超时的实际 smoke 正确返回 exit 1 和 FAILED JSON；检查确认没有残留的 smoke / demo / task manager / grasp generator 进程。
- 本机 WSL 没有 Docker；尚未构建干净镜像或运行 GitHub-hosted CI。这两个门槛仍未验收，不能用本机测试或 YAML 解析替代。

详见 [复现与 CI 说明](reproduction.md)。ROS smoke 记录位于 `artifacts/runs/release_ros_graph_smoke_20260904/`，故意超时记录位于 `artifacts/runs/release_ros_graph_timeout_20260904/`。

### 剩余门槛

| 优先级 | 发现 | 下一步验收 |
| --- | --- | --- |
| 进行中 | 2026-09-05 已初始化本地 Git 仓库，分支 main；尚无提交 | 完成公开内容审计后建立候选提交和参数快照，当前不能引用 commit id |
| 历史基线完成，当前源码待复验 | 2026-09-05 同指纹配置固定 10 次达 10/10，随机 20 次达 19/20；随后修改了场景、载荷状态和端点筛选 | 原始失败保留；不能将历史指标当作当前源码验收。见 fixed10_acceptance.md、random20_acceptance.md 和公开基线目录 |
| 待验收 | Docker 的依赖失败吞错已修复，镜像尚未实际构建 | 在干净环境实际构建和运行；记录镜像与依赖版本 |
| 已处理 | README 与安装入口的开发者专属路径已移除 | 带空格、非 cwd 检出路径和外部 workspace 测试通过 |
| 待验收 | CI 工作流已建立但尚未运行远程 job | 首次授权发布后验证 GitHub 执行结果，不能提前展示绿色徽章 |
| P1 | 贡献说明已建立，根目录许可证与第三方资产授权仍待核对 | package.xml 声明 MIT 不等同于第三方资产均获 MIT 授权 |
| P1 | 当前发布脚本默认只有单次 RGB-D 和障碍 smoke，缺少随机验收阶段 | 将 smoke 与正式验收区分，报告不能把 smoke 通过标成全系统发布通过 |
| 已处理 | 软件测试与物理验收分开记录 | 2026-09-05 当前完整回归 269 项通过；名义载荷路径前瞻加 110 mm 抓取偏移的固定 smoke 1/1、中央障碍 0/1。见 loaded_path_precheck_20260905.md，发布仍未验收 |
| P2 | 主演示、故障视频和干净环境复现尚未以同一候选版本确认 | 固定演示入口、输出材料与版本绑定，最后再申请远程发布确认 |

## 范围与统计约束

- 本日变更覆盖实验产物存储和复现工具，不改变碰撞检查、接触、放置和停止延迟判定阈值。
- 33 项定向单元测试与 204 项全量软件测试不是 Gazebo 物理回归；新增两次物理 smoke 仅覆盖第 3 阶段，正式 RGB-D 端到端发布回归仍待完成。
- 真值参与的在线监测、补偿和验证必须披露，不把仿真真值控制描述为纯视觉控制。
- 零命令发送延迟与机械臂实际停止时间分开表述。
- 已初始化本地 Git，尚未提交或推送，未创建远程仓库，未将旧长期目标标记完成。
