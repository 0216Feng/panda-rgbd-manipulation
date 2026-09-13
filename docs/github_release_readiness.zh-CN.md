[![English](https://img.shields.io/badge/lang-English-2563eb.svg)](github_release_readiness.md) [![简体中文](https://img.shields.io/badge/lang-简体中文-d73a49.svg)](github_release_readiness.zh-CN.md)

# GitHub 发布状态

更新日期：2026-09-13。目标：公开 `v1.0.0` 版本。

## 当前结论

发布验收门槛已经全部完成，最终判定为 `PASS`。物理仿真证据基于冻结源码 commit `2fa0440` 生成，并归档于 [`artifacts/baselines/v1.0.0`](../artifacts/baselines/v1.0.0/README.zh-CN.md)。文档和精选证据放在后续仅涉及发布包装的 commit 中；通过验收后没有再修改软件包源码。

## 已完成门槛

- 完整软件回归：318/318 项测试通过。
- 安装后合成 ROS graph smoke 通过，并明确记录 `physical_execution_verified: false`。
- 固定 RGB-D 物理验收：10/10，Wilson 95% 区间 72.2%-100%。
- 固定种子随机 RGB-D 验收：20/20，Wilson 95% 区间 83.9%-100%。
- 代表性静态障碍：3/3；三条未检查碰撞的直线路径均受阻，碰撞感知转运全部完成。
- Gazebo wrench 安全停止：1/1；注入 `16 N`、`100 ms` 扰动后 `171.21 ms` 发布零指令，低于 `200 ms` 门槛。
- 四个物理 cohort 的源码指纹一致。
- GitHub Actions run `34736305077` 已在源码 commit `2fa0440` 通过：全新 Docker 构建、确定性图表、全部测试和安装后 graph smoke 均成功。
- 主要公开文档均有英文/简体中文版本、页首语言切换按钮和自动配对检查。
- 公开证据包含 CSV/JSON/Markdown/SVG 报告、配置 sidecar 和源码 manifest；原始日志及生成 world 仅保留在本地。
- README 包含 RViz/Gazebo 截图、完整物理抓取视频，以及附有源码绑定元数据的真实 Gazebo 扰动安全停止视频。
- MIT 许可证、第三方归属、Docker/devcontainer、贡献指南和最小权限 CI workflow 完整。

## 发布边界

该标签打包已经验收的物理证据和公开文档。项目已达到完整仿真作品集发布质量，但不代表可以直接部署真机，也不构成工业安全认证。

## 对外表述规则

- 明确称为 Gazebo 物理仿真验证。
- 最终 `v1.0` cohort 与历史补充实验分开表述。
- 不把 Gazebo 真值辅助监测与评分称为纯视觉控制。
- 保留所有尝试行、固定分母、Wilson 区间及源码/配置 sidecar。
- 将真机部署、标定、时延评估与安全评估列为后续工作。
