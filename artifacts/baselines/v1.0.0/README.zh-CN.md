[![English](https://img.shields.io/badge/lang-English-2563eb.svg)](README.md) [![简体中文](https://img.shields.io/badge/lang-简体中文-d73a49.svg)](README.zh-CN.md)

# v1.0 验收证据

本目录保存最终 `v1.0` 物理仿真门槛的精选公开证据。完整矩阵于 2026-09-13 基于源码 commit `2fa0440a00afeb29ff1987e88ca6d676d2df65ac`，使用 `full` profile 执行。

## 结果

| Cohort | 结果 | 平均感知误差 | 平均放置误差 | 附加门槛 |
| --- | ---: | ---: | ---: | --- |
| 固定 RGB-D | 10/10 | 0.55 mm | 1.74 mm | 要求 100% |
| 固定种子随机 RGB-D | 20/20 | 1.29 mm | 2.35 mm | 要求不低于 85% |
| 代表性静态障碍 | 3/3 | 2.49 mm | 4.02 mm | 3/3 直线路径受阻 |
| Gazebo wrench 安全停止 | 1/1 | 不适用 | 不适用 | 171.21 ms，低于 200 ms 门槛 |

最终机器可读判定为 `PASS`。运行物理矩阵前，完整软件测试也已达到 318/318 通过。

## 方法

- 每次物理试验都启动全新的 Gazebo Harmonic world。
- 随机目标位置使用发布 runner 固定的随机种子与样本数。
- 独立验证器测量目标抬升、放置误差、最终倾角和直线路径阻断，不只依赖流水线返回的成功日志。
- CSV 保留每次尝试；配置与源码 sidecar 保存准确参数和软件包 SHA256 manifest。
- 安全停止 cohort 在下降阶段注入有界 `16 N`、`100 ms` Gazebo wrench，并要求系统在 `200 ms` 内发布零指令。

为控制仓库体积，原始进程日志和生成 world 没有进入公开版本。公开证据保留结果 CSV、生成报告、配置、源码指纹和汇总判定，足以审计项目对外指标。

## 边界

这些结果属于 Gazebo 接触物理仿真，不是真机验证或安全认证。Gazebo 真值参与独立评分和运行时安全监测，因此项目不声称实现了纯视觉控制器。

- [汇总报告](release_summary.md)
- [机器可读判定](release_summary.json)
- [固定 RGB-D 报告](fixed_rgbd/report.md)
- [随机 RGB-D 报告](random_rgbd/report.md)
- [静态障碍报告](static_obstacles/report.md)
- [安全停止报告](safe_stop/physical_disturbance_safety.md)
