[![English](https://img.shields.io/badge/lang-English-2563eb.svg)](README.md) [![简体中文](https://img.shields.io/badge/lang-简体中文-d73a49.svg)](README.zh-CN.md)

# 实验产物

生成的 benchmark 数据统一放在项目源码之外，便于快速浏览源码与运行入口。

## 目录结构

- `baselines/`：主 README 引用的精选 benchmark 报告。
- `current/`：支持当前开发工作的近期诊断运行。
- `runs/`：新本地实验的默认输出目录，Git 会忽略该目录。
- `archive/`：按主题整理的历史实验，Git 会忽略该目录。
- `archive_manifest_20260829.csv`：2026-08-29 整理时每个文件的原始名称、新路径、类型、大小和修改时间。

2026-08-29 整理过程中没有删除历史实验。

最终公开发布证据见 [`baselines/v1.0.0/README.zh-CN.md`](baselines/v1.0.0/README.zh-CN.md)。该目录保留精简报告与源码/配置指纹，原始日志和生成 world 不进入 Git。

## 新实验

每次实验应使用独立目录，不要将报告直接写入项目根目录：

```bash
mkdir -p artifacts/runs/my_experiment
python3 scripts/run_gazebo_physics_benchmark.py \
  --trials 1 \
  --csv artifacts/runs/my_experiment/results.csv \
  --markdown artifacts/runs/my_experiment/report.md \
  --svg artifacts/runs/my_experiment/chart.svg \
  --log-dir artifacts/runs/my_experiment/logs
```

只有在样本量、配置、成功标准与失败统计均记录清楚后，才能把 `runs/` 中的结果提升到 `baselines/`。
