# 抓取前的载荷放置端点筛选

日期：2026-09-05。状态：实验性开关，默认关闭；不是 v1.0 发布通过记录。

## 问题与证据

此前在抓取成功后才检查放置下降路径，导致机器人持物后发现没有通过碰撞检查的放置姿态。
`release_descent_yaw_survey_20260905` 的 7 条只读诊断均取得 IK 解，但状态无效；
额外 90/180/270 度朝向仍出现 panda_link5 / panda_link6 与 static_auxiliary_obstacle 接触。
该轮物理任务 0/1，153.4 秒。有限种子与朝向搜索不能证明全局不存在可行解。

## 本次改变

- 在目标物从预验证场景移除前，通过 GetPlanningScene 读取实际目标盒尺寸。
- 对每个已完成接近路径预验证的候选，构造仅用于请求的预测附着物和闭合手指状态。
- 使用与 after_lift 相同的载荷补偿关系生成名义放置手位姿，保持原放置目标和间隙。
- 每个候选最多使用 3 个种子进行碰撞感知 IK，并用 GetStateValidity 再检查带载荷端点。
- 未通过或未完成端点检查的候选，不能由原接近恢复回退逻辑重新选中。
- 服务观察使用 10 秒稳态时钟超时，丢弃超时后到达的回调；IK 自身预算为 1 秒。
- 筛选不发送执行动作，不发布预测附着物到全局场景，不提高任何碰撞或成功阈值。

开关为 launch 参数 `enable_place_endpoint_precheck` 或 benchmark 的
`--place-endpoint-precheck`。它依赖 `enable_grasp_candidate_prevalidation`；benchmark 会启用该依赖。

## 验证结果

| 实验 | 结果 | 解释 |
| --- | --- | --- |
| 全部软件回归 | 256 passed，7.36 秒 | 包含数组、载荷请求隔离、尺寸合法性、超时迟到回调、回退绕过检查 |
| colcon 构建 | 成功 | panda_manipulation |
| 中央障碍 RGB-D | 物理 0/1，27.0 秒 | 抓取前拒绝；2 个候选 OMPL 失败，其余 2 个均未找到通过检查的载荷放置端点 |
| 固定 RGB-D | 物理 1/1，106.1 秒 | 正常候选被放行，完成抓取、放置、撤离和回位 |

固定场景抬升 0.125974 m，最终放置误差 0.005589 m。
中央障碍日志的执行步骤为空；遥测只有等待、候选预验证和场景恢复阶段，没有进入抓取/转运。
两个场景不是成功率前后配对实验，不能据此声称整体成功率提高。

原始目录：

- `artifacts/runs/release_descent_yaw_survey_20260905/`
- `artifacts/runs/release_place_precheck_center_20260905/`
- `artifacts/runs/release_place_precheck_fixed_20260905/`

## 复验命令

先 source ROS 与本工程的 install/setup.bash。使用新的输出目录，保留旧结果。
实验性开关尚未写入 CSV 的配置列，不要将不同开关配置通过 `--resume` 合并。

```bash
python3 scripts/run_gazebo_physics_benchmark.py \
  --trials 1 --rgbd-perception --place-endpoint-precheck \
  --planner-id RRTConnectkConfigDefault --transfer-diagnostics --timeout-s 240 \
  --csv artifacts/runs/precheck_fixed_repeat/results.csv \
  --markdown artifacts/runs/precheck_fixed_repeat/report.md \
  --log-dir artifacts/runs/precheck_fixed_repeat/logs
```

中央障碍复验在命令中增加 `--challenge-obstacles --challenge-scenario center_tall_barrier`，
并更换输出目录。该任务目前预期仍可能失败，不能将安全拒绝算成物理成功。

## 限制与下一步

这是预测载荷的**端点**筛选，不是完整抓取、抬升、转运、下降、撤离路径的前瞻验证。
实际夹持偏移仍需在抓取后测量，已有路径与物理检查必须保留。
只支持当前单盒目标和 panda 手指模型；不支持多网格工件或任意夹具。
新增筛选仍为默认关闭的实验参数，未经过正式固定 10 / 随机 20 回归。

下一步需要以完整载荷路径可行为候选选择条件，并评估原放置任务在完整障碍场景中的可行性。
若研究允许区域内选择放置点，必须明确给出用户允许的区域边界与独立评测配置；不能悄悄挪走障碍物、
更换放置目标或放宽验收阈值来覆盖当前失败。此前固定 10 / 随机 20 的历史指标不能代表此次源码。

## 运行代码指纹

尚无候选提交号。以下 SHA256 是本轮两个 precheck 运行后的局部源码指纹，不替代完整发布清单。

| 文件 | SHA256 |
| --- | --- |
| placement_precheck.py | BD0869C3223C37B606BC0E78BE4912F7B0A6F9DD9AA14E2CA41B35ACA87D2505 |
| pick_plan_pipeline.py | FD8744AA1785C92198BBE351581338932132729767F88A61F297C873A2272832 |
| gazebo_pick.launch.py | 7CFBF06A0415811B483AF7D3C3DA65D8293B2794771192F5A598C194C29C9B7B |
| run_gazebo_physics_benchmark.py | F0F44D3321907358AAB438039BA4D9AC9382F5E0AC4BB65E3FC42A9EA4C13DBE |
