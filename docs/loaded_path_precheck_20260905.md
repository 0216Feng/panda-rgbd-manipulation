# 名义载荷路径前瞻与手掌间隙

日期：2026-09-05。状态：实验开关，默认关闭；不是完整发布验收。

## 实现

新增 `loaded_path_precheck.py`，在候选接近路径和放置端点检查后，预测以下机械臂路径：

1. 从该候选接近轨迹终点带载荷抬升。
2. 按当前任务配置使用 Cartesian 或 OMPL 转运至 pre-place。
3. 使用现有 `interpolated_place_descent_poses` 和配置的段数逐段下降。
4. 用下降终点的 FK 推算名义释放物体位姿，而不是把物体直接放在理想目标点。
5. 在请求局部场景中移除预测附着关系、恢复释放物体，检查短 OMPL 撤离路径。
6. 在包含已释放物体的同一局部场景中检查回位路径。

每段起点承接上一段轨迹终点，手指状态与预测附着物保留在 RobotState 中。
MoveGroup 请求强制 `plan_only=True`；模块没有执行动作客户端，也不发布全局场景修改。
失败或缺少前瞻结果的候选不能被接近恢复回退逻辑重新选中。

参数：`enable_loaded_path_precheck`。Benchmark 使用 `--loaded-path-precheck`，并自动启用其端点与候选预验证依赖。

## 发现并修正的模型问题

### 下降上限的作用范围

首轮把整段下降与实际执行使用的单段 0.5 rad 上限比较，误拒绝了总长 0.880 / 0.553 rad 的候选。
修正后逐段应用原上限，没有增大上限。新测试覆盖“每段合规、总和大于单段上限”的情况。

### 手掌与目标的名义重叠

分段修正后，三个候选的抬升、转运、六段下降和释放 FK 通过，但撤离起点被
`CheckStartStateCollision` 拒绝，接触对是 `panda_hand - target_cube`。

只读解析已安装的 `moveit_resources_panda_description/meshes/collision/hand.stl`：

- 200 个三角形。
- 手掌碰撞网格的局部 Z 最大值为 0.0659621507 m。
- 目标高度 0.08 m，当前名义手掌到物体中心距离 0.105 m。
- 正对抓取时，物体靠近手掌的一面在局部 Z = 0.105 - 0.04 = 0.065 m。

这给出了约 0.962 mm 的 Z 投影重叠；运行时的碰撞检查进一步确认该姿态有手掌接触。
将名义偏移显式改成 0.110 m 后，名义间隙约 4.038 mm。
没有添加碰撞豁免、删除释放物体或放宽验证阈值。

新增 `--grasp-height-offset` 用于隔离试验。原 RGB-D 默认 0.105 m 尚未更改，
新值仍需重复性验证；不能仅凭这次 smoke 就推广为所有场景默认值。

## 运行结果

| 目录（artifacts/runs 下） | 结果 | 总用时 | 说明 |
| --- | --- | --- | --- |
| release_loaded_precheck_fixed_20260905 | 0/1 | 20.4 s | 初版下降总长误用了单段上限，保留失败记录 |
| release_loaded_precheck_staged_20260905 | 0/1 | 20.3 s | 分段通过，释放起点手掌碰撞，未进入实际抓取 |
| release_loaded_precheck_clearance_20260905 | 1/1 | 128.0 s | 110 mm 偏移，完整前瞻及物理闭环通过 |
| release_loaded_precheck_center_clearance_20260905 | 0/1 | 34.8 s | 原中央障碍场景仍找不到通过检查的放置端点，未进入实际抓取 |

固定场景中，0/90 度两个候选通过名义全路径前瞻；-90 度接近失败，180 度转运失败，均被拒绝。
最终选中 0 度候选，实际抬升 0.127055 m，放置误差 0.004398 m，并完成回位。
上述用时包含启动、感知、前瞻和实际任务，不是纯规划耗时。
固定成功和中央障碍失败不能合并解释为障碍能力改善；本轮没有证明整体成功率提高。

## 软件检查

- colcon 构建成功；后续 Python 改动由既有 symlink install 加载。
- 最终全量测试：269 passed，7.78 s。
- 覆盖载荷跨段保留、预测释放位置使用 FK、开闭手指状态、plan-only 强制设置、失败终止、
  分段上限、Cartesian 转运模式保留、候选回退不能绕过、CLI 参数透传和非法偏移拒绝。

## 复验

完成构建并 source ROS 与工作区后：

```bash
python3 scripts/run_gazebo_physics_benchmark.py \
  --trials 1 --rgbd-perception --planner-id RRTConnectkConfigDefault \
  --loaded-path-precheck --grasp-height-offset 0.110 \
  --transfer-diagnostics --timeout-s 300 \
  --csv artifacts/runs/loaded_precheck_repeat/results.csv \
  --markdown artifacts/runs/loaded_precheck_repeat/report.md \
  --log-dir artifacts/runs/loaded_precheck_repeat/logs
```

中央障碍试验增加 `--challenge-obstacles --challenge-scenario center_tall_barrier`，并使用新输出目录。
这些实验开关尚未写入 CSV 配置列，禁止用 `--resume` 混合不同配置；历史原始结果不回写。

## 边界与后续

- 前瞻使用名义载荷变换，不预测摩擦、滑移或夹爪开合过程中的接触变化。
- 撤离前瞻证明的是存在受路径长度限制的 OMPL 路径，不表示实际执行会复用完全相同的轨迹。
- 真实抓取后的测量校正、碰撞检查、控制器限制和物理结果验收保持运行。
- 当前前瞻是运动学筛选，不能据此声明 Servo 动态闭环、所有控制器时序门槛或工业安全认证通过。
- 下一步重点是原中央障碍目标的抓取/放置姿态可行性，优先研究保持目标位置与直立约束的
  小角度抓取候选；不挪走障碍、不悄悄更换放置点，也不把安全拒绝算成物理成功。
- 正式发布仍需要同版本重复性回归、干净环境复现和展示材料。

## 局部代码指纹

这是本轮 110 mm 固定/中央场景运行代码的局部 SHA256，不替代完整发布 manifest：

| 文件 | SHA256 |
| --- | --- |
| loaded_path_precheck.py | 353B87CD3C99D151A849A05B98FD2A3C3930CD9077348D4EF2B5A0BC3B79FF5E |
| pick_plan_pipeline.py | B68F73C1C10595F9BADC25FE7BBBFF1471E719CEAF449E1D7768BFB7767FD330 |
| run_gazebo_physics_benchmark.py | 18F1BA7F2EDCCBBD5AF99FCC5247BB4D3FC5388BF9B179A44DAF60034D67A6DF |
