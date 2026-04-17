# Wheel融合中高风险评估报告（方案阶段）

> **维护说明（2026-04-17）**：本文件不再作为当前状态主文档维护。
>
> 当前唯一应优先阅读的 wheel 文档是：
> - `docs/WHEEL_THREE_AB_REPORT.md`
>
> 本文件仅保留为历史风险记录和旧方案背景。若与 `WHEEL_THREE_AB_REPORT.md` 冲突，以后者为准。

## 1. 评估范围与前提

- 本报告兼作实验进展记录，随实验更新。
- `Wheel.use` 不作为独立开关设计目标；运行时由 bash 参数控制是否启用 wheel。
- `G2oTypes` 中 `Encoder update error2` 日志暂时保留，用于观察量级和收敛趋势。
- 量化对比工具：`scripts/eval_traj_vs_wheel.py`（见 §7）。
- 量化对比工具：`scripts/eval_traj_vs_wheel.py`（见 §7）。

---

## 2. 关键结论（先看）


> **状态更新（2026-04-17）**：P0 方案 A 已实现并通过 Jacobian 数值自检；
> 最新 `full_wheel_graph vs pure_imu` 5-run 显示平均有正向改善（ratio +3.9%，loop_gap -23.8%），
> 但方差仍大、稳定性不足，尚未形成“严格门限下可直接上线”的结论。

- **P0（历史高风险，当前已部分缓解）模型一致性问题已进入可控状态**：
  `EdgeInertialGSE` 尾部残差与解析雅可比已按统一契约重推并做有限差分校验（`[WheelJacCheck]` mismatch=0）。
- **P1（仍需关注）回环行为仍存在不稳定风险**：历史数据中 wheel 可能压制回环候选；最新 5-run 显示
  `loop_gap` 平均改善但标准差较大（`wheel_on gap_std=0.5492`）。
- **P1 附（待验证）回环后预积分冻结失配**：回环/merge 会修改 `Pose/Velocity`，但没有统一的"按新状态重算 wheel 预积分"流程。当前因回环候选被压制未实际触发，但模型风险不变。
- **P2（中风险）建边门限对静止段不友好**：`encoder_velocity.norm()>1e-7` 会把静止段 wheel 边切掉，无法利用零速约束；该风险在低速、起停频繁场景更明显。

---

## 3. 证据摘录

## 3.1 P0 模型一致性

- 预积分中，wheel 累积量定义为：
  - `encoder_velocity += dR * Rbo * encoder_cast * dt`
  - 文件：`ORB_SLAM3/src/ImuTypes.cc`
- 因子尾部误差当前为：
  - `e_tail = Rbw1 * s * (twb2 - twb1) - Tbo + Rbw1*Rwb2*Tbo - encoder_velocity`
  - 文件：`ORB_SLAM3/src/G2oTypes.cc`
- 结论：
  - `e_tail` 写法已较旧版本更合理；
  - 但协方差传播（`covariance_enc`）与解析雅可比仍沿用历史线性化结构，未见“同一状态定义下的整体重推导”。

## 3.2 P1 回环与冻结预积分
、

- 回环/merge流程会对多关键帧执行 `SetPose`、`SetVelocity`、图优化与缩放旋转更新：
  - 文件：`ORB_SLAM3/src/LoopClosing.cc`、`ORB_SLAM3/src/Map.cc`
- 但该流程内未形成“姿态变化后统一重积分 wheel 预积分”的稳定路径。
- 虽然某些优化流程在 bias 变化时会触发 `Reintegrate()`，但这是 **bias 驱动**，不是 **pose 更新驱动**：
  - 文件：`ORB_SLAM3/src/Optimizer.cc`

## 3.3 P2 建边门限

- 建 wheel 边条件：
  - `encoder_velocity.norm() > 1e-7`
  - `covariance_enc.block<12,12>(0,0).norm() > 1e-12`
  - 文件：`ORB_SLAM3/src/Optimizer.cc`
- ROS2 wheel 采样在无数据时返回 0，且静止时本身接近 0：
  - 文件：`ORB_SLAM3_ROS2/src/monocular-inertial/mono-inertial-node.cpp`
- 这意味着静止段和短窗口更容易走回 `EdgeInertialGS`（无 wheel 尾部约束）。

---

## 4. 日志量化结果（最新对比实验 2025-04-15）

数据集：`large_single_0.db3`，camera_offset=0.09，两次实验配置完全相同，仅 wheel 开关不同。

| 指标 | wheel_on (`exp_test_0415_1105`) | wheel_off (`exp_test_0415_1108`) | 优胜方 |
|------|--------------------------------|----------------------------------|--------|
| SLAM 总路程 (m) | 33.02 | 40.91 | wheel_off |
| 轮速积分参考路程 (m) | 46.65 | 46.65 | — |
| 尺度 ratio (SLAM/wheel) | **0.707** | **0.877** | wheel_off |
| 回环候选触发次数 | **0** | **18** | wheel_off（有触发） |
| BAD LOOP 次数 | 0 | 17 | — |
| Loop gap 首尾距离 (m) | **0.739** | **0.367** | wheel_off |
| Encoder error2 条数 | 1035 | 0（未启用） | — |
| Encoder error2 first100 均值 | 0.093 | — | — |
| Encoder error2 last100 均值 | 0.034 | — | — |
| 收敛比（last/first） | 0.37 | — | — |

解读：

- **wheel_on 尺度偏小约 29%**：ratio=0.707 vs 0.877，差距远超正常噪声范围，说明 P0 模型不一致性在把尺度持续拉偏，不是标定问题。
- **wheel_on 回环候选为 0**：同一 bag 存在真实回环，wheel_off 有 18 次候选（均为 BAD LOOP，但 Sim3 验证过程本身对局部图有约束作用）。wheel_on 完全无候选，说明 wheel 引入的残差改变了关键帧位姿分布，使 BoW 相似度或几何验证门限未达到。
- **Encoder error2 收敛**（wheel_on 专有）：误差从 0.093 降至 0.034，比值 0.37，说明优化器在迭代收敛，但收敛到了错误的尺度解。收敛不等于模型正确。
- **结论**：当前 change 在该数据集上是净负效果。P0 修复优先级最高。

历史参考（`exp_test_0414_1808`，wheel_on）：err_count=936，mean=0.0702，first100/last100 比值≈0.405。当前 0415 比值 0.37 略有改善，但尺度问题更明显，说明调参未能解决根本问题。

---
## 5. 候选改造方案（只评估）

## 5.1 P0 一致化改造（高风险核心）

### 方案 A（最小改动，推荐先做）

- 保留当前 `e_tail` 几何形式；
- 以 `e_tail` 为准，重推导 `linearizeOplus` 的尾部 3 维雅可比；
- 对 `covariance_enc` 的 12x12 子块做一致性检查（保持结构，修正不一致项）。

优点：

- 改动集中在 `G2oTypes + ImuTypes` 局部，回归风险可控；
- 能快速判断“数学一致性修复”对性能的真实收益。

代价：

- 需要有限差分校验工具，避免手推导遗漏。

### 方案 B（中等改动）

- 重新定义 wheel 预积分状态（例如显式定义 `DeltaPw_b1`）；
- 重做传播矩阵 `F/V` 与噪声注入，统一到新状态定义。

优点：

- 物理语义更清晰。

代价：

- 侵入大，回归成本高；短期不利于定位主因。

### 方案 C（彻底重构）

- 将 wheel 作为独立观测因子（1D 或 3D）而非嵌入 IMU 预积分扩维。

优点：

- 模型边界清晰，可做鲁棒核和时序策略独立调优。

代价：

- 工程量最大，不适合当前迭代节奏。

---

## 5.2 P1 回环后不一致（高风险）

### 方案 A（稳妥）

- 回环/merge完成后，对受影响关键帧链执行 wheel 预积分重算（含 `encoder_velocity` 与 `covariance_enc`）。

优点：

- 约束一致性最好。

缺点：

- 耗时上升，需控制重算窗口。

### 方案 B（低成本）

- 回环后 `N` 帧内临时禁用 wheel 边（只保留 IMU 边），等待状态稳定后恢复。

优点：

- 实现简单，回归风险低。

缺点：

- 窗口期损失 wheel 约束，短时漂移可能上升。

---

## 5.3 P2 建边门限（中风险）

### 现状问题

- 以 `encoder_velocity.norm()` 作为唯一门限会把“静止但有意义的零速信息”过滤掉。

### 候选替代

- 规则 1：改成“有有效 wheel 消息且时间窗口有效”即建边；
- 规则 2：静止段建弱约束边（更大协方差）而不是直接跳过；
- 规则 3：增加显式参数（如 `wheel_edge_mode`）选择 `strict_norm / valid_msg / always_with_cov`。

建议：

- 先做规则 1（最小行为变化），观察静止段稳定性再决定是否引入规则 2。

---

## 6. 推荐执行顺序（实施草案）

1. **先做 P0 方案 A**：统一尾部误差、雅可比、协方差的一致性。
2. 做 **有限差分对照**：验证尾部 3 维解析雅可比。
3. 上 **P1 方案 B**（低成本保护）：回环后短窗禁用 wheel 边，验证是否减少 `BAD LOOP`。
4. 再做 **P2 规则 1**：建边条件从“范数门限”改为“消息有效性门限”。
5. 最后评估是否升级到 P1 方案 A（回环后重积分）。

---

## 7. 验证步骤与工具

### 7.1 运行 A/B 实验

```bash
# wheel on
./ORB_SLAM3_ROS2/run_orb_experiment.sh \
  mono-inertial /path/to/bag.db3 /path/to/config.yaml \
  experiments/exp_wheel_on 0.09 0.0 false

# wheel off（务必在 YAML 里设 Wheel.use: 0，仅关 use_wheel 不够干净）
./ORB_SLAM3_ROS2/run_orb_experiment.sh \
  mono-inertial /path/to/bag.db3 /path/to/config.yaml \
  experiments/exp_wheel_off 0.09 0.0 false
```

### 7.2 量化对比（需 source ROS2 环境）

```bash
source /opt/ros/humble/setup.bash
python3 scripts/eval_traj_vs_wheel.py \
    --traj experiments/exp_wheel_on/FrameTrajectory.txt:wheel_on \
           experiments/exp_wheel_off/FrameTrajectory.txt:wheel_off \
    --bag  /path/to/large_single_0.db3 \
    --topic /wheel_odom \
    --segments 10 \
    --save-plot experiments/ratio_comparison.png
```

输出说明：
- `Ratio`（SLAM 路程 / 轮速参考路程）越接近 1.0 越好；
- `LoopGap`（首尾帧距离）越小说明漂移越小（有真回环时参考意义更大）；
- 分段 ratio 趋势图显示尺度漂移是否被持续抑制。

### 7.3 Encoder error 提取（仅 wheel_on 有效）

```bash
python3 - << 'PY'
import re, statistics
log = "experiments/exp_wheel_on/run.log"
errs = []
bad_loop = 0
with open(log, "r", errors="ignore") as f:
    for line in f:
        m = re.search(r'Encoder update error2:\s*([\-0-9.eE]+)\s+([\-0-9.eE]+)\s+([\-0-9.eE]+)', line)
        if m:
            x, y, z = map(float, m.groups())
            errs.append((x*x + y*y + z*z)**0.5)
        if "BAD LOOP!!!" in line:
            bad_loop += 1
print("err_count", len(errs))
if errs:
    print("err_mean", statistics.mean(errs))
    print("first100_mean", statistics.mean(errs[:100]) if len(errs) >= 100 else statistics.mean(errs))
    print("last100_mean", statistics.mean(errs[-100:]) if len(errs) >= 100 else statistics.mean(errs))
print("bad_loop_count", bad_loop)
PY
```

判断标准：
- `first100_mean / last100_mean` 比值小于 0.5 说明收敛趋势良好；
- 收敛不等于模型正确，需结合 ratio 和 loop gap 综合判断。

---

## 8. 决策建议（当前阶段，2026-04-17 更新）

- **当前实验结论**：wheel 已显示平均正向收益（最新 5-run：ratio `0.8770 vs 0.8441`，loop_gap `0.6961 vs 0.9136`），
  但 run-to-run 波动仍大，暂不建议直接宣称“稳定上线”。
- **近期优先项**：
  1. 继续保持 `Wheel.UseVIBAEdge` 运行时开关，支持快速回退；
  2. 补齐 run 级 `reset/init_fail/bad_loop` 统计，与 ratio/loop_gap 做相关性归因；
  3. 在 P1/P2 方案中优先选低侵入手段提升稳定性（回环后短窗策略、建边门限策略）。
- **建议验收口径分层**：
  - 工程有效性：`full_wheel_graph` 相对 `pure_imu` 的均值改进为正；
  - 稳定上线门限：均值改进同时满足方差约束和异常 run 占比约束。
