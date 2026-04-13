# 轮速里程计融合说明（单目 + IMU）

参考实现：https://github.com/hongyeah314/ORB_SLAM3_with_wheel

## ROS2 参数

| 参数 | 含义 |
|------|------|
| `use_wheel` | 是否订阅轮速里程计，并把 `twist.linear.x`（m/s）挂到 IMU 采样上。 |
| `wheel_topic` | 话题名（`nav_msgs/msg/Odometry`）。 |
| `wheel_time_offset` | 在轮速消息时间戳上增加的秒数，用于与 IMU 对齐。 |
| `db_wheel_topic` | `data_source:=db` 时从 bag 里读的轮速话题名。 |

## ORB-SLAM3 YAML（`Wheel.*`）

- `Wheel.use: 1`：打开 `IMU::Calib::mbUseWheel`，预积分走带轮速分支 `Preintegrated::IntegrateNewMeasurement(..., encoder_v)`。
- `Wheel.T_imu_wheel` / `Wheel.R_imu_wheel`：轮速线速度在 IMU 体轴下的表达（杠杆臂 + 旋转外参）。
- `Wheel.NoiseVel` / `Wheel.WalkVel`：扩维噪声里轮速相关对角项（见 `Calib::SetWheel`）。
- 当前常用配置：`config/monocular-inertial/s100p_mono_imu_final.yaml`。

## 双层开关（做 A/B 时务必分清）

1. **ROS 参数 `use_wheel`**  
   为 `false` 时不挂轮速，`IMU::Point::encoder_v` 保持为 `0`。

2. **YAML `Wheel.use`**  
   为 `1` 时 `mbUseWheel` 为真，预积分仍走**轮速相关代码路径**（ROS 关轮速时只是速度为 0），与「从未加过 wheel 分支」的二进制行为不一定相同。

要在**同一套可执行文件**下做干净纯 IMU 基线，应在 YAML 里设 **`Wheel.use: 0`**（或不加载 wheel 字段），而不能只关 `use_wheel:=false`。

## 启动示例

```bash
./start_orb.sh --mode mono-inertial \
  --bag /path/to/your_bag_0.db3 \
  --exp_dir experiments/exp_wheel_$(date +%m%d_%H%M) \
  --camera_offset 0.09 --imu_offset 0.0 \
  --bag_rate 0.5 --viz false --use_wheel true
```

---

## 已知问题（与代码直接对应）

### 1）预积分 `encoder_velocity` 与 `EdgeInertialGSE` 后 3 维残差——定义可能不一致

**预积分**累加：

```text
encoder_velocity += dR * Rbo * (encoder_v, 0, 0)^T * dt
```

见 `ORB_SLAM3/src/ImuTypes.cc`（带轮速的 `IntegrateNewMeasurement`）。这是每一步用**当前已积出的 `dR`** 去旋转体轴速度再乘 `dt` 的**矢量链式累加**，并不保证等于「由位姿 1 到 2、在 `{b1}` 下写出的单一几何位移」，除非按同一运动学模型严格推导。

**残差**（`EdgeInertialGSE` 最后 3 行）为：

```text
e_tail = Rbw1 * s * (twb2 - twb1) - Tbo + Rbw1*Rwb2*Tbo - encoder_velocity
```

见 `ORB_SLAM3/src/G2oTypes.cc`（`EdgeInertialGSE::computeError`）。这是**位姿差 + 尺度 `s` + 杠杆臂**，在**第一帧体坐标系 `{b1}`** 下的几何量。

**风险：** 优化器把两边当成可减的同一物理量，但二者**物理含义**可能不同（尤其转弯、单目尺度时），这比单纯调 `NoiseVel` 更容易表现为「加 wheel 反而伤跟踪」。

**建议修法方向：** 先选定一种**基准定义**（推荐：**让预积分里保存/传播的量与上述几何残差严格一致**，即「积什么」与 `Rbw1*s*dpb + 杠杆臂` 对齐），再同步改 `covariance_enc`、信息矩阵与雅可比。

### 2）何时建 wheel 边——`encoder_velocity.norm()` 门限

`ORB_SLAM3/src/Optimizer.cc`（`InertialOptimization`）中，仅当 `encoder_velocity.norm()` 超过阈值时才建 `EdgeInertialGSE`。

**后果：** 长时间静止或区间很短、积分范数接近 0 时，**整条 wheel 边可能被跳过**，无法单独依赖该条件提供「零速轮速约束」。

**若需要静止也约束：** 应去掉或改写该门限（例如按「轮速话题有效 / 区间长度 / 用户开关」判断，而不是按范数是否为 0）。

### 3）尾部解析雅可比与 `computeError`——需数值验证

`EdgeInertialGSE::linearizeOplus` 对第 9–11 行残差填了多块雅可比，同文件内还有注释掉的备选写法。

**风险：** 解析雅可比与 `inner2` 在全局未必逐点一致。**建议：** 对尾部 3 维相对各顶点做有限差分核对，或按统一链式法则重写并删掉歧义分支。

### 4）回环修正与冻结的预积分

`LoopClosing` 修正关键帧位姿后，**通常不会**按新位姿重算 `KeyFrame::mpImuPreintegrated` 里的 `encoder_velocity`。wheel 约束可能与更新后的图不一致。

**缓解：** 回环后对受影响预积分**重积分**，或在回环后若干帧内**暂时不加** wheel 残差边。

### 5）信息矩阵回退策略

`EdgeInertialGSE` 构造里：优先用 `covariance_enc` 的 12×12 子块求逆；若该块接近奇异，则回退为 IMU 的 `C` 的 9×9 逆，并对最后 3×3 使用**较大对角**（见 `G2oTypes.cc`）。这是启发式，若 `cov12` 经常退化，尾部权重可能不合理。

### 6）单目尺度 `s` 与杠杆臂 `Tbo` 的度量约定

尾部残差对平移项使用 **`s * (twb2 - twb1)`**，而 **`Tbo`（`Wheel.T_imu_wheel`）通常按米制外参**写入。若地图中 `twb` 与 YAML 外参不在同一尺度约定下，会出现**量纲混用**：优化可能用错误的 `s` 或扭曲轨迹去拟合轮速。

**建议：** 在文档/配置里写死「`twb` 与 `Tbo` 是否同尺度」；单目初始化后确认尺度与标定一致再做 wheel A/B。

### 7）历史 commit `4da7b15`（`add-wheel`）审阅记录（对照用）

若对比该 commit 的 `G2oTypes.cc`（而非当前树），曾出现：

- **`computeError` 尾部**：`inner = s*dpb - Tbo + Rbw1*Rwb2*Tbo - enc`，再 **`ee = Rbw1 * inner`**，会把**已在 b1 系累加的 `encoder_velocity` 再左乘 `Rbw1`**，且 `inner` 内世界量与体轴量混写，与预积分语义易冲突。
- **`linearizeOplus` 尺度块**：`_jacobianOplus[7].block<3,1>(9,0) = Rbw1*Δtwb * s`，对 **`Rbw1 * s * Δtwb`** 关于 `s` 的常见偏导应为 **`Rbw1 * Δtwb`**，多出的 `*s` 疑似笔误。

当前主干若已改为 **`ee2 = Rbw1*s*dpb - Tbo + Rbw1*Rwb2*Tbo - enc`** 并修正尺度雅可比，上述两点应已缓解；合并分支时仍建议 **diff 核对**。

### 8）扩维噪声 `Nga_en` 与「只有 x 向线速度」

`Calib::SetWheel` 将 `Nga_en` 尾部 3 维对角设为同一 `NoiseVel^2`，但 ROS 侧通常只填 **`twist.linear.x`**。相当于在传播里为 y/z 维也加了噪声通道，**与真实 1 自由度观测不完全同构**；若 y/z 对角过大或过小，都会影响 `covariance_enc` 与尾部信息矩阵。

**建议：** 明确是否将 y/z 噪声压到极小（近似只约束 x），或改为严格 1 维观测模型。

### 9）ROS2：时间对齐、轴向与符号

- **`wheel_time_offset`**：与 IMU 不同步时，会把速度插到错误子区间。
- **`linear.x` 与 `Wheel.R_imu_wheel`**：车体前进方向、IMU 安装、右手系与 ORB 体轴定义必须一致；符号反了会系统性拉偏尺度与轨迹。

### 10）`covariance_enc`（21×21）与 `C`（9×9）

- 12×12 传播沿**预积分轨迹**线性化，与 **g2o 在当前顶点处**的线性化点未必一致，属常见近似。
- 下方块与 `NgaWalk_en` 的用法、**初始化是否处处清零**、是否与 IMU 协方差**重复或遗漏**交叉项，建议全局 `grep covariance_enc` 做一次审阅。

### 11）优化图：鲁棒核与 `EdgeInertialGS` ↔ `EdgeInertialGSE` 切换

- 若 `EdgeInertialGS` 与 `EdgeInertialGSE` 在鲁棒核、权重上不一致，Chi2 贡献不可比。
- 仅靠 **`encoder_velocity.norm()`** 等门限在两类边之间切换，边界附近可能出现**约束形式跳变**。

### 12）序列化与 Atlas

`Preintegrated` 增加 `mbUseWheel`、`encoder_velocity`、`covariance_enc` 等序列化字段后，**旧版保存的 atlas/地图与新版不兼容**。需保留 **wheel 关闭** 的基线地图或版本说明。

### 13）其它工程点

- **`EdgeInertialGSE::computeError` 内 `std::cout`**：若仍存在，会严重拖慢优化；应删除或 `#ifdef`。
- **轮速缺失段**：若 `encoder_v` 被置 0 重积分，与建图当时使用的预积分可能不一致；**bias 更新后 `Reintegrate`** 会重播 `enc`，需保证队列与 YAML 开关一致。
- **CMake 生成物误提交**：与算法无关，但说明该历史 commit 审查疏漏，合并时注意 `.gitignore`。

---

## 问题汇总（检查清单）

| 序号 | 主题 | 建议动作 |
|------|------|----------|
| 1 | 预积分量 vs 尾部几何残差 | 统一模型后改协方差与雅可比 |
| 2 | Optimizer 范数门限 | 静止/短区间是否需要 wheel 边 |
| 3 | 尾部解析雅可比 | 数值差分校验 `e_tail` 对各顶点 |
| 4 | 回环 vs 冻结预积分 | 重积分或短期内禁用 wheel 边 |
| 5 | 信息矩阵退化回退 | 监控 `cov12` 是否常退化 |
| 6 | `s` 与 `Tbo` 尺度 | 文档化约定并做 A/B |
| 7 | 历史 commit 残差/尺度雅可比 | 合并时 diff `G2oTypes.cc` |
| 8 | 3 维噪声 vs 1 维观测 | 调对角或改 1 维模型 |
| 9 | 时间戳/轴向/符号 | bag 标定与实车核对 |
| 10 | `covariance_enc` 全链路 | `grep` + 初始化审阅 |
| 11 | 鲁棒核与边切换 | 与纯 IMU 边对齐策略 |
| 12 | Atlas 兼容 | 版本说明 + 基线地图 |
| 13 | 日志/缺失数据/Reintegrate | 关调试输出、保证数据一致 |

---

## 建议的工程顺序

1. **统一模型：** 让 `encoder_velocity`（或替换为别的状态）与 `EdgeInertialGSE` 尾部几何量严格一致（收益最大）。
2. **修正/验证尾部雅可比：** 数值雅可比 vs 解析式。
3. **按需调整 Optimizer 门限：** 若希望静止也有 wheel 约束。
4. **回环：** 重积分或短期内禁用 wheel 边。
5. **标定：** 实测或离线标定 `T_imu_wheel` / `R_imu_wheel`；单靠调 `NoiseVel` 很难修正错误模型。

---

## 离线外参标定（简述）

使用专用 bag（前后加减速、左右转弯、尽量无打滑）。将轮速 `linear.x` 插值到 IMU 时间（与 `SampleWheelLinearX` 思路一致）。在代码中**采用与线上一致的残差/预积分定义**构造代价，优化 `(R_imu_wheel, T_imu_wheel)`，再固定 `Wheel.use` 与 `use_wheel` 文档化地做 A/B。
