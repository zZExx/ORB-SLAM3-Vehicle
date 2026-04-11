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
