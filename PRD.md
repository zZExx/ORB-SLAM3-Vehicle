# PRD：ORB-SLAM3 单目初始化 -> IMU 对齐 -> VIO 切换（可落地合并版）

基于：`ORB_SLAM3`

---

## 1. 目标

实现初始化阶段三段式流程：

```text
Stage 1: Mono map initialization (no metric scale)
Stage 2: IMU alignment (scale + gravity + bias)
Stage 3: Switch to VIO tracking
```

约束：

- 仅初始化阶段允许模式切换。
- 跟踪阶段不做模式来回切换。
- `LOST` 后必须回到初始化起点并重走完整流程。

---

## 2. DoD（必须达成）

功能：

- 单目初始化成功。
- IMU 初始化成功（`scale + gravity + bias`）。
- 自动切到 VIO。
- tracking lost 后自动重置并重启初始化。

性能与稳定性：

- 初始化总耗时 `< 5s`（目标值）。
- `scale` 收敛稳定、无明显跳变。
- 无崩溃、无死锁、线程行为与现有架构一致。

---

## 3. 改动范围（合并后最终约束）

允许修改：

- `ORB_SLAM3/src/Tracking.cc`
- `ORB_SLAM3/include/Tracking.h`
- `ORB_SLAM3/CMakeLists.txt`（仅用于新增文件编译可达性）

允许新增：

- `ORB_SLAM3/src/IMUInitializer.cc`
- `ORB_SLAM3/include/IMUInitializer.h`

最小放宽说明（必须）：

- 允许最小触达或复用 `LocalMapping` 现有 IMU 初始化路径（不重写其核心逻辑）。

禁止：

- 不改 `Optimizer` 核心 BA/惯性优化求解逻辑。
- 不删现有 VIO 主流程。
- 不新增额外线程（非必要禁止）。
- 不引入新外部依赖。

---

## 4. 现有 API 映射（替代 PRD 中不匹配命名）

| PRD 名称 | 现有对应 | 处理方式 |
|---|---|---|
| `src/Tracking.h` | `include/Tracking.h` | 使用真实路径。 |
| `TrackWithIMU()` | `Tracking::Track()` 内惯性分支 | 不新增同名 API。 |
| `SetImuBias(...)` | `SetNewBias(...)` | 统一映射到现有 bias 接口。 |
| `SetInertialMode(true)` | `Atlas::SetInertialSensor()` / `Map::SetInertialSensor()` | 使用 Atlas/Map 惯性标志。 |
| `IMUData` 缓存体系 | `IMU::Point` + 现有队列/时间窗逻辑 | 优先复用现有类型，避免并行数据体系。 |
| `ApplyIMUInitialization()` | `ApplyScaledRotation + UpdateFrameIMU + SetImuInitialized` | 采用现有完成链路。 |

---

## 5. 核心状态机（最小侵入）

```cpp
enum InitState {
    MONO_ONLY = 0,
    WAIT_IMU = 1,
    VIO_READY = 2
};
```

新增成员：

```cpp
InitState mInitState;
double mInitStartTime;
```

状态转移：

1. `MONO_ONLY -> WAIT_IMU`
   - 条件：`IMU_MONOCULAR` 且 `KeyFramesInMap >= 8` 且 `!isImuInitialized()`
   - 动作：记录 `mInitStartTime`，输出 init start 日志
2. `WAIT_IMU -> VIO_READY`
   - 条件：`mpAtlas->isImuInitialized() == true`
   - 动作：输出 init success + switch 日志
3. `WAIT_IMU -> MONO_ONLY`
   - 条件：超时（`> 3.0s`）
   - 动作：走现有 reset 路径并回退状态
4. `ANY -> MONO_ONLY`
   - 条件：`mState==LOST` / `mbBadImu` / 时间戳异常触发重置
   - 动作：与现有 `Track()` reset 分支一致

原则：

- 不重写 `Tracking::Track()` 主干，仅插入轻量状态门控。
- 不创建状态真值分裂，`isImuInitialized()` 作为唯一 IMU 就绪判据。

---

## 6. IMU 初始化策略

优先复用现有 `LocalMapping::InitializeIMU(...)` 能力，保持以下行为：

- 时间窗与关键帧数量门槛由现有逻辑驱动；
- scale/gravity/bias 通过现有惯性优化链路求解；
- map scale/rotation 更新通过现有地图更新接口应用；
- 初始化完成标志通过 `Atlas/Map` 现有标志位传播。

若新增 `IMUInitializer`，其职责仅为封装与桥接，不复制一套独立数学求解路径。

---

## 7. 超时与重置

超时函数：

```cpp
bool Timeout() { return (Now() - mInitStartTime) > 3.0; }
```

重置要求：

- `LOST` 或 `WAIT_IMU` 超时后，统一回退 `mInitState = MONO_ONLY`；
- reset 行为沿用现有分支（`ResetActiveMap` / `CreateMapInAtlas`）；
- reset 后必须可再次完成 `MONO_ONLY -> WAIT_IMU -> VIO_READY`。

---

## 8. 输入与传感器约束

IMU 设备约束（LPMS-BE2）：

- 仅使用 `accel`、`gyro`；
- 禁止使用姿态输出（Euler/Quaternion）。

建议参数（初始硬编码）：

```yaml
Init:
  MinKeyFrames: 8
  MaxKeyFrames: 20
  Timeout: 3.0
IMU:
  Freq: 200
```

---

## 9. 日志规范（必须可观测）

```text
[INIT] Mono initialized
[INIT] IMU initialization started
[INIT] IMU initialization success, scale=...
[INIT] Switch to VIO
[INIT] IMU init timeout, fallback to MONO_ONLY
[RESET] Tracking lost, reinitializing
```

ROS2 侧观测补充：

- `IMU topic freq (1s): ...`
- `Image topic freq (1s): ...`
- 连续空 IMU 样本窗口统计（用于定位 Case2）。

---

## 10. 验收测试（Case1/2/3）

### Case1: 正常启动

- 观测到完整日志序列：Mono init -> IMU start -> IMU success -> Switch to VIO
- `isImuInitialized: false -> true`
- 成功切换后连续稳定跟踪（建议 >= 10s）

### Case2: IMU 初始化失败

- 在 `WAIT_IMU` 超过 `3.0s` 后触发 timeout 日志
- 自动 reset 并回到 `MONO_ONLY`
- 可重复重试，不崩溃、不死锁

### Case3: Tracking lost

- VIO 运行中制造丢跟踪，出现 lost/reset 相关日志
- 状态机回到 `MONO_ONLY`
- 可重新走通 Case1 成功路径

---

## 11. 实施顺序

```text
1. 接入状态机成员与状态同步点
2. 完成 API 映射替换（使用现有真实符号）
3. 接入 WAIT_IMU 超时回退与 reset 联动
4. 补齐统一日志
5. 按 Case1/2/3 跑验收
```

> 定位：这是“初始化增强与流程编排”，不是重写 ORB-SLAM3。
