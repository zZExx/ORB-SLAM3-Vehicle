# 目标
1. 验证IMU与轮速计时间对齐误差，该误差是否为固定值
2. 估计轮速计精度模型

# 输入与输出

## 背景与范围
- 本文档用于离线分析 wheel odometry（`/wheel_odom`）与 IMU（`/imu/data`）之间的**时间对齐误差**与**轮速计测量噪声**，并给出**打滑检测**的可落地判据与输出格式，作为后续在 ORB-SLAM3 管线中引入轮速计时的参数依据。
- 本 PRD 的产出是**分析结果与参数文件**；不包含 ORB-SLAM3/ROS2 节点的集成实现（如需实现，另开 PRD/任务）。

## 关键定义（口径统一）
### 时间对齐
- 定义时间偏移 \( \delta t \)：采用
  - \( t_{wheel} = t_{imu} + \delta t \)
  - 含义：当 \( \delta t > 0 \) 时，wheel 时间戳相对 IMU **更晚**（wheel 落后 IMU）。
- 估计目标：找到能最小化两传感器“动态一致性残差”的 \( \delta t \)。

### 对比量与残差
- Wheel 侧（`nav_msgs/Odometry.twist.twist`）：
  - 线速度：\( v_{wheel}(t) := \text{twist.twist.linear.x} \)（m/s）
  - 偏航角速度：\( \omega_{wheel}(t) := \text{twist.twist.angular.z} \)（rad/s）
- IMU 侧（`sensor_msgs/Imu.angular_velocity.z`）：
  - \( \omega_{imu}(t) := \text{angular_velocity.z} \)（rad/s）
- 残差（对齐后）：
  - \( e_\omega(t) = \omega_{wheel}^{aligned}(t) - \omega_{imu}(t) \)

### 运动假设
- 默认假设车体近似平面运动：主要关注 yaw 轴（z 轴）角速度一致性；若存在明显俯仰/横滚激励，需要在风险项里单独标注并调整策略。

## 输入
提前录制好的bag，运动模式包含向前，向后，转向，S形移动
    1. IMU topic 为 /imu/data
    2. Wheel topic 为 /wheel_odom
    3. bag 的绝对路径为 /mnt/ssd/ros2_bag/IMU_WHEEL/bag1/bag1_0.db3

## 数据约束与前置条件
- `/wheel_odom`：`nav_msgs/Odometry`
  - 使用字段：`twist.twist.linear.x`、`twist.twist.angular.z`
  - 若消息自带协方差（`twist.covariance`），记录但不直接信任，后续用统计结果校核/覆盖（见“输出交付物”）。
- `/imu/data`：`sensor_msgs/Imu`
  - 使用字段：`angular_velocity.z`（至少 z），用于角速度对齐。
- 时间戳：使用消息 header timestamp（ROS2 time）；若两 topic 不同时间源或存在重映射/重采样，必须在“风险与待确认项”中记录。

## 输出
1. 时间对齐误差
2. 轮速计噪声
3. 轮速计有无打滑

## 方法
### 方法1：验证时间对齐误差是否为固定值（估计 \( \delta t \)）
#### 分段策略（与运动模式对齐）
- 将 bag 分成若干段：forward / backward / turning / S-shape。
- 分段方式优先顺序：
  - 优先：根据 wheel 的 \( |\omega_{wheel}| \)、\( |v_{wheel}| \) 阈值自动切段（实现简单、可复现）。
  - 备选：人工给定时间窗口（需在输出里记录窗口范围，保证可追溯）。

#### 估计策略（推荐：角速度互相关/滑窗搜索）
- 在候选区间 \( \delta t \in [\delta t_{min}, \delta t_{max}] \)（占位：例如 ±200 ms）内搜索最优 \( \delta t \)。
- 一个可实现定义（示例）：
  - 将 \( \omega_{wheel}(t) \)、\( \omega_{imu}(t) \) 重采样到同一时间栅格（占位：例如 200 Hz）
  - 去均值（必要时做带通，占位）
  - 互相关 \( C(\tau) = \sum_t \omega_{wheel}(t)\,\omega_{imu}(t+\tau) \)
  - \( \delta t = \arg\max_{\tau} C(\tau) \)

#### “固定值”判据（可验收口径）
- 对每个运动段估计 \( \delta t_i \)，计算 \( \delta t_{mean} \) 与 \( \text{std}(\delta t_i) \)
- 若 \( \text{std}(\delta t_i) < T_{\delta t} \) 则认为“近似固定”（阈值占位：例如 1–5 ms，后续用数据设定）

### 方法2：估计轮速计噪声模型（默认：测量噪声协方差）
#### 参数化
- 目标输出测量噪声（示例二维）：
  - \( R = \text{diag}(\sigma_v^2, \sigma_\omega^2) \)
  - \( \sigma_\omega \) 对应 \( \omega_{wheel} \)，\( \sigma_v \) 对应 \( v_{wheel} \)

#### 角速度噪声估计（默认落地）
- 使用方法1得到 \( \delta t_{mean} \)，对齐：
  - \( \omega_{wheel}^{aligned}(t) = \omega_{wheel}(t-\delta t_{mean}) \)
- 残差：
  - \( e_\omega(t) = \omega_{wheel}^{aligned}(t) - \omega_{imu}(t) \)
- 统计：
  - \( \mu_\omega = \text{mean}(e_\omega) \)（用于检查偏置）
  - \( \sigma_\omega^2 = \text{var}(e_\omega) \)

#### 线速度噪声估计（先给可执行口径 + 明确限制）
- 若缺少可靠 \( v_{ref}(t) \)，先输出 `sigma_v` 为 null，并记录原因；后续补充参考源后再估计。
- 若提供可靠 \( v_{ref}(t) \)，用 \( e_v(t)=v_{wheel}^{aligned}(t)-v_{ref}(t) \) 的均值/方差估计 \( \sigma_v \)。

### 方法3：打滑检测（输出“是否打滑/滑移程度”）
#### 判据（先落地为角速度一致性）
- 当 \( |e_\omega(t)| > T_\omega \) 持续超过时长 \( T_{dur} \) 则标记为 slip episode。
- \( T_\omega \)、\( T_{dur} \) 为阈值占位；建议用残差分布分位数（例如 95%/99%）设定初值。

#### 评分（建议输出）
- \( s(t) = \min(1, |e_\omega(t)|/T_\omega) \)
- 每段输出平均/最大评分。

## 实验设计
- 对每个运动模式分别输出：\( \delta t_i \)、\( \sigma_{\omega,i} \)、以及 slip episodes 统计。
- 对比项：
  - 对齐前后：`e_omega` 分布变化（直方图、时间序列）。
  - 分段间：\( \delta t_i \)、\( \sigma_{\omega,i} \) 的一致性。

## 输出交付物（建议格式，便于自动化消费）
> 说明：以下字段名与文件内容保持 English-only，便于脚本解析。

1. `time_offset.json`
   - `delta_t_mean_ms`: number
   - `delta_t_std_ms`: number
   - `segments`: array of objects
     - `name`: string (e.g. "forward_01")
     - `t_start_ns`: integer
     - `t_end_ns`: integer
     - `delta_t_ms`: number
     - `method`: string (e.g. "xcorr_omega_z")
     - `search_range_ms`: array [min, max]
     - `resample_hz`: number

2. `wheel_noise.json`
   - `sigma_omega_radps`: number
   - `mu_omega_radps`: number
   - `sigma_v_mps`: number | null
   - `sigma_v_method`: string (e.g. "ref_velocity" or "null_no_reference")
   - `segments`: array (optional)

3. `slip_report.json`
   - `threshold_omega_radps`: number
   - `min_duration_ms`: number
   - `episodes`: array
     - `t_start_ns`: integer
     - `t_end_ns`: integer
     - `max_abs_e_omega_radps`: number
     - `mean_score`: number

4. 图表清单（文件名建议）
   - `delta_t_hist.png`
   - `e_omega_timeseries.png`
   - `e_omega_hist.png`
   - `slip_episodes.png`

## 验收标准（可量化口径，阈值占位）
- 时间对齐“固定值”：
  - `delta_t_std_ms < T_delta_t_ms`
- 噪声估计稳定性：
  - 不同运动段的 `sigma_omega_radps` 相对差 < P%（占位）
- 打滑检测可用性：
  - `slip_report.json` 必须包含 `episodes`（允许为空数组），并记录阈值与持续时间参数。

## 风险与待确认项
- 时间戳同源性：wheel 与 IMU 是否使用同一时钟；bag 录制链路是否引入系统性延迟。
- wheel_odom 预处理：wheel 侧是否已经滤波/融合（会改变噪声统计口径）。
- 坐标系一致性：`angular_velocity.z` 与 `twist.angular.z` 的正方向是否一致；若不一致会导致“假残差”与错误 \( \delta t \)。
- 线速度参考缺失：若无可靠 `v_ref`，`sigma_v` 需置空并在报告中解释。

# 关联项目
/home/sunrise/ORB_SLAM3/ORB_SLAM3

/home/sunrise/ORB_SLAM3/ORB_SLAM3_ROS2
