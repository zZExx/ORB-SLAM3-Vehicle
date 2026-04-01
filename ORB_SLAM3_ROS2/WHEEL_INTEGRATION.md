# Wheel odometry integration (mono-inertial)

## ROS2 parameters

| Parameter | Meaning |
|-----------|---------|
| `use_wheel` | Subscribe to wheel odometry and attach linear-x to IMU samples. |
| `wheel_topic` | Topic name (`nav_msgs/msg/Odometry`). |
| `wheel_time_offset` | Seconds added to wheel message stamp before alignment with IMU. |
| `db_wheel_topic` | Bag topic when `data_source:=db`. |

## ORB-SLAM3 YAML (`Settings` / `ParseIMUParamFile`)

Uncomment and fill `Wheel.*` in your camera YAML (see `config/monocular-inertial/s100p_mono_imu.yaml`).

- `Wheel.use: 1` enables wheel in `IMU::Calib` and preintegration.
- Set `T_imu_wheel` / `R_imu_wheel` to express wheel-frame forward velocity in the IMU body frame.

## Baseline A/B

1. Run with `use_wheel:=false` and no `Wheel.use` (default): must match previous behavior.
2. Enable ROS2 `use_wheel` only: wheel is logged and fused into `IMU::Point.encoder_v`; optimization uses wheel only if `Wheel.use: 1`.

## Acceptance checks (manual)

1. **Initialization**: compare time-to-IMU-ready and failure rate with/without `Wheel.use`.
2. **Relocalization**: after forced lost-tracking, compare recovery success (wheel off vs on).
3. **Drift**: straight line, turn, and S-path; compare trajectory smoothness and scale stability.

## Compatibility note

`Preintegrated` serialization was extended for wheel-related state. Old atlas files may not load correctly; keep a wheel-disabled map baseline if you rely on `System.SaveAtlasToFile`.
