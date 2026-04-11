# ORB-SLAM3 Background

## System Setup
- Board: S100P
- Sensors:
  - Global shutter camera: 640x480, ~20 Hz, BGR
  - IMU: 200 Hz, Allan-calibrated noise
- ROS 2: Humble; rviz2/evo used for visualization and offline checks

## Datasets & Topics
- Local S100P dataset (mapping test):
  - Camera topic remapped from `/camera/image_raw`
  - IMU topic remapped from `/imu` or `/imu/data` (depends on bag)
- EuRoC V1_01_easy (sanity check):
  - Camera: `/cam0/image_raw`
  - IMU: `/imu0`

## Configurations
- Monocular config: `config/monocular/s100p_mono.yaml`
  - Camera intrinsics/distortion, fps=15–16 Hz
- Monocular-inertial config: `config/monocular-inertial/s100p_mono_imu_final.yaml`
  - Same camera
  - Tbc (camera→IMU) rotation from calibration; translation has ~3 cm uncertainty
  - IMU noise/random walk from Allan variance
  - IMU.Frequency = 200 Hz

## Run Scripts
- `start_orb.sh` (repo root)
  - Modes:
    - `mono`: `ros2 run orbslam3 mono ... config/monocular/s100p_mono.yaml`
    - `mono-inertial`: `ros2 run orbslam3 mono-inertial ... config/monocular-inertial/s100p_mono_imu_final.yaml`
  - Remaps and params:
    - `-r camera:=/camera/image_raw`
    - `-r imu:=/imu` or `-r imu:=/imu/data`
    - `-p camera_time_offset:=0.09`
    - `-p imu_time_offset:=0.0`
  - Visualization default is off to avoid Pangolin CPU spikes; can be enabled via script argument.
- `play_bag.sh`
  - Plays only camera+IMU topics and supports rate control.

## Current Behavior
- **Monocular (`mono`)**
  - Produces continuous trajectories on S100P and EuRoC.
- **Monocular-inertial (`mono-inertial`)**
  - Frequent `IMU is not or recently initialized` and `Fail to track local map`.
  - Atlas resets multiple times, leading to fragmented trajectories.
  - Applying kalibr time offset (`t_imu = t_cam + 0.09`) is conceptually correct but not sufficient alone.
- **Visualization**
  - Pangolin viewer causes high CPU load and freezes on S100P, especially over remote desktop.
  - Visualization disabled by default; rviz2/evo used for offline inspection.

## Known Issues / Open Points
- IMU fusion remains unstable:
  - Possible contributors: residual time-sync error, Tbc translation uncertainty (~3 cm), aggressive IMU weighting.
- Need systematic IMU noise tuning:
  - Relax IMU noise (increase NoiseGyro/NoiseAcc/GyroWalk/AccWalk) to give more weight to vision.
- Mapping quality evaluation:
  - Use monocular trajectory as baseline.
  - Compare mono-inertial trajectories after each tuning round (start/end error, relocalization).

1、 目标是实现一个完整的slam系统，包括建图定位
2、 现在是硬件受限的状态，相机，IMU经过kalibr标定，会有90ms左右的timeoffset

