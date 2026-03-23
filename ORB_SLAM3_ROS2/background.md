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
    - `-p camera_time_offset:=0.0523`
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
  - Applying kalibr time offset (`t_imu = t_cam + 0.0523`) is conceptually correct but not sufficient alone.
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
2、 现在是硬件受限的状态，相机，IMU经过kalibr标定，会有50ms左右的timeoffset
3、 bag 内 IMU 与相机首帧时间戳基本一致（偏差约毫秒级），硬件时钟源一致。
4、 现在无IMU纯mono的结果要优于加入了IMU的结果
5、 结果文件在/home/sunrise/Desktop/ORB_SLAM3_ROS2/mono_imu_result和/home/sunrise/Desktop/ORB_SLAM3_ROS2/mono_result

最新实验（offset=0.07, Tbc平移缩放0.5, IMU噪声倍率2.0）验证结果：/home/sunrise/Desktop/ORB_SLAM3_ROS2/experiments/final_validate_noise2_20260226_101606/KeyFrameTrajectory.txt，关键帧416，轨迹时长约95.455s，首末位移约0.837m，仍明显高于10cm目标。

偏移大的主要原因：IMU初始化反复失败与频繁重置导致地图碎片化；时间同步与Tbc平移误差使视觉与IMU约束不一致；IMU权重设置不当时会放大漂移。加IMU变差的直接表现是“IMU未初始化/丢跟踪/重置”频繁，使得融合无法稳定约束尺度与姿态，反而降低轨迹一致性。

大量BAD LOOP的原因是回环候选通过外观检索但几何/IMU一致性校验失败（漂移大、尺度/姿态不一致、重置导致地图片段不兼容）。影响：回环无法闭合，地图无法被纠正，误差只能累积，导致首末位移长期偏大。

后续运行要求：每次运行将终端输出重定向保存到同一实验目录下，例如：
LOG_DIR=/home/sunrise/Desktop/ORB_SLAM3_ROS2/experiments/exp_$(date +%Y%m%d_%H%M%S)
mkdir -p "$LOG_DIR"
bash /home/sunrise/Desktop/ORB_SLAM3_ROS2/run_orb_experiment.sh \
  mono-inertial \
  /mnt/ssd/ros2_bag/vins_bag_upgrade/bag2/bag2_0.db3 \
  /home/sunrise/Desktop/ORB_SLAM3_ROS2/config/monocular-inertial/s100p_mono_imu_final_final.yaml \
  "$LOG_DIR" \
  0.07 \
  0.00000 \
  false > "$LOG_DIR/run.log" 2>&1

## Mapping + Localization Workflow

### Mapping (save atlas)
- Config (mono): `/home/sunrise/ORB_SLAM3/ORB_SLAM3_ROS2/config/monocular/s100p_mono.yaml`
- Config (mono-inertial): `/home/sunrise/ORB_SLAM3/ORB_SLAM3_ROS2/config/monocular-inertial/s100p_mono_imu_final_final.yaml`
- `System.SaveAtlasToFile` is enabled and will write `*.osa` in the current working directory.
- `run_orb_experiment.sh` uses `pushd "$OUT_DIR"`, so the atlas file will be saved inside `OUT_DIR`.

### Localization (load atlas)
- Config (mono): `config/monocular/s100p_mono_localization.yaml`
- Config (mono-inertial): `config/monocular-inertial/s100p_mono_imu_final_localization.yaml`
- `System.LoadAtlasFromFile` is enabled. Use an absolute path or run from the map directory.
- **Run script**: `./run_localization.sh mono <MAP_DIR> [localization_only]` or `./run_localization.sh mono-inertial <MAP_DIR> [localization_only]` (MAP_DIR = directory containing the `.osa` file; script changes CWD to MAP_DIR so the config’s relative atlas name resolves correctly).

### Localization Mode Control
- Startup-only localization:
  - `--ros-args -p localization_only:=true`
- Runtime toggle:
  - Enable: `ros2 service call /localization_mode std_srvs/srv/SetBool "{data: true}"`
  - Disable: `ros2 service call /localization_mode std_srvs/srv/SetBool "{data: false}"`