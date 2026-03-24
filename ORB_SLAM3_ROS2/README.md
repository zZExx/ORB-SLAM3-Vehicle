# ORB_SLAM3_ROS2
This repository is ROS2 wrapping to use ORB_SLAM3

Repository layout options and `ORB_SLAM3_ROOT_DIR` are described in [`REPO_LAYOUT.md`](../REPO_LAYOUT.md) (parent of this package when using the recommended monorepo / side-by-side layout).

---

## Demo Video
[![orbslam3_ros2](https://user-images.githubusercontent.com/31432135/220839530-786b8a28-d5af-4aa5-b4ed-6234c2f4ca33.PNG)](https://www.youtube.com/watch?v=zXeXL8q72lM)

## Prerequisites

- **Tested**: Ubuntu 22.04, ROS 2 **Humble**, OpenCV 4.5.x, Eigen3, Pangolin, Boost (serialization).
- **Also reported upstream**: Ubuntu 20.04, ROS 2 Foxy, OpenCV 4.2 — see troubleshooting if you use older OpenCV.

Install ROS 2 dependencies (adjust distro):

```bash
sudo apt install ros-${ROS_DISTRO}-vision-opencv ros-${ROS_DISTRO}-message-filters
```

## Build ORB_SLAM3 (core library) first

From the `ORB_SLAM3` directory (same layout as in [`REPO_LAYOUT.md`](../REPO_LAYOUT.md)):

```bash
cd /path/to/ORB_SLAM3
./build.sh
```

This builds Thirdparty (DBoW2, g2o, Sophus), unpacks the vocabulary archive under `Vocabulary/`, and compiles `libORB_SLAM3.so` into `lib/`.

Optional wrapper (from monorepo root that contains `scripts/`, `ORB_SLAM3/`, `ORB_SLAM3_ROS2/`):

```bash
./scripts/build_core.sh
```

Override core location: `export ORB_SLAM3_CORE_DIR=/path/to/ORB_SLAM3`.

## How to build this ROS 2 package

CMake discovers the core library in this order:

1. `-DORB_SLAM3_ROOT_DIR=...` passed to CMake / colcon  
2. Environment variable `ORB_SLAM3_ROOT_DIR`  
3. **Default**: sibling folder `../ORB_SLAM3` next to this package (monorepo / side-by-side layout)

**Option A — build inside this package directory** (as in this workspace):

```bash
cd /path/to/ORB_SLAM3_ROS2
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install --packages-select orbslam3
```

**Option B — standard colcon workspace**:

```bash
mkdir -p ~/colcon_ws/src
# Place or clone ORB_SLAM3 and ORB_SLAM3_ROS2 so that ../ORB_SLAM3 exists from the ROS package,
# or set ORB_SLAM3_ROOT_DIR before building.
cd ~/colcon_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install --packages-select orbslam3
```

**If the core is not a sibling** of `ORB_SLAM3_ROS2`:

```bash
export ORB_SLAM3_ROOT_DIR=/absolute/path/to/ORB_SLAM3
colcon build --symlink-install --packages-select orbslam3
# or:
colcon build --symlink-install --packages-select orbslam3 --cmake-args -DORB_SLAM3_ROOT_DIR=/absolute/path/to/ORB_SLAM3
```

Optional wrapper from monorepo root:

```bash
./scripts/build_ros2.sh
```

Override ROS 2 package path: `export ORB_SLAM3_ROS2_DIR=/path/to/ORB_SLAM3_ROS2`. Extra arguments are forwarded to `colcon build` (e.g. `--cmake-clean-cache`).

## Troubleshooting

1. **`sophus/se3.hpp` not found**  
   The Find module adds `Thirdparty/Sophus` from your `ORB_SLAM3_ROOT_DIR`. Ensure `ORB_SLAM3_ROOT_DIR` points at the core tree that contains `Thirdparty/Sophus`. You only need a system-wide Sophus install if you deliberately removed the bundled headers.

2. **OpenCV version mismatches**  
   Older setups used OpenCV 4.2; see [upstream discussion](https://github.com/zang09/ORB_SLAM3_ROS2/issues/2#issuecomment-1251850857). Prefer the OpenCV version your ROS distro ships with (`ros-$ROS_DISTRO-vision-opencv`).

3. **`ORB_SLAM3 not found` at CMake time**  
   Set `ORB_SLAM3_ROOT_DIR` or use the sibling `ORB_SLAM3` layout; see [`REPO_LAYOUT.md`](../REPO_LAYOUT.md).

4. **`error while loading shared libraries: libORB_SLAM3.so`**  
   After `colcon build`, install the workspace and **source** `install/local_setup.bash` (or `setup.bash`). This package installs `libORB_SLAM3.so`, `libDBoW2.so`, and `libg2o.so` under `install/<ws>/orbslam3/lib/` and sets the node RPATH so they are found. If you copied only `install/orbslam3/lib/orbslam3/*` without the sibling `.so` files in `install/orbslam3/lib/`, or run the binary without sourcing setup, the loader will fail. Temporary workaround: `export LD_LIBRARY_PATH=/path/to/ORB_SLAM3/lib:$LD_LIBRARY_PATH`.

## How to use
1. Source the workspace  
```
$ source ~/colcon_ws/install/local_setup.bash
```

2. Run orbslam mode, which you want.  
This repository only support `MONO, STEREO, RGBD, STEREO-INERTIAL` mode now.  
You can find vocabulary file and config file in here. (e.g. `orbslam3_ros2/vocabulary/ORBvoc.txt`, `orbslam3_ros2/config/monocular/TUM1.yaml` for monocular SLAM).
  - `MONO` mode  
```
$ ros2 run orbslam3 mono PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE
```
  - `STEREO` mode  
```
$ ros2 run orbslam3 stereo PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY
```
  - `RGBD` mode  
```
$ ros2 run orbslam3 rgbd PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE
```
  - `STEREO-INERTIAL` mode  
```
$ ros2 run orbslam3 stereo-inertial PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY [BOOL_EQUALIZE]
```

## Run with rosbag
To play ros1 bag file, you should install `ros1 noetic` & `ros1 bridge`.  
Here is a [link](https://www.theconstructsim.com/ros2-qa-217-how-to-mix-ros1-and-ros2-packages/) to demonstrate example of `ros1-ros2 bridge` procedure.  
If you have `ros1 noetic` and `ros1 bridge` already, open your terminal and follow this:  
(Shell A, B, C, D is all different terminal, e.g. `stereo-inertial` mode)
1. Download EuRoC Dataset (`V1_02_medium.bag`)
```
$ wget -P ~/Downloads http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_02_medium/V1_02_medium.bag
```  

2. Launch Terminal  
(e.g. `ROS1_INSTALL_PATH`=`/opt/ros/noetic`, `ROS2_INSTALL_PATH`=`/opt/ros/foxy`)
```
#Shell A:
source ${ROS1_INSTALL_PATH}/setup.bash
roscore

#Shell B:
source ${ROS1_INSTALL_PATH}/setup.bash
source ${ROS2_INSTALL_PATH}/setup.bash
export ROS_MASTER_URI=http://localhost:11311
ros2 run ros1_bridge dynamic_bridge

#Shell C:
source ${ROS1_INSTALL_PATH}/setup.bash
rosbag play ~/Downloads/V1_02_medium.bag --pause /cam0/image_raw:=/camera/left /cam1/image_raw:=/camera/right /imu0:=/imu

#Shell D:
source ${ROS2_INSTALL_PATH}/setup.bash
ros2 run orbslam3 stereo-inertial PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY [BOOL_EQUALIZE]
```

3. Press `spacebar` in `Shell C` to resume bag file.

## Mapping and Localization (full SLAM workflow)

- **Mapping**: Use configs with `System.SaveAtlasToFile` (e.g. `config/monocular/s100p_mono.yaml`, `config/monocular-inertial/s100p_mono_imu_final.yaml`). When the node exits, it saves the atlas as `*.osa` in the current working directory. With `run_orb_experiment.sh`, the atlas is saved under `OUT_DIR`.
- **Localization**: Use configs with `System.LoadAtlasFromFile` (e.g. `config/monocular/s100p_mono_localization.yaml`, `config/monocular-inertial/s100p_mono_imu_localization.yaml`). Run from the directory that contains the `.osa` file, or set an absolute path in the config.
- **Run localization**: `./run_localization.sh mono <MAP_DIR> [true]` or `./run_localization.sh mono-inertial <MAP_DIR> [true]` (MAP_DIR = directory containing the `.osa` file; third arg = enable localization-only at startup).
- **Toggle localization mode at runtime**: `ros2 service call /localization_mode std_srvs/srv/SetBool "{data: true}"` to enable, `{data: false}` to disable. Or start with `-p localization_only:=true`.

See `background.md` for S100P-specific paths and details.

## Acknowledgments
This repository is modified from [this](https://github.com/curryc/ros2_orbslam3) repository.  
To add `stereo-inertial` mode and improve build difficulites.

paint result
MPLBACKEND=Agg ~/.local/bin/evo_traj tum ./KeyFrameTrajectory.txt  --plot_mode xyz --save_plot ./trajectory.png

./start_orb.sh --mode mono-inertial --bag /home/xzb/Desktop/Cache/dataset/ros2bag/bag15/bag15_0.db3 --exp_dir experiments/exp_test_$(date +%m%d_%H%M) --camera_offset 0.09 --imu_offset 0.0 --bag_rate 0.5 --viz true

./run_localization.sh mono-inertial   /home/xzb/Desktop/Cache/code/ORB_SLAM3/ORB_SLAM3_ROS2/experiments/exp_test_0314_1542   experiments/loc_exp_$(date +%m%d_%H%M)   false /camera/image_raw /imu/data 0.09 0.0

eval traj
python3 frame_revisit_eval.py \
  --input experiments/exp_test_0311_1603/FrameTrajectory.txt \
  --linear-speed-th 0.12 \
  --yaw-rate-th-deg 35 \
  --min-stop-duration 0.8 \
  --min-stop-samples 8 \
  --revisit-radius 1.0 \
  --csv revisit_errors.csv \
  --all-segments-csv all_segments.csv