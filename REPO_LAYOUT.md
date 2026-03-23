# Repository layout and Git collaboration

## Recommended directory layout (monorepo or side-by-side)

Place the ROS2 package and the ORB_SLAM3 core library **under the same parent directory** with these names:

```text
<workspace>/
  ORB_SLAM3/          # core library (this CMake project)
  ORB_SLAM3_ROS2/   # ROS2 package `orbslam3`
```

`FindORB_SLAM3.cmake` defaults to `ORB_SLAM3_ROOT_DIR = <parent of ORB_SLAM3_ROS2>/ORB_SLAM3` when that path contains `include/System.h`. No environment variable is required for this layout.

## If the core library lives elsewhere

Set either:

- `export ORB_SLAM3_ROOT_DIR=/absolute/path/to/ORB_SLAM3`, or
- `colcon build --cmake-args -DORB_SLAM3_ROOT_DIR=/absolute/path/to/ORB_SLAM3`

## Git strategies

| Strategy | When to use |
|----------|-------------|
| **Single repo (monorepo)** | Same team edits core and ROS2 together; keep `ORB_SLAM3/` and `ORB_SLAM3_ROS2/` in one repository. |
| **Submodule** | Core fork has its own lifecycle; in the ROS2 repo: `third_party/ORB_SLAM3` as a submodule and set `ORB_SLAM3_ROOT_DIR` to that path (or symlink to match the default sibling layout). |
| **Two repositories** | Minimal structural change; document `ORB_SLAM3_ROOT_DIR` in both READMEs and use consistent branch names. |

## Do not commit

- `build/`, `install/`, `log/` from colcon
- `ORB_SLAM3/lib/`, `ORB_SLAM3/build/`, third-party `build/` and `lib/` under `Thirdparty/`
- Generated trajectories and logs under `ORB_SLAM3_ROS2/experiments/`
- `compile_commands.json` (machine-specific paths)
