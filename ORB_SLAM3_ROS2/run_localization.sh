#!/bin/bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "$0")" && pwd)"
VOCAB="$WORKSPACE/vocabulary/ORBvoc.txt"

MODE="${1:?Usage: run_localization.sh <mono|mono-inertial> <MAP_DIR> [EXP_DIR] [localization_only] [camera_topic] [imu_topic] [camera_time_offset] [imu_time_offset]}"
MAP_DIR="${2:?MAP_DIR required}"

EXP_DIR="${3:-$WORKSPACE/experiments/loc_$(date +%Y%m%d_%H%M%S)}"
if [[ "$EXP_DIR" != /* ]]; then
  EXP_DIR="$WORKSPACE/$EXP_DIR"
fi
LOCALIZATION_ONLY="${4:-false}"
CAM_TOPIC="${5:-/camera/image_raw}"
IMU_TOPIC="${6:-/imu/data}"
CAM_OFFSET="${7:-0.05}"
IMU_OFFSET="${8:-0.0}"
LOG_NAME="run.log"

set +u
source /opt/ros/humble/setup.bash
[ -f "$WORKSPACE/install/setup.bash" ] && source "$WORKSPACE/install/setup.bash"
set -u

if [ ! -d "$MAP_DIR" ]; then
  echo "Map directory not found: $MAP_DIR"
  exit 1
fi

mkdir -p "$EXP_DIR"

if compgen -G "$MAP_DIR/*.osa" > /dev/null; then
  cp "$MAP_DIR"/*.osa "$EXP_DIR"/
fi

run_with_log() {
  local cmd=("$@")
  "${cmd[@]}" > "$EXP_DIR/$LOG_NAME" 2>&1
}

if [ "$MODE" = "mono-inertial" ]; then
  CONFIG="$WORKSPACE/config/monocular-inertial/s100p_mono_imu_localization.yaml"

  pushd "$EXP_DIR" >/dev/null
  run_with_log ros2 run orbslam3 mono-inertial \
    "$VOCAB" "$CONFIG" false true \
    --ros-args \
    -r camera:="$CAM_TOPIC" \
    -r imu:="$IMU_TOPIC" \
    -p camera_time_offset:="$CAM_OFFSET" \
    -p imu_time_offset:="$IMU_OFFSET" \
    -p localization_only:="$LOCALIZATION_ONLY"
  popd >/dev/null

elif [ "$MODE" = "mono" ]; then
  CONFIG="$WORKSPACE/config/monocular/s100p_mono_localization.yaml"

  pushd "$EXP_DIR" >/dev/null
  run_with_log ros2 run orbslam3 mono \
    "$VOCAB" "$CONFIG" false \
    --ros-args \
    -r camera:="$CAM_TOPIC" \
    -p localization_only:="$LOCALIZATION_ONLY"
  popd >/dev/null

else
  echo "Unknown mode: $MODE (use mono or mono-inertial)"
  exit 1
fi

echo "Localization results saved under: $EXP_DIR"
echo "  - KeyFrameTrajectory.txt / FrameTrajectory.txt"
echo "  - *.osa (copied from $MAP_DIR)"
echo "  - $LOG_NAME"