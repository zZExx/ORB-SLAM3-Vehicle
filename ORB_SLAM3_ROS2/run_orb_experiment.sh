#!/bin/bash
set -euo pipefail

MODE="${1:-mono-inertial}"
BAG_PATH="${2:-/mnt/ssd/ros2_bag/vins_bag_upgrade/bag2/bag2_0.db3}"
REPO="$(cd "$(dirname "$0")" && pwd)"
CONFIG_PATH="${3:-$REPO/config/monocular-inertial/s100p_mono_imu.yaml}"
OUT_DIR="${4:-$REPO/experiments/run_$(date +%Y%m%d_%H%M%S)}"
CAM_OFFSET="${5:-0.050}"
IMU_OFFSET="${6:-0.00000}"
VIZ="${7:-true}"
CAM_TOPIC="${8:-/camera/image_raw}"
IMU_TOPIC="${9:-/imu/data}"
RATE="${10:-1.0}"

VOCAB="$REPO/vocabulary/ORBvoc.txt"

set +u
source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash"
set -u

if [ ! -f "$BAG_PATH" ]; then
  echo "Bag file not found: $BAG_PATH"
  exit 1
fi

mkdir -p "$OUT_DIR"

cleanup() {
  if [ -n "${SLAM_PGID:-}" ] && kill -0 "-$SLAM_PGID" 2>/dev/null; then
    kill -INT "-$SLAM_PGID" || true
    wait "$SLAM_PGID" || true
  fi
  if [ -n "${BAG_PID:-}" ] && kill -0 "$BAG_PID" 2>/dev/null; then
    kill -INT "$BAG_PID" || true
    wait "$BAG_PID" || true
  fi
}

trap cleanup EXIT

pushd "$OUT_DIR" >/dev/null

ros2 bag play "$BAG_PATH" --topics "$CAM_TOPIC" "$IMU_TOPIC" -r "$RATE" &
BAG_PID=$!

sleep 1

if [ "$MODE" = "mono-inertial" ]; then
  setsid ros2 run orbslam3 mono-inertial "$VOCAB" "$CONFIG_PATH" false "$VIZ" \
    --ros-args \
    -r camera:="$CAM_TOPIC" \
    -r imu:="$IMU_TOPIC" \
    -p camera_time_offset:="$CAM_OFFSET" \
    -p imu_time_offset:="$IMU_OFFSET" &
elif [ "$MODE" = "mono" ]; then
  setsid ros2 run orbslam3 mono "$VOCAB" "$CONFIG_PATH" "$VIZ" \
    --ros-args \
    -r camera:="$CAM_TOPIC" &
else
  echo "Unknown mode: $MODE (use mono or mono-inertial)"
  exit 1
fi

SLAM_PGID=$!

wait "$BAG_PID"
if kill -0 "-$SLAM_PGID" 2>/dev/null; then
  kill -INT "-$SLAM_PGID"
  for _ in $(seq 1 30); do
    if ! kill -0 "-$SLAM_PGID" 2>/dev/null; then
      break
    fi
    sleep 1
  done
  if kill -0 "-$SLAM_PGID" 2>/dev/null; then
    kill -TERM "-$SLAM_PGID" || true
    sleep 5
  fi
  if kill -0 "-$SLAM_PGID" 2>/dev/null; then
    kill -KILL "-$SLAM_PGID" || true
  fi
  wait "$SLAM_PGID" || true
fi

popd >/dev/null

echo "Saved trajectory to $OUT_DIR/KeyFrameTrajectory.txt"
