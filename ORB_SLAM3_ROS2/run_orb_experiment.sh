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
INPUT_SOURCE="${11:-auto}"
USE_WHEEL="${12:-false}"
WHEEL_TOPIC="${13:-/wheel_odom}"
WHEEL_OFFSET="${14:-0.0}"
DB_WHEEL_TOPIC="${15:-/wheel_odom}"
DB_TIMEOUT_SEC=""

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

USE_DB_READER="false"
if [ "$MODE" = "mono-inertial" ] || [ "$MODE" = "mono" ]; then
  if [ "$INPUT_SOURCE" = "db" ]; then
    USE_DB_READER="true"
  elif [ "$INPUT_SOURCE" = "auto" ]; then
    USE_DB_READER="true"
  elif [ "$INPUT_SOURCE" = "subscribe" ]; then
    USE_DB_READER="false"
  else
    echo "Unknown input source: $INPUT_SOURCE (use auto, subscribe, or db)"
    exit 1
  fi
fi

if [ "$MODE" = "mono-inertial" ]; then
  if [ "$USE_DB_READER" = "true" ]; then
    setsid ros2 run orbslam3 mono-inertial "$VOCAB" "$CONFIG_PATH" false "$VIZ" \
      --ros-args \
      -p camera_time_offset:="$CAM_OFFSET" \
      -p imu_time_offset:="$IMU_OFFSET" \
      -p data_source:=db \
      -p db_bag_path:="$BAG_PATH" \
      -p db_camera_topic:="$CAM_TOPIC" \
      -p db_imu_topic:="$IMU_TOPIC" \
      -p db_play_rate:="$RATE" \
      -p use_wheel:="$USE_WHEEL" \
      -p wheel_time_offset:="$WHEEL_OFFSET" \
      -p db_wheel_topic:="$DB_WHEEL_TOPIC" &
  else
    setsid ros2 run orbslam3 mono-inertial "$VOCAB" "$CONFIG_PATH" false "$VIZ" \
      --ros-args \
      -r camera:="$CAM_TOPIC" \
      -r imu:="$IMU_TOPIC" \
      -p camera_time_offset:="$CAM_OFFSET" \
      -p imu_time_offset:="$IMU_OFFSET" \
      -p data_source:=subscribe \
      -p use_wheel:="$USE_WHEEL" \
      -p wheel_topic:="$WHEEL_TOPIC" \
      -p wheel_time_offset:="$WHEEL_OFFSET" &
  fi
elif [ "$MODE" = "mono" ]; then
  if [ "$USE_DB_READER" = "true" ]; then
    setsid ros2 run orbslam3 mono "$VOCAB" "$CONFIG_PATH" "$VIZ" \
      --ros-args \
      -p data_source:=db \
      -p db_bag_path:="$BAG_PATH" \
      -p db_camera_topic:="$CAM_TOPIC" \
      -p db_play_rate:="$RATE" &
  else
    setsid ros2 run orbslam3 mono "$VOCAB" "$CONFIG_PATH" "$VIZ" \
      --ros-args \
      -r camera:="$CAM_TOPIC" \
      -p data_source:=subscribe &
  fi
else
  echo "Unknown mode: $MODE (use mono or mono-inertial)"
  exit 1
fi

SLAM_PGID=$!

if [ "$USE_DB_READER" = "false" ]; then
  # Ensure subscriptions are ready before starting bag playback.
  sleep 30
  BAG_TOPICS="$CAM_TOPIC $IMU_TOPIC"
  if [ "$USE_WHEEL" = "true" ]; then
    BAG_TOPICS="$BAG_TOPICS $WHEEL_TOPIC"
  fi
  ros2 bag play "$BAG_PATH" --topics $BAG_TOPICS -r "$RATE" &
  BAG_PID=$!
  wait "$BAG_PID"
else
  DB_TIMEOUT_SEC=$(python3 - "$BAG_PATH" "$RATE" <<'PY'
import sqlite3
import sys

bag_path = sys.argv[1]
rate = float(sys.argv[2]) if float(sys.argv[2]) > 0 else 1.0
conn = sqlite3.connect(bag_path)
cur = conn.cursor()
cur.execute("SELECT MIN(timestamp), MAX(timestamp) FROM messages;")
row = cur.fetchone()
conn.close()
if not row or row[0] is None or row[1] is None:
    print(120)
    raise SystemExit(0)
duration_sec = (row[1] - row[0]) / 1e9
# Add a fixed safety buffer so SLAM can flush and save outputs.
timeout_sec = int(duration_sec / rate + 30)
print(timeout_sec if timeout_sec > 30 else 30)
PY
)
  # DB mode has no ros2 bag play process; wait roughly bag_duration/rate then stop SLAM.
  for _ in $(seq 1 "$DB_TIMEOUT_SEC"); do
    if ! kill -0 "-$SLAM_PGID" 2>/dev/null; then
      break
    fi
    sleep 1
  done
fi
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
