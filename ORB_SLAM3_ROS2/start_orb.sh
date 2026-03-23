#!/bin/bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "$0")" && pwd)"
VOCAB=$WORKSPACE/vocabulary/ORBvoc.txt

MODE="mono-inertial"
VIZ="false"
CONFIG="$WORKSPACE/config/monocular-inertial/s100p_mono_imu_final.yaml"
CONFIG_SET="0"
EXP_DIR="$WORKSPACE/experiments/exp_$(date +%Y%m%d_%H%M%S)"
LOG_NAME="run.log"
CAM_OFFSET="0.07"
IMU_OFFSET="0.00000"
CAM_TOPIC="/camera/image_raw"
IMU_TOPIC="/imu/data"
BAG_PATH=""
BAG_RATE="0.5"
INPUT_SOURCE="auto"
TBC_SCALE="0.5"
NOISE_SCALES="2.0,4.0,8.0"

usage() {
  cat <<'EOF'
Usage:
  ./start_orb.sh [options]

Options:
  --mode mono|mono-inertial|sweep-noise
  --config PATH
  --viz true|false
  --exp_dir PATH
  --log NAME
  --camera_offset FLOAT
  --imu_offset FLOAT
  --camera_topic TOPIC
  --imu_topic TOPIC
  --bag PATH
  --bag_rate FLOAT
  --input_source auto|subscribe|db
  --tbc_scale FLOAT
  --noise_scales CSV
  --help
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    --mode) MODE="$2"; shift 2 ;;
    --config) CONFIG="$2"; CONFIG_SET="1"; shift 2 ;;
    --viz) VIZ="$2"; shift 2 ;;
    --exp_dir) EXP_DIR="$2"; shift 2 ;;
    --log) LOG_NAME="$2"; shift 2 ;;
    --camera_offset) CAM_OFFSET="$2"; shift 2 ;;
    --imu_offset) IMU_OFFSET="$2"; shift 2 ;;
    --camera_topic) CAM_TOPIC="$2"; shift 2 ;;
    --imu_topic) IMU_TOPIC="$2"; shift 2 ;;
    --bag) BAG_PATH="$2"; shift 2 ;;
    --bag_rate) BAG_RATE="$2"; shift 2 ;;
    --input_source) INPUT_SOURCE="$2"; shift 2 ;;
    --tbc_scale) TBC_SCALE="$2"; shift 2 ;;
    --noise_scales) NOISE_SCALES="$2"; shift 2 ;;
    --help) usage; exit 0 ;;
    *) echo "Unknown option: $1"; usage; exit 1 ;;
  esac
done

# Select default config based on mode if user did not override --config
if [ "$CONFIG_SET" = "0" ]; then
  if [ "$MODE" = "mono" ]; then
    CONFIG="$WORKSPACE/config/monocular/s100p_mono.yaml"
  elif [ "$MODE" = "mono-inertial" ] || [ "$MODE" = "sweep-noise" ]; then
    CONFIG="$WORKSPACE/config/monocular-inertial/s100p_mono_imu_final.yaml"
  fi
fi

set +u
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u

mkdir -p "$EXP_DIR"

run_with_log() {
  local cmd=("$@")
  "${cmd[@]}" > "$EXP_DIR/$LOG_NAME" 2>&1

  if [ -f "$EXP_DIR/$LOG_NAME" ]; then
    local bad_loop
    local loop_detect
    local reset_map
    local imu_uninit
    bad_loop=$(awk '/BAD LOOP/{c++} END{print c+0}' "$EXP_DIR/$LOG_NAME")
    loop_detect=$(awk '/Loop detected/{c++} END{print c+0}' "$EXP_DIR/$LOG_NAME")
    reset_map=$(awk '/Reseting active map/{c++} END{print c+0}' "$EXP_DIR/$LOG_NAME")
    imu_uninit=$(awk '/IMU is not or recently initialized/{c++} END{print c+0}' "$EXP_DIR/$LOG_NAME")
    echo "Log summary: bad_loop=${bad_loop} loop_detected=${loop_detect} reset_map=${reset_map} imu_uninit=${imu_uninit}"
  fi
}

if [ "$MODE" = "sweep-noise" ]; then
  if [ -z "$BAG_PATH" ]; then
    echo "Missing --bag for sweep-noise"
    exit 1
  fi
  IFS=',' read -r -a SCALE_LIST <<< "$NOISE_SCALES"
  for SCALE in "${SCALE_LIST[@]}"; do
    RUN_DIR="$EXP_DIR/noise_${SCALE}"
    CONFIG_PATH="$EXP_DIR/config_noise_${SCALE}.yaml"
    mkdir -p "$RUN_DIR"

    python3 "$WORKSPACE/imu_config_variant.py" \
      --input "$CONFIG" \
      --output "$CONFIG_PATH" \
      --tbc-translation-scale "$TBC_SCALE" \
      --imu-noise-scale "$SCALE"

    ( \
      LOG_NAME="run.log" \
      EXP_DIR="$RUN_DIR" \
      run_with_log bash "$WORKSPACE/run_orb_experiment.sh" \
        mono-inertial \
        "$BAG_PATH" \
        "$CONFIG_PATH" \
        "$RUN_DIR" \
        "$CAM_OFFSET" \
        "$IMU_OFFSET" \
        "$VIZ" \
        "$CAM_TOPIC" \
        "$IMU_TOPIC" \
        "$BAG_RATE" \
        "$INPUT_SOURCE" \
    )
  done
  exit 0
fi

if [ -n "$BAG_PATH" ]; then
  run_with_log bash "$WORKSPACE/run_orb_experiment.sh" \
    "$MODE" \
    "$BAG_PATH" \
    "$CONFIG" \
    "$EXP_DIR" \
    "$CAM_OFFSET" \
    "$IMU_OFFSET" \
    "$VIZ" \
    "$CAM_TOPIC" \
    "$IMU_TOPIC" \
    "$BAG_RATE" \
    "$INPUT_SOURCE"
  exit 0
fi

pushd "$EXP_DIR" >/dev/null
if [ "$MODE" = "mono" ]; then
  run_with_log ros2 run orbslam3 mono "$VOCAB" "$CONFIG" "$VIZ" \
    --ros-args -r camera:="$CAM_TOPIC"
elif [ "$MODE" = "mono-inertial" ]; then
  run_with_log ros2 run orbslam3 mono-inertial "$VOCAB" "$CONFIG" false "$VIZ" \
    --ros-args -r camera:="$CAM_TOPIC" -r imu:="$IMU_TOPIC" \
    -p camera_time_offset:="$CAM_OFFSET" -p imu_time_offset:="$IMU_OFFSET"
else
  echo "Unknown mode: $MODE (use mono, mono-inertial, or sweep-noise)"
  popd >/dev/null
  exit 1
fi
popd >/dev/null
