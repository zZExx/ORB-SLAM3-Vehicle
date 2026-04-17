#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 5 ]; then
  echo "Usage: $0 <bag_path> <config_path> <output_root> <old_wheel_dir> <alt_noise> [baseline_runs] [mode]"
  echo "Example: $0 /data/large_single_0.db3 ORB_SLAM3_ROS2/config/monocular-inertial/s100p_mono_imu_final.yaml ORB_SLAM3_ROS2/experiments/three_checks /path/to/old_wheel_on 0.02 3 mono-inertial"
  exit 1
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG_PATH="$1"
CONFIG_PATH="$2"
OUTPUT_ROOT="$3"
OLD_WHEEL_DIR="$4"
ALT_NOISE="$5"
BASELINE_RUNS="${6:-3}"
MODE="${7:-mono-inertial}"

if [ -f "$REPO_ROOT/$CONFIG_PATH" ]; then
  CONFIG_PATH="$REPO_ROOT/$CONFIG_PATH"
fi

if [ ! -f "$CONFIG_PATH" ]; then
  echo "Config file not found: $CONFIG_PATH"
  exit 1
fi

if [ ! -f "$BAG_PATH" ]; then
  echo "Bag file not found: $BAG_PATH"
  exit 1
fi

if [ ! -d "$OLD_WHEEL_DIR" ]; then
  echo "Old wheel-on directory not found: $OLD_WHEEL_DIR"
  exit 1
fi

ABS_OUTPUT_ROOT="$REPO_ROOT/$OUTPUT_ROOT"
mkdir -p "$ABS_OUTPUT_ROOT"

BASELINE_ROOT="$OUTPUT_ROOT/exp1_baseline"
CURRENT_ON_DIR="$ABS_OUTPUT_ROOT/exp2_current_wheel_on"
ALT_ON_DIR="$ABS_OUTPUT_ROOT/exp3_alt_noise_on"
ALT_CONFIG="$ABS_OUTPUT_ROOT/tmp_alt_noise.yaml"

echo "[STEP 1] Baseline wheel_off vs wheel_on"
bash "$REPO_ROOT/scripts/run_wheel_ab_batch.sh" \
  "$BAG_PATH" \
  "$CONFIG_PATH" \
  "$BASELINE_ROOT" \
  "$BASELINE_RUNS" \
  "$MODE"

python3 "$REPO_ROOT/scripts/analyze_ab_results.py" \
  "$REPO_ROOT/$BASELINE_ROOT" \
  "$BAG_PATH"

echo
echo "[STEP 2] Old wheel_on vs current wheel_on"
mkdir -p "$CURRENT_ON_DIR"
(
  cd "$REPO_ROOT/ORB_SLAM3_ROS2"
  bash ./run_orb_experiment.sh \
    "$MODE" \
    "$BAG_PATH" \
    "$CONFIG_PATH" \
    "$CURRENT_ON_DIR" \
    0.050 0.00000 false \
    /camera/image_raw /imu/data 1.0 auto \
    true /wheel_odom 0.0 /wheel_odom 2>&1 | tee "$CURRENT_ON_DIR/run.log"
)

python3 "$REPO_ROOT/scripts/analyze_pair_vs_wheel.py" \
  "$OLD_WHEEL_DIR" old_wheel_on \
  "$CURRENT_ON_DIR" current_wheel_on \
  "$BAG_PATH"

echo
echo "[STEP 3] Current wheel_on vs alternate NoiseVel wheel_on"
python3 - "$CONFIG_PATH" "$ALT_CONFIG" "$ALT_NOISE" <<'PY'
import re
import sys

src_path = sys.argv[1]
dst_path = sys.argv[2]
alt_noise = sys.argv[3]

with open(src_path, "r", encoding="utf-8") as handle:
    text = handle.read()

updated, count = re.subn(
    r"^(Wheel\.NoiseVel:\s*).*$",
    rf"\g<1>{alt_noise}",
    text,
    count=1,
    flags=re.MULTILINE,
)
if count != 1:
    raise SystemExit("Failed to update Wheel.NoiseVel in temp config")

with open(dst_path, "w", encoding="utf-8") as handle:
    handle.write(updated)
PY

mkdir -p "$ALT_ON_DIR"
(
  cd "$REPO_ROOT/ORB_SLAM3_ROS2"
  bash ./run_orb_experiment.sh \
    "$MODE" \
    "$BAG_PATH" \
    "$ALT_CONFIG" \
    "$ALT_ON_DIR" \
    0.050 0.00000 false \
    /camera/image_raw /imu/data 1.0 auto \
    true /wheel_odom 0.0 /wheel_odom 2>&1 | tee "$ALT_ON_DIR/run.log"
)

python3 "$REPO_ROOT/scripts/analyze_pair_vs_wheel.py" \
  "$CURRENT_ON_DIR" current_wheel_on \
  "$ALT_ON_DIR" "alt_noise_${ALT_NOISE}" \
  "$BAG_PATH"

echo
echo "[DONE] Outputs saved under: $ABS_OUTPUT_ROOT"
