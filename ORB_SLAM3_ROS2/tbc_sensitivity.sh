#!/bin/bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "$0")" && pwd)"
OUT_ROOT="${1:-$WORKSPACE/experiments/tbc_sensitivity_$(date +%Y%m%d_%H%M%S)}"
BAG_PATH="${2:-/mnt/ssd/ros2_bag/vins_bag_upgrade/bag2/bag2_0.db3}"

BASE_CONFIG="$WORKSPACE/config/monocular-inertial/s100p_mono_imu.yaml"
SUMMARY_CSV="$OUT_ROOT/summary.csv"

mkdir -p "$OUT_ROOT"

declare -a SCALES=("1.0" "0.5" "0.25")

for SCALE in "${SCALES[@]}"; do
  RUN_DIR="$OUT_ROOT/scale_${SCALE}"
  CONFIG_PATH="$OUT_ROOT/config_scale_${SCALE}.yaml"

  python3 "$WORKSPACE/imu_config_variant.py" \
    --input "$BASE_CONFIG" \
    --output "$CONFIG_PATH" \
    --tbc-translation-scale "$SCALE"

  bash "$WORKSPACE/run_orb_experiment.sh" \
    mono-inertial \
    "$BAG_PATH" \
    "$CONFIG_PATH" \
    "$RUN_DIR" \
    "0.050" \
    "0.00000" \
    "false"

  python3 "$WORKSPACE/trajectory_stats.py" \
    --input "$RUN_DIR/KeyFrameTrajectory.txt" \
    --label "tbc_scale_${SCALE}" \
    --output "$SUMMARY_CSV" \
    --append
done

RUN_DIR="$OUT_ROOT/translation_zero"
CONFIG_PATH="$OUT_ROOT/config_translation_zero.yaml"

python3 "$WORKSPACE/imu_config_variant.py" \
  --input "$BASE_CONFIG" \
  --output "$CONFIG_PATH" \
  --tbc-translation-zero

bash "$WORKSPACE/run_orb_experiment.sh" \
  mono-inertial \
  "$BAG_PATH" \
  "$CONFIG_PATH" \
  "$RUN_DIR" \
  "0.050" \
  "0.00000" \
  "false"

python3 "$WORKSPACE/trajectory_stats.py" \
  --input "$RUN_DIR/KeyFrameTrajectory.txt" \
  --label "tbc_translation_zero" \
  --output "$SUMMARY_CSV" \
  --append

echo "Sensitivity summary saved to $SUMMARY_CSV"
