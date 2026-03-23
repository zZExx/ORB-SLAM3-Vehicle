#!/bin/bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "$0")" && pwd)"
OUT_ROOT="${1:-$WORKSPACE/experiments/imu_noise_tune_$(date +%Y%m%d_%H%M%S)}"
BAG_PATH="${2:-/mnt/ssd/ros2_bag/vins_bag_upgrade/bag2/bag2_0.db3}"

BASE_CONFIG="$WORKSPACE/config/monocular-inertial/s100p_mono_imu.yaml"
SUMMARY_CSV="$OUT_ROOT/summary.csv"

mkdir -p "$OUT_ROOT"

declare -a SCALES=("1.0" "2.0" "4.0" "8.0")

for SCALE in "${SCALES[@]}"; do
  RUN_DIR="$OUT_ROOT/noise_${SCALE}"
  CONFIG_PATH="$OUT_ROOT/config_noise_${SCALE}.yaml"

  python3 "$WORKSPACE/imu_config_variant.py" \
    --input "$BASE_CONFIG" \
    --output "$CONFIG_PATH" \
    --imu-noise-scale "$SCALE"

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
    --label "noise_${SCALE}" \
    --output "$SUMMARY_CSV" \
    --append
done

echo "Noise tuning summary saved to $SUMMARY_CSV"
