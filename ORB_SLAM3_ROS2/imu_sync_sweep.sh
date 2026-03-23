#!/bin/bash
set -euo pipefail

START="${1:-0.00}"
END="${2:-0.08}"
STEP="${3:-0.01}"
WORKSPACE="$(cd "$(dirname "$0")" && pwd)"
OUT_ROOT="${4:-$WORKSPACE/experiments/sync_sweep_$(date +%Y%m%d_%H%M%S)}"
BAG_PATH="${5:?Usage: $0 START END STEP [OUT_ROOT] BAG_PATH [CAM_TOPIC] [IMU_TOPIC] [BAG_RATE]. BAG_PATH required.}"
CAM_TOPIC="${6:-/camera/image_raw}"
IMU_TOPIC="${7:-/imu/data}"
BAG_RATE="${8:-1.0}"

BASE_CONFIG="$WORKSPACE/config/monocular-inertial/s100p_mono_imu.yaml"
SUMMARY_CSV="$OUT_ROOT/summary.csv"

# Mirror start_orb.sh: source ROS before any runs
set +u
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u

mkdir -p "$OUT_ROOT"

# Same run_with_log as start_orb.sh
run_with_log() {
  local cmd=("$@")
  "${cmd[@]}" > "$EXP_DIR/$LOG_NAME" 2>&1
  if [ -f "$EXP_DIR/$LOG_NAME" ]; then
    local bad_loop loop_detect reset_map imu_uninit
    bad_loop=$(awk '/BAD LOOP/{c++} END{print c+0}' "$EXP_DIR/$LOG_NAME")
    loop_detect=$(awk '/Loop detected/{c++} END{print c+0}' "$EXP_DIR/$LOG_NAME")
    reset_map=$(awk '/Reseting active map/{c++} END{print c+0}' "$EXP_DIR/$LOG_NAME")
    imu_uninit=$(awk '/IMU is not or recently initialized/{c++} END{print c+0}' "$EXP_DIR/$LOG_NAME")
    echo "  Log summary: bad_loop=${bad_loop} loop_detected=${loop_detect} reset_map=${reset_map} imu_uninit=${imu_uninit}"
  fi
}

# Reliable float loop (seq skips 0.04, 0.05 on some systems)
# Use 4 decimals so step 0.001 yields 0.0200, 0.0210, 0.0220... (unique dirs)
OFFSETS=$(python3 -c "
s, e, st = float('$START'), float('$END'), float('$STEP')
n = int(round((e - s) / st)) + 1
for k in range(n):
    i = round(s + k * st, 6)
    if i <= e + 1e-9:
        print(f'{i:.4f}')
")

for OFFSET in $OFFSETS; do
  RUN_DIR="$OUT_ROOT/offset_${OFFSET}"
  mkdir -p "$RUN_DIR"
  echo "Running offset=${OFFSET}"
  # Same pattern as start_orb sweep-noise: subshell + run_with_log
  (
    LOG_NAME="run.log"
    EXP_DIR="$RUN_DIR"
    run_with_log bash "$WORKSPACE/run_orb_experiment.sh" \
      mono-inertial \
      "$BAG_PATH" \
      "$BASE_CONFIG" \
      "$RUN_DIR" \
      "$OFFSET" \
      "0.00000" \
      "false" \
      "$CAM_TOPIC" \
      "$IMU_TOPIC" \
      "$BAG_RATE"
  )

  if [ -s "$RUN_DIR/KeyFrameTrajectory.txt" ]; then
    python3 "$WORKSPACE/trajectory_stats.py" \
      --input "$RUN_DIR/KeyFrameTrajectory.txt" \
      --label "offset_${OFFSET}" \
      --output "$SUMMARY_CSV" \
      --append
  else
    echo "  Skipped stats (empty or missing trajectory)"
  fi
done

echo "Sweep summary saved to $SUMMARY_CSV"
if [ -f "$SUMMARY_CSV" ] && [ -s "$SUMMARY_CSV" ]; then
  echo ""
  python3 "$WORKSPACE/select_best_offset.py" --input "$SUMMARY_CSV" || true
fi
