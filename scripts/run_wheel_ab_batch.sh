#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 4 ]; then
  echo "Usage: $0 <bag_path> <config_path> <output_root> <runs> [mode]"
  echo "Example: $0 /data/large_single_0.db3 ORB_SLAM3_ROS2/config/monocular-inertial/s100p_mono_imu_final.yaml ORB_SLAM3_ROS2/experiments/ab_0415 5 mono-inertial"
  exit 1
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG_PATH="$1"
CONFIG_PATH="$2"
OUT_ROOT="$3"
RUNS="$4"
MODE="${5:-mono-inertial}"

if [ ! -f "$BAG_PATH" ]; then
  echo "Bag file not found: $BAG_PATH"
  exit 1
fi

if [ ! -f "$REPO_ROOT/$CONFIG_PATH" ] && [ ! -f "$CONFIG_PATH" ]; then
  echo "Config file not found: $CONFIG_PATH"
  exit 1
fi

if [ -f "$REPO_ROOT/$CONFIG_PATH" ]; then
  CONFIG_PATH="$REPO_ROOT/$CONFIG_PATH"
fi

if [ "$RUNS" -le 0 ]; then
  echo "runs must be > 0"
  exit 1
fi

mkdir -p "$REPO_ROOT/$OUT_ROOT"
RUN_LOG="$REPO_ROOT/$OUT_ROOT/batch.log"
SUMMARY="$REPO_ROOT/$OUT_ROOT/summary.tsv"

{
  echo -e "label\tratio\tloop_gap"
} > "$SUMMARY"

run_one() {
  local tag="$1"
  local use_wheel="$2"
  local out_dir="$REPO_ROOT/$OUT_ROOT/$tag"
  mkdir -p "$out_dir"
  echo "[INFO] Running $tag (use_wheel=$use_wheel)"
  (
    cd "$REPO_ROOT/ORB_SLAM3_ROS2"
    bash ./run_orb_experiment.sh \
      "$MODE" \
      "$BAG_PATH" \
      "$CONFIG_PATH" \
      "$out_dir" \
      0.050 0.00000 false \
      /camera/image_raw /imu/data 1.0 auto \
      "$use_wheel" /wheel_odom 0.0 /wheel_odom 2>&1 | tee "$out_dir/run.log"
  )
}

collect_metrics() {
  local traj_a="$1"
  local traj_b="$2"
  local eval_out
  eval_out="$(python3 "$REPO_ROOT/scripts/eval_traj_vs_wheel.py" \
    --traj "$traj_a" "$traj_b" \
    --bag "$BAG_PATH" \
    --topic /wheel_odom)"

  echo "$eval_out" | grep -E "^SUMMARY|^Label|^---|^wheel_on_|^wheel_off_" || true

  local on_line off_line
  on_line="$(echo "$eval_out" | grep -E "^wheel_on_" | awk 'NR==1{print; exit}' || true)"
  off_line="$(echo "$eval_out" | grep -E "^wheel_off_" | awk 'NR==1{print; exit}' || true)"

  if [ -n "$on_line" ]; then
    local on_ratio on_gap
    on_ratio="$(echo "$on_line" | awk '{print $(NF-1)}')"
    on_gap="$(echo "$on_line" | awk '{print $NF}')"
    echo -e "wheel_on\t$on_ratio\t$on_gap" >> "$SUMMARY"
  fi
  if [ -n "$off_line" ]; then
    local off_ratio off_gap
    off_ratio="$(echo "$off_line" | awk '{print $(NF-1)}')"
    off_gap="$(echo "$off_line" | awk '{print $NF}')"
    echo -e "wheel_off\t$off_ratio\t$off_gap" >> "$SUMMARY"
  fi
}

pick_traj_file() {
  local dir="$1"
  if [ -f "$dir/FrameTrajectory.txt" ]; then
    echo "$dir/FrameTrajectory.txt"
    return 0
  fi
  if [ -f "$dir/KeyFrameTrajectory.txt" ]; then
    echo "$dir/KeyFrameTrajectory.txt"
    return 0
  fi
  return 1
}

{
  echo "[INFO] Start batch"
  echo "[INFO] bag=$BAG_PATH"
  echo "[INFO] config=$CONFIG_PATH"
  echo "[INFO] output=$REPO_ROOT/$OUT_ROOT"
  echo "[INFO] runs=$RUNS"
} | tee "$RUN_LOG"

for i in $(seq 1 "$RUNS"); do
  ts="$(date +%Y%m%d_%H%M%S)"
  on_tag="wheel_on_${i}_${ts}"
  off_tag="wheel_off_${i}_${ts}"

  run_one "$on_tag" true | tee -a "$RUN_LOG"
  run_one "$off_tag" false | tee -a "$RUN_LOG"

  on_file="$(pick_traj_file "$REPO_ROOT/$OUT_ROOT/$on_tag")"
  off_file="$(pick_traj_file "$REPO_ROOT/$OUT_ROOT/$off_tag")"
  on_traj="$on_file:wheel_on_$i"
  off_traj="$off_file:wheel_off_$i"
  collect_metrics "$on_traj" "$off_traj" | tee -a "$RUN_LOG"
done

python3 - "$SUMMARY" <<'PY' | tee -a "$RUN_LOG"
import csv
import math
import sys

path = sys.argv[1]
vals = {"wheel_on": {"ratio": [], "gap": []}, "wheel_off": {"ratio": [], "gap": []}}
with open(path, "r", encoding="utf-8") as f:
    rd = csv.DictReader(f, delimiter="\t")
    for r in rd:
        key = r["label"]
        if key not in vals:
            continue
        vals[key]["ratio"].append(float(r["ratio"]))
        vals[key]["gap"].append(float(r["loop_gap"]))

def mean(xs):
    return sum(xs) / len(xs) if xs else float("nan")

def std(xs):
    if len(xs) < 2:
        return 0.0
    m = mean(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))

print("===== Batch Aggregate =====")
for k in ["wheel_on", "wheel_off"]:
    r = vals[k]["ratio"]
    g = vals[k]["gap"]
    print(
        f"{k}: n={len(r)} ratio_mean={mean(r):.4f} ratio_std={std(r):.4f} "
        f"gap_mean={mean(g):.4f} gap_std={std(g):.4f}"
    )
PY

echo "[INFO] Done. Logs: $RUN_LOG"
echo "[INFO] Summary: $SUMMARY"
