#!/usr/bin/env python3
"""
Compare ORB-SLAM3 FrameTrajectory files against wheel odometry reference.

Metrics:
  1. Total path length  (SLAM vs wheel integral)
  2. Scale ratio per segment  (drift trend over time)
  3. Loop-closure gap  (Euclidean distance between first and last frame)

Usage (no bag - SLAM-only metrics):
    python3 scripts/eval_traj_vs_wheel.py \
        --traj exp_A/FrameTrajectory.txt:wheel_on \
               exp_B/FrameTrajectory.txt:wheel_off

Usage (with bag - full comparison):
    python3 scripts/eval_traj_vs_wheel.py \
        --traj exp_A/FrameTrajectory.txt:wheel_on \
               exp_B/FrameTrajectory.txt:wheel_off \
        --bag  /path/to/large_single_0.db3 \
        --topic /wheel_odom \
        --segments 10 \
        --signed \
        --plot
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import List, Optional, Tuple

# (t_sec, x, y, z)
TrajPoint = Tuple[float, float, float, float]


# ---------------------------------------------------------------------------
# Trajectory helpers
# ---------------------------------------------------------------------------

def load_traj(path: str) -> List[TrajPoint]:
    """Load TUM-format FrameTrajectory.txt.

    Columns: timestamp tx ty tz qx qy qz qw
    Timestamp may be in nanoseconds (>1e15) or seconds; both are handled.
    Lines starting with '#' or empty lines are skipped.
    """
    points: List[TrajPoint] = []
    with open(path, "r", errors="ignore") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 4:
                continue
            t_raw = float(parts[0])
            # ORB-SLAM3 FrameTrajectory uses nanosecond timestamps as floats
            t_sec = t_raw * 1e-9 if t_raw > 1e15 else t_raw
            x = float(parts[1])
            y = float(parts[2])
            z = float(parts[3])
            points.append((t_sec, x, y, z))
    if not points:
        raise ValueError(f"No valid trajectory points found in: {path}")
    points.sort(key=lambda p: p[0])
    return points


def path_length(traj: List[TrajPoint]) -> float:
    """Sum of Euclidean distances between consecutive frames."""
    total = 0.0
    for i in range(1, len(traj)):
        dx = traj[i][1] - traj[i - 1][1]
        dy = traj[i][2] - traj[i - 1][2]
        dz = traj[i][3] - traj[i - 1][3]
        total += math.sqrt(dx * dx + dy * dy + dz * dz)
    return total


def loop_gap(traj: List[TrajPoint]) -> float:
    """Euclidean distance between first and last frame position."""
    p0 = traj[0]
    p1 = traj[-1]
    dx = p1[1] - p0[1]
    dy = p1[2] - p0[2]
    dz = p1[3] - p0[3]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


# ---------------------------------------------------------------------------
# Wheel odometry helpers
# ---------------------------------------------------------------------------

def _load_wheel_messages(
    bag_path: str,
    topic_name: str,
) -> List[Tuple[float, float]]:
    """Read all (t_sec, velocity_mps) pairs from a ROS2 bag wheel topic."""
    try:
        import rosbag2_py
        from nav_msgs.msg import Odometry
        from rclpy.serialization import deserialize_message
    except ImportError as exc:
        raise RuntimeError(
            "Missing ROS2 Python modules. Source your ROS2 environment first."
        ) from exc

    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id="sqlite3",
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic_name]))

    msgs: List[Tuple[float, float]] = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != topic_name:
            continue
        odom: Odometry = deserialize_message(data, Odometry)
        sec = int(odom.header.stamp.sec)
        nsec = int(odom.header.stamp.nanosec)
        t = float(sec) + float(nsec) * 1e-9
        v = float(odom.twist.twist.linear.x)
        msgs.append((t, v))

    msgs.sort(key=lambda m: m[0])
    return msgs


def wheel_distance_in_range(
    wheel_msgs: List[Tuple[float, float]],
    t0: float,
    t1: float,
    signed: bool,
) -> float:
    """Integrate wheel velocity within [t0, t1) using pre-loaded messages."""
    total = 0.0
    prev_t: Optional[float] = None
    prev_v: float = 0.0
    for t, v in wheel_msgs:
        if t < t0:
            prev_t = t
            prev_v = v
            continue
        if t >= t1:
            break
        # integrate from max(t0, prev_t) to t
        if prev_t is not None:
            dt = t - prev_t
        else:
            dt = t - t0
        vel = prev_v if prev_t is not None else v
        total += (vel if signed else math.fabs(vel)) * dt
        prev_t = t
        prev_v = v
    return total


def segment_ratios(
    traj: List[TrajPoint],
    wheel_msgs: List[Tuple[float, float]],
    n_seg: int,
    signed: bool,
) -> Tuple[List[float], List[float], List[float]]:
    """Split trajectory into n_seg time-equal segments.

    Returns (seg_midpoints_sec, slam_lengths, wheel_lengths).
    Segments where wheel distance is ~0 are kept but ratio is marked as NaN.
    """
    t_start = traj[0][0]
    t_end = traj[-1][0]
    duration = t_end - t_start
    if duration <= 0:
        raise ValueError("Trajectory duration is non-positive.")

    step = duration / n_seg
    midpoints: List[float] = []
    slam_lens: List[float] = []
    wheel_lens: List[float] = []

    for i in range(n_seg):
        seg_t0 = t_start + i * step
        seg_t1 = t_start + (i + 1) * step
        mid = (seg_t0 + seg_t1) / 2.0

        # SLAM path in this segment
        seg_pts = [p for p in traj if seg_t0 <= p[0] < seg_t1]
        # include the last point of previous segment as anchor if available
        earlier = [p for p in traj if p[0] < seg_t0]
        if earlier and seg_pts:
            anchor = earlier[-1]
            seg_pts = [anchor] + seg_pts

        slam_len = path_length(seg_pts) if len(seg_pts) >= 2 else 0.0
        wheel_len = wheel_distance_in_range(wheel_msgs, seg_t0, seg_t1, signed)

        midpoints.append(mid)
        slam_lens.append(slam_len)
        wheel_lens.append(wheel_len)

    return midpoints, slam_lens, wheel_lens


# ---------------------------------------------------------------------------
# Printing helpers
# ---------------------------------------------------------------------------

def _fmt(v: Optional[float], decimals: int = 4) -> str:
    if v is None or math.isnan(v):
        return "  N/A  "
    return f"{v:>{decimals + 6}.{decimals}f}"


def print_summary_table(
    labels: List[str],
    slam_total_lens: List[float],
    wheel_total: Optional[float],
    loop_gaps: List[float],
) -> None:
    col_w = 14
    header = f"{'Label':<20} {'SLAM_len(m)':>{col_w}} {'Wheel_len(m)':>{col_w}} {'Ratio':>{col_w}} {'LoopGap(m)':>{col_w}}"
    print()
    print("=" * len(header))
    print("SUMMARY")
    print("=" * len(header))
    print(header)
    print("-" * len(header))
    for label, slam_len, gap in zip(labels, slam_total_lens, loop_gaps):
        if wheel_total and wheel_total > 0:
            ratio = slam_len / wheel_total
            ratio_str = f"{ratio:>{col_w}.4f}"
            wheel_str = f"{wheel_total:>{col_w}.4f}"
        else:
            ratio_str = f"{'N/A':>{col_w}}"
            wheel_str = f"{'N/A':>{col_w}}"
        print(
            f"{label:<20} {slam_len:>{col_w}.4f} {wheel_str} {ratio_str} {gap:>{col_w}.4f}"
        )
    print("=" * len(header))
    print()


def print_segment_table(
    label: str,
    seg_idx: List[int],
    midpoints: List[float],
    slam_lens: List[float],
    wheel_lens: List[float],
) -> None:
    print(f"\n--- Segment ratios: {label} ---")
    print(f"{'Seg':>4} {'t_mid(s)':>12} {'SLAM(m)':>10} {'Wheel(m)':>10} {'Ratio':>8}")
    for i, (mid, sl, wl) in enumerate(zip(midpoints, slam_lens, wheel_lens)):
        ratio_str = f"{sl / wl:>8.4f}" if wl > 1e-6 else "     N/A"
        print(f"{seg_idx[i]:>4} {mid:>12.2f} {sl:>10.4f} {wl:>10.4f} {ratio_str}")


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_segment_ratios(
    labels: List[str],
    all_midpoints: List[List[float]],
    all_slam_lens: List[List[float]],
    all_wheel_lens: List[List[float]],
    save_path: Optional[str] = None,
) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[WARN] matplotlib not available; skipping plot.")
        return

    fig, axes = plt.subplots(2, 1, figsize=(10, 8))

    # Panel 1: per-segment scale ratio
    ax1 = axes[0]
    for label, mids, slam_lens, wheel_lens in zip(
        labels, all_midpoints, all_slam_lens, all_wheel_lens
    ):
        ratios = [
            sl / wl if wl > 1e-6 else float("nan")
            for sl, wl in zip(slam_lens, wheel_lens)
        ]
        ax1.plot(mids, ratios, marker="o", label=label)
    ax1.axhline(1.0, color="gray", linestyle="--", linewidth=0.8)
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("SLAM / Wheel ratio")
    ax1.set_title("Per-segment scale ratio (closer to 1.0 is better)")
    ax1.legend()
    ax1.grid(True, alpha=0.4)

    # Panel 2: per-segment SLAM path lengths overlaid
    ax2 = axes[1]
    for label, mids, slam_lens, _ in zip(
        labels, all_midpoints, all_slam_lens, all_wheel_lens
    ):
        ax2.plot(mids, slam_lens, marker="s", label=f"{label} SLAM")
    if all_wheel_lens[0]:
        # plot wheel reference once (same for all experiments)
        ax2.plot(
            all_midpoints[0],
            all_wheel_lens[0],
            marker="x",
            linestyle="--",
            color="black",
            label="Wheel ref",
        )
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Path length per segment (m)")
    ax2.set_title("Segment path lengths")
    ax2.legend()
    ax2.grid(True, alpha=0.4)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150)
        print(f"[INFO] Plot saved to: {save_path}")
    else:
        plt.show()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def _parse_traj_arg(s: str) -> Tuple[str, str]:
    """Parse 'path/to/traj.txt:label' or 'path/to/traj.txt'."""
    if ":" in s:
        idx = s.rfind(":")
        return s[:idx], s[idx + 1:]
    return s, Path(s).parent.name


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Evaluate ORB-SLAM3 FrameTrajectory vs wheel odometry."
    )
    parser.add_argument(
        "--traj",
        nargs="+",
        required=True,
        metavar="FILE[:LABEL]",
        help="One or more FrameTrajectory.txt paths, optionally with :label suffix.",
    )
    parser.add_argument(
        "--bag",
        default=None,
        metavar="BAG_PATH",
        help="ROS2 bag path (directory containing metadata.yaml / .db3 file).",
    )
    parser.add_argument(
        "--topic",
        default="/wheel_odom",
        help="Wheel odometry topic name (default: /wheel_odom).",
    )
    parser.add_argument(
        "--segments",
        type=int,
        default=10,
        metavar="N",
        help="Number of time segments for ratio trend (default: 10).",
    )
    parser.add_argument(
        "--signed",
        action="store_true",
        help="Integrate signed wheel velocity (use when vehicle reverses).",
    )
    parser.add_argument(
        "--plot",
        action="store_true",
        help="Show/save segment ratio plot.",
    )
    parser.add_argument(
        "--save-plot",
        default=None,
        metavar="PNG_PATH",
        help="Save plot to file instead of displaying (implies --plot).",
    )
    args = parser.parse_args()

    # --- Load trajectories ---
    traj_list: List[List[TrajPoint]] = []
    labels: List[str] = []
    for traj_arg in args.traj:
        path, label = _parse_traj_arg(traj_arg)
        print(f"[INFO] Loading trajectory: {path}  label={label}")
        try:
            traj = load_traj(path)
        except Exception as e:
            print(f"[ERROR] {e}", file=sys.stderr)
            return 1
        traj_list.append(traj)
        labels.append(label)
        print(
            f"       frames={len(traj)}  "
            f"t=[{traj[0][0]:.2f}, {traj[-1][0]:.2f}]s  "
            f"duration={traj[-1][0]-traj[0][0]:.2f}s"
        )

    # --- Load wheel messages once (if bag given) ---
    wheel_msgs: Optional[List[Tuple[float, float]]] = None
    wheel_total: Optional[float] = None
    if args.bag:
        print(f"[INFO] Loading wheel messages from bag: {args.bag}")
        try:
            wheel_msgs = _load_wheel_messages(args.bag, args.topic)
        except RuntimeError as e:
            print(f"[ERROR] {e}", file=sys.stderr)
            return 1
        if not wheel_msgs:
            print(f"[WARN] No wheel messages found on topic {args.topic}; ratio will be N/A.")
        else:
            print(f"       wheel messages={len(wheel_msgs)}  "
                  f"t=[{wheel_msgs[0][0]:.2f}, {wheel_msgs[-1][0]:.2f}]s")
            # total wheel distance over the full bag time range
            t0_all = min(t[0][0] for t in traj_list)
            t1_all = max(t[-1][0] for t in traj_list)
            wheel_total = wheel_distance_in_range(
                wheel_msgs, t0_all, t1_all, args.signed
            )
            print(f"       wheel total distance (over traj range): {wheel_total:.4f} m")

    # --- Per-trajectory metrics ---
    slam_total_lens: List[float] = []
    loop_gaps_list: List[float] = []
    all_midpoints: List[List[float]] = []
    all_slam_seg: List[List[float]] = []
    all_wheel_seg: List[List[float]] = []

    for traj, label in zip(traj_list, labels):
        total_len = path_length(traj)
        gap = loop_gap(traj)
        slam_total_lens.append(total_len)
        loop_gaps_list.append(gap)

        if wheel_msgs is not None and len(wheel_msgs) > 1:
            mids, slam_seg, wheel_seg = segment_ratios(
                traj, wheel_msgs, args.segments, args.signed
            )
            all_midpoints.append(mids)
            all_slam_seg.append(slam_seg)
            all_wheel_seg.append(wheel_seg)
            print_segment_table(label, list(range(1, args.segments + 1)), mids, slam_seg, wheel_seg)
        else:
            all_midpoints.append([])
            all_slam_seg.append([])
            all_wheel_seg.append([])

    # --- Summary table ---
    print_summary_table(labels, slam_total_lens, wheel_total, loop_gaps_list)

    # --- Interpretation hints ---
    print("Interpretation:")
    print("  Ratio close to 1.0     -> scale well-calibrated vs wheel reference")
    print("  Ratio drifts over time -> scale drift not suppressed by SLAM")
    print("  LoopGap near 0.0       -> trajectory closes the loop correctly")
    print("  LoopGap large          -> accumulated drift (no loop closure, or suppressed)")
    print()

    # --- Plot ---
    do_plot = args.plot or args.save_plot
    if do_plot and wheel_msgs is not None and len(wheel_msgs) > 1:
        plot_segment_ratios(
            labels,
            all_midpoints,
            all_slam_seg,
            all_wheel_seg,
            save_path=args.save_plot,
        )
    elif do_plot:
        print("[WARN] --plot requested but no bag data; skipping plot.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
