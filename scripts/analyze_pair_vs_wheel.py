#!/usr/bin/env python3
"""Compare two experiment directories against wheel distance reference."""

from __future__ import annotations

import argparse
import os
import sys
from typing import Optional, Tuple


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

from analyze_ab_results import parse_run_log, parse_summary_line, run_eval  # noqa: E402


def find_traj(exp_dir: str) -> Optional[str]:
    for name in ("FrameTrajectory.txt", "KeyFrameTrajectory.txt"):
        path = os.path.join(exp_dir, name)
        if os.path.isfile(path):
            return path
    return None


def read_log_stats(exp_dir: str) -> Tuple[int, int]:
    return parse_run_log(os.path.join(exp_dir, "run.log"))


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare two experiment directories.")
    parser.add_argument("dir_a", help="First experiment directory")
    parser.add_argument("label_a", help="First experiment label")
    parser.add_argument("dir_b", help="Second experiment directory")
    parser.add_argument("label_b", help="Second experiment label")
    parser.add_argument("bag_path", help="Rosbag path used for eval_traj_vs_wheel.py")
    parser.add_argument("--topic", default="/wheel_odom", help="Wheel topic for eval_traj_vs_wheel.py")
    args = parser.parse_args()

    traj_a = find_traj(args.dir_a)
    traj_b = find_traj(args.dir_b)
    if traj_a is None or traj_b is None:
        print(f"[ERROR] Missing trajectory file: dir_a={traj_a} dir_b={traj_b}")
        sys.exit(1)

    output = run_eval(traj_a, args.label_a, traj_b, args.label_b, args.bag_path, args.topic)
    ratio_a, gap_a = parse_summary_line(output, args.label_a)
    ratio_b, gap_b = parse_summary_line(output, args.label_b)
    if ratio_a is None or gap_a is None or ratio_b is None or gap_b is None:
        print("[ERROR] Failed to parse eval output.")
        print(output[-800:])
        sys.exit(1)

    loop_a, bad_a = read_log_stats(args.dir_a)
    loop_b, bad_b = read_log_stats(args.dir_b)

    print(
        f"{'Label':<18} {'ratio':>9} {'loop_gap':>10} {'loop_detected':>14} {'bad_loop':>10}"
    )
    print("-" * 68)
    print(f"{args.label_a:<18} {ratio_a:>9.4f} {gap_a:>10.4f} {loop_a:>14d} {bad_a:>10d}")
    print(f"{args.label_b:<18} {ratio_b:>9.4f} {gap_b:>10.4f} {loop_b:>14d} {bad_b:>10d}")
    print("=" * 68)
    print(f"delta ratio ({args.label_a}-{args.label_b}): {ratio_a - ratio_b:+.4f}")
    print(f"delta gap   ({args.label_a}-{args.label_b}): {gap_a - gap_b:+.4f}")
    print(f"delta loop  ({args.label_a}-{args.label_b}): {loop_a - loop_b:+d}")
    print(f"delta bad   ({args.label_a}-{args.label_b}): {bad_a - bad_b:+d}")


if __name__ == "__main__":
    main()
