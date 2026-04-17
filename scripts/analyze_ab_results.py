#!/usr/bin/env python3
"""Analyze wheel A/B batch results from experiment directories."""

from __future__ import annotations

import argparse
import os
import re
import statistics
import subprocess
import sys
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple


@dataclass
class Metrics:
    ratio: float
    loop_gap: float
    loop_detected: int
    bad_loop: int
    scale_after: float
    bg_delta_norm: float
    ba_delta_norm: float


@dataclass
class VibaMetrics:
    scale_after: float
    bg_delta_norm: float
    ba_delta_norm: float


def find_traj(exp_dir: str) -> Optional[str]:
    for name in ("FrameTrajectory.txt", "KeyFrameTrajectory.txt"):
        path = os.path.join(exp_dir, name)
        if os.path.isfile(path):
            return path
    return None


def run_eval(traj_a: str, label_a: str, traj_b: str, label_b: str, bag_path: str, topic: str) -> str:
    script = os.path.join(os.path.dirname(__file__), "eval_traj_vs_wheel.py")
    result = subprocess.run(
        [
            "python3",
            script,
            "--traj",
            f"{traj_a}:{label_a}",
            f"{traj_b}:{label_b}",
            "--bag",
            bag_path,
            "--topic",
            topic,
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    return result.stdout + result.stderr


def parse_summary_line(output: str, label: str) -> Tuple[Optional[float], Optional[float]]:
    for line in output.splitlines():
        if line.strip().startswith(label):
            parts = line.split()
            if len(parts) >= 5:
                try:
                    return float(parts[-2]), float(parts[-1])
                except ValueError:
                    return None, None
    return None, None


def parse_batch_sections(batch_log: str) -> Dict[str, Tuple[int, int]]:
    if not os.path.isfile(batch_log):
        return {}

    sections: Dict[str, Tuple[int, int]] = {}
    current_tag: Optional[str] = None
    loop_count = 0
    bad_count = 0
    pattern = re.compile(r"^\[INFO\] Running ([^ ]+) \(use_wheel=(?:true|false)\)$")

    with open(batch_log, "r", encoding="utf-8", errors="ignore") as handle:
        for raw_line in handle:
            line = raw_line.rstrip("\n")
            match = pattern.match(line)
            if match:
                if current_tag is not None:
                    sections[current_tag] = (loop_count, bad_count)
                current_tag = match.group(1)
                loop_count = 0
                bad_count = 0
                continue

            if current_tag is None:
                continue

            if "*Loop detected" in line:
                loop_count += 1
            elif "BAD LOOP!!!" in line:
                bad_count += 1

    if current_tag is not None:
        sections[current_tag] = (loop_count, bad_count)

    return sections


def parse_run_log(run_log: str) -> Tuple[int, int]:
    if not os.path.isfile(run_log):
        return 0, 0

    loop_count = 0
    bad_count = 0
    with open(run_log, "r", encoding="utf-8", errors="ignore") as handle:
        for line in handle:
            if "*Loop detected" in line:
                loop_count += 1
            elif "BAD LOOP!!!" in line:
                bad_count += 1
    return loop_count, bad_count


def parse_vec3(raw: str) -> Optional[Tuple[float, float, float]]:
    parts = raw.strip().split()
    if len(parts) != 3:
        return None
    try:
        return float(parts[0]), float(parts[1]), float(parts[2])
    except ValueError:
        return None


def vec_delta_norm(
    before: Tuple[float, float, float], after: Tuple[float, float, float]
) -> float:
    dx = after[0] - before[0]
    dy = after[1] - before[1]
    dz = after[2] - before[2]
    return (dx * dx + dy * dy + dz * dz) ** 0.5


def parse_viba_metrics(run_log: str) -> Optional[VibaMetrics]:
    if not os.path.isfile(run_log):
        return None

    begin_pattern = re.compile(
        r"\[VIBA\] begin .*scale_before=([^\s]+) .*bg_before=\[([^\]]+)\] .*ba_before=\[([^\]]+)\]"
    )
    end_pattern = re.compile(
        r"\[VIBA\] end .*scale_after=([^\s]+) .*bg_after=\[([^\]]+)\] .*ba_after=\[([^\]]+)\]"
    )

    pending_begin: Optional[Tuple[float, Tuple[float, float, float], Tuple[float, float, float]]] = None
    latest: Optional[VibaMetrics] = None

    with open(run_log, "r", encoding="utf-8", errors="ignore") as handle:
        for line in handle:
            begin_match = begin_pattern.search(line)
            if begin_match:
                bg_before = parse_vec3(begin_match.group(2))
                ba_before = parse_vec3(begin_match.group(3))
                if bg_before is None or ba_before is None:
                    pending_begin = None
                    continue
                try:
                    scale_before = float(begin_match.group(1))
                except ValueError:
                    pending_begin = None
                    continue
                pending_begin = (scale_before, bg_before, ba_before)
                continue

            end_match = end_pattern.search(line)
            if end_match and pending_begin is not None:
                bg_after = parse_vec3(end_match.group(2))
                ba_after = parse_vec3(end_match.group(3))
                if bg_after is None or ba_after is None:
                    continue
                try:
                    scale_after = float(end_match.group(1))
                except ValueError:
                    continue
                _scale_before, bg_before, ba_before = pending_begin
                latest = VibaMetrics(
                    scale_after=scale_after,
                    bg_delta_norm=vec_delta_norm(bg_before, bg_after),
                    ba_delta_norm=vec_delta_norm(ba_before, ba_after),
                )
                pending_begin = None

    return latest


def read_loop_stats(exp_dir: str, tag: str, batch_sections: Dict[str, Tuple[int, int]]) -> Tuple[int, int]:
    run_log = os.path.join(exp_dir, "run.log")
    if os.path.isfile(run_log):
        return parse_run_log(run_log)
    return batch_sections.get(tag, (0, 0))


def stat(values: Iterable[float]) -> Tuple[float, float]:
    data = list(values)
    mean = sum(data) / len(data)
    std = statistics.stdev(data) if len(data) > 1 else 0.0
    return mean, std


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze wheel on/off batch results.")
    parser.add_argument("exp_root", help="Experiment root containing wheel_on_* and wheel_off_* directories")
    parser.add_argument("bag_path", help="Rosbag path used for eval_traj_vs_wheel.py")
    parser.add_argument("--topic", default="/wheel_odom", help="Wheel topic for eval_traj_vs_wheel.py")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    exp_root = args.exp_root
    bag_path = args.bag_path

    entries = sorted(os.listdir(exp_root))
    on_dirs = [entry for entry in entries if re.match(r"wheel_on_\d+_", entry)]
    off_dirs = [entry for entry in entries if re.match(r"wheel_off_\d+_", entry)]

    def run_idx(name: str) -> Tuple[int, str]:
        match = re.match(r"wheel_(?:on|off)_(\d+)_(\d+)", name)
        if match is None:
            return 0, ""
        return int(match.group(1)), match.group(2)

    on_map = {run_idx(entry): entry for entry in on_dirs}
    off_map = {run_idx(entry): entry for entry in off_dirs}
    keys = sorted(set(on_map) & set(off_map))

    if not keys:
        print(f"[ERROR] No matched on/off pairs found in {exp_root}")
        sys.exit(1)

    batch_sections = parse_batch_sections(os.path.join(exp_root, "batch.log"))
    on_metrics: List[Metrics] = []
    off_metrics: List[Metrics] = []

    print(
        f"{'Run':<16} {'on_ratio':>9} {'on_gap':>9} {'on_loop':>8} {'on_bad':>7} "
        f"{'on_scale':>9} {'on_dbg':>9} {'on_dba':>9} "
        f"{'off_ratio':>10} {'off_gap':>9} {'off_loop':>9} {'off_bad':>8} "
        f"{'off_scale':>10} {'off_dbg':>10} {'off_dba':>10}"
    )
    print("-" * 154)

    for key in keys:
        on_tag = on_map[key]
        off_tag = off_map[key]
        on_dir = os.path.join(exp_root, on_tag)
        off_dir = os.path.join(exp_root, off_tag)
        on_traj = find_traj(on_dir)
        off_traj = find_traj(off_dir)
        if on_traj is None or off_traj is None:
            print(f"[SKIP] run {key}: missing traj file (on={on_traj}, off={off_traj})")
            continue

        output = run_eval(on_traj, "wheel_on", off_traj, "wheel_off", bag_path, args.topic)
        on_ratio, on_gap = parse_summary_line(output, "wheel_on")
        off_ratio, off_gap = parse_summary_line(output, "wheel_off")

        if on_ratio is None or on_gap is None or off_ratio is None or off_gap is None:
            print(f"[WARN] run {key}: parse failed, raw output tail below")
            print(output[-800:])
            continue

        on_loop, on_bad = read_loop_stats(on_dir, on_tag, batch_sections)
        off_loop, off_bad = read_loop_stats(off_dir, off_tag, batch_sections)
        on_viba = parse_viba_metrics(os.path.join(on_dir, "run.log"))
        off_viba = parse_viba_metrics(os.path.join(off_dir, "run.log"))

        if on_viba is None or off_viba is None:
            print(f"[WARN] run {key}: missing VIBA metrics (on={on_viba is not None}, off={off_viba is not None})")
            continue

        on_metrics.append(
            Metrics(
                on_ratio,
                on_gap,
                on_loop,
                on_bad,
                on_viba.scale_after,
                on_viba.bg_delta_norm,
                on_viba.ba_delta_norm,
            )
        )
        off_metrics.append(
            Metrics(
                off_ratio,
                off_gap,
                off_loop,
                off_bad,
                off_viba.scale_after,
                off_viba.bg_delta_norm,
                off_viba.ba_delta_norm,
            )
        )
        print(
            f"{str(key):<16} {on_ratio:>9.4f} {on_gap:>9.4f} {on_loop:>8d} {on_bad:>7d} "
            f"{on_viba.scale_after:>9.4f} {on_viba.bg_delta_norm:>9.4f} {on_viba.ba_delta_norm:>9.4f} "
            f"{off_ratio:>10.4f} {off_gap:>9.4f} {off_loop:>9d} {off_bad:>8d} "
            f"{off_viba.scale_after:>10.4f} {off_viba.bg_delta_norm:>10.4f} {off_viba.ba_delta_norm:>10.4f}"
        )

    if not on_metrics:
        print("[ERROR] No valid pairs evaluated.")
        sys.exit(1)

    print("=" * 154)
    on_ratio_mean, on_ratio_std = stat(metric.ratio for metric in on_metrics)
    on_gap_mean, on_gap_std = stat(metric.loop_gap for metric in on_metrics)
    off_ratio_mean, off_ratio_std = stat(metric.ratio for metric in off_metrics)
    off_gap_mean, off_gap_std = stat(metric.loop_gap for metric in off_metrics)

    on_loop_mean, _ = stat(metric.loop_detected for metric in on_metrics)
    on_bad_mean, _ = stat(metric.bad_loop for metric in on_metrics)
    off_loop_mean, _ = stat(metric.loop_detected for metric in off_metrics)
    off_bad_mean, _ = stat(metric.bad_loop for metric in off_metrics)
    on_scale_mean, on_scale_std = stat(metric.scale_after for metric in on_metrics)
    off_scale_mean, off_scale_std = stat(metric.scale_after for metric in off_metrics)
    on_bg_mean, on_bg_std = stat(metric.bg_delta_norm for metric in on_metrics)
    off_bg_mean, off_bg_std = stat(metric.bg_delta_norm for metric in off_metrics)
    on_ba_mean, on_ba_std = stat(metric.ba_delta_norm for metric in on_metrics)
    off_ba_mean, off_ba_std = stat(metric.ba_delta_norm for metric in off_metrics)

    print(
        f"wheel_on : ratio={on_ratio_mean:.4f}±{on_ratio_std:.4f}  "
        f"loop_gap={on_gap_mean:.4f}±{on_gap_std:.4f}  "
        f"loop_detected_mean={on_loop_mean:.2f}  bad_loop_mean={on_bad_mean:.2f}  "
        f"scale_after={on_scale_mean:.4f}±{on_scale_std:.4f}  "
        f"dbg_norm={on_bg_mean:.4f}±{on_bg_std:.4f}  "
        f"dba_norm={on_ba_mean:.4f}±{on_ba_std:.4f}  "
        f"n={len(on_metrics)}"
    )
    print(
        f"wheel_off: ratio={off_ratio_mean:.4f}±{off_ratio_std:.4f}  "
        f"loop_gap={off_gap_mean:.4f}±{off_gap_std:.4f}  "
        f"loop_detected_mean={off_loop_mean:.2f}  bad_loop_mean={off_bad_mean:.2f}  "
        f"scale_after={off_scale_mean:.4f}±{off_scale_std:.4f}  "
        f"dbg_norm={off_bg_mean:.4f}±{off_bg_std:.4f}  "
        f"dba_norm={off_ba_mean:.4f}±{off_ba_std:.4f}  "
        f"n={len(off_metrics)}"
    )
    print()
    print(f"delta ratio (on-off): {on_ratio_mean - off_ratio_mean:+.4f}")
    print(f"delta gap   (on-off): {on_gap_mean - off_gap_mean:+.4f}  (negative = on is better)")
    print(
        f"delta loop  (on-off): {on_loop_mean - off_loop_mean:+.2f}  "
        "(positive = on triggers more loop candidates)"
    )
    print(
        f"delta bad   (on-off): {on_bad_mean - off_bad_mean:+.2f}  "
        "(negative = on rejects fewer loop candidates)"
    )
    print(f"delta scale (on-off): {on_scale_mean - off_scale_mean:+.4f}")
    print(f"delta dbg   (on-off): {on_bg_mean - off_bg_mean:+.4f}  (negative = on changes bg less)")
    print(f"delta dba   (on-off): {on_ba_mean - off_ba_mean:+.4f}  (negative = on changes ba less)")


if __name__ == "__main__":
    main()
