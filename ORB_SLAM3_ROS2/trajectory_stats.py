#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import os
from dataclasses import dataclass
from typing import List, Tuple


@dataclass(frozen=True)
class TrajectoryStats:
    label: str
    count: int
    start_time: float
    end_time: float
    duration_s: float
    start_xyz: Tuple[float, float, float]
    end_xyz: Tuple[float, float, float]
    displacement_m: float


def parse_trajectory_lines(lines: List[str]) -> List[List[float]]:
    values: List[List[float]] = []
    for line in lines:
        stripped: str = line.strip()
        if not stripped:
            continue
        parts: List[str] = stripped.split()
        if len(parts) < 8:
            continue
        values.append([float(p) for p in parts[:8]])
    return values


def compute_stats(label: str, path: str) -> TrajectoryStats:
    with open(path, "r", encoding="utf-8") as file:
        values: List[List[float]] = parse_trajectory_lines(file.readlines())

    if not values:
        raise ValueError(f"Empty trajectory: {path}")

    start: List[float] = values[0]
    end: List[float] = values[-1]

    start_time: float = start[0]
    end_time: float = end[0]
    duration_s: float = end_time - start_time

    start_xyz: Tuple[float, float, float] = (start[1], start[2], start[3])
    end_xyz: Tuple[float, float, float] = (end[1], end[2], end[3])
    dx: float = end_xyz[0] - start_xyz[0]
    dy: float = end_xyz[1] - start_xyz[1]
    dz: float = end_xyz[2] - start_xyz[2]
    displacement_m: float = math.sqrt(dx * dx + dy * dy + dz * dz)

    return TrajectoryStats(
        label=label,
        count=len(values),
        start_time=start_time,
        end_time=end_time,
        duration_s=duration_s,
        start_xyz=start_xyz,
        end_xyz=end_xyz,
        displacement_m=displacement_m,
    )


def format_csv_row(stats: TrajectoryStats) -> str:
    start_xyz: str = f"{stats.start_xyz[0]:.7f} {stats.start_xyz[1]:.7f} {stats.start_xyz[2]:.7f}"
    end_xyz: str = f"{stats.end_xyz[0]:.7f} {stats.end_xyz[1]:.7f} {stats.end_xyz[2]:.7f}"
    return (
        f"{stats.label},"
        f"{stats.count},"
        f"{stats.start_time:.6f},"
        f"{stats.end_time:.6f},"
        f"{stats.duration_s:.6f},"
        f"\"{start_xyz}\","
        f"\"{end_xyz}\","
        f"{stats.displacement_m:.6f}"
    )


def write_csv(path: str, stats: TrajectoryStats, append: bool) -> None:
    header: str = (
        "label,count,start_time,end_time,duration_s,start_xyz,end_xyz,displacement_m"
    )
    write_header: bool = not append or not os.path.exists(path)
    mode: str = "a" if append else "w"

    with open(path, mode, encoding="utf-8") as file:
        if write_header:
            file.write(header + "\n")
        file.write(format_csv_row(stats) + "\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compute trajectory summary stats.")
    parser.add_argument("--input", required=True, help="Path to KeyFrameTrajectory.txt")
    parser.add_argument("--label", required=True, help="Label for this run")
    parser.add_argument("--output", help="Optional CSV output path")
    parser.add_argument("--append", action="store_true", help="Append to CSV output")
    return parser.parse_args()


def main() -> None:
    args: argparse.Namespace = parse_args()
    stats: TrajectoryStats = compute_stats(args.label, args.input)

    if args.output:
        write_csv(args.output, stats, args.append)
        print(f"Wrote stats to {args.output}")
    else:
        print(format_csv_row(stats))


if __name__ == "__main__":
    main()
