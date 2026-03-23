#!/usr/bin/env python3
from __future__ import annotations

import argparse
import re
from typing import List


def format_matrix(values: List[float]) -> List[str]:
    fmt = lambda v: f"{v:.8f}"
    v = [fmt(x) for x in values]
    return [
        f"   data: [{v[0]}, {v[1]}, {v[2]}, {v[3]},",
        f"         {v[4]}, {v[5]}, {v[6]}, {v[7]},",
        f"         {v[8]}, {v[9]}, {v[10]}, {v[11]},",
        f"         {v[12]}, {v[13]}, {v[14]}, {v[15]}]",
    ]


def update_tbc(lines: List[str], scale: float, zero: bool) -> List[str]:
    start_idx: int = -1
    end_idx: int = -1
    buffer: List[str] = []

    for idx, line in enumerate(lines):
        if "data:" in line and start_idx == -1:
            start_idx = idx
        if start_idx != -1:
            buffer.append(line)
            if "]" in line:
                end_idx = idx
                break

    if start_idx == -1 or end_idx == -1:
        raise ValueError("Tbc data block not found")

    numbers: List[float] = []
    for buf_line in buffer:
        numbers.extend([float(x) for x in re.findall(r"[-+]?\d*\.\d+|[-+]?\d+", buf_line)])

    if len(numbers) < 16:
        raise ValueError("Tbc data block does not contain 16 values")

    values: List[float] = numbers[:16]
    if zero:
        values[3] = 0.0
        values[7] = 0.0
        values[11] = 0.0
    else:
        values[3] = values[3] * scale
        values[7] = values[7] * scale
        values[11] = values[11] * scale

    new_block: List[str] = format_matrix(values)
    return lines[:start_idx] + new_block + lines[end_idx + 1 :]


def update_imu_noise(lines: List[str], scale: float) -> List[str]:
    keys = ["IMU.NoiseGyro", "IMU.NoiseAcc", "IMU.GyroWalk", "IMU.AccWalk"]
    new_lines: List[str] = []
    for line in lines:
        replaced = False
        for key in keys:
            if line.strip().startswith(f"{key}:"):
                parts = line.split(":")
                current = float(parts[1].strip())
                updated = current * scale
                new_lines.append(f"{key}: {updated:.12f}\n")
                replaced = True
                break
        if not replaced:
            new_lines.append(line)
    return new_lines


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate IMU config variants.")
    parser.add_argument("--input", required=True, help="Base YAML path")
    parser.add_argument("--output", required=True, help="Output YAML path")
    parser.add_argument("--tbc-translation-scale", type=float, default=1.0)
    parser.add_argument("--tbc-translation-zero", action="store_true")
    parser.add_argument("--imu-noise-scale", type=float, default=1.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    with open(args.input, "r", encoding="utf-8") as file:
        lines: List[str] = file.readlines()

    lines = update_tbc(lines, args.tbc_translation_scale, args.tbc_translation_zero)
    lines = update_imu_noise(lines, args.imu_noise_scale)

    with open(args.output, "w", encoding="utf-8") as file:
        file.writelines(lines)

    print(f"Wrote config to {args.output}")


if __name__ == "__main__":
    main()
