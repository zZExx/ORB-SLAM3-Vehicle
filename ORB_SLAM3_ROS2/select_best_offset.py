#!/usr/bin/env python3
"""Parse sweep summary CSV and select the offset with minimal displacement_m."""

from __future__ import annotations

import argparse
import csv
import re
import sys


def parse_label_offset(label: str) -> float | None:
    """Extract numeric offset from label like 'offset_0.05'."""
    match = re.search(r"offset_([\d.]+)", label)
    return float(match.group(1)) if match else None


def select_best(
    csv_path: str,
    min_count: int = 20,
    min_duration_s: float = 10.0,
    fallback: bool = True,
) -> tuple[float, float, int, bool] | None:
    """
    Read summary CSV and return (best_offset, displacement_m, count, used_fallback) or None.
    Filters out runs with count < min_count or duration_s < min_duration_s.
    If fallback and no run passes filter, return least-bad (min displacement) with warning.
    """
    best_valid: tuple[float, float, int] | None = None
    best_any: tuple[float, float, int] | None = None

    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            label = row.get("label", "")
            offset = parse_label_offset(label)
            if offset is None:
                continue
            try:
                count = int(row["count"])
                duration_s = float(row["duration_s"])
                displacement_m = float(row["displacement_m"])
            except (KeyError, ValueError):
                continue
            if best_any is None or displacement_m < best_any[1]:
                best_any = (offset, displacement_m, count)
            if count >= min_count and duration_s >= min_duration_s:
                if best_valid is None or displacement_m < best_valid[1]:
                    best_valid = (offset, displacement_m, count)

    if best_valid is not None:
        return (*best_valid, False)
    if fallback and best_any is not None:
        return (*best_any, True)
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description="Select best camera_time_offset from sweep summary.")
    parser.add_argument("--input", "-i", required=True, help="Path to summary.csv")
    parser.add_argument("--min-count", type=int, default=20, help="Min keyframes to consider valid")
    parser.add_argument("--min-duration", type=float, default=10.0, help="Min duration (s) to consider valid")
    parser.add_argument("--no-fallback", action="store_true", help="Do not fall back to least-bad when no valid run")
    args = parser.parse_args()

    result = select_best(
        args.input, args.min_count, args.min_duration, fallback=not args.no_fallback
    )
    if result is None:
        print("No valid run found in summary", file=sys.stderr)
        return 1
    offset, displacement, count, used_fallback = result
    if used_fallback:
        print(
            "Warning: no run passed filters (min_count=%d, min_duration=%.1fs). "
            "Using least-bad displacement."
            % (args.min_count, args.min_duration),
            file=sys.stderr,
        )
    print(f"Best offset: {offset:.4f}, displacement={displacement:.6f} m, keyframes={count}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
