#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class PoseSample:
    t_sec: float
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass(frozen=True)
class Segment:
    index: int
    start_idx: int
    end_idx: int
    start_t: float
    end_t: float
    duration_s: float
    sample_count: int
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    yaw_deg: float


@dataclass(frozen=True)
class RevisitError:
    segment_index: int
    start_t: float
    end_t: float
    duration_s: float
    translation_error_m: float
    rotation_error_deg: float
    yaw_error_deg: float
    x: float
    y: float
    z: float


def parse_args() -> argparse.Namespace:
    parser: argparse.ArgumentParser = argparse.ArgumentParser(
        description=(
            "Detect stationary segments from FrameTrajectory and compute revisit errors "
            "against a reference segment."
        )
    )
    parser.add_argument("--input", required=True, help="Path to FrameTrajectory.txt")
    parser.add_argument(
        "--linear-speed-th",
        type=float,
        default=0.03,
        help="Stationary linear speed threshold in m/s (default: 0.03)",
    )
    parser.add_argument(
        "--yaw-rate-th-deg",
        type=float,
        default=8.0,
        help="Stationary yaw rate threshold in deg/s (default: 8.0)",
    )
    parser.add_argument(
        "--min-stop-duration",
        type=float,
        default=2.0,
        help="Minimum stationary segment duration in seconds (default: 2.0)",
    )
    parser.add_argument(
        "--min-stop-samples",
        type=int,
        default=15,
        help="Minimum stationary segment sample count (default: 15)",
    )
    parser.add_argument(
        "--merge-gap",
        type=float,
        default=0.30,
        help="Merge adjacent stationary segments if the gap <= this value in seconds (default: 0.30)",
    )
    parser.add_argument(
        "--reference-segment-index",
        type=int,
        default=0,
        help="Reference segment index after detection (default: 0)",
    )
    parser.add_argument(
        "--revisit-radius",
        type=float,
        default=0.50,
        help="Only include segments within this distance to reference in meters (default: 0.50)",
    )
    parser.add_argument(
        "--csv",
        help="Optional path to write revisit error rows as CSV",
    )
    parser.add_argument(
        "--all-segments-csv",
        help="Optional path to write all detected stationary segments",
    )
    return parser.parse_args()


def parse_trajectory(path: Path) -> List[PoseSample]:
    samples: List[PoseSample] = []
    with path.open("r", encoding="utf-8") as file:
        for raw_line in file:
            line: str = raw_line.strip()
            if not line:
                continue
            parts: List[str] = line.split()
            if len(parts) < 8:
                continue
            values: List[float] = [float(token) for token in parts[:8]]
            samples.append(
                PoseSample(
                    t_sec=values[0],
                    x=values[1],
                    y=values[2],
                    z=values[3],
                    qx=values[4],
                    qy=values[5],
                    qz=values[6],
                    qw=values[7],
                )
            )

    if not samples:
        raise ValueError(f"No valid trajectory rows found: {path}")

    return normalize_timebase(samples)


def normalize_timebase(samples: Sequence[PoseSample]) -> List[PoseSample]:
    timestamps: List[float] = [sample.t_sec for sample in samples]
    if len(timestamps) < 2:
        return list(samples)

    max_ts: float = max(timestamps)
    diffs: List[float] = [timestamps[i] - timestamps[i - 1] for i in range(1, len(timestamps))]
    positive_diffs: List[float] = [delta for delta in diffs if delta > 0.0]
    median_dt: float = statistics.median(positive_diffs) if positive_diffs else 0.0

    scale: float = 1.0
    if max_ts > 1.0e14:
        scale = 1.0e9
    elif max_ts > 1.0e11 and median_dt > 1.0e5:
        scale = 1.0e9
    elif max_ts > 1.0e8 and 1.0e3 < median_dt < 1.0e6:
        scale = 1.0e6

    if scale == 1.0:
        return list(samples)

    normalized: List[PoseSample] = []
    for sample in samples:
        normalized.append(
            PoseSample(
                t_sec=sample.t_sec / scale,
                x=sample.x,
                y=sample.y,
                z=sample.z,
                qx=sample.qx,
                qy=sample.qy,
                qz=sample.qz,
                qw=sample.qw,
            )
        )
    return normalized


def wrap_to_pi(angle_rad: float) -> float:
    wrapped: float = (angle_rad + math.pi) % (2.0 * math.pi) - math.pi
    return wrapped


def quat_to_yaw_rad(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp: float = 2.0 * (qw * qz + qx * qy)
    cosy_cosp: float = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_dot(a: Tuple[float, float, float, float], b: Tuple[float, float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]


def normalize_quat(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    norm: float = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if norm <= 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm)


def quaternion_average(samples: Sequence[PoseSample]) -> Tuple[float, float, float, float]:
    if not samples:
        return (0.0, 0.0, 0.0, 1.0)

    ref: Tuple[float, float, float, float] = normalize_quat(
        (samples[0].qx, samples[0].qy, samples[0].qz, samples[0].qw)
    )
    sum_qx: float = 0.0
    sum_qy: float = 0.0
    sum_qz: float = 0.0
    sum_qw: float = 0.0

    for sample in samples:
        q: Tuple[float, float, float, float] = normalize_quat((sample.qx, sample.qy, sample.qz, sample.qw))
        if quat_dot(q, ref) < 0.0:
            q = (-q[0], -q[1], -q[2], -q[3])
        sum_qx += q[0]
        sum_qy += q[1]
        sum_qz += q[2]
        sum_qw += q[3]

    return normalize_quat((sum_qx, sum_qy, sum_qz, sum_qw))


def compute_stationary_mask(
    samples: Sequence[PoseSample],
    linear_speed_th: float,
    yaw_rate_th_rad: float,
) -> List[bool]:
    if len(samples) < 2:
        return [False for _ in samples]

    mask: List[bool] = [False for _ in samples]
    prev_yaw: float = quat_to_yaw_rad(samples[0].qx, samples[0].qy, samples[0].qz, samples[0].qw)

    for i in range(1, len(samples)):
        curr: PoseSample = samples[i]
        prev: PoseSample = samples[i - 1]
        dt: float = curr.t_sec - prev.t_sec
        if dt <= 1.0e-9:
            continue

        dx: float = curr.x - prev.x
        dy: float = curr.y - prev.y
        dz: float = curr.z - prev.z
        distance: float = math.sqrt(dx * dx + dy * dy + dz * dz)
        linear_speed: float = distance / dt

        yaw: float = quat_to_yaw_rad(curr.qx, curr.qy, curr.qz, curr.qw)
        yaw_rate: float = abs(wrap_to_pi(yaw - prev_yaw)) / dt
        prev_yaw = yaw

        stationary: bool = linear_speed <= linear_speed_th and yaw_rate <= yaw_rate_th_rad
        mask[i] = stationary
        mask[i - 1] = mask[i - 1] or stationary

    return mask


def mask_to_segments(mask: Sequence[bool]) -> List[Tuple[int, int]]:
    segments: List[Tuple[int, int]] = []
    start: Optional[int] = None
    for idx, value in enumerate(mask):
        if value and start is None:
            start = idx
        if (not value) and start is not None:
            segments.append((start, idx - 1))
            start = None
    if start is not None:
        segments.append((start, len(mask) - 1))
    return segments


def merge_close_segments(
    segments: Sequence[Tuple[int, int]],
    samples: Sequence[PoseSample],
    max_gap_s: float,
) -> List[Tuple[int, int]]:
    if not segments:
        return []
    merged: List[Tuple[int, int]] = [segments[0]]
    for curr_start, curr_end in segments[1:]:
        prev_start, prev_end = merged[-1]
        gap_s: float = samples[curr_start].t_sec - samples[prev_end].t_sec
        if gap_s <= max_gap_s:
            merged[-1] = (prev_start, curr_end)
        else:
            merged.append((curr_start, curr_end))
    return merged


def build_segment(index: int, start_idx: int, end_idx: int, samples: Sequence[PoseSample]) -> Segment:
    chunk: List[PoseSample] = list(samples[start_idx : end_idx + 1])
    xs: List[float] = [sample.x for sample in chunk]
    ys: List[float] = [sample.y for sample in chunk]
    zs: List[float] = [sample.z for sample in chunk]
    qx, qy, qz, qw = quaternion_average(chunk)
    yaw_rad: float = quat_to_yaw_rad(qx, qy, qz, qw)

    start_t: float = samples[start_idx].t_sec
    end_t: float = samples[end_idx].t_sec
    duration_s: float = max(0.0, end_t - start_t)

    return Segment(
        index=index,
        start_idx=start_idx,
        end_idx=end_idx,
        start_t=start_t,
        end_t=end_t,
        duration_s=duration_s,
        sample_count=(end_idx - start_idx + 1),
        x=statistics.median(xs),
        y=statistics.median(ys),
        z=statistics.median(zs),
        qx=qx,
        qy=qy,
        qz=qz,
        qw=qw,
        yaw_deg=math.degrees(yaw_rad),
    )


def detect_stationary_segments(
    samples: Sequence[PoseSample],
    linear_speed_th: float,
    yaw_rate_th_deg: float,
    min_stop_duration: float,
    min_stop_samples: int,
    merge_gap_s: float,
) -> List[Segment]:
    yaw_rate_th_rad: float = math.radians(yaw_rate_th_deg)
    mask: List[bool] = compute_stationary_mask(samples, linear_speed_th, yaw_rate_th_rad)
    coarse_segments: List[Tuple[int, int]] = mask_to_segments(mask)
    merged_segments: List[Tuple[int, int]] = merge_close_segments(coarse_segments, samples, merge_gap_s)

    segments: List[Segment] = []
    for start_idx, end_idx in merged_segments:
        sample_count: int = end_idx - start_idx + 1
        duration_s: float = max(0.0, samples[end_idx].t_sec - samples[start_idx].t_sec)
        if sample_count < min_stop_samples:
            continue
        if duration_s < min_stop_duration:
            continue
        segments.append(build_segment(len(segments), start_idx, end_idx, samples))

    return segments


def quaternion_angle_deg(
    qa: Tuple[float, float, float, float],
    qb: Tuple[float, float, float, float],
) -> float:
    qa_n: Tuple[float, float, float, float] = normalize_quat(qa)
    qb_n: Tuple[float, float, float, float] = normalize_quat(qb)
    dot: float = abs(quat_dot(qa_n, qb_n))
    dot_clamped: float = min(1.0, max(-1.0, dot))
    angle_rad: float = 2.0 * math.acos(dot_clamped)
    return math.degrees(angle_rad)


def build_revisit_errors(
    segments: Sequence[Segment],
    ref_index: int,
    revisit_radius_m: float,
) -> List[RevisitError]:
    if not segments:
        return []
    if ref_index < 0 or ref_index >= len(segments):
        raise IndexError(f"reference-segment-index out of range: {ref_index} (detected {len(segments)})")

    reference: Segment = segments[ref_index]
    ref_q: Tuple[float, float, float, float] = (reference.qx, reference.qy, reference.qz, reference.qw)

    revisits: List[RevisitError] = []
    for segment in segments:
        dx: float = segment.x - reference.x
        dy: float = segment.y - reference.y
        dz: float = segment.z - reference.z
        translation_error_m: float = math.sqrt(dx * dx + dy * dy + dz * dz)
        if translation_error_m > revisit_radius_m:
            continue

        seg_q: Tuple[float, float, float, float] = (segment.qx, segment.qy, segment.qz, segment.qw)
        rotation_error_deg: float = quaternion_angle_deg(ref_q, seg_q)
        yaw_error_deg: float = math.degrees(
            abs(wrap_to_pi(math.radians(segment.yaw_deg - reference.yaw_deg)))
        )

        revisits.append(
            RevisitError(
                segment_index=segment.index,
                start_t=segment.start_t,
                end_t=segment.end_t,
                duration_s=segment.duration_s,
                translation_error_m=translation_error_m,
                rotation_error_deg=rotation_error_deg,
                yaw_error_deg=yaw_error_deg,
                x=segment.x,
                y=segment.y,
                z=segment.z,
            )
        )
    return revisits


def mean_or_nan(values: Sequence[float]) -> float:
    return statistics.mean(values) if values else float("nan")


def std_or_nan(values: Sequence[float]) -> float:
    if len(values) < 2:
        return 0.0 if len(values) == 1 else float("nan")
    return statistics.pstdev(values)


def write_revisit_csv(path: Path, rows: Sequence[RevisitError]) -> None:
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(
            [
                "segment_index",
                "start_t",
                "end_t",
                "duration_s",
                "translation_error_m",
                "rotation_error_deg",
                "yaw_error_deg",
                "x",
                "y",
                "z",
            ]
        )
        for row in rows:
            writer.writerow(
                [
                    row.segment_index,
                    f"{row.start_t:.6f}",
                    f"{row.end_t:.6f}",
                    f"{row.duration_s:.3f}",
                    f"{row.translation_error_m:.6f}",
                    f"{row.rotation_error_deg:.6f}",
                    f"{row.yaw_error_deg:.6f}",
                    f"{row.x:.6f}",
                    f"{row.y:.6f}",
                    f"{row.z:.6f}",
                ]
            )


def write_all_segments_csv(path: Path, rows: Sequence[Segment]) -> None:
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(
            [
                "segment_index",
                "start_idx",
                "end_idx",
                "start_t",
                "end_t",
                "duration_s",
                "sample_count",
                "x",
                "y",
                "z",
                "yaw_deg",
            ]
        )
        for row in rows:
            writer.writerow(
                [
                    row.index,
                    row.start_idx,
                    row.end_idx,
                    f"{row.start_t:.6f}",
                    f"{row.end_t:.6f}",
                    f"{row.duration_s:.3f}",
                    row.sample_count,
                    f"{row.x:.6f}",
                    f"{row.y:.6f}",
                    f"{row.z:.6f}",
                    f"{row.yaw_deg:.6f}",
                ]
            )


def print_segment_table(segments: Sequence[Segment]) -> None:
    print("Detected stationary segments:")
    print("idx  start_t      end_t        dur_s  count   x         y         z         yaw_deg")
    for segment in segments:
        print(
            f"{segment.index:<4d}"
            f"{segment.start_t:>11.3f}  "
            f"{segment.end_t:>11.3f}  "
            f"{segment.duration_s:>6.2f}  "
            f"{segment.sample_count:>5d}  "
            f"{segment.x:>8.3f}  "
            f"{segment.y:>8.3f}  "
            f"{segment.z:>8.3f}  "
            f"{segment.yaw_deg:>8.2f}"
        )


def print_revisit_table(rows: Sequence[RevisitError], ref_index: int) -> None:
    print("")
    print(f"Revisit errors within radius (reference segment index: {ref_index}):")
    print("seg  start_t      end_t        dur_s  trans_m   rot_deg   yaw_deg")
    for row in rows:
        print(
            f"{row.segment_index:<4d}"
            f"{row.start_t:>11.3f}  "
            f"{row.end_t:>11.3f}  "
            f"{row.duration_s:>6.2f}  "
            f"{row.translation_error_m:>7.4f}  "
            f"{row.rotation_error_deg:>7.3f}  "
            f"{row.yaw_error_deg:>7.3f}"
        )


def print_summary(rows: Sequence[RevisitError]) -> None:
    trans_errors: List[float] = [row.translation_error_m for row in rows]
    rot_errors: List[float] = [row.rotation_error_deg for row in rows]
    yaw_errors: List[float] = [row.yaw_error_deg for row in rows]

    print("")
    print("Summary:")
    print(
        "translation_error_m: "
        f"mean={mean_or_nan(trans_errors):.5f}, "
        f"std={std_or_nan(trans_errors):.5f}, "
        f"max={max(trans_errors) if trans_errors else float('nan'):.5f}"
    )
    print(
        "rotation_error_deg:  "
        f"mean={mean_or_nan(rot_errors):.5f}, "
        f"std={std_or_nan(rot_errors):.5f}, "
        f"max={max(rot_errors) if rot_errors else float('nan'):.5f}"
    )
    print(
        "yaw_error_deg:       "
        f"mean={mean_or_nan(yaw_errors):.5f}, "
        f"std={std_or_nan(yaw_errors):.5f}, "
        f"max={max(yaw_errors) if yaw_errors else float('nan'):.5f}"
    )


def main() -> None:
    args: argparse.Namespace = parse_args()
    input_path: Path = Path(args.input)
    if not input_path.exists():
        raise FileNotFoundError(f"Input file not found: {input_path}")

    samples: List[PoseSample] = parse_trajectory(input_path)
    segments: List[Segment] = detect_stationary_segments(
        samples=samples,
        linear_speed_th=args.linear_speed_th,
        yaw_rate_th_deg=args.yaw_rate_th_deg,
        min_stop_duration=args.min_stop_duration,
        min_stop_samples=args.min_stop_samples,
        merge_gap_s=args.merge_gap,
    )

    if not segments:
        print("No stationary segments were detected. Try relaxing thresholds.")
        return

    print_segment_table(segments)
    revisits: List[RevisitError] = build_revisit_errors(
        segments=segments,
        ref_index=args.reference_segment_index,
        revisit_radius_m=args.revisit_radius,
    )

    if not revisits:
        print("")
        print("No revisit segment found within the given radius.")
        return

    print_revisit_table(revisits, args.reference_segment_index)
    print_summary(revisits)

    if args.csv:
        output_csv: Path = Path(args.csv)
        write_revisit_csv(output_csv, revisits)
        print(f"\nSaved revisit CSV: {output_csv}")

    if args.all_segments_csv:
        all_segments_csv: Path = Path(args.all_segments_csv)
        write_all_segments_csv(all_segments_csv, segments)
        print(f"Saved segment CSV: {all_segments_csv}")


if __name__ == "__main__":
    main()
