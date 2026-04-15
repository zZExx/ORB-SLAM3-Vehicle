#!/usr/bin/env python3
"""
Compute traveled distance from ROS2 bag wheel odometry.

The script integrates absolute wheel speed (twist.linear.x) over time:
    distance = sum(|v_i| * dt_i)
"""

from __future__ import annotations

import argparse
import math
import sys
from typing import Optional


def _build_arg_parser() -> argparse.ArgumentParser:
    parser: argparse.ArgumentParser = argparse.ArgumentParser(
        description="Read ROS2 bag and compute total wheel distance."
    )
    parser.add_argument(
        "bag_path",
        help="Path to ros2 bag directory (the folder containing metadata.yaml).",
    )
    parser.add_argument(
        "--topic",
        default="/wheel/odometry",
        help="Wheel odometry topic (nav_msgs/msg/Odometry).",
    )
    parser.add_argument(
        "--time-offset",
        type=float,
        default=0.0,
        help="Time offset (seconds) added to each wheel timestamp.",
    )
    parser.add_argument(
        "--signed",
        action="store_true",
        help="Integrate signed velocity instead of absolute velocity.",
    )
    return parser


def compute_distance(
    bag_path: str,
    topic_name: str,
    time_offset_sec: float,
    integrate_signed: bool,
) -> float:
    try:
        import rosbag2_py
        from nav_msgs.msg import Odometry
        from rclpy.serialization import deserialize_message
    except ImportError as exc:
        raise RuntimeError(
            "Missing ROS2 Python modules. Source your ROS2 environment first."
        ) from exc

    storage_options: rosbag2_py.StorageOptions = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id="sqlite3",
    )
    converter_options: rosbag2_py.ConverterOptions = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader: rosbag2_py.SequentialReader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    filter_obj: rosbag2_py.StorageFilter = rosbag2_py.StorageFilter(topics=[topic_name])
    reader.set_filter(filter_obj)

    last_t_sec: Optional[float] = None
    total_distance_m: float = 0.0
    msg_count: int = 0

    while reader.has_next():
        topic: str
        data: bytes
        bag_timestamp_ns: int
        topic, data, bag_timestamp_ns = reader.read_next()
        if topic != topic_name:
            continue

        odom_msg: Odometry = deserialize_message(data, Odometry)

        sec: int = int(odom_msg.header.stamp.sec)
        nanosec: int = int(odom_msg.header.stamp.nanosec)
        msg_t_sec: float = float(sec) + float(nanosec) * 1e-9 + time_offset_sec

        velocity_mps: float = float(odom_msg.twist.twist.linear.x)
        if not integrate_signed:
            velocity_mps = math.fabs(velocity_mps)

        if last_t_sec is not None:
            dt_sec: float = msg_t_sec - last_t_sec
            if dt_sec > 0.0:
                total_distance_m += velocity_mps * dt_sec

        last_t_sec = msg_t_sec
        msg_count += 1

    if msg_count == 0:
        raise RuntimeError(f"No messages found on topic: {topic_name}")

    return total_distance_m


def main() -> int:
    parser: argparse.ArgumentParser = _build_arg_parser()
    args: argparse.Namespace = parser.parse_args()

    try:
        total_distance_m: float = compute_distance(
            bag_path=args.bag_path,
            topic_name=args.topic,
            time_offset_sec=args.time_offset,
            integrate_signed=args.signed,
        )
    except Exception as exc:  # pylint: disable=broad-except
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    print(f"Total distance (m): {total_distance_m:.6f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
