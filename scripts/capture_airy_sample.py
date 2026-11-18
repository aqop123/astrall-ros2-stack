#!/usr/bin/env python3
"""Capture one RoboSense PointCloud2 frame and a short IMU sample."""

import argparse
import csv
import json
import math
from pathlib import Path
import struct
import time

import rclpy
from sensor_msgs.msg import Imu, PointCloud2


def write_pcd(message: PointCloud2, path: Path) -> int:
    offsets = {field.name: field.offset for field in message.fields}
    required = ("x", "y", "z", "intensity")
    if not all(name in offsets for name in required):
        raise ValueError(f"PointCloud2 lacks XYZI fields: {list(offsets)}")

    input_format = (">" if message.is_bigendian else "<") + "f"
    points = bytearray()
    for row in range(message.height):
        for col in range(message.width):
            base = row * message.row_step + col * message.point_step
            values = [
                struct.unpack_from(input_format, message.data, base + offsets[name])[0]
                for name in required
            ]
            if all(math.isfinite(value) for value in values[:3]):
                points.extend(struct.pack("<ffff", *values))

    count = len(points) // 16
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {count}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {count}\n"
        "DATA binary\n"
    )
    path.write_bytes(header.encode("ascii") + points)
    return count


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--timeout", type=float, default=12)
    parser.add_argument("--cloud-topic", default="/rslidar_points")
    parser.add_argument("--imu-topic", default="/rslidar_imu_data")
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = rclpy.create_node("airy_sample_capture")
    cloud = None
    imu_rows = []

    def on_cloud(message: PointCloud2) -> None:
        nonlocal cloud
        if cloud is None:
            cloud = message

    def on_imu(message: Imu) -> None:
        if len(imu_rows) < 20:
            imu_rows.append([
                message.header.stamp.sec,
                message.header.stamp.nanosec,
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
            ])

    node.create_subscription(PointCloud2, args.cloud_topic, on_cloud, 10)
    node.create_subscription(Imu, args.imu_topic, on_imu, 10)
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline and (cloud is None or len(imu_rows) < 20):
        rclpy.spin_once(node, timeout_sec=0.5)

    node.destroy_node()
    rclpy.shutdown()
    if cloud is None:
        raise SystemExit(f"No {args.cloud_topic} frame received")

    count = write_pcd(cloud, args.output_dir / "airy_one_frame.pcd")
    with (args.output_dir / "airy_imu.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["sec", "nanosec", "ax", "ay", "az", "gx", "gy", "gz"])
        writer.writerows(imu_rows)

    metadata = {
        "cloud_topic": args.cloud_topic,
        "imu_topic": args.imu_topic,
        "frame_id": cloud.header.frame_id,
        "pointcloud_stamp": [cloud.header.stamp.sec, cloud.header.stamp.nanosec],
        "source_width": cloud.width,
        "source_height": cloud.height,
        "finite_points_saved": count,
        "imu_samples_saved": len(imu_rows),
    }
    (args.output_dir / "airy_sample.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(json.dumps(metadata, ensure_ascii=False))


if __name__ == "__main__":
    main()
