#!/usr/bin/env python3
"""点云点数统计：统计 /merged_points（或指定话题）每帧点数与空帧。

用法：
    python3 scripts/slam_tests/pointcloud_stats.py --duration 60 --out /tmp/points.csv
"""

import argparse
import csv
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import PointCloud2


class PointCounter(Node):
    def __init__(self, topic):
        super().__init__('slam_pointcloud_stats')
        self.counts = []
        self.stamps = []
        self.empty_frames = 0
        self.sub = self.create_subscription(PointCloud2, topic, self.on_cloud,
                                            qos_profile_system_default)

    def on_cloud(self, msg):
        n = msg.width * msg.height
        self.counts.append(n)
        self.stamps.append(time.time())
        if n == 0:
            self.empty_frames += 1


def main():
    parser = argparse.ArgumentParser(description='点云点数统计')
    parser.add_argument('--topic', default='/merged_points')
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--out', default=None)
    args = parser.parse_args()

    rclpy.init()
    node = PointCounter(args.topic)
    end = time.time() + args.duration
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

    counts = node.counts
    print(f'--- point cloud stats on {args.topic} ---')
    if not counts:
        print('FAIL: no point cloud received')
    else:
        hz = len(counts) / (node.stamps[-1] - node.stamps[0]) if len(counts) > 1 else 0.0
        print(f'frames={len(counts)} empty={node.empty_frames} hz={hz:.2f}')
        print(f'points/frame: min={min(counts)} max={max(counts)} '
              f'mean={statistics.mean(counts):.0f} std={statistics.pstdev(counts):.0f}')
    if args.out:
        with open(args.out, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['wall_sec', 'points'])
            for t, n in zip(node.stamps, counts):
                writer.writerow([f'{t:.6f}', n])
        print(f'written to {args.out}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
