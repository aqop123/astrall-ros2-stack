#!/usr/bin/env python3
"""通用话题频率统计：记录每个话题的到达时刻，输出实际频率与抖动。

用法（需先 source install/setup.bash）：
    python3 scripts/slam_tests/topic_hz_collector.py --duration 60
    python3 scripts/slam_tests/topic_hz_collector.py --topics /imu/data /merged_points --out /tmp/hz.csv

默认话题覆盖 FAST-LIO2 前端测试的关键链路；未知话题会跳过并提示。
"""

import argparse
import csv
import os
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry

try:
    from interface.msg import LioDiagnostics
except ImportError:
    LioDiagnostics = None

TOPIC_TYPES = {
    '/imu/data': (Imu, qos_profile_sensor_data),
    '/merged_points': (PointCloud2, qos_profile_system_default),
    '/rslidar_points': (PointCloud2, qos_profile_system_default),
    '/rslidar_rear_points': (PointCloud2, qos_profile_system_default),
    '/fastlio2/lio_odom': (Odometry, qos_profile_system_default),
    '/fastlio2/lio_diagnostics': (LioDiagnostics, qos_profile_system_default)
    if LioDiagnostics else (None, None),
    # 直接用 ros2 run 启动 lio_node（无命名空间）时的等价话题
    '/lio_odom': (Odometry, qos_profile_system_default),
    '/lio_diagnostics': (LioDiagnostics, qos_profile_system_default)
    if LioDiagnostics else (None, None),
}


class HzCollector(Node):
    def __init__(self, topics):
        super().__init__('slam_topic_hz_collector')
        self.samples = {t: [] for t in topics}
        self.subs = []
        for topic in topics:
            entry = TOPIC_TYPES.get(topic)
            if not entry or entry[0] is None:
                self.get_logger().warning(f'Unknown topic or message type unavailable: {topic}')
                continue
            msg_type, qos = entry
            self.subs.append(self.create_subscription(msg_type, topic,
                                                      lambda msg, t=topic: self.on_msg(t), qos))

    def on_msg(self, topic):
        self.samples[topic].append(time.time())


def summarize(topic, stamps):
    if len(stamps) < 2:
        return f'{topic}: samples={len(stamps)} (not enough for frequency)'
    dts = [b - a for a, b in zip(stamps[:-1], stamps[1:])]
    mean_hz = (len(stamps) - 1) / (stamps[-1] - stamps[0])
    return (f'{topic}: n={len(stamps)} mean={mean_hz:.2f} Hz '
            f'inst=[{1 / max(dts):.2f}, {1 / min(dts):.2f}] Hz '
            f'dt_mean={statistics.mean(dts) * 1000:.1f} ms '
            f'dt_std={statistics.pstdev(dts) * 1000:.1f} ms')


def main():
    parser = argparse.ArgumentParser(description='FAST-LIO2 话题频率统计')
    parser.add_argument('--topics', nargs='+', default=list(TOPIC_TYPES.keys()))
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--out', default=None, help='可选：把原始时间戳写入 CSV')
    args = parser.parse_args()

    rclpy.init()
    node = HzCollector(args.topics)
    end = time.time() + args.duration
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f'--- topic hz summary (duration {args.duration:.1f}s) ---')
    for topic in args.topics:
        print(summarize(topic, node.samples[topic]))
    if args.out:
        with open(args.out, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['topic', 'arrival_wall_sec'])
            for topic, stamps in node.samples.items():
                for s in stamps:
                    writer.writerow([topic, f'{s:.6f}'])
        print(f'raw timestamps written to {os.path.abspath(args.out)}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
