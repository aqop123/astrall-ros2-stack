#!/usr/bin/env python3
"""IMU 数据频率与数值统计：频率、加速度/角速度模长、时间戳跳变。

静止时 |acc| 应接近 9.81 / imu_acc_scale（当前配置为 9.81 m/s²），
|gyro| 应接近 0；二者同时偏离说明 IMU 单位、朝向或安装状态异常。
"""

import argparse
import csv
import math
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuStats(Node):
    def __init__(self, topic):
        super().__init__('slam_imu_stats')
        self.recv = []
        self.stamps = []
        self.acc_norm = []
        self.gyro_norm = []
        self.sub = self.create_subscription(Imu, topic, self.on_imu, qos_profile_sensor_data)

    def on_imu(self, msg):
        t = time.time()
        self.recv.append(t)
        self.stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        a = msg.linear_acceleration
        g = msg.angular_velocity
        self.acc_norm.append(math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z))
        self.gyro_norm.append(math.sqrt(g.x * g.x + g.y * g.y + g.z * g.z))


def report(name, values, unit=''):
    if not values:
        return f'{name}: no data'
    return (f'{name}: min={min(values):.4f} mean={statistics.mean(values):.4f} '
            f'max={max(values):.4f} std={statistics.pstdev(values):.4f} {unit}')


def main():
    parser = argparse.ArgumentParser(description='IMU 频率与数值统计')
    parser.add_argument('--topic', default='/imu/data')
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--out', default=None)
    args = parser.parse_args()

    rclpy.init()
    node = ImuStats(args.topic)
    end = time.time() + args.duration
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

    print(f'--- IMU stats on {args.topic} (duration {args.duration:.1f}s) ---')
    if not node.recv:
        print('FAIL: no IMU received (check QoS: subscriber must use SensorDataQoS)')
    else:
        hz = len(node.recv) / (node.recv[-1] - node.recv[0]) if len(node.recv) > 1 else 0.0
        print(f'frames={len(node.recv)} hz={hz:.2f}')
        print(report('|acc|', node.acc_norm, 'm/s^2'))
        print(report('|gyro|', node.gyro_norm, 'rad/s'))
        dts = [b - a for a, b in zip(node.stamps[:-1], node.stamps[1:])]
        rollback = sum(1 for dt in dts if dt < -0.001)
        gaps = sum(1 for dt in dts if dt > 0.05)
        print(f'timestamp rollback={rollback} gaps>50ms={gaps} dt_mean={statistics.mean(dts) * 1000:.2f} ms')
    if args.out:
        with open(args.out, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['wall_sec', 'stamp_sec', 'acc_norm', 'gyro_norm'])
            for r, s, a, g in zip(node.recv, node.stamps, node.acc_norm, node.gyro_norm):
                writer.writerow([f'{r:.6f}', f'{s:.6f}', f'{a:.6f}', f'{g:.6f}'])
        print(f'written to {args.out}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
