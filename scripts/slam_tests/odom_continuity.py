#!/usr/bin/env python3
"""LIO 里程计连续性统计：时间戳倒退、静默、相邻位姿跳变。

注意：跳变只是诊断信号，不能据此直接判定定位失效（急加速/旋转也会产生跳变）。
"""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from nav_msgs.msg import Odometry


class OdomCheck(Node):
    def __init__(self, topic):
        super().__init__('slam_odom_continuity')
        self.records = []
        self.sub = self.create_subscription(Odometry, topic, self.on_odom,
                                            qos_profile_system_default)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.records.append((stamp, time.time(), (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)))


def angle_deg(q1, q2):
    dot = abs(sum(a * b for a, b in zip(q1, q2)))
    return 2 * math.degrees(math.acos(min(1.0, dot)))


def main():
    parser = argparse.ArgumentParser(description='LIO 里程计连续性统计')
    parser.add_argument('--topic', default='/fastlio2/lio_odom')
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--trans-thresh', type=float, default=0.5, help='相邻位置变化告警阈值 (m)')
    parser.add_argument('--rot-thresh-deg', type=float, default=10.0, help='相邻角度变化告警阈值 (deg)')
    args = parser.parse_args()

    rclpy.init()
    node = OdomCheck(args.topic)
    end = time.time() + args.duration
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

    rec = node.records
    print(f'--- odom continuity on {args.topic} ---')
    if len(rec) < 2:
        print('FAIL: fewer than 2 odom messages received')
    else:
        rollbacks = 0
        jumps_t, jumps_r = [], []
        gaps = []
        for (s0, r0, p0, q0), (s1, r1, p1, q1) in zip(rec[:-1], rec[1:]):
            if s1 < s0 - 0.001:
                rollbacks += 1
            d = math.dist(p0, p1)
            a = angle_deg(q0, q1)
            if d > args.trans_thresh:
                jumps_t.append((s1, d))
            if a > args.rot_thresh_deg:
                jumps_r.append((s1, a))
            gaps.append(r1 - r0)
        hz = (len(rec) - 1) / (rec[-1][1] - rec[0][1])
        print(f'frames={len(rec)} hz={hz:.2f} timestamp_rollback={rollbacks}')
        print(f'max_silence={max(gaps) * 1000:.1f} ms, mean_dt={sum(gaps) / len(gaps) * 1000:.1f} ms')
        print(f'trans jumps >{args.trans_thresh} m: {len(jumps_t)} '
              f'(max={max([d for _, d in jumps_t], default=0):.3f} m)')
        print(f'rot jumps >{args.rot_thresh_deg} deg: {len(jumps_r)} '
              f'(max={max([a for _, a in jumps_r], default=0):.2f} deg)')
        print(f'start_pos={rec[0][2]} end_pos={rec[-1][2]}')
        if jumps_t or jumps_r or rollbacks:
            print('WARN: 以上跳变/倒退仅为诊断信号，请结合录包与实机运动状态分析')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
