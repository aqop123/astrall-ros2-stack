#!/usr/bin/env python3
# 端到端收敛测试: 模拟 astrall_sensor_node(250Hz IMU, best-effort, acc 单位 m/s²)
#   + 双雷达静态"房间"点云 -> dual_lidar_fusion_node(/merged_points) -> fastlio2 lio_node
#
# 相比旧版(仅 4+2 点, 触发 "NO Effective Points!" 导致 IESKF 无法更新), 本版:
#   ① 前雷达发布稠密平面点云(地面 + 4 面墙, ~2200 点/帧), near_search_num=5 有足够共面近邻做 esti_plane
#   ② 订阅 /lio_odom, 验证静止工况下 z 收敛在 0 附近 (imu_acc_scale=1.0);
#      若 scale 仍为 10.0, 静止 acc=98.1 而内部 g 固定 9.81 -> z 发散到上万, 测试失败
#
# 运行(需先 source install/setup.bash 并启动 fusion_node + lio_node):
#   python3 scripts/e2e_lio_test.py [duration_sec]
# 注意: 节点刚启动后 DDS 需数秒完成 pub/sub 匹配, 首跑可能收不到 odom, 直接重跑即可。
import os
import struct
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from sensor_msgs.msg import Imu, PointCloud2, PointField
from nav_msgs.msg import Odometry

F32, U16, F64 = PointField.FLOAT32, PointField.UINT16, PointField.FLOAT64
STEP = 26
FRAME_PERIOD = 0.1          # 10Hz 点云
PHASE_DIFF = 0.0015         # 前后雷达相位差 < sync_slop 30ms
IMU_HZ = 250.0
WARMUP_SEC = 2.5            # 跳过 IMU 初始化 + 地图初始化阶段
Z_LIMIT = 3.0               # 静止工况 |z| 允许上限(m)

T0 = int(time.time()) + 1.0  # 1.7e9 量级绝对秒, 与 IMU time.time() 同基准


def make_xyzirt(stamp, points):
    msg = PointCloud2()
    msg.header.frame_id = 'rslidar'
    msg.header.stamp.sec = int(stamp)
    msg.header.stamp.nanosec = int(round((stamp - int(stamp)) * 1e9))
    msg.fields = [
        PointField(name='x', offset=0, datatype=F32, count=1),
        PointField(name='y', offset=4, datatype=F32, count=1),
        PointField(name='z', offset=8, datatype=F32, count=1),
        PointField(name='intensity', offset=12, datatype=F32, count=1),
        PointField(name='ring', offset=16, datatype=U16, count=1),
        PointField(name='timestamp', offset=18, datatype=F64, count=1),
    ]
    buf = bytearray()
    for (x, y, z, inten, ring, t) in points:
        buf += struct.pack('<fff', x, y, z)
        buf += struct.pack('<fH', inten, ring)
        buf += struct.pack('<d', t)
    msg.height, msg.width = 1, len(points)
    msg.is_bigendian = False
    msg.point_step, msg.row_step = STEP, STEP * len(points)
    msg.data = bytes(buf)
    msg.is_dense = True
    return msg


def dense_room(fs):
    """静态房间: 地面 z=-1 + 四面墙, 0.5m 网格 (~2200 点)。
    全部落在自车包围盒外、min/max_range [0.4,30] 内, 且每个点近邻共面 -> 可稳定 esti_plane。"""
    pts = []
    step = 0.5
    n = 16  # -8..8

    def add(x, y, z, ring, dt):
        pts.append((float(x), float(y), float(z), 1.0, ring, fs + dt))

    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            add(i * step, j * step, -1.0, 1, 0.0001)        # 地面
    for i in range(-n, n + 1):
        for k in range(0, 9):
            z = -1.0 + k * 0.5
            add(-8.0, i * step, z, 2, 0.0002)               # 墙 x=-8
            add(8.0, i * step, z, 3, 0.0003)                # 墙 x=+8
            add(i * step, -8.0, z, 4, 0.0004)               # 墙 y=-8
            add(i * step, 8.0, z, 5, 0.0005)                # 墙 y=+8
    return pts


def rear_probe(fs):
    """后雷达少量点, 仅用于维持 ApproximateTime 配对(避免 2s 后降级), 不贡献有效特征。"""
    return [(8.02, 0.0, 0.0, 1.5, 1, fs + PHASE_DIFF + 0.0006),
            (8.02, 0.5, 0.0, 1.5, 1, fs + PHASE_DIFF + 0.0007)]


class Driver(Node):
    def __init__(self):
        super().__init__('e2e_driver')
        self.imu_pub = self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
        self.front_pub = self.create_publisher(PointCloud2, '/rslidar_points', qos_profile_system_default)
        self.rear_pub = self.create_publisher(PointCloud2, '/rslidar_rear_points', qos_profile_system_default)
        self.z_samples = []       # (t_rel, z)
        self.odom_count = 0
        self.sub = self.create_subscription(Odometry, '/lio_odom', self.on_odom, qos_profile_system_default)
        self.k = 0
        self.t_start = time.time()
        self.room = dense_room(T0)

    def on_odom(self, msg):
        self.odom_count += 1
        self.z_samples.append((time.time() - self.t_start, msg.pose.pose.position.z))

    def tick(self):
        now = time.time()
        imu = Imu()
        imu.header.stamp.sec = int(now)
        imu.header.stamp.nanosec = int((now - int(now)) * 1e9)
        imu.header.frame_id = 'imu_link'
        imu.angular_velocity.x = 0.0
        imu.angular_velocity.y = 0.0
        imu.angular_velocity.z = 0.0
        imu.linear_acceleration.x = 0.0
        imu.linear_acceleration.y = 0.0
        imu.linear_acceleration.z = 9.81  # m/s², 与 SDK AstrallImuData 单位一致
        self.imu_pub.publish(imu)          # ~250Hz best-effort
        if self.k % 25 == 0:               # 10Hz 点云
            fs = T0 + (self.k // 25 + 1) * FRAME_PERIOD
            self.front_pub.publish(make_xyzirt(fs, self.room))
            self.rear_pub.publish(make_xyzirt(fs + PHASE_DIFF, rear_probe(fs)))
        self.k += 1


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
    rclpy.init()
    node = Driver()
    end = time.time() + duration
    spin_dt = 1.0 / IMU_HZ
    next_tick = time.time()
    while time.time() < end:
        now = time.time()
        if now >= next_tick:
            node.tick()
            next_tick = now + spin_dt
        rclpy.spin_once(node, timeout_sec=0.001)
    warm = [(t, z) for (t, z) in node.z_samples if t >= WARMUP_SEC]
    warm_z = [z for (_, z) in warm]
    all_z = [z for (_, z) in node.z_samples]
    print(f'e2e driver done: {node.k} imu ticks, {node.odom_count} odom msgs, '
          f'{len(node.room)} pts/frame, duration={duration}s')

    if node.odom_count == 0:
        print('FAIL: no /lio_odom received (lio_node never reached MAPPING)')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(2)
    if not warm_z:
        print('FAIL: no odom after warmup')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(2)

    z_max = max(abs(z) for z in warm_z)
    z_last = warm_z[-1]
    z_first = warm_z[0]
    print(f'z after warmup: first={z_first:.4f} last={z_last:.4f} '
          f'max|z|={z_max:.4f} (limit {Z_LIMIT})')
    print(f'z range over run: [{min(all_z):.4f}, {max(all_z):.4f}]')

    node.destroy_node()
    rclpy.shutdown()
    if z_max < Z_LIMIT:
        print('PASS: LIO converged with imu_acc_scale=1.0 (z bounded)')
    else:
        print(f'FAIL: z diverged (max|z|={z_max:.2f} >= {Z_LIMIT}); '
              f'check imu_acc_scale / IMU unit')
        sys.exit(1)


if __name__ == '__main__':
    main()
