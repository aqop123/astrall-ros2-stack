#!/usr/bin/env python3
"""判定雷达安装姿态: 采一帧点云, 看地面点落在哪个坐标面 -> 扫描面是水平还是侧躺。

用法(需先 source install/setup.bash 且雷达在发布):
    python3 scripts/check_lidar_axis.py /rslidar_points 12
    python3 scripts/check_lidar_axis.py /rslidar_rear_points 12

判据:
    地面点应在【雷达原生系 z≈常数】的平面上(速腾 RSAIRY 解码公式 z=d*sin(phi)+RZ)。
    - 若最低那层点 z 明显的常数面、数量多 -> 水平扫描面(安装正常, roll/pitch≈0)。
    - 若最低层点几乎都在 x≈常数 或 y≈常数 -> 雷达侧躺, 此时 FAST-LIO2 无法建图,
      必须重新安装或据实填写外参的 roll/pitch。
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


def unpack_xyz(msg):
    off = {f.name: f.offset for f in msg.fields}
    step = msg.point_step
    endian = ">" if msg.is_bigendian else "<"
    import struct
    pts = []
    n = msg.width * msg.height
    for i in range(0, n, max(1, n // 20000)):  # 最多取2万点
        b = i * step
        x = struct.unpack_from(endian + "f", msg.data, b + off["x"])[0]
        y = struct.unpack_from(endian + "f", msg.data, b + off["y"])[0]
        z = struct.unpack_from(endian + "f", msg.data, b + off["z"])[0]
        if all(math.isfinite(v) for v in (x, y, z)):
            pts.append((x, y, z))
    return pts


def spread(vals):
    vals = sorted(vals)
    if not vals:
        return 0.0
    n = len(vals)
    # 用 5%~95% 分位距衡量集中程度, 抗离群点
    return vals[int(n * 0.95)] - vals[int(n * 0.05)]


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else "/rslidar_points"
    timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0
    rclpy.init()
    node = Node("axis_check")
    got = []
    node.create_subscription(PointCloud2, topic, lambda m: got.append(m), 1)
    end = time.time() + timeout
    while time.time() < end and not got:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not got:
        print(f"FAIL: {timeout}s 内未收到 {topic} 的点云, 确认雷达节点在发布")
        rclpy.shutdown()
        return
    pts = unpack_xyz(got[-1])
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    zs = [p[2] for p in pts]
    z_min = min(zs)
    # 取最低 8% 的点, 近似地面层, 看它在哪个轴上最集中
    k = max(1, int(len(pts) * 0.08))
    ground = sorted(pts, key=lambda p: p[2])[:k]
    print(f"topic={topic} 采样 {len(pts)} 点")
    print(f"全点云范围: x[{min(xs):.2f},{max(xs):.2f}] y[{min(ys):.2f},{max(ys):.2f}] z[{min(zs):.2f},{max(zs):.2f}]")
    print(f"最低 {k} 点(疑似地面) 各轴 5~95% 展宽: "
          f"dx={spread([p[0] for p in ground]):.3f} "
          f"dy={spread([p[1] for p in ground]):.3f} "
          f"dz={spread([p[2] for p in ground]):.3f} m")
    print(f"地面层 z_min={min(p[2] for p in ground):.3f}  (RSAIRY 理论地面 z≈0.045+RZ偏移)")
    sx, sy, sz = (spread([p[0] for p in ground]), spread([p[1] for p in ground]),
                  spread([p[2] for p in ground]))
    if sz <= sx and sz <= sy:
        print("PASS: 地面点 z 最集中 -> 水平扫描面, 安装正常, roll/pitch 取 0 即可")
    else:
        axis = "x" if sx < sy else "y"
        print(f"WARN: 地面点在 {axis} 轴最集中 -> 疑似侧躺安装, 请据实填 roll/pitch "
              f"或重新安装雷达(竖直扫描面无法做 FAST-LIO2)")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
