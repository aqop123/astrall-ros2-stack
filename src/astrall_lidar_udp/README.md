# 泰坦雷达 UDP 接收与协议确认

这个 ROS 2 Humble 包直接监听泰坦文档列出的前后雷达 UDP 端口，将收到的**原始报文**发布到 ROS 2。它不通过泰坦运动控制 SDK，也不发送雷达配置或机器狗运动命令。

本包只发布原始 UDP 报文。前后雷达均为 RoboSense Airy；需要解码点云与 IMU 时，请使用工作空间中已有的 `rslidar_sdk`。接入和故障排查过程见[独立文档](../../RoboSense_Airy_双雷达接入与故障排查.md)。

## 网络与端口

泰坦 SDK 文档第 3 节给出的默认值：

| 雷达 | IP | MSOP | DIFOP | 内置 IMU |
| --- | --- | ---: | ---: | ---: |
| 前雷达 | `10.18.0.120` | UDP `6699` | UDP `7788` | UDP `6688` |
| 后雷达 | `10.18.0.121` | UDP `6969` | UDP `7878` | UDP `6868` |

本机连接雷达网络的网卡应有 `10.18.0.200/24`（文档默认值）。默认监听 `0.0.0.0` 上的六个端口，并按前后雷达 IP 过滤来源。

## 编译与运行

本机用户 Python 环境中的 `setuptools` 与 `packaging` 版本不兼容，生成 ROS 自定义消息时可能报 `canonicalize_version()` 错误。因此这里用 `PYTHONNOUSERSITE=1` 让构建使用系统 Python 包：

```bash
cd <SDK根目录>/astrall_ws
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --packages-select astrall_lidar_udp
source install/setup.bash
ros2 launch astrall_lidar_udp lidar_udp.launch.py
```

节点每 10 秒打印各端口累计收到的报文数；每个端口首次收到报文时，还打印长度、来源 IP/端口和前 16 字节，便于识别协议。默认话题如下，类型均为 `astrall_lidar_udp/msg/UdpPacket`：

| 话题 | 默认本机 UDP 端口 |
| --- | ---: |
| `/lidar/front/msop_packets` | `6699` |
| `/lidar/front/difop_packets` | `7788` |
| `/lidar/front/imu_packets` | `6688` |
| `/lidar/rear/msop_packets` | `6969` |
| `/lidar/rear/difop_packets` | `7878` |
| `/lidar/rear/imu_packets` | `6868` |

`UdpPacket` 包含 ROS 接收时间、前/后雷达帧名、源 IP、源端口、目标端口和完整原始 UDP 负载。时间戳是本机接收时间，**不是雷达硬件时间**。话题使用传感器数据 QoS（best effort）。

另开终端检查：

```bash
cd <SDK根目录>/astrall_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic list
ros2 topic echo /lidar/front/msop_packets --qos-reliability best_effort --once
```

自定义端口或源 IP 请修改 [`config/lidar_udp.yaml`](config/lidar_udp.yaml)，也可通过 `params_file` 指定复制后的配置：

```bash
ros2 launch astrall_lidar_udp lidar_udp.launch.py params_file:=/绝对路径/lidar_udp.yaml
```

