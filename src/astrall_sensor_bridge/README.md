# 泰坦传感器 ROS 2 桥接包

`astrall_sensor_bridge` 面向 ROS 2 Humble。它从本目录上级的泰坦 SDK 1.0.5 订阅机身 IMU 数据，用其中的 `odomX`、`odomY` 和 `yaw` 发布平面里程计；另外通过 SDK 开启相机码流订阅，监听 UDP 6000，并尝试用 FFmpeg 将 H.264 解码成 ROS 图像。本包**不申请运动控制权，也不发送移动命令**。

## 当前能发布什么

| 话题 | 消息类型 | 内容 |
| --- | --- | --- |
| `/imu/data` | `sensor_msgs/msg/Imu` | SDK 加速度、角速度，以及由 roll/pitch/yaw 构造的姿态 |
| `/odom` | `nav_msgs/msg/Odometry` | SDK 的 `odomX`、`odomY` 和 `yaw`；速度未知，未计算 |
| `/camera/h264_packet` | `sensor_msgs/msg/CompressedImage` | UDP 6000 收到的每个原始包；`format=h264/udp-packet`，**不是可直接显示的完整图像** |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | FFmpeg 成功解码后输出的 `rgb8` 图像 |

话题使用 ROS 传感器数据 QoS（best effort）。原始包话题主要用于确认数据是否到达和分析视频封装，普通图像工具应订阅 `/camera/image_raw`。

## 前提条件

- Linux x86-64 或 ARM64；ROS 2 Humble、`colcon`、C++ 编译器、CMake、FFmpeg 开发库（`libavcodec-dev`、`libavutil-dev`、`libswscale-dev`）和 `pkg-config`。
- 工作空间自包含：SDK 头文件与两种架构的动态库已放在 `<工作空间>/sdk/`，直接克隆本仓库即可编译，无需外部 SDK 目录。编译时会按本机架构链接 `sdk/x86-64/` 或 `sdk/arm64/` 中的动态库；旧的 `<SDK根目录>/astrall_ws` 布局仍兼容（当工作空间内没有 `sdk/` 时自动回退到上一级 SDK 目录）。
- 运行电脑接入机器狗的运控网络。SDK 文档给出的默认配置是电脑 `10.18.0.200/24`、机器狗 `10.18.0.100`、双方 UDP 端口 `3600`；相机数据发往电脑 UDP `6000`。这些地址是文档默认值，以现场设备配置为准。
- SDK 文档要求建立连接前，机器狗处于阻尼或锁定趴下状态。确保没有其他进程占用本机 UDP `3600` 和 `6000`。

查看连接机器狗的网卡名：

```bash
ip -br link
```

如果该网卡尚未设置文档要求的 IP，可在确认网卡名后配置（不要重复添加已有地址）：

```bash
sudo ip addr add 10.18.0.200/24 dev <网卡名>
sudo ip link set <网卡名> up
ping -c 3 10.18.0.100
```

`ping` 仅是辅助检查；设备禁用 ICMP 时仍可能通过 SDK 通信。网络配置和运行均应在连接机器狗的同一台电脑上完成。

## 编译和运行

```bash
cd <SDK根目录>/astrall_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select astrall_sensor_bridge
source install/setup.bash
ros2 launch astrall_sensor_bridge sensors.launch.py
```

默认 SDK 连接超时为 60 秒。程序显示 `Sensor bridge ready` 表示 SDK 连接及 IMU 订阅成功；`Receiving camera UDP packets` 表示相机包已经到达；`Publishing decoded camera images` 表示已经成功解码。如果相机订阅失败，节点仍会继续发布 IMU 和里程计。

另开一个终端检查：

```bash
cd <SDK根目录>/astrall_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic list
ros2 topic echo /imu/data --qos-reliability best_effort --once
ros2 topic echo /odom --qos-reliability best_effort --once
ros2 topic hz /camera/h264_packet
ros2 topic hz /camera/image_raw
```

`/camera/h264_packet` 有频率而 `/camera/image_raw` 没有频率，表示收到 UDP 包，但现有 H.264 解码方式无法从该流得到完整帧；需要抓取原始包确认是否为 Annex B 码流、RTP 或厂家私有封装。两者都没有频率时，先检查 SDK 相机订阅结果、电脑 UDP `6000`、网卡和相机状态。

## 配置

参数在 [`config/sensors.yaml`](config/sensors.yaml) 中。常用参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `imu_frequency_hz` | `50` | SDK 支持 `1`、`25`、`50`、`125`、`250` Hz |
| `camera_enabled` | `true` | 关闭时不订阅或接收相机流 |
| `camera_udp_port` | `6000` | 相机码流的本地 UDP 端口 |
| `camera_source_ip` | 空 | 可选的相机 UDP 源地址过滤；先确认实际源地址再填写 |
| `imu_frame_id` | `imu_link` | IMU 消息坐标系名称 |
| `odom_frame_id` | `odom` | 里程计参考坐标系名称 |
| `base_frame_id` | `base_link` | 机身坐标系名称 |
| `camera_frame_id` | `camera_link` | 图像坐标系名称 |
| `odom_xy_variance` / `odom_yaw_variance` | `100.0` | 仅为保守占位值，需实测后调整 |

可复制 YAML 后通过 `params_file` 指定自己的参数文件：

```bash
ros2 launch astrall_sensor_bridge sensors.launch.py params_file:=/绝对路径/自己的参数.yaml
```

## 用于 SLAM 和导航前需要验证

1. **时间戳**：SDK `timestamp` 的单位与时间基准没有说明，所以 ROS 消息使用接收时的本机时间。与雷达融合前需要测量延迟并建立时间同步方案。
2. **里程计语义**：文档给出 `odomX/Y` 和 `yaw`，但没有说明 `odomX/Y` 单位、起点、轴方向、重置规则及 `yaw` 相对于里程计坐标系的关系。本包按米和弧度解释，现场应低速沿各轴运动并核对。`/odom` 中的速度字段保持零值并给出很大的未知速度协方差，**不能当作实测速度**。
3. **姿态与坐标系**：本包由 SDK 的 roll/pitch/yaw 计算四元数，未使用 SDK 中顺序不明的原始四元数字段。包不发布 TF；在确认雷达、相机、IMU 安装角度和机身坐标后，再发布静态变换和 `odom -> base_link`。默认协方差只是占位，不能直接当作已标定的不确定度输入融合器。
4. **图像封装**：SDK 文档只说明 UDP `6000` 推送 H.264，没有给出包结构。本包对收到的字节按 H.264 裸码流解析；如果是 RTP 或私有封装，原始包话题可收到数据，但标准图像话题不会有帧。
5. **雷达**：当前 SDK 头文件没有雷达点云订阅接口，目录中也没有雷达型号或驱动。工作空间中的 [`astrall_lidar_udp`](../astrall_lidar_udp/README.md) 包可监听文档给出的雷达 UDP 端口并发布原始报文；确认型号和数据格式后，才能进一步解码为 `PointCloud2`。

本包的接口依据是 SDK 根目录中的 `sdk/include/interface.h` 和《吠云-泰坦 SDK_1_0_5》文档。没有连接真实机器狗时，编译通过只说明代码可构建，不代表现场传感器流已验证。
