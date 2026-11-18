# RoboSense Airy 双雷达接入与故障排查

本文记录 2026-10-21 在泰坦工作空间接入两台 RoboSense Airy 的实际排查过程，供以后现场手动复现。驱动配置见 [`src/rslidar_sdk/config/config.yaml`](src/rslidar_sdk/config/config.yaml)。

## 一、问题原因

电脑连接雷达的网卡 `enp7s0` 使用 `10.18.0.200/24`。两台雷达网页都可以打开，也都能 `ping` 通；最初测试前雷达厂家驱动时持续出现 `ERRCODE_MSOPTIMEOUT`，点云话题没有实际消息。

只读查询雷达网页配置后发现，两台雷达的 **Destination IP Address** 原来都是 `10.18.0.100`。这个字段是测量数据的**接收方地址**；电脑是 `10.18.0.200`，自然收不到原本发给 `10.18.0.100` 的 UDP 包。`ping` 只能证明电脑能访问雷达的设备 IP，不能证明雷达正在向电脑发送 MSOP、DIFOP 或 IMU 数据。

| 设备 | Device IP（雷达自身） | 原 Destination IP | 修改后 Destination IP | MSOP | DIFOP | IMU |
| --- | --- | --- | --- | ---: | ---: | ---: |
| 前雷达 | `10.18.0.120` | `10.18.0.100` | `255.255.255.255` | 6699 | 7788 | 6688 |
| 后雷达 | `10.18.0.121` | `10.18.0.100` | `255.255.255.255` | 6969 | 7878 | 6868 |

经确认，采用广播地址 `255.255.255.255`，以便电脑在同一网络上接收两台雷达的数据。保存后回读完整配置：两台雷达均**只有 `DstIp` 改变**，设备 IP、各端口和 IMU 开关保持原值。RoboSense [Airy 产品手册](https://robosense-robotics.github.io/product-manual/en/Airy/)说明了网页配置和广播模式；[rslidar_sdk 在线雷达高级说明](https://github.com/RoboSense-LiDAR/rslidar_sdk/blob/main/doc/howto/07_online_lidar_advanced_topics_CN.md)给出了广播和多雷达配置示例。

## 二、下次现场处理步骤

### 1. 检查电脑网卡、雷达供电和网络连通

```bash
ip -br addr
ping -c 3 10.18.0.120
ping -c 3 10.18.0.121
```

这次电脑雷达网卡为 `enp7s0`，IP 为 `10.18.0.200/24`。如果网页和 `ping` 都不可达，先检查线缆、供电、网卡地址、子网掩码。若 `ping` 正常但没有点云，继续检查目标 IP 和数据端口。

### 2. 登录雷达网页，重点检查 Destination IP

在连接雷达网络的电脑浏览器中分别打开：

- 前雷达：`http://10.18.0.120/`
- 后雷达：`http://10.18.0.121/`

首页点击 **Setting**，进入 **General Setting**。当前固件的设置页也可直接打开 `http://10.18.0.120/Parameter_Setting.html` 或 `http://10.18.0.121/Parameter_Setting.html`。设备型号可在首页 **Device** 页面确认，本次两台设备均报告为 `Airy`。

逐项核对：

| 网页字段 | 含义 | 前雷达 | 后雷达 |
| --- | --- | --- | --- |
| **Device IP Address** | 雷达自身管理地址 | `10.18.0.120` | `10.18.0.121` |
| **Destination IP Address** | 数据发往的目标地址 | `255.255.255.255` | `255.255.255.255` |
| **MSOP Port Number** | 点云原始包的目标端口 | `6699` | `6969` |
| **DIFOP Port Number** | 设备配置、标定包的目标端口 | `7788` | `7878` |
| **Imu Ctrl** | IMU 输出开关 | `ON` | `ON` |
| **Imu Port Number** | IMU 包的目标端口 | `6688` | `6868` |

本次只修改 **Destination IP Address**，然后点击 **General Setting 区域的 Save**。不要把 **Device IP Address** 改成广播地址，也不要误改端口。网页若在保存后短暂断开，重新打开页面核对；本次保存请求的 HTTP 连接被关闭，但设置已经生效。

也可以只读查询当前设置；`--noproxy '*'` 避免电脑的 HTTP 代理接管局域网请求：

```bash
curl --noproxy '*' -sS http://10.18.0.120/setting_data.json
curl --noproxy '*' -sS http://10.18.0.121/setting_data.json
```

重点看 `SrcIp`、`DstIp`、`MPort`、`DPort`、`IPort`、`ImuCtrl`。修改前最好保存这份 JSON，保存后再次读取并对比。本次备份为：

- 前雷达：[修改前](results/lidar/airy_settings_before_broadcast.json)、[修改后](results/lidar/airy_settings_after_broadcast.json)
- 后雷达：[修改前](results/lidar/rear_airy_settings_before_broadcast.json)、[修改后](results/lidar/rear_airy_settings_after_broadcast.json)

如果只要求 `10.18.0.200` 这一台电脑接收，可把目标地址设成 `10.18.0.200`，采用单播；这样原目标 `10.18.0.100` 将不再收到该单播流。本次改用广播后，已验证 `10.18.0.200` 能接收，**没有验证 `10.18.0.100` 上的软件是否仍正常处理广播包**。

### 3. 确认 UDP 包是否抵达本机

先停止占用相同端口的雷达接收节点，再在本机抓包。例如后雷达：

```bash
sudo tcpdump -ni enp7s0 -c 20 'src host 10.18.0.121 and udp'
```

检查输出中的**目标 IP 和目标端口**。源端口可能是设备临时端口，本次实际观察为 `49610`、`49608`、`49607`；不要将其误认为 MSOP、DIFOP、IMU 的目标端口。没有 `tcpdump` 权限时，可用 Wireshark，或运行工作空间中的原始报文节点 `ros2 launch astrall_lidar_udp lidar_udp.launch.py` 观察计数；这类诊断节点不要与厂家驱动同时争用相同 UDP 端口。

广播包的本机 UDP 监听地址应设为 `0.0.0.0`。我们曾把直接测试程序绑定到 `10.18.0.200`，因此漏掉了目的地址为 `255.255.255.255` 的包；改为 `0.0.0.0` 后就收到了。工作空间原始报文节点的 [`bind_address`](src/astrall_lidar_udp/config/lidar_udp.yaml) 已设为 `0.0.0.0`。厂家驱动未显式配置 `host_address`，默认也是 `0.0.0.0`。

广播设置生效后，每台雷达各测 10 秒：

| 雷达 | MSOP | DIFOP | IMU |
| --- | ---: | ---: | ---: |
| 前 | 22,399 | 10 | 2,000 |
| 后 | 22,397 | 10 | 2,000 |

### 4. 配置并编译 RoboSense 厂家驱动

这个工作空间已有 `rslidar_sdk`、`rs_driver`、`rslidar_msg`。当前 [`config.yaml`](src/rslidar_sdk/config/config.yaml) 的关键点：

- `common.msg_source: 1`：从在线雷达接收；`send_point_cloud_ros: true`：发布点云。
- `lidar` 中有两组 `driver`：型号都为 `RSAIRY`，三路端口分别与网页一致。
- 两台雷达使用不同的点云、IMU 话题和 frame ID，前雷达原有话题保持不变。
- `use_lidar_clock: false`：使用本机时间。前雷达诊断页当时显示时间同步为 `UnLock`，直接使用雷达时间曾得到明显不正确的 ROS 时间戳。

IMU 解析编译选项在厂家 CMake 文件中的默认值为 `OFF`。如果重新编译，需明确打开：

```bash
cd /home/ubuntu/astrall_sdk_1_0_5/astrall_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONNOUSERSITE=1 colcon build --packages-select rslidar_sdk \
  --cmake-args -DENABLE_IMU_DATA_PARSE=ON
source install/setup.bash
```

本机使用 `PYTHONNOUSERSITE=1`，是为了避开用户 Python 包与 ROS 构建工具的版本冲突。可用 `rg -n 'ENABLE_IMU_DATA_PARSE:BOOL' build/rslidar_sdk/CMakeCache.txt` 核对编译缓存为 `ON`。[官方在线雷达配置说明](https://github.com/RoboSense-LiDAR/rslidar_sdk/blob/main/doc/howto/06_how_to_decode_online_lidar_CN.md)

### 5. 启动一个节点并验证四个话题

```bash
cd /home/ubuntu/astrall_sdk_1_0_5/astrall_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rslidar_sdk rslidar_sdk_node
```

该节点会同时监听两组端口并解码。另开终端，加载同一 ROS 环境后检查**实际消息**：

```bash
ros2 topic echo /rslidar_points --field width --once
ros2 topic echo /rslidar_rear_points --field width --once
ros2 topic echo /rslidar_imu_data --field angular_velocity --once
ros2 topic echo /rslidar_rear_imu_data --field angular_velocity --once
```

| 雷达 | 点云话题 | IMU 话题 | frame ID |
| --- | --- | --- | --- |
| 前 | `/rslidar_points` | `/rslidar_imu_data` | `rslidar` |
| 后 | `/rslidar_rear_points` | `/rslidar_rear_imu_data` | `rslidar_rear` |

`ros2 topic list` 或 `ros2 topic info` 只能说明发布者已经创建，不能证明有数据。首次启动后给 DIFOP 标定包和 ROS 发现留几秒时间。正常配置保留厂家驱动默认的 `wait_for_difop=true`；曾短暂关闭该等待以定位问题，验证完毕已恢复，不能靠跳过标定长期运行。

### 6. 保存一帧点云和一段 IMU 作证据

保持厂家驱动运行，在另一终端执行：

```bash
cd /home/ubuntu/astrall_sdk_1_0_5/astrall_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/capture_airy_sample.py results/lidar/新的前雷达样本
python3 scripts/capture_airy_sample.py results/lidar/新的后雷达样本 \
  --cloud-topic /rslidar_rear_points --imu-topic /rslidar_rear_imu_data
```

脚本保存二进制 PCD、IMU CSV 和元数据 JSON。本次已保存的[前雷达样本](results/lidar/sample_20261021_host_clock/airy_sample.json)为 `96×900` 点云、22,302 个有限坐标点和 20 条 IMU；[后雷达样本](results/lidar/rear_sample_20261021_host_clock/airy_sample.json)为 `96×900`、10,913 个有限坐标点和 20 条 IMU。当前 `dense_points: false` 会保留无效点，PCD 样本只写入有限坐标点，因此保存点数可能小于消息的宽×高。

## 三、按现象快速定位

| 现象 | 优先核对 |
| --- | --- |
| 网页、`ping` 都不通 | 供电、线缆、网卡 IP、子网掩码 |
| `ping` 通，但驱动报 `ERRCODE_MSOPTIMEOUT` | 网页 **Destination IP Address**、MSOP 目标端口、本机抓包结果 |
| 广播已设置，自己写的 UDP 程序仍收不到 | 是否绑定 `0.0.0.0`；是否有别的程序占用或争用端口 |
| MSOP 有包，暂时没有点云 | DIFOP 包是否到达、端口和型号是否正确；给标定和首次 ROS 订阅留时间 |
| 点云有数据，但 IMU 没数据 | 网页 **Imu Ctrl=ON**、IMU 目标端口、`ENABLE_IMU_DATA_PARSE=ON` |
| 话题名称存在但 `echo --once` 一直无值 | 发布者不等于实际数据；回到 UDP 包、DIFOP 和驱动日志检查 |
| 时间戳明显异常 | 网页 **Diagnostic** 的时间同步状态；同步未建立时使用 `use_lidar_clock: false` |

网页首页的 **Diagnostic** 可查看激光状态、转速、温度和时间同步状态。当前固件还提供只读数据地址 `http://<雷达IP>/diag_data.json`，其中 `laser_sts`、`rpm`、`sync_sts` 便于排查。前雷达当时显示激光 `ON`、约 `599 RPM`，但同步状态为 `UnLock`。
