# 双 3D 激光雷达（速腾 RoboSense）接入、同步与轻量级拼接融合技术方案

> 修订版（v4→v5）：对接目标确认为 `FASTLIO2_ROS2-main/fastlio2`（`lio_node.cpp` + `config/lio.yaml`）。
> 依据其源码（`utils.cpp::cloud2PCL`、`imu_processor.cpp::undistort`）做三点关键修正：
> ① 融合节点**保留逐点 float64 绝对时间戳**，去畸变完全交给 fastlio2 原生逐点补偿（禁止预去畸变/清零时间戳）；
> ② `/imu/data` 为 best-effort 发布，fastlio2 IMU 订阅必须改 QoS，否则收不到任何 IMU 数据；
> ③ PointCloud2 线上布局为紧凑无填充（`point_step == 26`），解析方式相应更正。
> v5：明确文档中所有外参数值均为**占位示例**（仓库中不存在真实外参），新增 §7 给出无专用工具的外参获取流程；
> 明确 `lio.yaml` 的 `r_il/t_il` 按构造恒为单位阵/零向量，不是标定参数。
> 基线配置（`rslidar_sdk/config.yaml`、`lidar_udp.yaml`、`sensors.yaml`）均为实车验证过、曾配合 ORB-SLAM3 运行的现网配置，本方案只做标注过的增量修改。
> 文中 CPU 占用、点数量等指标均为**目标值/估计值，以实测为准**。

## 1. 现有硬件与底层通信拓扑分析（基于实车已验证配置）

1. **雷达型号与通信协议**（`rslidar_sdk/config/config.yaml`，实车运行验证）：
   * **前后两台雷达 `lidar_type` 均为 `RSAIRY`**——已在实车运行验证，型号以此为准，禁止凭猜测改为 RSHELIOS/RSBP 等其他型号。
   * **前向主雷达**：IP `10.18.0.120` | MSOP 端口 `6699` | DIFOP 端口 `7788` | 内置 IMU 端口 `6688`。
   * **后向/补盲雷达**：IP `10.18.0.121` | MSOP 端口 `6969` | DIFOP 端口 `7878` | 内置 IMU 端口 `6868`。
   * 输出话题为现网命名：`/rslidar_points`（前）与 `/rslidar_rear_points`（后），**沿用不重命名**，融合节点直接订阅。
   * `use_lidar_clock: false`（主机时钟）、`ts_first_point: true`（帧头 = 首点时刻）——现网已验证，保持不变。
   * **线上格式（XYZIRT，紧凑无填充）**：字段偏移 `x@0, y@4, z@8, intensity@12, ring@16, timestamp@18`，`point_step = 26`；`timestamp` 为 **FLOAT64 绝对秒**（主机时钟，含包内偏移修正）。此布局是本方案时间戳链路的基础（见 §3.1/§3.2）。
   * **前置条件**：两台雷达 Web 界面的 UDP 目标已指向工控机（`lidar_udp_node` 能收到包即证明雷达端无需改动）。
2. **UDP 转发节点**（`astrall_lidar_udp/config/lidar_udp.yaml`，实车运行验证）：
   * msop/difop/imu 分属三个话题，与 `rslidar_sdk` 的 `msg_source: 2` 路径（要求 msop+difop 同话题）不能直接对接——**不走转发路径**。
3. **底盘主 IMU 与里程计**（`astrall_sensor_bridge/config/sensors.yaml`，实车运行验证）：
   * 发布 `/imu/data` 与 `/odom`（frame_id `imu_link` / `odom`），现网 `imu_frequency_hz: 50`。
   * **必须将 `imu_frequency_hz` 从 50 改为 250**（节点支持 `1, 25, 50, 125, 250`；50Hz 不足以支撑 fastlio2 的 IMU 积分）。
   * **QoS 事实（对接 fastlio2 的关键）**：`astrall_sensor_node` 以 **SensorDataQoS（best-effort）** 发布 `/imu/data`；而 `lio_node.cpp` 的 IMU 订阅默认 **RELIABLE**——best-effort 发布 + reliable 订阅在 DDS 层不匹配，**fastlio2 将收不到任何 IMU 数据**。必须修改 fastlio2 的 IMU 订阅 QoS（见 §5）。
   * `camera_enabled: true` 为 ORB-SLAM3 视觉方案所需；纯激光 SLAM 测试阶段可临时关闭（见 §4），恢复视觉方案须改回。
4. **架构决策（砍掉冗余 UDP 转发）**：
   * `lidar_udp_node` 与 `rslidar_sdk`（`msg_source: 1`）绑定相同 UDP 端口，**不能同时运行**。运行本方案时**停用 `lidar_udp_node`**，由 `rslidar_sdk` 直连解码，省去 "UDP→ROS→UDP→再解码" 的中转开销（仍需经 DDS 发布消息，并非零拷贝）。

---

## 2. 第一步：速腾 `rslidar_sdk` 直连配置（基于已验证现网配置的增量修改）

> **编译前置条件（必须核实！）**：`rslidar_sdk/CMakeLists.txt` 中 `POINT_TYPE` 当前为 `XYZI`。**必须改为 `XYZIRT` 后 clean rebuild**——否则输出点云不含 `ring`/`timestamp` 字段，fastlio2 将退化为"无逐点时间 → 运动补偿关闭"（`utils.cpp` 会打印 `per-point motion compensation disabled`）。此改动只增字段，不影响已验证的解码链路。雷达内置 IMU 解析（`ENABLE_IMU_DATA_PARSE`，当前 OFF）本方案不需要，保持 OFF。

以实车验证过的 `config/config.yaml` 为基线，仅做下表增量修改：

| 参数 | 现网已验证值 | 本方案改为 | 原因 |
|---|---|---|---|
| `dense_points` | `false` | **`true`** | `true` 才丢弃无效 NaN 点（输出无序 `height=1` 点云）；`false` 保留 NaN，NaN 会穿透包围盒裁剪（`NaN > min` 恒 false）流入 SLAM |
| `min_distance` | `0.2` | `0.4` | 硬件层过滤近处自遮挡（与 fastlio2 `lidar_min_range: 0.4` 一致） |
| `max_distance` | `200` | `30.0`（前后一致） | 与 fastlio2 `lidar_max_range: 30.0` 对齐，减少上游无效点量；更远点 fastlio2 反正会丢弃 |
| `ros_queue_length` | `100` | `1`（可选） | 降低延迟；`rslidar_sdk` ROS2 发布端为 RELIABLE、深度即此值。不改也可运行 |
| 其余全部参数 | — | **保持现网值** | 型号、端口、时钟、话题名均已实车验证 |

改后的 `config/config.yaml`（**`[改]` 行为本方案增量，其余与现网一致**）：

```yaml
common:
  msg_source: 1                         # 1: packet message comes from online Lidar（现网已验证）
  send_packet_ros: false
  send_point_cloud_ros: true
lidar:
  - driver:
      lidar_type: RSAIRY             #  实车已验证型号，勿改。仅 RSAIRY/RSAIRYLITE_ETH/RSFAIRY/RSE1 支持 imu_port 解析
      msop_port: 6699
      difop_port: 7788
      imu_port: 6688                  #  现网值保持；仅当 ENABLE_IMU_DATA_PARSE=ON 才有输出，本方案不需要
      user_layer_bytes: 0
      tail_layer_bytes: 0
      min_distance: 0.4            #  [改]（现网 0.2）
      max_distance: 30             #  [改]（现网 200；与 fastlio2 lidar_max_range 30 对齐）
      use_lidar_clock: false       #  现网已验证，保持（主机系统时钟，与 astrall_sensor_node now() 同基准）
      dense_points: true           #  [改] true: discard NAN points（现网 false，必须改！）
      ts_first_point: true         #  现网已验证，保持
      start_angle: 0
      end_angle: 360
    ros:
      ros_frame_id: rslidar                           #  保持现网值（输入点云 frame_id 无所谓，融合节点统一变换到 imu_link）
      ros_recv_packet_topic: /rslidar_packets
      ros_send_packet_topic: /rslidar_packets
      ros_send_imu_data_topic: /rslidar_imu_data
      ros_send_point_cloud_topic: /rslidar_points      #  现网值，融合节点订阅此话题
      ros_queue_length: 1                              #  [改，可选]（现网 100）
  - driver:
      lidar_type: RSAIRY                               #  实车已验证型号，勿改
      msop_port: 6969
      difop_port: 7878
      imu_port: 6868
      user_layer_bytes: 0
      tail_layer_bytes: 0
      min_distance: 0.4                                #  [改]
      max_distance: 30                                 #  [改]
      use_lidar_clock: false                           #  必须与前雷达一致（主机系统时钟）
      dense_points: true                               #  [改]
      ts_first_point: true
      start_angle: 0
      end_angle: 360
    ros:
      ros_frame_id: rslidar                           #  [改] 必须与 RViz Fixed Frame / 前雷达一致;
                                                     #  若写 rslidar_rear, rslidar_sdk 不发 TF, RViz 会静默丢弃点云
                                                     #  (话题可见但无点云). 融合节点按话题区分, 不受影响.
      ros_recv_packet_topic: /rslidar_rear_packets
      ros_send_packet_topic: /rslidar_rear_packets
      ros_send_imu_data_topic: /rslidar_rear_imu_data
      ros_send_point_cloud_topic: /rslidar_rear_points  #  现网值，融合节点订阅此话题
      ros_queue_length: 1                              #  [改，可选]
```

> **型号提示**：rs_driver 不会根据 DIFOP 自动识别真实型号，填错只会静默输出错误角度的点云或泛化报错。本方案沿用实车已验证的 `RSAIRY`，无需确认动作；日后换型号务必先经雷达 Web 页面/DIFOP 字节确认。

---

## 3. 第二步：双 3D 雷达轻量级拼接融合节点 (`dual_lidar_fusion_node`)

### 3.1 输入解析与时间戳链路（本方案的正确性核心）

**输入布局（`rslidar_sdk` ROS2 线上格式，紧凑无填充）**：

```
字段:    x      y      z      intensity  ring       timestamp
偏移:    0      4      8      12         16         18
类型:    F32    F32    F32    F32        U16        F64(绝对秒)
point_step = 26 字节（无对齐填充！）
```

* **解析方式**：`timestamp` 位于偏移 18 且不满足 8 字节对齐，**禁止**把 buffer 强转成 `RsPointXYZIRT` 结构体（PCL 内存布局 32 字节含填充，与线上布局不一致）。使用 `PointCloud2ConstIterator<double>("timestamp")` 或按上表偏移 `memcpy` 读取。
* **运行时校验（必须实现）**：`point_step == 26`、`timestamp` 字段存在且 `datatype == FLOAT64`、`x/y/z` 为 FLOAT32。不满足则报错退出（典型事故：忘改 `POINT_TYPE` 时输入为 XYZI 布局）。

**时间戳精度红线（对应 `utils.cpp` 中已有告警）**：

> `float32` 在 1.7e9 量级（Unix 秒）下 ULP 约 128~256 秒——若融合节点用 float32 存绝对秒级时间戳，**所有点坍缩到同一时刻，逐点运动补偿完全失效**。`fastlio2/src/utils.cpp` 对此已显式打印告警（`time field is FLOAT32 with absolute time...`）。

因此融合节点的输出时间戳字段**固定如下**（代码需对字段类型显式断言并告警）：

* `timestamp` 字段 **FLOAT64 绝对秒**，原样透传 rslidar 值（不做任何换算/清零）；
* fastlio2 配置 `lidar_time_unit: sec` + `lidar_time_absolute: true`（现配置即此），内部换算为相对最早点的毫秒偏移，精度无损；
* 若输出被写成 float32 绝对秒或相对偏移语义不符，均为实现错误，节点校验直接报错（见上）。

### 3.2 核心处理流水线（去畸变职责全部移交 fastlio2）

**设计要点（与初版方案的关键差异）**：本仓库 fastlio2 的 `undistort`（`imu_processor.cpp`）按逐点 `curvature`（相对扫描起始的毫秒偏移）做**完整 SE3 补偿（旋转 + 平移）**，且对合并帧的 ~130ms 跨度同样适用（`syncPackage` 会等到 IMU 覆盖 `cloud_end_time` 才处理）。因此：

* **融合节点不做预去畸变、不清零时间戳**——若融合节点先做旋转预去畸变又保留原时间戳，fastlio2 会**二次补偿**（过补偿）；若清零时间戳，fastlio2 补偿退化为恒等，只剩融合节点残缺的纯旋转补偿（丢失平移项）。两种做法都错误。正确做法：**原样透传逐点绝对时间戳**，运动补偿 100% 由 fastlio2 完成（它用的 `/imu/data` 与融合节点本要用的是同一数据源）。
* 融合节点**无需订阅 IMU 与 /odom**，结构更简、CPU 更省（目标值：单核 < 8%）。

流水线共 3 步：

1. **双雷达帧同步 (`ApproximateTime`)**：
   * 订阅 `/rslidar_points` 与 `/rslidar_rear_points`（10Hz，现网已验证话题），同步容差 `slop = 0.03s`。发布端 `rslidar_sdk` 为 RELIABLE，订阅端用默认 RELIABLE 或 SensorDataQoS 均可匹配。
   * **相位同步前置检查（部署必做）**：两台 10Hz 雷达自由运行、统一主机时钟，帧起始相位差近似恒定。**若实测相位差 > 30ms（很常见），ApproximateTime 将持续配对失败，输出恒为空**。上线前必须实测相位差（对比两 topic `header.stamp` 小数部分），并采取其一：放宽 `sync_slop_sec`（**有效上限 < 0.05s**，即半个扫描周期）；或经雷达 Web/DIFOP 对齐起始相位。
   * 合并帧基准时刻 = 两帧 `header.stamp` 较早者 $t_{\text{base}}$（`ts_first_point: true` 即帧头 = 首点时刻）；最晚点时间戳可比 $t_{\text{base}}$ 晚约 100ms + 相位差（≤ slop），fastlio2 会覆盖该跨度。
2. **外参变换 + 自遮挡裁剪（同一单次循环完成）**：
   * 利用外参 $T_{\text{imu} \leftarrow \text{front}}$、$T_{\text{imu} \leftarrow \text{rear}}$ 将每个点变换至 `imu_link`（与 `imu_frame_id` 严格一致）。**每点保留其原 `timestamp` 不变**——点在"其采集时刻的 IMU 系"中的表达，正是 fastlio2 逐点补偿模型所要求的输入。
   * 同一循环内做机身包围盒裁剪（单位：米，相对于 imu_link；迈步时腿部摆出静态盒属已知局限，残留腿点由 fastlio2 当作外点处理，行走工况验收时确认无地图拖尾，见 §6.5）：
     ```cpp
     // 剔除机身包围盒内的自遮挡点 (单位: 米，相对于 imu_link)
     if (pt_imu.x() > box_min_x && pt_imu.x() < box_max_x &&
         pt_imu.y() > box_min_y && pt_imu.y() < box_max_y &&
         pt_imu.z() > box_min_z && pt_imu.z() < box_max_z) {
       continue;
     }
     ```
3. **哈希表快速体素降采样 (`0.10m`) 与发布**：
   * `std::unordered_map<uint64_t, 点>` 体素滤波，每体素保留首个落入点（含其 `timestamp`/`intensity`）。**目标**：双雷达约 5万~7万点压缩至 1.2万~1.6万（估计值）。
   * 发布至 `/merged_points`（与 fastlio2 `lidar_topic` 一致），`frame_id = "imu_link"`，`stamp = t_base`。**QoS 必须为 RELIABLE**（`lio_node` 的点云订阅是默认 RELIABLE；若融合节点用 SensorDataQoS/best-effort 发布，pub best-effort + sub reliable 不匹配，fastlio2 将收不到点云——与 §1 的 IMU QoS 问题同理）。
   * 输出字段与输入一致：`x, y, z, intensity, ring, timestamp`（紧凑布局，`point_step=26`，`timestamp` 保持 **FLOAT64 绝对秒**，见 §3.1）。

### 3.3 降级策略（单雷达失效）
* rear 雷达断流超过 `rear_timeout_sec`（建议 2.0s）：发布 front 单雷达点云（仍执行外参/裁剪/降采样，时间戳链路不变），按 1Hz 告警；恢复后自动回到双雷达合并。
* IMU 断流由 fastlio2 侧兜底：`syncPackage` 会因 IMU 未覆盖 `cloud_end_time` 而停住（不输出脏数据）；融合节点若订阅 IMU 仅做健康告警（可选）。

---

## 4. 节点配置与启动参数 (`dual_lidar_fusion.yaml`)

```yaml
# 1. 修改 astrall_sensor_bridge/config/sensors.yaml（实车现网文件，改动项已标注）
astrall_sensor_node:
  ros__parameters:
    imu_frequency_hz: 250               # [改] 现网 50，必须提升至 250Hz
    sdk_init_timeout_ms: 60000          # 现网值，保持
    imu_frame_id: imu_link              # 现网值，保持（融合节点输出 frame_id 与其一致）
    odom_frame_id: odom                 # 现网值，保持
    base_frame_id: base_link            # 现网值，保持
    camera_frame_id: camera_link        # 现网值，保持
    camera_enabled: true                # [注意] 现网 true 是 ORB-SLAM3 所需；仅纯激光 SLAM 测试可临时改 false（省约 20% CPU，估计值），恢复视觉方案前必须改回
    camera_udp_port: 6000               # 现网值，保持
    camera_source_ip: ""                # 现网值，保持
    odom_xy_variance: 100.0             # 现网值，保持
    odom_yaw_variance: 100.0            # 现网值，保持

# 2. 双雷达拼接融合节点参数（新节点；无 IMU/odom 相关参数，见 §3.2）
dual_lidar_fusion_node:
  ros__parameters:
    front_lidar_topic: "/rslidar_points"        # 现网已验证话题
    rear_lidar_topic: "/rslidar_rear_points"    # 现网已验证话题
    output_topic: "/merged_points"              # 与 fastlio2 lio.yaml 的 lidar_topic 一致
    output_frame_id: "imu_link"

    sync_slop_sec: 0.03                 # 前后雷达帧间最大允许时间差；上线前按实测相位差调整，有效上限 <0.05s
    rear_timeout_sec: 2.0               # 后雷达断流降级阈值（见 §3.3）
    voxel_leaf_size: 0.10               # 降采样体素尺寸 (m)
    output_qos_reliable: true           # 必须 RELIABLE（fastlio2 点云订阅为 RELIABLE，见 §3.2）

    # 泰坦机身自遮挡裁剪盒 (相对于 imu_link)
    self_filter_box:
      min_x: -0.65
      max_x: 0.65
      min_y: -0.40
      max_y: 0.40
      min_z: -0.60
      max_z: 0.30

    # 前雷达 -> imu_link 外参（！！以下数值为占位示例，仓库中不存在真实外参，
    #   实际值必须按 §7 流程获取：Step1 卷尺粗测 + Step2 RViz 精调）
    extrinsic_front_to_imu:
      translation: [0.35, 0.0, 0.12]    # [占位示例]
      quaternion: [0.0, 0.0, 0.0, 1.0]  # [占位示例] [qx, qy, qz, qw]；朝前安装通常为单位四元数

    # 后雷达 -> imu_link 外参（!! 数值同样为占位示例，按 §7 获取）
    extrinsic_rear_to_imu:
      translation: [-0.35, 0.0, 0.12]   # [占位示例]
      quaternion: [0.0, 0.0, 1.0, 0.0]  # [占位示例] 仅当后雷达确实水平朝后安装时才近似取绕Z 180度
```

---

## 5. 第三步：fastlio2 对接（`FASTLIO2_ROS2-main/fastlio2`）

融合节点输出的点云已完成外参变换（统一到 `imu_link`）且保留逐点 float64 绝对时间戳；fastlio2 的点云解析（`utils.cpp`）与逐点运动补偿（`imu_processor.cpp::undistort`）**原生支持该输入，点云解析无需改代码**。需要的改动只有两处：

**改动 1（必须，代码）：IMU 订阅 QoS**。`lio_node.cpp` 中 `/imu/data` 订阅当前为默认 RELIABLE，而 `astrall_sensor_node` 以 best-effort 发布，DDS 不匹配会导致 fastlio2 收不到任何 IMU：

```cpp
// lio_node.cpp 构造函数中，IMU 订阅改用 SensorDataQoS 与发布端匹配
m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
    m_node_config.imu_topic, rclcpp::SensorDataQoS(),
    std::bind(&LIONode::imuCB, this, std::placeholders::_1), imu_sub_options);
```

**改动 2（必须，配置）：`config/lio.yaml` 增量修改**：

```yaml
imu_topic: /imu/data                # 保持
lidar_topic: /merged_points         # 与融合节点输出一致
body_frame: base_link               # 保持；注意 fastlio 发布的是 IMU(imu_link) 位姿，
                                    # 若 imu_link 与 base_link 存在 cm 级安装偏差，可改为 imu_link 消除歧义
world_frame: odom                   # 保持（fastlio 广播 odom->base_link TF；astrall_sensor_node 只发 /odom 话题不广播 TF，无冲突）

lidar_time_unit: sec                # 保持：rslidar timestamp 为绝对秒
lidar_time_absolute: true           # 保持：内部换算为相对最早点的毫秒偏移（时间戳方案固定见 §3.1）

lidar_filter_num: 6                 # 保持（注意：与融合节点 0.10m 体素、scan_resolution 0.15 三级降采样叠加，行走点密度骤降时减小此值）
lidar_min_range: 0.4                # 保持（与 rslidar min_distance 0.4 一致）
lidar_max_range: 30.0               # 保持（与 rslidar max_distance 30 对齐）
scan_resolution: 0.15               # 保持
r_il: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]   # 按构造必须为单位阵（见下），不是标定参数
t_il: [0.0, 0.0, 0.0]               # [改] 同上，按构造取零。r_il/t_il 是"融合节点输出点云所在系 -> fastlio body 系"
                                    #      的变换；本方案融合节点输出已在 imu_link 系，而 fastlio 的 body 即
                                    #      /imu/data 的 imu_link，二者重合，故恒为单位阵/零向量。
                                    #      现网值 [-0.011, -0.02329, 0.04412] 对应旧融合节点的输出坐标约定
                                    #      （点云不在 imu_link 系），切换本方案后必须改零，否则引入 ~4cm 系统偏差。
                                    #      真正需要标定的外参全部在融合节点侧（获取流程见 §7）。
esti_il: false                      # 保持（外参已由融合节点消化，在线估计会吸收融合误差）
gravity_align: true                 # 保持
```

**IMU 加速度量纲（已核实，代码已修正）**：
* 早期 `lio_node.cpp::imuCB` 中加速度硬编码乘 `* 10.0`，仅为适配「0.1 m/s²」单位的旧数据源。**本数据源不适用**：`sdk/include/interface.h:191` 明确 `AstrallImuData::accelerometer` 单位为 `m/(s*s)`，`astrall_sensor_node::on_imu()` 原样透传（170-172 行），故 `/imu/data` 静止时 `|acc|≈9.81 m/s²`。若再 ×10，静止等效重力变为 98.1 m/s²，而 `ieskf.cpp:3` 内部 `g=9.81` 固定，IESKF 预测项 `r_wi·(acc-ba)+g` 每步产生 ≈88 m/s² 净加速度 → z 持续发散。已把系数改成可配置参数 `imu_acc_scale`（`config/lio.yaml`，默认 1.0，容错读取兼容旧配置），启动日志会打印当前值。
* 换算规则（换数据源时）：`scale = 9.81 / |acc_静止|`，实车用 `ros2 topic echo /imu/data --field linear_acceleration` 读静止值复核。
* e2e 收敛脚本 `scripts/e2e_lio_test.py`（见 §6）已验证：scale=1.0 静止 14s z≈0；scale=10.0 同工况 z 发散到 8.9m/14s（长时间即 z≈8.7e4）。

**其他需要核实的点（fastlio2 代码内，联调时确认）**：
* 时间戳全链路同基准：rslidar（主机时钟）+ `/imu/data`（`now()` 主机时钟），无跨时钟问题；`lidar_time_offset` 类参数保持 0。
* fastlio2 点云订阅为默认 RELIABLE 深度 10，融合节点发布频率 10Hz、单帧约 400KB，带宽无压力。

---

## 6. Agent 执行与联调验收步骤

1. **网络与端口验证**：
   * 确认工控机网卡已配置 `10.18.0.x` 网段 IP，能 `ping` 通 `10.18.0.120/121`。
   * 确保未同时运行 `lidar_udp_node`（双方绑定相同 UDP 端口，不能共存）。
2. **编译**：
   * `rslidar_sdk`：`POINT_TYPE` 由 `XYZI` 改为 `XYZIRT`，**clean rebuild**。
   * `dual_lidar_fusion_node`：编译后确认启动时字段布局校验通过（§3.1：`point_step==26`、`timestamp` FLOAT64）。
   * `fastlio2`：应用 §5 改动 1（IMU 订阅 QoS）后重编译。
3. **相位差实测（决定双雷达同步成败）**：
   * 对比 `/rslidar_points` 与 `/rslidar_rear_points` 的 `header.stamp` 相位差；若 > `sync_slop_sec`，放宽（上限 <0.05s）或对齐雷达起始相位；确认 `/merged_points` 连续输出 ≥ 60s 无中断。
4. **时间戳链路验证（本方案的核心验收项）**：
   * `ros2 topic echo --once /merged_points` 检查字段：`timestamp` 为 FLOAT64、数值为绝对秒（1.7e9 量级）且逐点不同；融合节点日志无 float32 精度告警。
   * `ros2 topic hz /imu/data` 确认 250Hz；`ros2 topic info /imu/data --verbose` 确认 fastlio2 订阅端 QoS 改为 best-effort 后与发布端匹配（改前该命令会显示 Reliability 不兼容，fastlio2 无 IMU 数据）。
   * fastlio2 日志确认进入初始化/MAPPING（若 IMU 收不到，`syncPackage` 会一直等待、无输出）。
5. **外参粗调与 RViz 拼图验收**：
   * RViz2 Fixed Frame `imu_link`，加载 `/merged_points`。
   * **静止站立**：前后雷达扫同一面墙若错位，按 §7 Step2 流程微调 `extrinsic_front_to_imu` / `extrinsic_rear_to_imu`（先地面 z/pitch/roll，后墙缝 Yaw/x/y）至严丝合缝；初值尚未标定时先执行 §7 Step0/Step1。
   * **行走工况**：直行 + 迈步越障，确认：① 无腿部/云台点残留；② 地面无"拖尾"双影——拖尾说明逐点补偿未生效，优先回查 §6.4 时间戳链路（float32 坍缩/POINT_TYPE 未改）。
   * 外参平移分量用已知尺寸特征复核一次，不只依赖目视。
6. **降级与容错验收**：
   * 断开后雷达网线：2s 内降级为 front 单雷达输出并告警；恢复后自动合并。
   * 停掉 IMU 流：fastlio2 停止输出（`syncPackage` 等待）并可在融合节点/lio 日志观察到告警，而非输出未补偿的脏数据。
7. **e2e 收敛自测（无需实车，仿真数据）**：
   * 脚本 `scripts/e2e_lio_test.py`：发布 250Hz 静止 IMU（acc.z=9.81 m/s²）+ 稠密静态"房间"点云（~2277 点/帧，保证 IESKF 有足够共面近邻），订阅 `/lio_odom` 检查静止工况 z 收敛。
   * 依次启动（各占一终端，先 `source install/setup.bash`）：
     `ros2 run dual_lidar_fusion dual_lidar_fusion_node --ros-args --params-file scripts/e2e_fusion_test.yaml`
     → `ros2 run fastlio2 lio_node --ros-args -p config_path:=<install>/fastlio2/config/lio.yaml`
     → `python3 scripts/e2e_lio_test.py 14`。
   * 判据：`PASS`（收到 odom、静止 z≈0）；若 `z` 发散（max|z|≥3m），优先查 `imu_acc_scale` 与 IMU 单位（§5）。该脚本曾复现：scale=10.0 时 z 14s 发散到 8.9m，scale=1.0 时 z≈0。
   * 点云必须够稠密：早期复用融合单测的 4+2 点 fixture 会触发 `NO Effective Points!`（`lidar_processor.cpp:228`），IESKF 无法更新、测试无意义。

---

## 7. 附：外参从哪来（无专用标定工具的获取流程）

**先澄清两件事（回答"这些参数是哪来的"）：**

1. **文档中的所有外参数值都是占位示例**（`translation: [0.35, 0, 0.12]`、`quaternion: [0,0,1,0]` 等）。仓库代码与配置中**不存在**任何真实外参——这些变换描述的是"雷达光学中心在 imu_link 坐标系中的位置与朝向"，物理上只存在于你的实车装配里，只能通过本节流程获得。旧方案（跑 ORB-SLAM3 时）不涉及双雷达拼接，所以也没有可继承的双雷达外参。
2. **`lio.yaml` 的 `r_il`/`t_il` 不是标定参数，不需要你"标"**。它的含义是"融合节点输出点云所在系 → fastlio body 系"的变换。本方案中融合节点已把两台雷达的点都变到 imu_link 系，而 fastlio2 的 body 就是 `/imu/data` 的 imu_link，两者重合，所以**按构造恒为单位阵/零向量**。现网那个非零的 `t_il = [-0.011, -0.02329, 0.04412]` 是旧的输出坐标约定留下的（当时融合节点输出的点云不在 imu_link 系），切换本方案时直接改成零即可，不要试图"标"出它。
   ——换句话说：**全部标定量集中在融合节点的 `extrinsic_front_to_imu` / `extrinsic_rear_to_imu` 两个参数上**，lio.yaml 侧不承担任何标定。

**用 SDK 文档外参直接给初值（吠云-泰坦 SDK_1_0_5.md §4.5）**：若手上没有标定工具，可先用文档给的传感器位置算出外参初值，再按 Step 0/2 收敛。文档 §4.5 的"传感器外参"是**相对质心、在机体系（X 朝头/Y 朝左/Z 朝上）下的位置**，单位 mm：前雷达 `(444,0,-32)`、后雷达 `(-420,0,19)`、IMU `(127,-85,82)`。注意此 IMU 是机身 SDK 的 IMU（`AstrallImuData`→`/imu/data`），**不是雷达 6688/6868 端口的内置 IMU**（本方案未启用）。fastlio 的 body 系 = `/imu/data` 的 `imu_link`，融合节点需要的是"雷达原点在 imu_link 中的位置"，故取 imu_link 原点在 IMU 芯片处时：

   `t = p_雷达(机体系) − p_IMU(机体系)` → 前 `(0.317, 0.085, -0.114)`、后 `(-0.547, 0.085, -0.063)`（米），已填入 `dual_lidar_fusion.yaml`；`self_filter_box` 相应平移。若选择把 imu_link 原点放质心，则 t 用文档原始位置、box 用机体系值，二者只需一致。

   **旋转按现场确认的侧躺安装填写**：经现场确认，两台雷达都是**侧躺**装在头部/尾部，即雷达原生 Z 轴绕机身 Y 轴转约 90°。§4.5 的 4.12°/2.59° 经核验恰为位置矢量仰角 `atan(|z|/x)`，不是安装姿态，文档未提供姿态信息。当前 `dual_lidar_fusion.yaml`：
   * 前雷达绕 Y +90°：`[0, 0.707107, 0, 0.707107]`
   * 后雷达绕 Y +100°：`[0, 0.766044, 0, 0.642788]`（用户描述"比 90 度多一点"，**100° 为暂定值，确切角度待现场给出后修正**）
   * 若转完点云上下颠倒（地面跑到 z>0），把两个四元数的 `qy` 取负号即可（绕 Y −90°/−100°）。
   * **实车验算（必做）**：侧躺安装下 FAST-LIO2 能否稳定建图取决于外参精度，务必按 §6.5 行走验证（地面无拖尾、墙角无重影）；若发现系统性上下/左右错位，优先微调这两个角度与 translation。可辅以 `python3 scripts/check_lidar_axis.py` 采一帧核对雷达轴向。
   * 另：IMU 轴向若非标准（z 朝下等），须把轴向修正吸收进这两个外参的旋转，不要改 IMU 节点。

**Step 0：先确定 imu_link 的轴向（5 分钟，决定后续所有符号）**

IMU 藏在底盘内、安装朝向未必是"前x左y上z"，先实测确认：
1. 机器人静止水平放置，`ros2 topic echo /imu/data`：`linear_acceleration` 应近似 `(0, 0, ±9.8)`——确定 z 轴朝上还是朝下；
2. 手推机器人沿前后/左右方向平移、绕竖直轴缓慢旋转，对照 `angular_velocity` 与 `linear_acceleration` 各分量的符号变化，确定 x/y 轴指向及是否右手系。
   若 z 朝下或轴向不标准，把该修正吸收进外参旋转矩阵，不要改 IMU 节点。

**Step 1：卷尺 + 安装朝向，给出初值（cm 级精度即可）**

* **平移**：卷尺量每台雷达外壳中心到 IMU 安装点的 x/y/z 距离，按 Step 0 确定的轴向填符号。注意量的是外壳，与光学中心差 1~2cm，靠 Step 2 修正——初值只求"量级正确"。
* **旋转**：按支架安装朝向取特殊角即可：前雷达朝前通常取单位四元数 `[0,0,0,1]`；后雷达朝后取绕 Z 180° `[0,0,1,0]`（[qx,qy,qz,qw] 顺序）。倒装/侧装按实际角度填。

**Step 2：RViz 精调（静止工况，两步收敛）**

1. 启动融合节点，RViz2 Fixed Frame `imu_link`，加载 `/merged_points`。
2. **先调地面（z + pitch/roll）**：两台雷达各自扫出的地面必须重合为**同一张水平面**。先单独调某台外参的 z 与 pitch/roll，使两地平面高度一致且水平（可对点云做平面拟合对比法线与截距，比目视准）。
3. **再调墙缝（yaw + x/y）**：选一面前后雷达都能看到的长直墙，微调 Yaw 使墙线拼接无错位，再微调 x/y 平移。
4. 迭代 2↔3，收敛判据：重叠区错位 < 0.1m（约一个体素）。调好立即固化进 `dual_lidar_fusion.yaml` 并记录日期。

**Step 3：行走验证（§6.5）**

直行 + 迈步时地面无拖尾、墙角无重影即收敛。若静止时对齐、行走时前后点云相对错位随速度增大，优先怀疑后雷达外参的平移分量或其 Yaw。

**（可选进阶）自动化精标**：静止录一段 rosbag，对前后雷达重叠区点云做 ICP 配准求解相对位姿，再与 Step1/2 结果比对验证。没有现成工具链前不作为阻塞项，手调流程对 0.10m 体素的 SLAM 足够。
