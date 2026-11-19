# SLAM 使用说明（`FASTLIO2_ROS2-main`）

> 本文替换旧的“整体执行方案/规划”文档，以当前仓库里真实存在的代码为准。  
> 目标包：`src/FASTLIO2_ROS2-main`（ROS 2 Humble）。

**结论先说**：

- 这个仓库包含 **LIO 前端、PGO 回环建图、ICP 重定位、HBA 一致性地图优化** 四个可用能力。
- 上游必须提供 `/merged_points`（双雷达融合点云）和 `/imu/data`（机身 IMU）；**本包不负责双雷达融合**。
- **建图用 `pgo_launch.py`，定位用 `localizer_launch.py`，二者互斥**；它们都会自动再启动一个 `lio_node`，不要再单独启动 `fastlio2`。
- 当前包**没有** RTK/GPS 因子、Scan Context 描述子、自动粗定位、`slam_watchdog`；旧文档里这些描述已不适用。

---

## 1. 包结构与职责

```text
src/FASTLIO2_ROS2-main/
├── fastlio2/       # 激光惯性里程计：IESKF + ikd-Tree，输出 odom 与局部/全局点云
├── pgo/            # 建图后端：位置先验 + ICP 回环，GTSAM ISAM2 位姿图优化，保存地图
├── localizer/      # 定位：两阶段 ICP（粗 + 精），加载先验 PCD，输出 map→odom 修正
├── hba/            # 离线一致性地图优化：读取 PGO 保存的 patches + poses，优化并可视化
└── interface/      # 公共服务定义：SaveMaps / Relocalize / IsValid / RefineMap / SavePoses
```

| 包 | 节点 | 自带 launch | 作用 |
|---|---|---|---|
| `fastlio2` | `/fastlio2/lio_node` | `lio_launch.py` | 纯里程计 / 前端调试 |
| `pgo` | `/pgo/pgo_node` | `pgo_launch.py` | **建图**：同时启动 lio + pgo + RViz |
| `localizer` | `/localizer/localizer_node` | `localizer_launch.py` | **定位**：同时启动 lio + localizer + RViz |
| `hba` | `/hba/hba` | `hba_launch.py` | **离线优化**：只启动 hba + RViz |
| `interface` | 无 | 无 | 仅定义 srv |

---

## 2. 上游输入要求

### 2.1 必备话题

| 话题 | 发布方 | 类型 | QoS | 说明 |
|---|---|---|---|---|
| `/merged_points` | `dual_lidar_fusion_node` | `sensor_msgs/PointCloud2` | RELIABLE | 双雷达融合后的点云 |
| `/imu/data` | `astrall_sensor_node` | `sensor_msgs/Imu` | SensorDataQoS（best-effort） | 机身 SDK IMU，250Hz |

`fastlio2` 的 IMU 订阅使用 `rclcpp::SensorDataQoS()`，点云订阅使用深度 10 的普通 RELIABLE QoS。**QoS 不匹配会直接收不到数据**。

### 2.2 点云时间戳要求

`/merged_points` 必须满足：

- 含逐点时间字段 `time` 或 `timestamp`（代码优先找 `time`，其次 `timestamp`）；
- 推荐 **FLOAT64 绝对秒**；
- `lio.yaml` 中对应配置为：

```yaml
lidar_time_unit: sec
lidar_time_absolute: true
```

代码会先减去最早点时刻，再转换成相对扫描起始的毫秒偏移写入 `curvature`，供 FAST-LIO2 做逐点运动补偿。  
如果时间字段是 **FLOAT32 且为绝对秒**，在 1.7e9 量级下精度只剩约 100 秒量级，去畸变失效；如果没有时间字段，会退化为不做逐点补偿。

### 2.3 坐标系约定

- 融节点已把点云变换到 `imu_link`；
- `/imu/data` 的 `frame_id` 也是 `imu_link`；
- 因此 `lio.yaml` 中：

```yaml
r_il: [1,0,0, 0,1,0, 0,0,1]
t_il: [0,0,0]
```

**必须保持单位阵/零向量**。IMU 与雷达的外参全部在 `dual_lidar_fusion.yaml` 的 `extrinsic_*_to_imu` 中消化，不能在这里再填一次，否则会二次变换。

---

## 3. 编译

```bash
cd /home/cersei/astrall_ws
source /opt/ros/humble/setup.bash

# 只编译 LIO
colcon build --packages-up-to fastlio2 --cmake-args -DCMAKE_BUILD_TYPE=Release

# 编译建图 / 定位 / 离线优化
colcon build --packages-up-to pgo localizer hba --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

主要系统依赖：`pcl`、`eigen`、`gtsam`、`yaml-cpp`、`message_filters`、`tf2_ros` 等。  
仓库自带 `fastlio2/third_party/sophus`，优先使用仓库版本；若系统 Sophus 与之冲突，按包内 README 处理。

---

## 4. 四种运行模式

启动前先确认上游已经运行：

```bash
ros2 topic hz /merged_points
ros2 topic hz /imu/data
```

### 4.1 纯 LIO：调试前端 / 标定

```bash
ros2 launch fastlio2 lio_launch.py
```

输出（均在 `/fastlio2` 命名空间下）：

| 话题 | 说明 |
|---|---|
| `/fastlio2/lio_odom` | `nav_msgs/Odometry`，`header=odom`，`child=imu_link` |
| `/fastlio2/body_cloud` | 机体坐标系点云，`frame=imu_link`，供 PGO / localizer 使用 |
| `/fastlio2/world_cloud` | 世界坐标系点云，`frame=odom` |
| `/fastlio2/lio_path` | 轨迹，`frame=odom` |

广播 TF：`odom → imu_link`。

适合用来检查：IMU 是否正常、点云是否正常、Z 轴是否漂移、`r_il/t_il` 是否为零。

---

### 4.2 PGO 建图：回环 + 保存地图

```bash
ros2 launch pgo pgo_launch.py
```

**注意：这个 launch 已经包含 `lio_node`，不要再单独启动 `fastlio2`。**

额外输出：

| 话题 / 服务 | 说明 |
|---|---|
| `/pgo/loop_markers` | 回环可视化 MarkerArray |
| `/pgo/save_maps` | 保存地图服务 |

广播 TF：`map → odom`，与 LIO 的 `odom → imu_link` 串成完整链路。

保存地图：

```bash
mkdir -p /home/cersei/astrall_ws/maps/run01

ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '/home/cersei/astrall_ws/maps/run01', save_patches: true}"
```

生成物：

```text
maps/run01/
├── map.pcd          # 全局地图
├── poses.txt        # 关键帧位姿
└── patches/0.pcd ...  # 逐关键帧子图，HBA 必需
```

**`save_patches` 建议始终为 `true`**：否则 `poses.txt` 为空、没有 `patches/`，后续 HBA 无法使用。

回环算法是**位置先验近邻搜索 + ICP 子图匹配 + GTSAM ISAM2**，不是 Scan Context。关键参数在 `pgo.yaml`：

```yaml
key_pose_delta_deg: 10      # 每转 10° 取一个关键帧
key_pose_delta_trans: 0.5   # 每移动 0.5m 取一个关键帧
loop_search_radius: 1.0     # 近邻搜索半径
loop_time_tresh: 60.0       # 时间间隔 >60s 才允许成为回环候选
loop_score_tresh: 0.15      # ICP fitness 阈值
```

---

### 4.3 Localizer 定位：加载先验地图 + 手动初值

```bash
ros2 launch localizer localizer_launch.py
```

**同样已经包含 `lio_node`，且与 PGO 互斥**：两者都广播 `map → odom`，同时运行会造成 TF 冲突。

调用重定位：

```bash
ros2 service call /localizer/relocalize interface/srv/Relocalize \
  "{pcd_path: '/home/cersei/astrall_ws/maps/run01/map.pcd', x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, pitch: 0.0, roll: 0.0}"
```

- 位姿单位为 **米 / 弧度**；
- 初值是机器人在 **map 系**下的估计位姿；
- 当前包**不会自动全局粗定位**，必须给一个合理初值，否则两阶段 ICP 可能收敛到错误位置。

检查结果：

```bash
ros2 service call /localizer/relocalize_check interface/srv/IsValid "{code: 0}"
```

`code: 0` 返回真实 `localize_success`；`code: 1` 在代码里恒返回 `valid=true`，不要用它判断成功。

输出：

| 话题 | 说明 |
|---|---|
| `/localizer/map_cloud` | 对齐后的地图点云，可视化用 |
| TF `map → odom` | 低频漂移修正，默认 `update_hz: 1.0` |

`localizer.yaml` 里 `local_frame: lidar` 会在收到第一帧 odom 后被代码改写成 odom 的 `header.frame_id`，实际广播的是 `map → odom`。

两阶段 ICP 参数：

```yaml
rough_scan_resolution: 0.25
rough_map_resolution: 0.25
rough_max_iteration: 5
rough_score_thresh: 0.2

refine_scan_resolution: 0.1
refine_map_resolution: 0.1
refine_max_iteration: 10
refine_score_thresh: 0.1
```

---

### 4.4 HBA 一致性地图优化：离线后处理

前提：已经用 PGO 保存过 `patches/` + `poses.txt`（即 `save_patches: true`）。

```bash
ros2 launch hba hba_launch.py

ros2 service call /hba/refine_map interface/srv/RefineMap \
  "{maps_path: '/home/cersei/astrall_ws/maps/run01'}"
```

优化过程会在 `/hba/map_points` 上按迭代发布地图点云（`frame=map`），可在 RViz 中观察。

导出优化后位姿：

```bash
ros2 service call /hba/save_poses interface/srv/SavePoses \
  "{file_path: '/home/cersei/astrall_ws/maps/run01/poses_refined.txt'}"
```

注意：

- HBA 是**离线服务式**节点，不订阅在线雷达/IMU，也不广播 TF；
- 当前 `hba_node.cpp` **不会自动保存优化后的 PCD**，只发布到 `/hba/map_points` 并可通过 `save_poses` 导出位姿；
- 仓库里的 `hba_node copy.cpp` 有硬编码保存 PCD 的代码，但**未参与编译**，不要依赖它。

---

## 5. 模式互斥关系

| 同时启动 | 是否允许 | 原因 |
|---|---|---|
| `lio_launch.py` + `pgo_launch.py` | 否 | PGO launch 已含 lio，会重复发布话题/TF |
| `lio_launch.py` + `localizer_launch.py` | 否 | 同上 |
| `pgo_launch.py` + `localizer_launch.py` | **否** | 两者都广播 `map → odom` |
| `hba_launch.py` + 任意在线模式 | 可以，但无必要 | HBA 离线运行，不占用在线话题/TF |

---

## 6. 配置文件说明

各 launch 通过 `config_path` 参数传入 yaml，C++ 端用 yaml-cpp 读取。

| 文件 | 被谁加载 | 作用 |
|---|---|---|
| `fastlio2/config/lio.yaml` | `lio_node` | LIO 前端核心参数 |
| `pgo/config/pgo.yaml` | `pgo_node` | 关键帧、回环、位姿图参数 |
| `localizer/config/localizer.yaml` | `localizer_node` | ICP 重定位参数 |
| `hba/config/hba.yaml` | `hba_node` | 一致性优化参数 |

### `lio.yaml` 关键参数

```yaml
imu_topic: /imu/data
lidar_topic: /merged_points
body_frame: imu_link
world_frame: odom

lidar_time_unit: sec
lidar_time_absolute: true
imu_acc_scale: 1.0

lidar_filter_num: 6
lidar_min_range: 0.4
lidar_max_range: 30.0
scan_resolution: 0.15
map_resolution: 0.3

na: 0.01
ng: 0.01
nba: 0.0001
nbg: 0.0001
lidar_cov_inv: 1000.0

imu_init_num: 20
near_search_num: 5
ieskf_max_iter: 5
gravity_align: true
esti_il: false
```

重点说明：

- `imu_acc_scale: 1.0`：机身 SDK IMU 加速度单位是 m/s²；旧代码硬编码 `*10.0` 会让静止重力变成 98.1 m/s²，导致发散。换 IMU 数据源时按 `scale = 9.81 / |a_static|` 重新计算。
- `lidar_filter_num: 6`：每 6 个点取 1 个进前端，低算力调优项。
- `lidar_min_range: 0.4`：与 `rslidar_sdk` 的 `min_distance: 0.4` 保持一致，先做硬件层近处滤除。
- `r_il/t_il` 保持单位阵/零向量，理由见 §2.3。

### `pgo.yaml` / `localizer.yaml` / `hba.yaml`

见 §4.2 / §4.3 / §4.4 的对应片段。若仍通过 `astrall_bringup` 统一启动，以 bringup 中集中管理的 config 为准，改完需要重新 `colcon build --packages-select astrall_bringup`。

---

## 7. 标准作业流程

### A. 建图

1. 启动雷达驱动、`astrall_sensor_node`、`dual_lidar_fusion_node`，确认 `/merged_points` 与 `/imu/data` 正常。
2. `ros2 launch pgo pgo_launch.py`
3. 检查 TF：`map → odom → imu_link`。
4. 机器人走一遍待建区域，尽量形成闭环；观察 `/pgo/loop_markers`。
5. `mkdir -p <map_dir>` 后调用 `/pgo/save_maps`，`save_patches: true`。
6. 用 RViz 打开 `map.pcd`，检查重影、地面拖尾、闭环误差。

### B. 定位

1. 启动同样的上游传感器与融合节点。
2. `ros2 launch localizer localizer_launch.py`
3. 调用 `/localizer/relocalize` 并提供 `map.pcd` 与合理初值。
4. 用 `/localizer/relocalize_check`（`code: 0`）确认收敛。
5. 观察 `/localizer/map_cloud` 与 TF `map → odom` 是否稳定。

### C. 离线优化

1. 使用 A 中保存的目录，确认存在 `patches/` 与 `poses.txt`。
2. `ros2 launch hba hba_launch.py`
3. 调用 `/hba/refine_map`，观察 `/hba/map_points`。
4. 调用 `/hba/save_poses` 导出优化位姿。

---

## 8. 验收与排错

```bash
ros2 topic hz /merged_points
ros2 topic hz /imu/data
ros2 topic echo /imu/data --field linear_acceleration --once
ros2 topic echo /fastlio2/lio_odom --field header.frame_id --once
ros2 run tf2_tools view_frames
```

| 现象 | 优先检查 |
|---|---|
| `/imu/data` 有发布但 LIO 无数据 | IMU 订阅是 SensorDataQoS；发布端必须是 best-effort 兼容配置 |
| LIO 收不到点云 | `/merged_points` 是否为 RELIABLE；是否有 `time`/`timestamp` 字段 |
| 静止时 Z 轴发散 | `imu_acc_scale`；用 `ros2 topic echo` 测静止 `|acc|` 是否约 9.81 |
| 点云无去畸变 | 时间字段是否 FLOAT32 绝对秒 / 是否缺失 |
| 没有 `map → odom` | PGO 或 localizer 是否运行；是否被重复 launch 造成冲突 |
| 出现两个 `lio_node` | 检查是否重复启动 launch；PGO/localizer 自带 lio |
| `/pgo/save_maps` 失败 | `file_path` 目录必须已存在；是否有关键帧（`NO POSES!`） |
| HBA 报 `patches not exists` | 保存地图时 `save_patches` 必须为 `true` |
| 重定位失败 | 初值是否合理；`pcd_path` 是否存在；用 `code: 0` 查真实状态 |

---

## 9. 与旧规划文档的差异

旧 `slam.md` 是方案规划，当前实现已经收敛为以下事实：

- **没有** RTK/GPS 因子与协方差门控；
- **没有** Scan Context 描述子，PGO 回环是“位置先验 + ICP”；
- **没有** 自动全局重定位，localizer 需要手动给初值；
- **没有** `slam_watchdog` / 轮足里程计兜底；
- `titan_fastlio2.yaml` / `timestamp_unit: 3` 等旧参数名已不存在，以当前 `lio.yaml` 为准。

新增这些能力时应另开功能包或修改本包源码，不要假设它们已经存在。
