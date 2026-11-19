# FAST-LIO2 双雷达前端实机测试方案

> 本文档用于指导**实机建图前的前端诊断数据采集与分析**。
> 诊断工具准备完成**不代表 FAST-LIO2 建图质量已通过验收**；所有结论必须基于实机录包与实测记录，不得填写推测数据。

## 1. 测试范围与前置条件

- 被测对象：`fastlio2` 的 `lio_node`（仅前端 LIO + 局部地图），双雷达融合输入 `/merged_points`。
- 不修改 IESKF 核心、IMU 噪声参数、双雷达外参、PGO/Localizer 算法。
- 环境：Ubuntu 22.04 + ROS 2 Humble；已执行 `colcon build` 并 `source install/setup.bash`。
- 启动（**只用 FAST-LIO2 前端，不启动 PGO**）：

  ```bash
  source /opt/ros/humble/setup.bash && source /home/cersei/astrall_ws/install/setup.bash
  ros2 launch astrall_bringup fastlio.launch.py
  ```

  `fastlio.launch.py` 只启动雷达驱动、IMU 桥接、双雷达融合、`fastlio2::lio_node` 和 RViz；
  `slam.launch.py` 中的 `pgo_node` 当前被注释掉，若未启用回环，两者节点集相同，但本方案统一用 `fastlio.launch.py`。
  定位模式用 `localizer.launch.py`，**不要同时运行多个入口**（会重复启动 `lio_node`）。

- **话题归属（避免误判为 PGO 输出）**：

  | 话题 | 发布者 | 是否本测试需记录 |
  |---|---|---|
  | `/imu/data` | `astrall_sensor_node` | 是 |
  | `/rslidar_points`、`/rslidar_rear_points` | `rslidar_sdk_node` | 是 |
  | `/merged_points` | `dual_lidar_fusion_node` | 是 |
  | `/fastlio2/lio_odom`、`/fastlio2/lio_diagnostics`、`/fastlio2/body_cloud`、`/fastlio2/world_cloud`、`/fastlio2/lio_path` | `fastlio2::lio_node`（由 launch `namespace="fastlio2"` 得来） | 是 |
  | `/pgo/loop_markers` | `pgo_node`（仅 PGO 启动时存在） | 否（本方案不涉及） |

  PGO 本身**订阅** `/fastlio2/lio_odom` 与 `/fastlio2/body_cloud`（见 `config/pgo.yaml:1-2`），
  并不发布 `lio_odom`；当前两个 launch 都未启动 `pgo_node`，因此记录上述话题不会混入 PGO 输出。
- 每次测试前确认：

  ```bash
  ros2 topic list | grep -E 'merged_points|lio_odom|lio_diagnostics|imu/data'
  ros2 topic hz /fastlio2/lio_diagnostics   # 期望约 2 Hz
  ```

  若 `/fastlio2/lio_diagnostics` 无数据，检查 `config/lio.yaml` 中 `diagnostics.enable` 是否为 `true`，并确认已重新 build `astrall_bringup`（配置随包安装）。

## 2. 诊断数据来源

### 2.1 诊断话题 `/fastlio2/lio_diagnostics`

默认 2 Hz 发布，消息类型 `interface/msg/LioDiagnostics`，可直接被 rosbag 录制：

| 字段 | 含义 |
|---|---|
| `builder_status` / `imu_initialized` | 0=IMU_INIT, 1=MAP_INIT, 2=MAPPING；后者表示 IMU 初始化完成 |
| `imu_frames_received` / `cloud_frames_received` | 累计 IMU / 点云接收帧数 |
| `cloud_frames_processed` / `failed_sync_count` | 累计成功处理帧数 / 同步失败次数（同步失败表示点云等不到足够 IMU） |
| `odom_frames_published` | 累计实际发布的里程计帧数 |
| `cloud_process_hz` / `odom_publish_hz` | 最近 5 s 窗口的处理频率 / 里程计输出频率 |
| `last_process_cost_ms` / `avg_process_cost_ms` | 最近一帧 / 窗口平均 `process()` 耗时 |
| `last_match_cost_ms` | 最近一帧地图匹配（最近邻搜索+平面拟合）累计耗时 |
| `valid_point_num` / `matched_point_num` | 下采样后送入匹配的点数 / 有效匹配点数 |
| `last_process_stamp` | 最近一次成功处理的数据时间戳 |
| `pose_delta_trans` / `pose_delta_rot_deg` | 相邻里程计位置变化 (m) / 角度变化 (deg) |
| `pose_timestamp_rollback` | 最近一帧是否出现里程计时间戳倒退 |
| `last_odom_stamp` / `odom_silence_sec` / `odom_timeout` | 里程计时间戳 / 静默时长 / 是否超时未输出 |
| `imu_static` / `static_drift_trans` / `static_drift_rot_deg` | 是否判定静止 / 当前静止窗口累计最大位置与姿态漂移 |
| `pose_jump_warning` / `static_drift_warning` / `diagnostic_warning_count` | 超阈值诊断标志与累计告警数 |

阈值全部在 `src/astrall_bringup/config/lio.yaml` 的 `diagnostics:` 段配置，均有默认值。
**注意：告警只是诊断信号，不能直接认定为定位失效；急加速、原地旋转、地面上台阶都可能触发跳变。**

### 2.2 测试脚本

位于 `scripts/slam_tests/`，均需 `source install/setup.bash` 后运行：

| 脚本 | 用途 | 示例 |
|---|---|---|
| `topic_hz_collector.py` | 多话题频率/抖动统计 | `python3 scripts/slam_tests/topic_hz_collector.py --duration 60 --out /tmp/hz.csv` |
| `pointcloud_stats.py` | 每帧点云点数/空帧统计 | `python3 scripts/slam_tests/pointcloud_stats.py --duration 60` |
| `imu_stats.py` | IMU 频率、`|acc|`/`|gyro|`、时间戳跳变 | `python3 scripts/slam_tests/imu_stats.py --duration 60` |
| `odom_continuity.py` | 里程计连续性/跳变/静默统计 | `python3 scripts/slam_tests/odom_continuity.py --duration 60 --trans-thresh 0.5` |
| `record_test_bag.sh` | rosbag 录制（含诊断话题） | `bash scripts/slam_tests/record_test_bag.sh /data/slam_test/static_01 60` |

## 3. 通用测试流程

1. 机器人放置在测试区域，确认安全，启动 bringup。
2. 等待 `builder_status == 2 (MAPPING)` 且 `odom_frames_published` 持续增长（通常数秒）。
3. 开始 rosbag 录制；同时另开终端运行对应的统计脚本。
4. 按下列各节的路线操作机器人，记录开始/结束时刻与现场观察。
5. 停止录制，将 bag 路径、脚本输出与关键截图填入 `docs/slam_frontend_test_report_template.md`。
6. 离线回放分析：`ros2 bag play <bag> --clock`（回放时注意不要与实时节点同时运行造成话题冲突）。

## 4. 测试项

### 4.1 静止测试

- **目的**：验证 IMU 初始化后静止状态下里程计是否漂移，检查 IMU 单位与零偏。
- **操作方法**：机器人在平地上静止不动，连续采集 **60 s**（建议覆盖上电后前 30 s 与稳定后 30 s）。
- **应记录数据**：`imu_initialized`、`static_drift_trans`、`static_drift_rot_deg`、`pose_delta_trans`、`pose_delta_rot_deg`、`matched_point_num`、`|acc|` 统计、结束时 `t_wi` 与起始 `t_wi`。
- **结果分析方法**：
  - 若 `|acc|` 远离 9.81 m/s²，先检查 `imu_acc_scale` 与 `/imu/data` 单位，不要调 LIO 噪声参数。
  - 若 `static_drift_trans` 持续增长且远大于配置的 `static_drift_trans_thresh`，结合 RViz 观察局部地图是否有重影/分层，判断是双雷达外参、自车盒裁剪还是 IMU 零偏问题。
  - 静止漂移为 0 也不是充分条件：需要在运动测试中交叉验证。

### 4.2 直线运动测试

- **目的**：验证匀速直线下的位姿连续性与有效匹配点数量。
- **操作方法**：沿空旷直线缓慢前进 5–10 m，再停止；重复 3 次。速度保持平稳，避免急启停。
- **应记录数据**：每段起止时刻与真实位移（卷尺测量）、`pose_delta_trans` 序列、`matched_point_num` 均值与最低值、`cloud_process_hz`、`last_process_cost_ms`、脚本 `odom_continuity.py` 输出。
- **结果分析方法**：
  - 对比里程计累计位移与卷尺实测位移，记录相对误差（不作硬性阈值，只作为前端一致性参考）。
  - 若直线段 `matched_point_num` 长时间为 0 或接近 0，说明局部地图缺乏可匹配结构（如面对空旷墙面/地面），需换场地验证。
  - 处理耗时若显著超过 20 ms 定时器周期，说明工控机算力不足，后续需记录丢帧情况。

### 4.3 原地旋转测试

- **目的**：验证角速度激励下姿态估计与地图匹配的稳定性。
- **操作方法**：机器人原地缓慢旋转，分别完成 ±90°、±180°、360° 各一组，每组之间静止 5 s。
- **应记录数据**：旋转前后航向角、`pose_delta_rot_deg` 峰值、`matched_point_num`、`static_drift_rot_deg`、是否出现 `pose_jump_warning`、RViz 中局部地图是否出现撕裂。
- **结果分析方法**：
  - 旋转中 `pose_delta_rot_deg` 超阈值属预期现象，不应视为定位失效；重点看旋转结束静止后姿态是否收敛。
  - 若旋转后地图明显重影或姿态持续转动不收敛，优先复核双雷达外参和时间同步，而不是直接改 IMU 噪声。

### 4.4 往返运动测试

- **目的**：评估前端在短闭环下的累计漂移（不含 PGO 回环优化，PGO 另测）。
- **操作方法**：从 A 点沿直线走到 B 点（建议 ≥10 m），再原路返回 A 点；到达后静止 10 s。
- **应记录数据**：A/B 点实际坐标标记、出发与返回时的 `lio_odom` 位置、回程时间序列、`pose_delta_trans` 峰值、bag 路径。
- **结果分析方法**：计算返回 A 点时的位置与出发位置之差，作为**前端闭环漂移参考值**；不要把它当作 PGO 闭环误差。若需要 PGO 闭环精度验收，另按 `docs/slam.md` 的 PGO 流程测试。

### 4.5 连续转弯测试

- **目的**：验证持续角速度 + 平动耦合下的前端鲁棒性。
- **操作方法**：以低速沿 S 形路线或直径 ≥5 m 的圆连续转弯 3 圈，全程保持雷达视野内有稳定结构（墙、柱子）。
- **应记录数据**：全程 `matched_point_num` 曲线、`last_match_cost_ms`、`cloud_process_hz`、`failed_sync_count` 增量、`pose_jump_warning` 次数、RViz 局部地图截图。
- **结果分析方法**：关注匹配点数是否在转弯中骤降到 0、处理耗时是否持续升高、`failed_sync_count` 是否增长（多为 IMU/点云断流或算力不足）。出现上述现象先查传感器数据质量，不要先改滤波参数。

### 4.6 长时间运行测试

- **目的**：验证连续运行稳定性与资源泄漏。
- **操作方法**：在固定区域内按预设路线连续运行 **≥30 min**（建议 60 min），期间每 10 min 记录一次状态；使用 `top`/`ros2 topic hz` 观察资源与频率。
- **应记录数据**：每小时 CPU/内存占用、`cloud_process_hz` 与 `odom_publish_hz` 趋势、`failed_sync_count`、`odom_timeout` 出现次数、`pose_timestamp_rollback` 出现次数、`diagnostic_warning_count` 增量、进程是否重启。
- **结果分析方法**：
  - 频率单调下降或内存单调上升提示资源泄漏（注意 `path` 话题累积无上限，长时间运行时 path 消息体积会持续增大）。
  - `odom_timeout` 与同步失败暴增应优先排查网络/UDP 断流与主机负载。
  - 本项只验证稳定性，不评价建图精度。

## 5. 结果汇报原则

- **只记录实际观测值与实测数据**；未做的测试写“未测试”，不得推测或填充示例数值。
- 诊断告警与建图质量是两件事；本报告只完成前端诊断数据采集与初步分析。
- 建图质量验收（闭环误差、地图一致性、定位精度）需在 PGO/Localizer 流程中另行完成。
