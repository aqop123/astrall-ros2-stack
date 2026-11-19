# FAST-LIO2 双雷达前端实机测试报告

> 填写说明：**删除“示例”占位、只填实测数据；未做的项目写“未测试”**。
> 本报告只记录前端诊断测试，不代表建图质量验收通过。

## 1. 基本信息

| 项目 | 内容 |
|---|---|
| 测试日期 | |
| 操作者 | |
| 机器人 / 编号 | 吠云·泰坦 / |
| 场地 / 环境 | |
| ROS 版本 | Ubuntu 22.04 / ROS 2 Humble |
| 代码版本（git commit） | |
| 启动命令 | `ros2 launch astrall_bringup fastlio.launch.py` |
| 实际使用的 lio 配置路径 | |
| 固件 / 驱动版本 | |

## 2. 配置快照（测试时实际生效值）

| 参数 | 值 | 备注 |
|---|---|---|
| `imu_topic` / `lidar_topic` | / | |
| `imu_acc_scale` | | |
| `lidar_time_unit` / `lidar_time_absolute` | / | |
| `lidar_filter_num` / `lidar_min_range` / `lidar_max_range` | / / | |
| `scan_resolution` / `map_resolution` | / | |
| `na` / `ng` / `nba` / `nbg` | / / / | **未修改** |
| 双雷达外参（fusion 配置） | | **未修改**，填实际生效值 |
| `diagnostics.enable` / `publish_hz` | / | |
| `diagnostics.static_*_thresh` | / / | |
| `diagnostics.pose_*_thresh` | / | |
| `diagnostics.odom_timeout_sec` | | |

## 3. 诊断话题与脚本可用性确认

| 检查项 | 结果（实测） |
|---|---|
| `ros2 topic hz /fastlio2/lio_diagnostics` | |
| `builder_status` 是否达到 2 (MAPPING) | |
| `topic_hz_collector.py` 输出路径 | |
| `pointcloud_stats.py` 输出路径 | |
| `imu_stats.py` 输出路径 | |
| `odom_continuity.py` 输出路径 | |

## 4. 测试记录

### 4.1 静止测试

| 项目 | 实测 |
|---|---|
| 录制时长 / bag 路径 | |
| `|acc|` min/mean/max（m/s²） | |
| `|gyro|` min/mean/max（rad/s） | |
| 起始 / 结束位置 (x,y,z) | |
| `static_drift_trans` 最大值 (m) | |
| `static_drift_rot_deg` 最大值 (deg) | |
| `matched_point_num` 均值 / 最低 | |
| 现场观察 / 分析 | |

### 4.2 直线运动测试

| 项目 | 第 1 次 | 第 2 次 | 第 3 次 |
|---|---|---|---|
| 实测位移（卷尺, m） | | | |
| 里程计累计位移 (m) | | | |
| 相对误差 | | | |
| `matched_point_num` 均值 / 最低 | / | / | / |
| `cloud_process_hz` 均值 | | | |
| `last_process_cost_ms` 最大值 | | | |
| bag 路径 | | | |
| 现场观察 / 分析 | | | |

### 4.3 原地旋转测试

| 项目 | +90° | -90° | +180° | -180° | 360° |
|---|---|---|---|---|---|
| 旋转前 / 后航向角 | | | | | |
| `pose_delta_rot_deg` 峰值 | | | | | |
| `pose_jump_warning` 次数 | | | | | |
| 旋转结束静止后是否收敛 | | | | | |
| 局部地图是否重影/撕裂 | | | | | |
| bag 路径 | | | | | |

### 4.4 往返运动测试

| 项目 | 实测 |
|---|---|
| A 点标记 / 出发位置 | |
| B 点标记 / 到达位置 | |
| 返回 A 点时的里程计位置 | |
| 前端闭环位置差 (m) | |
| `pose_delta_trans` 峰值 (m) | |
| bag 路径 | |
| 现场观察 / 分析 | |

### 4.5 连续转弯测试

| 项目 | 实测 |
|---|---|
| 路线（S 形 / 圆）与圈数 | |
| `matched_point_num` 最低值 | |
| `last_match_cost_ms` 最大 / 均值 | |
| `failed_sync_count` 增量 | |
| `pose_jump_warning` 次数 | |
| 局部地图截图路径 | |
| bag 路径 | |
| 现场观察 / 分析 | |

### 4.6 长时间运行测试

| 项目 | 0 min | 10 min | 20 min | 30 min | 60 min |
|---|---|---|---|---|---|
| CPU 占用 (%) | | | | | |
| 内存占用 (MB) | | | | | |
| `cloud_process_hz` | | | | | |
| `odom_publish_hz` | | | | | |
| `failed_sync_count` | | | | | |
| `odom_timeout` 出现次数 | | | | | |
| `pose_timestamp_rollback` 出现次数 | | | | | |
| 进程是否重启 | | | | | |

## 5. 汇总

| 维度 | 实测结论（基于数据） | 待验证项 |
|---|---|---|
| IMU 初始化与数据单位 | | |
| 静止漂移 | | |
| 运动连续性 | | |
| 地图匹配点数量 | | |
| 双雷达融合地图一致性 | | |
| 长时间稳定性 | | |

## 6. 遗留问题与下一步

- 未测试项：
- 发现的异常及证据（bag 路径 + 截图 + 时间区间）：
- 建议的下一步（例如 PGO 闭环测试 / 外参复核 / 算力优化）：
- 本报告是否可支撑建图质量验收：**否**（诊断工具准备完成 ≠ 建图质量通过验收）

记录人：__________   审核人：__________   日期：__________
