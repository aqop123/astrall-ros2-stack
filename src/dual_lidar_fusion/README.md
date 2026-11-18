# dual_lidar_fusion

双 3D 激光雷达（速腾 RoboSense RSAIRY）轻量级拼接融合节点，输出给 `fastlio2` 做 LIO。

技术方案与验收步骤见 `docs/3d_robosense.md`（§3 节点设计 / §4 参数 / §6 联调 / §7 外参标定）。

## 处理流水线（去畸变 100% 由 fastlio2 完成）

1. **双雷达帧同步**：`message_filters` ApproximateTime，容差 `sync_slop_sec`（默认 0.03s，有效上限 < 0.05s）。
2. **外参变换 + 自遮挡裁剪**：单次循环内将每点变换到 `imu_link`（逐点 `timestamp` 原样保留，禁止预去畸变/清零），并剔除机身包围盒内的自遮挡点。
3. **体素降采样与发布**：哈希表体素滤波（`voxel_leaf_size`，每体素保留首个落入点），发布到 `/merged_points`，**RELIABLE QoS**。

## 话题

| 方向 | 话题 | 说明 |
|---|---|---|
| 订阅 | `/rslidar_points` | 前雷达（rslidar_sdk，XYZIRT 紧凑布局 point_step=26） |
| 订阅 | `/rslidar_rear_points` | 后雷达（同上） |
| 发布 | `/merged_points` | 合并点云，frame_id `imu_link`，`stamp = t_base`（两帧较早者） |

输入布局校验（§3.1）：`point_step == 26`、`timestamp` 为 FLOAT64 绝对秒、`x/y/z` FLOAT32、`intensity` FLOAT32、`ring` UINT16；不满足则报错退出（典型事故：rslidar_sdk 忘改 `POINT_TYPE=XYZIRT`）。

## 降级（§3.3）

后雷达断流超过 `rear_timeout_sec`（默认 2.0s）后自动降级为 front 单雷达输出并按 1Hz 告警，恢复后自动回到双雷达合并。

## 运行

```bash
ros2 launch dual_lidar_fusion dual_lidar_fusion.launch.py            # 默认参数
ros2 launch dual_lidar_fusion dual_lidar_fusion.launch.py params_file:=/path/to/dual_lidar_fusion.yaml
```

注意：运行本方案时**停用 `lidar_udp_node`**（与 `rslidar_sdk` 绑定相同 UDP 端口，不能共存，§1）。

## 外参

配置中的 `extrinsic_front_to_imu` / `extrinsic_rear_to_imu` 数值均为**占位示例**，必须按 `docs/3d_robosense.md` §7 流程实测获取（Step1 卷尺粗测 → Step2 RViz 精调），调好后固化进 `config/dual_lidar_fusion.yaml`。
