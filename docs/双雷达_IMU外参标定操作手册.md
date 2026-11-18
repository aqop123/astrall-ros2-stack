# 泰坦双雷达 + 机身 IMU 外参标定操作手册

> 适用范围：吠云-泰坦机器狗，头部雷达 `10.18.0.120`（MSOP 6699）、尾部雷达 `10.18.0.121`（MSOP 6969），两雷达均为 RoboSense Airy；IMU 使用机身 SDK 的 `AstrallImuData`（`/imu/data`，250 Hz），**不是**雷达 6688/6868 端口的内置 IMU。
>
> 标定目标：得到每台雷达到 `imu_link` 的外参，填入 `dual_lidar_fusion/config/dual_lidar_fusion.yaml` 的 `extrinsic_front_to_imu` / `extrinsic_rear_to_imu`。

---

## 0. 本次已经完成的修改

| 文件 | 改动 | 说明 |
| --- | --- | --- |
| `src/calibration/LiDAR_IMU_Init_ROS2-main/src/laserMapping.cpp` | IMU 订阅改为 `rclcpp::SensorDataQoS()` | `/imu/data` 是 best-effort 发布；原代码用默认 reliable 订阅，收不到数据 |
| `src/calibration/LiDAR_IMU_Init_ROS2-main/config/front_airy_li.yaml` | 新增 | 前雷达 `/rslidar_points` + `/imu/data` |
| `src/calibration/LiDAR_IMU_Init_ROS2-main/config/rear_airy_li.yaml` | 新增 | 后雷达 `/rslidar_rear_points` + `/imu/data` |
| 功能包内 Livox 相关代码与配置 | 移除 `livox_ros_driver2` 依赖、`CustomMsg` 回调、AVIA 枚举、Livox 专用 config/launch | 本版本只支持 `sensor_msgs/PointCloud2` 雷达；不再需要 Livox SDK2 |
| `src/calibration/LiDAR_IMU_Init_ROS2-main/result/` | 新建空目录 | 结果文件写在这里；原仓库没有该目录，程序会静默写失败 |

**没有改动** `dual_lidar_fusion.yaml` 里的外参数值：那些值是按 SDK 文档位置推算的初值/占位值，真实值必须由下面的实车标定得到。

**编译验证状态**：已在本机执行 `colcon build --packages-select lidar_imu_init --symlink-install` 并成功（`1 package finished`）。不再需要安装 Livox SDK2。

---

## 1. 工具与数据流

`astrall_ws/src/calibration` 下只有两个项目：

- **`LiDAR_IMU_Init_ROS2-main`**（包名 `lidar_imu_init`）：LI-Init 标定工具。一次只订阅 **1 路雷达点云 + 1 路 IMU**，输出雷达到 IMU 的外参、时间偏移、重力向量和 IMU bias。不需要标定板，但要求运功激励。
- **`livox_ros_driver2-master`**：Livox 雷达驱动目录保留在原处，但 LI-Init 已移除对它的全部依赖；本机标定时无需启动它，也无需安装 Livox SDK2。

数据流：

```
[前雷达 10.18.0.120] ──UDP 6699/7788──┐
                                       ├─> rslidar_sdk_node ─> /rslidar_points        (前, PointCloud2 XYZIRT)
[后雷达 10.18.0.121] ──UDP 6969/7878──┘                     └> /rslidar_rear_points  (后, PointCloud2 XYZIRT)

[机身 SDK IMU] ──UDP 3600 / AstrallImuData──> astrall_sensor_node ─> /imu/data
                                                                  (sensor_msgs/Imu, 250 Hz, frame=imu_link,
                                                                   gyro=rad/s, acc=m/s², best-effort QoS)

标定前雷达：/rslidar_points        + /imu/data ──> LI-Init ─> T_front
标定后雷达：/rslidar_rear_points   + /imu/data ──> LI-Init ─> T_rear
```

`dual_lidar_fusion` 在拿到这两个外参后，把两雷达点云统一变换到 `imu_link`，输出 `/merged_points` 给 FASTLIO2。

**坐标约定（全链路一致）**：

$$p_{imu}=R\cdot p_{lidar}+t$$

其中 `t` 是“雷达原点在 `imu_link` 系中的位置”。LI-Init 输出的 `Translation LiDAR to IMU` 就是 `t`，可直接填进融合节点的 `translation`。

---

## 2. 环境准备

### 2.1 依赖说明

LI-Init 已移除 `livox_ros_driver2` / `CustomMsg` / AVIA 相关代码，只订阅 `sensor_msgs::PointCloud2`。**不需要安装 Livox SDK2**，也不需要启动 Livox 驱动。

### 2.2 编译

```bash
cd /home/cersei/astrall_sdk_1_0_5/astrall_ws
source /opt/ros/humble/setup.bash
PYTHONNOUSERSITE=1 colcon build --packages-select lidar_imu_init --symlink-install
source install/setup.bash
```

`--symlink-install` 让后续改 YAML 不需要重新编译。本版本已在本机验证编译通过。

### 2.3 依赖确认

- Ubuntu 22.04 + ROS2 Humble
- `libceres-dev 2.0.0`（本机已安装）
- PCL / Eigen3 / OpenMP / Python3-dev（随 ROS 环境）

---

## 3. 标定前检查

1. 工控机摄像头/雷达网口配在 `10.18.0.x`（SDK 默认 `10.18.0.200/24`），能 ping 通 `10.18.0.120` 和 `10.18.0.121`。
2. 每个新终端都要执行：

   ```bash
   source /opt/ros/humble/setup.bash
   source /home/cersei/astrall_sdk_1_0_5/astrall_ws/install/setup.bash
   ```

3. **不要启动 `astrall_lidar_udp`**：它和 `rslidar_sdk` 抢同一组 UDP 端口。
4. 确认 IMU 轴向：机器人静止水平放置时，`ros2 topic echo /imu/data` 的 `linear_acceleration` 应有某一轴接近 `±9.8`；若轴向非标准，把修正吸收进外参旋转，不要改 IMU 节点。

---

## 4. 单次标定流程

### 4.1 启动数据源

终端 A（雷达点云）：

```bash
ros2 launch rslidar_sdk start.py
```

终端 B（机身 IMU）：

```bash
ros2 launch astrall_sensor_bridge sensors.launch.py
```

确认：

```bash
ros2 topic hz /rslidar_points        # 约 10 Hz
ros2 topic hz /rslidar_rear_points   # 约 10 Hz
ros2 topic hz /imu/data              # 250 Hz
```

### 4.2 标定前雷达

终端 C：

```bash
ros2 run lidar_imu_init li_init --ros-args \
  --params-file /home/cersei/astrall_sdk_1_0_5/astrall_ws/src/calibration/LiDAR_IMU_Init_ROS2-main/config/front_airy_li.yaml
```

可选 RViz：

```bash
rviz2 -d /home/cersei/astrall_sdk_1_0_5/astrall_ws/src/calibration/LiDAR_IMU_Init_ROS2-main/rviz_cfg/airy.rviz
```

### 4.3 激励动作（关键）

1. 启动后**先静止 5–10 秒**，让算法积累初始地图。终端会打印 `Acceleration norm` 和 `IMU data frequency`，正常应接近 `9.8` 和 `250 Hz`。
2. 然后按终端提示做激励：前后/左右平移，并绕 X/Y/Z 各轴缓慢转动。机器人可搬动、手推或低速运动。
3. 幅度要足够让 IMU 有可观测性，但不要剧烈震动或快速甩动。
4. 等 `data_accum_length`（配置 500）满足、`online_refine_time`（20 s）走完，终端出现 `[Final Result]` 即完成。

### 4.4 标定后雷达

Ctrl+C 结束终端 C，改用后雷达配置：

```bash
ros2 run lidar_imu_init li_init --ros-args \
  --params-file /home/cersei/astrall_sdk_1_0_5/astrall_ws/src/calibration/LiDAR_IMU_Init_ROS2-main/config/rear_airy_li.yaml
```

重复 §4.3 的静止 + 激励流程。

### 4.5 取结果

结果写在**源码目录**（终端打印的 install 路径是错的）：

```
/home/cersei/astrall_sdk_1_0_5/astrall_ws/src/calibration/LiDAR_IMU_Init_ROS2-main/result/Initialization_result.txt
```

关键字段：

- `Rotation LiDAR to IMU (degree)`：滚转/俯仰/偏航，单位度
- `Translation LiDAR to IMU (meter)`：外参平移 `t`
- `Time Lag IMU to LiDAR (second)`：时间偏移
- `Homogeneous Transformation Matrix from LiDAR to IMU`：完整 4×4 矩阵

建议把两次结果抄下来，避免后一次覆盖概念混淆：

| 项目 | 前雷达 | 后雷达 |
| --- | --- | --- |
| Translation (m) | | |
| Rotation (deg) | | |
| Time Lag (s) | | |
| Gyro bias (rad/s) | | |
| Acc bias (m/s²) | | |

---

## 5. 把结果填进融合配置

编辑 `src/dual_lidar_fusion/config/dual_lidar_fusion.yaml`：

```yaml
extrinsic_front_to_imu:
  translation: [x, y, z]          # 前雷达 LI-Init 的 Translation LiDAR to IMU
  quaternion: [qx, qy, qz, qw]    # 见下方转换
extrinsic_rear_to_imu:
  translation: [x, y, z]          # 后雷达结果
  quaternion: [qx, qy, qz, qw]
```

欧拉角转四元数（LI-Init 打印顺序是 roll, pitch, yaw，单位度）：

```bash
python3 - <<'PY'
from scipy.spatial.transform import Rotation as R
roll, pitch, yaw = 0.0, 90.0, 0.0   # 换成实测值
print(R.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_quat())  # [x,y,z,w]
PY
```

若直接用结果文件里的矩阵，更稳妥：

```python
from scipy.spatial.transform import Rotation as R
# T 为 Initialization_result.txt 中的 4x4 矩阵（numpy 数组）
print(R.from_matrix(T[:3, :3]).as_quat())  # [x,y,z,w]
```

两雷达的相对外参（前系到后系）为：

$$T_{front}^{-1}\,T_{rear}$$

**不要改** `FASTLIO2_ROS2-main/fastlio2/config/lio.yaml` 里的 `r_il` / `t_il`：融合节点已把点云变到 `imu_link`，而 FASTLIO2 的 body 系就是 `imu_link`，因此它们恒为单位阵/零向量。

---

## 6. 收敛验证

按正常 SLAM 流程启动：

```bash
# 终端 A/B 继续运行 rslidar_sdk 与 sensors.launch.py
ros2 launch dual_lidar_fusion dual_lidar_fusion.launch.py
ros2 launch fastlio2 lio_launch.py
```

检查项：

1. RViz2 的 Fixed Frame 设为 `imu_link`，加载 `/merged_points`：两雷达扫出的地面应是**同一张水平面**，墙角无重影、无上下错位。
2. `/imu/data` 为 250 Hz；QoS 匹配（best-effort）。
3. 静止时 `ros2 topic echo /lio_odom` 的 z 应稳定；若 z 发散，优先检查 `lio.yaml` 的 `imu_acc_scale` 是否为 `1.0`（本机 IMU 已是 m/s²，不能再乘 10）。
4. 若地面出现双层/错位，说明某一台雷达的旋转或 `t` 仍有误差，回到 §5 微调对应外参；必要时用 `python3 scripts/check_lidar_axis.py` 采一帧核对轴向。

---

## 7. 备选：用融合点云做残差标定

若想一次跑完，可让 LI-Init 订阅 `/merged_points`（需先按初值启动 `dual_lidar_fusion`）+ `/imu/data`：

```bash
ros2 run lidar_imu_init li_init --ros-args --params-file <把 lid_topic 改为 /merged_points 的 yaml>
```

此时 LI-Init 的结果是“融合后点云系 → IMU”的**公共残差** $T_{res}$（理想为单位阵/零），需要合成回原外参：

$$T_{new}=T_{res}\cdot T_{old}$$

前后两台都要做同样的左乘。限制：它只能修正公共误差，**校不正两雷达之间的相对错位**。因此推荐以 §4 的分别标定为主，本方法只做收尾验证。

---

## 8. 参数速查与故障排查

| 现象 | 原因 / 处理 |
| --- | --- |
| 编译失败 | 确认 ROS Humble / Ceres / PCL 环境；本版本已不再依赖 Livox SDK2 |
| 启动报 `package lidar_imu_init2 not found` | 该 launch 是坏的；用 `ros2 run lidar_imu_init li_init --ros-args --params-file ...` |
| 终端没有 IMU 数据 / `Acceleration norm` 为 0 | IMU QoS 不匹配；确认 `laserMapping.cpp` 已用 `rclcpp::SensorDataQoS()` |
| 打印 `Error LiDAR Type` 或无有效点 | `lidar_type` 必须是 `7`（AIRY）；本机点云是 XYZIRT + 绝对时间戳 |
| `mean_acc_norm` 打印异常 | 必须是 `9.805`（机身 IMU 为 m/s²，静止时加速度模长约 9.8） |
| `Initialization_result.txt` 不存在/为空 | 未创建 `result/` 目录，或看错了路径（应在源码目录） |
| 点云收不到 | `rslidar_sdk start.py` 没起，或误启了 `astrall_lidar_udp` 抢占端口 |
| 时间偏移很大 | 检查两端是否同一主机时钟；`/imu/data` 用的是本机接收时间 |
| FASTLIO2 的 z 发散 | `imu_acc_scale` 必须为 `1.0`；不要二次变换外参 |
| 地面双层/重影 | 外参旋转或 `t` 不准，回到 §5 微调对应雷达 |

关键参数含义：

- `cut_frame_num: 3` + `orig_odom_freq: 10`：把 10 Hz 原始帧切成 30 Hz 子帧，便于里程计收敛。
- `data_accum_length: 500`：判断数据是否够初始化；太小结果质量差。
- `online_refine_time: 20.0`：初始化完成后再用 FASTLIO2 精修 20 秒。
- `filter_size_surf` / `filter_size_map`：室内 0.05–0.15 / 0.15–0.25；室外可放大到 0.5。

---

## 9. 命令速查卡

```bash
# 环境
cd /home/cersei/astrall_sdk_1_0_5/astrall_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

# 数据源
ros2 launch rslidar_sdk start.py
ros2 launch astrall_sensor_bridge sensors.launch.py

# 前雷达标定
ros2 run lidar_imu_init li_init --ros-args --params-file \
  src/calibration/LiDAR_IMU_Init_ROS2-main/config/front_airy_li.yaml

# 后雷达标定
ros2 run lidar_imu_init li_init --ros-args --params-file \
  src/calibration/LiDAR_IMU_Init_ROS2-main/config/rear_airy_li.yaml

# 结果
cat src/calibration/LiDAR_IMU_Init_ROS2-main/result/Initialization_result.txt

# 上线验证
ros2 launch dual_lidar_fusion dual_lidar_fusion.launch.py
ros2 launch fastlio2 lio_launch.py
```
