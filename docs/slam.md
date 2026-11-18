# “吠云·泰坦”重载轮足机器人 SLAM 系统整体执行方案（低算力版）

## 1. 任务背景与硬件约束
- **目标平台**：吠云·泰坦（重载四轮足巡检机器人，自重约 75kg，负载 60kg+，变电站/园区室内外混合工况）。
- **传感器配置**：双 3D 激光雷达（主雷达+补盲雷达）、双目相机（不进 SLAM 前端以节省算力，留作下游避障/识别）、IMU、轮足运动学里程计、室外 RTK-GPS。
- **算力约束**：低功耗边缘工控机（CPU 算力受限，无强力独显），要求 SLAM 总 CPU 占用率控制在 **35%（单核等效 < 1.5 核）以内**，定位重复精度 **$< 5\text{cm}$**。
- **核心架构**：`双雷达硬同步轻量拼接` + `FAST-LIO2`（核心里程计） + `SC-PGO / GTSAM`（建图后端融合 RTK/回环） + `FAST-LIO-Localization`（巡检低频重定位纠偏）。

---

## 2. 系统目录结构规划
Agent 需在 ROS 工作空间（默认按 ROS 2 Humble 编写，若为 ROS 1 Noetic 需做对应接口转换）下建立如下工程结构：

```text
titan_slam_ws/src/
├── dual_lidar_fusion/          # [独立包] 双3D雷达时间同步、自遮挡裁剪与体素拼接节点（详见配套文档）
├── FAST_LIO/                   # [核心包] 适配多线/混合激光雷达与轮足IMU调优的 FAST-LIO2
├── SC-PGO/                     # [建图包] 后端位姿图优化（接入 RTK-GPS 因子与 Scan Context 回环）
├── FAST_LIO_LOCALIZATION/      # [定位包] 加载先验 PCD 地图的低频全局重定位与漂移修正节点
├── slam_watchdog/              # [安全包] 轮足里程计对比监控与极端退化兜底节点
└── titan_slam_bringup/         # [启动包] 统一的参数文件 (config/) 与启动脚本 (launch/)
```

---

## 3. 核心算法选型与模块职责

### 3.1 建图模式（Mapping Mode - 离线/建图阶段运行）
1. **前端里程计 (`FAST-LIO2`)**：
   - 订阅 `/lidar/merged_points` (`10Hz`) 和 `/imu/data` (`200Hz`)。
   - 利用基于 `ikd-Tree` 的直接法面元配准与迭代误差状态卡尔曼滤波（ESIKF），输出高频里程计 `/Odometry` 与当前帧去畸变点云 `/cloud_registered`。
2. **后端全局图优化 (`SC-PGO` / `GTSAM`)**：
   - 订阅 `/Odometry`、`/cloud_registered` 与 `/gps/fix`。
   - **RTK-GPS 融合策略（门控机制）**：实时检查 `NavSatFix.status.status` 与位置协方差矩阵 `position_covariance[0]`。**仅当** RTK 处于固定解（`Fixed`）且水平协方差 $< 0.03\text{m}^2$ 时，向 GTSAM 因子图添加 GPS 绝对位置因子；进入室内或变压器遮挡区（协方差跳变或降为浮点解）立即断开 GPS 因子，防止多路径效应拉歪地图。
   - **闭环检测**：每移动 $1.5\text{m}$ 或旋转 $15^\circ$ 提取一帧关键帧并计算 `Scan Context` 描述子，检测到回环后执行 ICP 精配准并触发全局图优化，最终保存降采样后的全局地图 `global_map.pcd` 及关键帧描述子库。

### 3.2 巡检定位模式（Localization Mode - 日常自主巡检运行）
- **算力优化策略**：关闭 `SC-PGO` 建图后端与地图写入进程，系统进入纯定位模式。
1. **初始化位姿获取**：
   - **室外启动**：读取 RTK 固定解坐标 + 航向角，转换至地图坐标系作为初始猜测，局部 ICP 收敛。
   - **室内启动**：利用当前帧点云生成 `Scan Context`，在先验描述子库中进行全局搜索粗定位，随后执行 GICP 精匹配（耗时 $< 2\text{s}$）。
2. **高低频双线程融合定位**：
   - **高频线程 (`100Hz`)**：`FAST-LIO2` 持续积分 IMU 与配准局部点云，直接向底层运控输出平滑无跳变的 `odom -> base_link` 变换。
   - **低频纠偏线程 (`1Hz ~ 2Hz`)**：`FAST-LIO-Localization` 截取当前去畸变点云，与内存中常驻的先验地图（`0.2m` 体素降采样，总点数控制在 $30\text{w}$ 以内）进行 **FastGICP / NDT** 配准，计算出全局漂移修正变换 `map -> odom`，消除长距离巡检的累积误差。

### 3.3 轮足里程计看门狗兜底 (`slam_watchdog`)
- 订阅 `/Odometry`（SLAM 输出）与 `/legged_odom`（底盘运动学里程计）。
- 计算滑动窗口（`0.5s`）内的线速度与角速度残差。若 SLAM 出现协方差发散或速度突变（如大雾、暴雨、强镜面反射导致激光退化），在 `100ms` 内切换发布 `/legged_odom` 作为临时位姿输出并上报 `SLAM_DEGRADED` 状态码请求底盘降速。

---

## 4. 低算力与重载轮足专属参数配置 (`titan_fastlio2.yaml`)

Agent 在配置 `FAST-LIO2` 参数文件时，必须严格应用以下针对低算力与轮足冲击的调优参数：

```yaml
common:
  lid_topic: "/lidar/merged_points"
  imu_topic: "/imu/data"
  time_sync_en: false                 # 已在 dual_lidar_fusion 节点完成硬同步对齐，此处设为 false
  time_offset_lidar_to_imu: 0.0       # 若标定出固定延迟则填入对应秒数

preprocess:
  lidar_type: 1                       # 1: 自定义合并后的标准 PointXYZIRT 格式点云
  scan_line: 32                       # 合并后的等效线束（按实际主雷达线束填写）
  timestamp_unit: 3                   # 3: 纳秒 (ns) 或按驱动统一转换为秒/毫秒
  blind: 0.8                          # 盲区过滤半径(m)，配合自遮挡包络彻底滤除泰坦机身与机械臂

mapping:
  acc_cov: 0.8                        # [关键] 重载轮足迈步触地冲击大，调大加速度计噪声协方差（默认0.1），抑制Z轴跳变
  gyr_cov: 0.1                        # 陀螺仪测量噪声协方差
  b_acc_cov: 0.0005                   # 加速度计零偏游走
  b_gyr_cov: 0.0001                   # 陀螺仪零偏游走
  fov_degree: 360                     # 双雷达拼接后实现水平 360 度全向视场
  det_range: 50.0                     # [算力优化] 限制最大有效测距为 50m，丢弃远处稀疏噪点
  extrinsic_est_en: false             # [算力优化] 离线标定好外参后关闭在线外参估计，节省矩阵求逆算力
  extrinsic_T: [0.0, 0.0, 0.0]        # 合并点云已转换至 IMU 坐标系下，此处设为单位变换
  extrinsic_R: [1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0]

publish:
  path_en: false                      # [算力优化] 巡检模式下关闭历史轨迹发布
  scan_publish_en: true
  dense_publish_en: false             # [算力优化] 仅发布降采样后的特征/体素点云，禁止发布稠密全量点云
  scan_bodyframe_pub_en: false

# 编译器与体素降采样核心参数（需同步修改 C++ 源码或 launch 参数）
# point_filter_num: 3                 # 每 3 个点抽样 1 个进入前端
# filter_size_surf: 0.25              # 当前帧表面点体素滤波尺寸 (m)
# filter_size_map: 0.35               # ikd-Tree 全局局部地图体素尺寸 (m)
```

---

## 5. Agent 分步执行清单 (Execution Checklist)

请执行 Agent 严格按以下顺序分阶段完成代码编写、编译与测试验证：

- **Step 1：环境与基础依赖检查**
  - 检查并安装 `PCL (>=1.10)`、`Eigen3`、`GTSAM (>=4.1)`、`Sophus` 及 `livox_ros_driver2` / 对应 3D 雷达驱动。
  - 确认工控机已开启 `chrony` 或 `ptp4l` 时间同步服务，确保双雷达与主机时钟偏差 $< 1\text{ms}$。
- **Step 2：开发并验证双雷达融合模块 (`dual_lidar_fusion`)**
  - 参照配套文档《双 3D 激光雷达时间同步、去畸变与轻量级拼接融合节点开发规范》实现 C++ 节点。
  - **验收标准**：单核 CPU 占用 $< 8\%$，输出话题 `/lidar/merged_points` 稳定 `10Hz`，RViz 中观察墙面、地面无重影，机器人本体及机械臂点云已完全剔除。
- **Step 3：部署与标定调优 `FAST-LIO2`**
  - 拉取 `FAST-LIO2` 源码，修改点云输入解析接口以兼容 `/lidar/merged_points`（支持 `PointXYZIRT` 带逐点时间戳格式）。
  - 写入第 4 节的 `titan_fastlio2.yaml` 参数配置，编译（开启 `-O3 -march=native` 编译优化）。
  - **验收标准**：让机器人在平坦路面跑动并切换步态迈步越障，`FAST-LIO2` 节点单核 CPU 占用 $< 30\%$，里程计输出无发散、无 Z 轴剧烈跳变。
- **Step 4：集成 `SC-PGO` 建图后端与 RTK-GPS 门控**
  - 接入 `SC-PGO`，编写 GPS 协方差门控回调逻辑，确保仅在 RTK `Fixed` 解时插入 GPS 因子。
  - **验收标准**：完成一次室内外混合闭环建图（室外绕行 + 进入室内走廊折返），导出 `global_map.pcd`，闭环起点与终点重合误差 $< 5\text{cm}$。
- **Step 5：集成 `FAST-LIO-Localization` 与 `slam_watchdog`**
  - 配置重定位节点加载 `global_map.pcd`（体素降采样至 `0.2m`），实现基于 `Scan Context` 的自动初始重定位与 `1Hz` FastGICP 漂移修正。
  - 启动 `slam_watchdog` 监控进程，完成全流程联合压测，记录整体 CPU 占用率与关键巡检点位重复停靠精度（目标 $< 5\text{cm}$）。