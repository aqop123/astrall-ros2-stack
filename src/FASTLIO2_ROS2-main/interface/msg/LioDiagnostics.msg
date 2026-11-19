# FAST-LIO2 前端诊断快照（仅用于测试诊断，不参与建图算法）
# 话题：/fastlio2/lio_diagnostics ，默认 2 Hz 发布，可被 rosbag 直接录制

std_msgs/Header header

# ---------- 任务1：处理状态统计 ----------
uint8 builder_status              # 0=IMU_INIT, 1=MAP_INIT, 2=MAPPING
bool imu_initialized              # 已完成 IMU 初始化（status != IMU_INIT）
uint64 imu_frames_received        # 累计接收 IMU 帧数
uint64 cloud_frames_received      # 累计接收并成功入队的点云帧数
uint64 cloud_frames_processed     # 累计成功处理的点云帧数
uint64 failed_sync_count          # 累计同步失败（无足够 IMU / 无点云）次数
uint64 odom_frames_published      # 累计实际发布的里程计帧数
float32 cloud_process_hz          # 最近窗口点云处理频率
float32 odom_publish_hz           # 最近窗口里程计实际发布频率
float32 last_process_cost_ms      # 最近一帧 m_builder->process() 耗时
float32 avg_process_cost_ms       # 最近窗口平均处理耗时
float32 last_match_cost_ms        # 最近一帧地图匹配（最近邻搜索+平面拟合）累计耗时
uint32 valid_point_num            # 最近一帧下采样后送入匹配的点数
uint32 matched_point_num          # 最近一帧有效匹配点数（共面近邻通过点数）
float64 last_process_stamp        # 最近一次成功处理的 ROS 时间（秒）

# ---------- 任务2：位姿稳定性诊断（仅告警，不代表定位失效） ----------
float32 pose_delta_trans          # 相邻里程计之间的位置变化 (m)
float32 pose_delta_rot_deg        # 相邻里程计之间的角度变化 (deg)
bool pose_timestamp_rollback      # 最近一帧是否出现时间戳倒退
float64 last_odom_stamp           # 最近一次里程计的 ROS 时间（秒）
float32 odom_silence_sec          # 距最近一次里程计输出的时长 (s)
bool odom_timeout                 # 超过配置阈值没有新里程计
bool imu_static                   # 依据 IMU 阈值判定的静止状态
float32 static_drift_trans        # 当前静止窗口内最大位置漂移 (m)
float32 static_drift_rot_deg      # 当前静止窗口内最大姿态漂移 (deg)
bool pose_jump_warning            # 相邻位姿变化超过阈值
bool static_drift_warning         # 静止期漂移超过阈值
uint64 diagnostic_warning_count   # 累计诊断告警帧数（每次发布最多 +1）
