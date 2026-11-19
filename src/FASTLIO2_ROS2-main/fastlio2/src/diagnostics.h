#pragma once

#include <atomic>
#include <deque>
#include <utility>
#include <Eigen/Eigen>
#include <interface/msg/lio_diagnostics.hpp>

// 所有字段都有合理默认值；YAML 中缺少 diagnostics 段时使用默认值。
struct DiagnosticsConfig
{
    bool enable = true;
    double publish_hz = 2.0;            // 诊断快照发布频率
    double warn_log_hz = 0.5;           // 告警日志最大频率
    double rate_window_sec = 5.0;       // 频率/耗时统计滑动窗口

    // IMU 静止判定：|gyro| < 阈值 且 ||acc| - 9.81| < 阈值，并持续 hold 秒
    double static_gyro_thresh = 0.05;   // rad/s
    double static_acc_thresh = 0.15;    // m/s^2（加速度已按 imu_acc_scale 缩放）
    double static_hold_sec = 0.5;       // s

    // 位姿稳定性阈值（只作诊断信号，不判定定位失效）
    double pose_trans_thresh = 0.5;          // m
    double pose_rot_thresh_deg = 10.0;       // deg
    double static_drift_trans_thresh = 0.05; // m
    double static_drift_rot_thresh_deg = 2.0;// deg
    double timestamp_rollback_tol = 0.001;   // s
    double odom_timeout_sec = 2.0;           // s
};

// 纯数据统计容器：不访问 ROS 话题、不调用算法，可被多线程回调安全写入。
// imuCB 写入的计数器/静止标志使用原子量；其余字段只在 timer 回调组内访问，
// 与 report()/publishDiagnostics() 共用同一个 MutuallyExclusive 回调组，无需加锁。
class LioDiagnostics
{
public:
    explicit LioDiagnostics(DiagnosticsConfig cfg) : cfg_(cfg) {}

    void onImu(double gyro_norm, double acc_norm, double stamp_sec);
    void onCloud();
    void onSyncFailed();
    // msg_stamp_sec: 数据自身时间戳（用于时间戳倒退检测与对外展示）
    // now_sec: ROS 当前时间（用于频率窗口、静默超时，避免受数据源时钟跳变影响）
    void onProcess(uint8_t status, double msg_stamp_sec, double now_sec, double cost_ms,
                   double match_ms, uint32_t valid_pts, uint32_t matched_pts);
    void onOdom(double msg_stamp_sec, double now_sec,
                const Eigen::Vector3d &pos, const Eigen::Quaterniond &q);

    interface::msg::LioDiagnostics report(double now_sec);

    const DiagnosticsConfig &config() const { return cfg_; }

private:
    void updateStatic(double gyro_norm, double acc_norm, double stamp_sec);
    static double rateHz(const std::deque<double> &stamps, double now_sec);

    DiagnosticsConfig cfg_;

    std::atomic<uint64_t> imu_frames_{0};
    std::atomic<uint64_t> cloud_frames_{0};
    std::atomic<uint64_t> sync_failures_{0};
    std::atomic<uint64_t> processed_{0};
    std::atomic<uint64_t> odom_published_{0};
    std::atomic<uint64_t> warning_count_{0};
    std::atomic<bool> imu_static_{false};
    std::atomic<bool> static_enter_pending_{false};

    double static_since_ = -1.0; // 仅 imuCB 访问

    std::deque<double> process_stamps_;
    std::deque<double> odom_stamps_;
    std::deque<std::pair<double, double>> cost_samples_;

    uint8_t status_ = 0;
    double last_process_stamp_ = 0.0;
    double last_process_cost_ms_ = 0.0;
    double avg_process_cost_ms_ = 0.0;
    double last_match_cost_ms_ = 0.0;
    uint32_t valid_point_num_ = 0;
    uint32_t matched_point_num_ = 0;

    bool has_odom_ = false;
    double last_odom_stamp_ = 0.0;
    double last_odom_recv_ = 0.0;
    Eigen::Vector3d last_pos_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond last_q_ = Eigen::Quaterniond::Identity();
    double pose_delta_trans_ = 0.0;
    double pose_delta_rot_deg_ = 0.0;
    bool timestamp_rollback_ = false;
    bool pose_jump_warning_ = false;
    bool static_drift_warning_ = false;

    Eigen::Vector3d static_ref_pos_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond static_ref_q_ = Eigen::Quaterniond::Identity();
    double static_drift_trans_ = 0.0;
    double static_drift_rot_deg_ = 0.0;
};
