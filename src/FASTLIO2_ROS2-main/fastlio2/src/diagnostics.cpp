#include "diagnostics.h"

#include <algorithm>
#include <cmath>

namespace
{
constexpr double G = 9.81;
constexpr double RAD2DEG = 180.0 / M_PI;
}

void LioDiagnostics::onImu(double gyro_norm, double acc_norm, double stamp_sec)
{
    imu_frames_.fetch_add(1, std::memory_order_relaxed);
    updateStatic(gyro_norm, acc_norm, stamp_sec);
}

void LioDiagnostics::onCloud()
{
    cloud_frames_.fetch_add(1, std::memory_order_relaxed);
}

void LioDiagnostics::onSyncFailed()
{
    sync_failures_.fetch_add(1, std::memory_order_relaxed);
}

void LioDiagnostics::updateStatic(double gyro_norm, double acc_norm, double stamp_sec)
{
    const bool motion = (gyro_norm > cfg_.static_gyro_thresh) ||
                        (std::fabs(acc_norm - G) > cfg_.static_acc_thresh);
    if (motion)
    {
        static_since_ = -1.0;
        if (imu_static_.exchange(false))
        {
            // 退出静止：清空上一静止窗口的漂移基准
            static_drift_trans_ = 0.0;
            static_drift_rot_deg_ = 0.0;
        }
        return;
    }

    if (static_since_ < 0.0)
    {
        static_since_ = stamp_sec;
        return;
    }
    if (!imu_static_.load() && (stamp_sec - static_since_ >= cfg_.static_hold_sec))
    {
        imu_static_.store(true);
        static_enter_pending_.store(true); // 下一帧位姿输出时重置静止基准
        static_drift_trans_ = 0.0;
        static_drift_rot_deg_ = 0.0;
    }
}

void LioDiagnostics::onProcess(uint8_t status, double msg_stamp_sec, double now_sec, double cost_ms,
                               double match_ms, uint32_t valid_pts, uint32_t matched_pts)
{
    status_ = status;
    last_process_stamp_ = msg_stamp_sec;
    last_process_cost_ms_ = static_cast<float>(cost_ms);
    last_match_cost_ms_ = static_cast<float>(match_ms);
    valid_point_num_ = valid_pts;
    matched_point_num_ = matched_pts;
    processed_.fetch_add(1, std::memory_order_relaxed);

    process_stamps_.push_back(now_sec);
    cost_samples_.emplace_back(now_sec, cost_ms);
    while (!process_stamps_.empty() && now_sec - process_stamps_.front() > cfg_.rate_window_sec)
        process_stamps_.pop_front();
    while (!cost_samples_.empty() && now_sec - cost_samples_.front().first > cfg_.rate_window_sec)
        cost_samples_.pop_front();
}

void LioDiagnostics::onOdom(double msg_stamp_sec, double now_sec,
                            const Eigen::Vector3d &pos, const Eigen::Quaterniond &q_in)
{
    odom_published_.fetch_add(1, std::memory_order_relaxed);
    odom_stamps_.push_back(now_sec);
    while (!odom_stamps_.empty() && now_sec - odom_stamps_.front() > cfg_.rate_window_sec)
        odom_stamps_.pop_front();

    Eigen::Quaterniond q = q_in.normalized();
    timestamp_rollback_ = has_odom_ && (msg_stamp_sec + cfg_.timestamp_rollback_tol < last_odom_stamp_);
    pose_delta_trans_ = 0.0;
    pose_delta_rot_deg_ = 0.0;

    if (has_odom_)
    {
        pose_delta_trans_ = static_cast<float>((pos - last_pos_).norm());
        // abs(dot) 处理 q 与 -q 等价，避免 360° 跳变
        const double dot = std::min(1.0, std::abs(q.dot(last_q_)));
        pose_delta_rot_deg_ = static_cast<float>(2.0 * std::acos(dot) * RAD2DEG);
    }
    pose_jump_warning_ = (pose_delta_trans_ > cfg_.pose_trans_thresh) ||
                         (pose_delta_rot_deg_ > cfg_.pose_rot_thresh_deg);

    if (imu_static_.load())
    {
        if (static_enter_pending_.exchange(false))
        {
            static_ref_pos_ = pos;
            static_ref_q_ = q;
        }
        static_drift_trans_ = std::max(static_drift_trans_, (pos - static_ref_pos_).norm());
        const double dot = std::min(1.0, std::abs(q.dot(static_ref_q_)));
        static_drift_rot_deg_ = std::max(static_drift_rot_deg_, 2.0 * std::acos(dot) * RAD2DEG);
    }
    else
    {
        static_enter_pending_.store(false);
    }

    static_drift_warning_ = imu_static_.load() &&
                            ((static_drift_trans_ > cfg_.static_drift_trans_thresh) ||
                             (static_drift_rot_deg_ > cfg_.static_drift_rot_thresh_deg));

    last_pos_ = pos;
    last_q_ = q;
    last_odom_stamp_ = msg_stamp_sec;
    last_odom_recv_ = now_sec;
    has_odom_ = true;
}

double LioDiagnostics::rateHz(const std::deque<double> &stamps, double now_sec)
{
    if (stamps.size() < 2)
        return 0.0;
    const double span = now_sec - stamps.front();
    return span > 0.0 ? static_cast<double>(stamps.size() - 1) / span : 0.0;
}

interface::msg::LioDiagnostics LioDiagnostics::report(double now_sec)
{
    interface::msg::LioDiagnostics msg;
    msg.builder_status = status_;
    msg.imu_initialized = (status_ > 0);
    msg.imu_frames_received = imu_frames_.load(std::memory_order_relaxed);
    msg.cloud_frames_received = cloud_frames_.load(std::memory_order_relaxed);
    msg.cloud_frames_processed = processed_.load(std::memory_order_relaxed);
    msg.failed_sync_count = sync_failures_.load(std::memory_order_relaxed);
    msg.odom_frames_published = odom_published_.load(std::memory_order_relaxed);
    msg.cloud_process_hz = static_cast<float>(rateHz(process_stamps_, now_sec));
    msg.odom_publish_hz = static_cast<float>(rateHz(odom_stamps_, now_sec));
    msg.last_process_cost_ms = last_process_cost_ms_;
    msg.last_match_cost_ms = last_match_cost_ms_;
    msg.valid_point_num = valid_point_num_;
    msg.matched_point_num = matched_point_num_;
    msg.last_process_stamp = last_process_stamp_;

    if (!cost_samples_.empty())
    {
        double sum = 0.0;
        for (const auto &s : cost_samples_)
            sum += s.second;
        avg_process_cost_ms_ = sum / static_cast<double>(cost_samples_.size());
    }
    msg.avg_process_cost_ms = static_cast<float>(avg_process_cost_ms_);

    msg.pose_delta_trans = pose_delta_trans_;
    msg.pose_delta_rot_deg = pose_delta_rot_deg_;
    msg.pose_timestamp_rollback = timestamp_rollback_;
    msg.last_odom_stamp = last_odom_stamp_;
    msg.odom_silence_sec = has_odom_ ? static_cast<float>(std::max(0.0, now_sec - last_odom_recv_)) : 0.0f;
    msg.odom_timeout = has_odom_ && (msg.odom_silence_sec > static_cast<float>(cfg_.odom_timeout_sec));
    msg.imu_static = imu_static_.load();
    msg.static_drift_trans = static_drift_trans_;
    msg.static_drift_rot_deg = static_drift_rot_deg_;
    msg.pose_jump_warning = pose_jump_warning_;
    msg.static_drift_warning = static_drift_warning_;

    if (msg.pose_jump_warning || msg.static_drift_warning || msg.odom_timeout ||
        msg.pose_timestamp_rollback)
        warning_count_.fetch_add(1, std::memory_order_relaxed);
    msg.diagnostic_warning_count = warning_count_.load(std::memory_order_relaxed);
    return msg;
}
