#include <mutex>
#include <vector>
#include <queue>
#include <memory>
#include <iostream>
#include <chrono>
#include <algorithm>
#include <type_traits>
// #include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "utils.h"
#include "diagnostics.h"
#include "map_builder/commons.h"
#include "map_builder/map_builder.h"

#include <pcl_conversions/pcl_conversions.h>
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

namespace
{
// 诊断阈值容错读取：缺少 diagnostics 段或个别字段时使用代码内合理默认值
DiagnosticsConfig loadDiagnosticsConfig(const YAML::Node &root)
{
    DiagnosticsConfig cfg;
    if (!root || !root["diagnostics"])
        return cfg;
    const YAML::Node d = root["diagnostics"];
    auto get = [&d](const char *key, auto &out)
    {
        if (d[key])
            out = d[key].as<std::decay_t<decltype(out)>>();
    };
    get("enable", cfg.enable);
    get("publish_hz", cfg.publish_hz);
    get("warn_log_hz", cfg.warn_log_hz);
    get("rate_window_sec", cfg.rate_window_sec);
    get("static_gyro_thresh", cfg.static_gyro_thresh);
    get("static_acc_thresh", cfg.static_acc_thresh);
    get("static_hold_sec", cfg.static_hold_sec);
    get("pose_trans_thresh", cfg.pose_trans_thresh);
    get("pose_rot_thresh_deg", cfg.pose_rot_thresh_deg);
    get("static_drift_trans_thresh", cfg.static_drift_trans_thresh);
    get("static_drift_rot_thresh_deg", cfg.static_drift_rot_thresh_deg);
    get("timestamp_rollback_tol", cfg.timestamp_rollback_tol);
    get("odom_timeout_sec", cfg.odom_timeout_sec);
    return cfg;
}
} // namespace

struct NodeConfig
{
    std::string imu_topic = "/livox/imu";
    std::string lidar_topic = "/merged_points";
    std::string body_frame = "body";
    std::string world_frame = "lidar";
    bool print_time_cost = false;
    // 单点时间字段(time/timestamp)的单位: sec / ms / us / ns, 速腾驱动默认秒级绝对时间戳
    std::string lidar_time_unit = "sec";
    // true: 单点时间为绝对时间戳; false: 已是相对扫描起始的偏移
    bool lidar_time_absolute = true;
    // IMU 加速度缩放系数(/imu/data 为 m/s² 时应取 1.0, 见 imuCB 注释)
    double imu_acc_scale = 1.0;
};
struct StateData
{
    bool lidar_pushed = false;
    std::mutex imu_mutex;
    std::mutex lidar_mutex;
    double last_lidar_time = -1.0;
    double last_imu_time = -1.0;
    std::deque<IMUData> imu_buffer;
    std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> lidar_buffer;
    nav_msgs::msg::Path path;
};

class LIONode : public rclcpp::Node
{
public:
    LIONode() : Node("lio_node")
    {
        RCLCPP_INFO(this->get_logger(), "LIO Node Started");
        loadParameters();

        // 多线程执行器下, 为 IMU/雷达/定时器分配独立回调组, 使其可并行执行, 避免重回调互相阻塞
        m_imu_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        m_lidar_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        m_timer_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions imu_sub_options;
        imu_sub_options.callback_group = m_imu_cb_group;
        // IMU 订阅必须用 SensorDataQoS(best-effort): 发布端 astrall_sensor_node 为 SensorDataQoS,
        // best-effort pub + reliable sub 在 DDS 层不匹配, 将收不到任何 IMU 数据 (docs/3d_robosense.md §5)
        m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(m_node_config.imu_topic, rclcpp::SensorDataQoS(), std::bind(&LIONode::imuCB, this, std::placeholders::_1), imu_sub_options);

        rclcpp::SubscriptionOptions lidar_sub_options;
        lidar_sub_options.callback_group = m_lidar_cb_group;
        m_lidar_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(m_node_config.lidar_topic, 10, std::bind(&LIONode::lidarCB, this, std::placeholders::_1), lidar_sub_options);

        m_body_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("body_cloud", 10000);
        m_world_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("world_cloud", 10000);
        m_path_pub = this->create_publisher<nav_msgs::msg::Path>("lio_path", 10000);
        m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("lio_odom", 10000);
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        m_state_data.path.poses.clear();
        m_state_data.path.header.frame_id = m_node_config.world_frame;

        m_kf = std::make_shared<IESKF>();
        m_builder = std::make_shared<MapBuilder>(m_builder_config, m_kf);
        m_timer = this->create_wall_timer(20ms, std::bind(&LIONode::timerCB, this), m_timer_cb_group);

        m_diagnostics = std::make_shared<LioDiagnostics>(m_diag_config);
        m_warn_throttle_ms = static_cast<uint64_t>(std::max(1.0, 1000.0 / m_diag_config.warn_log_hz));
        if (m_diag_config.enable)
        {
            m_diagnostics_pub = this->create_publisher<interface::msg::LioDiagnostics>("lio_diagnostics", 10);
            auto period = std::chrono::duration<double>(std::max(0.1, 1.0 / m_diag_config.publish_hz));
            // 与主 timerCB 同一 MutuallyExclusive 回调组：诊断数据无需加锁，且不增加主线程并发压力
            m_diag_timer = this->create_wall_timer(period, std::bind(&LIONode::publishDiagnostics, this), m_timer_cb_group);
            RCLCPP_INFO(this->get_logger(), "Diagnostics enabled: /fastlio2/lio_diagnostics @ %.1f Hz", m_diag_config.publish_hz);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Diagnostics disabled by config");
        }
    }

    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);

        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

        m_diag_config = loadDiagnosticsConfig(config);

        m_node_config.imu_topic = config["imu_topic"].as<std::string>();
        m_node_config.lidar_topic = config["lidar_topic"].as<std::string>();
        m_node_config.body_frame = config["body_frame"].as<std::string>();
        m_node_config.world_frame = config["world_frame"].as<std::string>();
        m_node_config.print_time_cost = config["print_time_cost"].as<bool>();

        // 新增配置项做容错读取, 兼容旧配置文件
        if (config["lidar_time_unit"])
            m_node_config.lidar_time_unit = config["lidar_time_unit"].as<std::string>();
        if (config["lidar_time_absolute"])
            m_node_config.lidar_time_absolute = config["lidar_time_absolute"].as<bool>();
        if (config["imu_acc_scale"])
            m_node_config.imu_acc_scale = config["imu_acc_scale"].as<double>();
        RCLCPP_INFO(this->get_logger(), "IMU acceleration scale: %.3f (requires /imu/data |acc|=%.3f m/s^2 when static)",
                    m_node_config.imu_acc_scale, 9.81 / m_node_config.imu_acc_scale);

        m_time_unit = Utils::parseTimeUnit(m_node_config.lidar_time_unit);
        RCLCPP_INFO(this->get_logger(), "Lidar point time field: unit='%s', absolute=%d (converted to ms offset relative to scan start)",
                    m_node_config.lidar_time_unit.c_str(), m_node_config.lidar_time_absolute ? 1 : 0);

        m_builder_config.lidar_filter_num = config["lidar_filter_num"].as<int>();
        m_builder_config.lidar_min_range = config["lidar_min_range"].as<double>();
        m_builder_config.lidar_max_range = config["lidar_max_range"].as<double>();
        m_builder_config.scan_resolution = config["scan_resolution"].as<double>();
        m_builder_config.map_resolution = config["map_resolution"].as<double>();
        m_builder_config.cube_len = config["cube_len"].as<double>();
        m_builder_config.det_range = config["det_range"].as<double>();
        m_builder_config.move_thresh = config["move_thresh"].as<double>();
        m_builder_config.na = config["na"].as<double>();
        m_builder_config.ng = config["ng"].as<double>();
        m_builder_config.nba = config["nba"].as<double>();
        m_builder_config.nbg = config["nbg"].as<double>();

        m_builder_config.imu_init_num = config["imu_init_num"].as<int>();
        m_builder_config.near_search_num = config["near_search_num"].as<int>();
        m_builder_config.ieskf_max_iter = config["ieskf_max_iter"].as<int>();
        m_builder_config.gravity_align = config["gravity_align"].as<bool>();
        m_builder_config.esti_il = config["esti_il"].as<bool>();
        std::vector<double> t_il_vec = config["t_il"].as<std::vector<double>>();
        std::vector<double> r_il_vec = config["r_il"].as<std::vector<double>>();
        m_builder_config.t_il << t_il_vec[0], t_il_vec[1], t_il_vec[2];
        m_builder_config.r_il << r_il_vec[0], r_il_vec[1], r_il_vec[2], r_il_vec[3], r_il_vec[4], r_il_vec[5], r_il_vec[6], r_il_vec[7], r_il_vec[8];
        m_builder_config.lidar_cov_inv = config["lidar_cov_inv"].as<double>();
    }

    void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);
        double timestamp = Utils::getSec(msg->header);
        if (timestamp < m_state_data.last_imu_time)
        {
            RCLCPP_WARN(this->get_logger(), "IMU Message is out of order");
            std::deque<IMUData>().swap(m_state_data.imu_buffer);
        }
        // m_node_config.imu_acc_scale: /imu/data 单位为 m/s²(astrall sdk interface.h: accelerometer "m/(s*s)",
        // astrall_sensor_node 原样透传), fastlio2 内部按 m/s² 建模, 故应为 1.0。
        // 历史代码硬编码 *10.0 仅适用于 0.1 m/s² 单位的旧数据源, 会使静止等效重力变为 98.1 m/s²,
        // IESKF 重力项失配 → 位置发散 (docs/3d_robosense.md §5)
        m_state_data.imu_buffer.emplace_back(V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z) * m_node_config.imu_acc_scale,
                                             V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                                             timestamp);
        m_state_data.last_imu_time = timestamp;
        if (m_diagnostics)
        {
            const IMUData &imu = m_state_data.imu_buffer.back();
            m_diagnostics->onImu(imu.gyro.norm(), imu.acc.norm(), timestamp);
        }
    }
    void lidarCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        double scan_start_time = 0.0;
        CloudType::Ptr cloud = Utils::cloud2PCL(msg, m_builder_config.lidar_filter_num, m_builder_config.lidar_min_range,
                                                m_builder_config.lidar_max_range, m_time_unit, m_node_config.lidar_time_absolute,
                                                scan_start_time);
        if (cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Receive empty/invalid point cloud on %s", m_node_config.lidar_topic.c_str());
            return;
        }
        std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
        double timestamp = scan_start_time;
        if (timestamp < m_state_data.last_lidar_time)
        {
            RCLCPP_WARN(this->get_logger(), "Lidar Message is out of order");
            std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>>().swap(m_state_data.lidar_buffer);
            m_state_data.lidar_pushed = false;  // buffer 已清空, 丢弃未完成的同步包
        }
        m_state_data.lidar_buffer.emplace_back(timestamp, cloud);
        m_state_data.last_lidar_time = timestamp;
        if (m_diagnostics)
            m_diagnostics->onCloud();
    }

    bool syncPackage()
    {
        // 多线程执行器下 timerCB 与 imuCB/lidarCB 并发, 访问缓冲区必须加锁(锁顺序: lidar -> imu, 与其他回调无交叉, 无死锁)
        bool need_sort = false;
        {
            std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
            if (m_state_data.lidar_buffer.empty())
                return false;
            if (!m_state_data.lidar_pushed)
            {
                m_package.cloud = m_state_data.lidar_buffer.front().second;
                m_package.cloud_start_time = m_state_data.lidar_buffer.front().first;
                m_state_data.lidar_pushed = true;
                need_sort = true;
            }
        }
        if (need_sort)
        {
            // m_package.cloud 与 buffer 共享所有权, 但 lidar 线程入队后不再修改, 排序无需持锁
            std::sort(m_package.cloud->points.begin(), m_package.cloud->points.end(), [](PointType &p1, PointType &p2)
                      { return p1.curvature < p2.curvature; });
            m_package.cloud_end_time = m_package.cloud_start_time + m_package.cloud->points.back().curvature / 1000.0;
        }
        {
            std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);
            if (m_state_data.imu_buffer.empty() || m_state_data.last_imu_time < m_package.cloud_end_time)
                return false;

            Vec<IMUData>().swap(m_package.imus);
            while (!m_state_data.imu_buffer.empty() && m_state_data.imu_buffer.front().time < m_package.cloud_end_time)
            {
                m_package.imus.emplace_back(m_state_data.imu_buffer.front());
                m_state_data.imu_buffer.pop_front();
            }
        }
        {
            std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
            m_state_data.lidar_buffer.pop_front();
            m_state_data.lidar_pushed = false;
        }
        return true;
    }

    void publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, CloudType::Ptr cloud, std::string frame_id, const double &time)
    {
        if (pub->get_subscription_count() <= 0)
            return;
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = frame_id;
        cloud_msg.header.stamp = Utils::getTime(time);
        pub->publish(cloud_msg);
    }

    void publishOdometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub, std::string frame_id, std::string child_frame, const double &time)
    {
        if (odom_pub->get_subscription_count() <= 0)
            return;
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = frame_id;
        odom.header.stamp = Utils::getTime(time);
        odom.child_frame_id = child_frame;
        odom.pose.pose.position.x = m_kf->x().t_wi.x();
        odom.pose.pose.position.y = m_kf->x().t_wi.y();
        odom.pose.pose.position.z = m_kf->x().t_wi.z();
        Eigen::Quaterniond q(m_kf->x().r_wi);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        V3D vel = m_kf->x().r_wi.transpose() * m_kf->x().v;
        odom.twist.twist.linear.x = vel.x();
        odom.twist.twist.linear.y = vel.y();
        odom.twist.twist.linear.z = vel.z();
        odom_pub->publish(odom);
        if (m_diagnostics)
        {
            m_diagnostics->onOdom(time, this->now().seconds(), m_kf->x().t_wi,
                                  Eigen::Quaterniond(m_kf->x().r_wi));
        }
    }

    void publishPath(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub, std::string frame_id, const double &time)
    {
        if (path_pub->get_subscription_count() <= 0)
            return;
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frame_id;
        pose.header.stamp = Utils::getTime(time);
        pose.pose.position.x = m_kf->x().t_wi.x();
        pose.pose.position.y = m_kf->x().t_wi.y();
        pose.pose.position.z = m_kf->x().t_wi.z();
        Eigen::Quaterniond q(m_kf->x().r_wi);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        m_state_data.path.poses.push_back(pose);
        path_pub->publish(m_state_data.path);
    }

    void broadCastTF(std::shared_ptr<tf2_ros::TransformBroadcaster> broad_caster, std::string frame_id, std::string child_frame, const double &time)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = frame_id;
        transformStamped.child_frame_id = child_frame;
        transformStamped.header.stamp = Utils::getTime(time);
        Eigen::Quaterniond q(m_kf->x().r_wi);
        V3D t = m_kf->x().t_wi;
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        broad_caster->sendTransform(transformStamped);
    }

    void timerCB()
    {
        if (!syncPackage())
        {
            if (m_diagnostics)
                m_diagnostics->onSyncFailed();
            return;
        }
        auto t1 = std::chrono::high_resolution_clock::now();
        m_builder->process(m_package);
        auto t2 = std::chrono::high_resolution_clock::now();
        double cost_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();

        if (m_node_config.print_time_cost)
        {
            RCLCPP_WARN(this->get_logger(), "Time cost: %.2f ms", cost_ms);
        }

        if (m_diagnostics)
        {
            auto lp = m_builder->lidar_processor();
            m_diagnostics->onProcess(static_cast<uint8_t>(m_builder->status()), m_package.cloud_end_time,
                                     this->now().seconds(), cost_ms, lp->lastMatchCostMs(),
                                     static_cast<uint32_t>(lp->lastValidPointNum()),
                                     static_cast<uint32_t>(lp->lastMatchedPointNum()));
        }

        if (m_builder->status() != BuilderStatus::MAPPING)
            return;

        broadCastTF(m_tf_broadcaster, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);

        publishOdometry(m_odom_pub, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);

        CloudType::Ptr body_cloud = m_builder->lidar_processor()->transformCloud(m_package.cloud, m_kf->x().r_il, m_kf->x().t_il);

        publishCloud(m_body_cloud_pub, body_cloud, m_node_config.body_frame, m_package.cloud_end_time);

        CloudType::Ptr world_cloud = m_builder->lidar_processor()->transformCloud(m_package.cloud, m_builder->lidar_processor()->r_wl(), m_builder->lidar_processor()->t_wl());

        publishCloud(m_world_cloud_pub, world_cloud, m_node_config.world_frame, m_package.cloud_end_time);

        publishPath(m_path_pub, m_node_config.world_frame, m_package.cloud_end_time);
    }

    void publishDiagnostics()
    {
        if (!m_diagnostics_pub || !m_diagnostics)
            return;
        interface::msg::LioDiagnostics msg = m_diagnostics->report(this->now().seconds());
        msg.header.stamp = this->now();
        msg.header.frame_id = m_node_config.body_frame;
        m_diagnostics_pub->publish(msg);

        // 诊断信号只告警，不判定定位失效；节流避免刷屏
        if (msg.pose_jump_warning)
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), m_warn_throttle_ms,
                                 "pose jump: %.3f m / %.2f deg (diagnostic only)",
                                 msg.pose_delta_trans, msg.pose_delta_rot_deg);
        if (msg.static_drift_warning)
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), m_warn_throttle_ms,
                                 "static drift: %.3f m / %.2f deg (diagnostic only)",
                                 msg.static_drift_trans, msg.static_drift_rot_deg);
        if (msg.pose_timestamp_rollback)
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), m_warn_throttle_ms,
                                 "odom timestamp rollback detected (diagnostic only)");
        if (msg.odom_timeout)
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), m_warn_throttle_ms,
                                 "no odom for %.2f s (diagnostic only)", msg.odom_silence_sec);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_lidar_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;

    rclcpp::CallbackGroup::SharedPtr m_imu_cb_group;
    rclcpp::CallbackGroup::SharedPtr m_lidar_cb_group;
    rclcpp::CallbackGroup::SharedPtr m_timer_cb_group;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_body_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_world_cloud_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;

    rclcpp::TimerBase::SharedPtr m_timer;
    StateData m_state_data;
    SyncPackage m_package;
    NodeConfig m_node_config;
    Config m_builder_config;
    LidarTimeUnit m_time_unit = LidarTimeUnit::SEC;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<MapBuilder> m_builder;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

    DiagnosticsConfig m_diag_config;
    std::shared_ptr<LioDiagnostics> m_diagnostics;
    rclcpp::Publisher<interface::msg::LioDiagnostics>::SharedPtr m_diagnostics_pub;
    rclcpp::TimerBase::SharedPtr m_diag_timer;
    uint64_t m_warn_throttle_ms = 2000;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // 多线程执行器: IMU/雷达/定时器回调在不同回调组中并行执行,
    // 防止建图(timerCB)或点云转换(lidarCB)等耗时回调阻塞其他回调, 适配低算力工控机
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<LIONode>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}