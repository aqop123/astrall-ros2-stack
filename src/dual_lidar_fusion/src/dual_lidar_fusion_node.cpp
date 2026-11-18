// 双 3D 激光雷达（速腾 RoboSense）轻量级拼接融合节点
// 技术方案: docs/3d_robosense.md §3
//
// 职责（共 3 步, 去畸变 100% 交给 fastlio2 原生逐点补偿）:
//   1. ApproximateTime 双雷达帧同步（/rslidar_points + /rslidar_rear_points, slop 可配）
//   2. 外参变换到 imu_link + 机身包围盒自遮挡裁剪（同一单次循环, 逐点 timestamp 原样保留）
//   3. 哈希表体素降采样（每体素保留首个落入点）后发布到 /merged_points（RELIABLE QoS）
//
// 关键约束（方案 §3.1, 违反将导致 fastlio2 逐点运动补偿失效）:
//   - 输入必须为 rslidar_sdk XYZIRT 线上紧凑布局（point_step=26, timestamp 为 FLOAT64 绝对秒）
//   - 输出逐点 timestamp 保持 FLOAT64 绝对秒原样透传, 禁止预去畸变 / 清零 / 降精度
//
// 降级（§3.3）: 后雷达断流超过 rear_timeout_sec 后发布 front 单雷达点云（1Hz 告警）,
//               恢复后自动回到双雷达合并。

#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Geometry>

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

// rslidar_sdk（POINT_TYPE=XYZIRT）线上紧凑布局, 无对齐填充:
//   x@0(F32) y@4(F32) z@8(F32) intensity@12(F32) ring@16(U16) timestamp@18(F64 绝对秒)
// timestamp 位于偏移 18 不满足 8 字节对齐, 禁止把 buffer 强转成结构体读取（方案 §3.1）
static constexpr size_t kPointStep = 26;

class DualLidarFusionNode : public rclcpp::Node
{
public:
  DualLidarFusionNode() : Node("dual_lidar_fusion_node")
  {
    loadParameters();

    // 输出 QoS 必须为 RELIABLE: lio_node 的点云订阅是默认 RELIABLE,
    // 若本节点用 best-effort 发布, pub best-effort + sub reliable 不匹配, fastlio2 收不到点云（§3.2）
    if (output_qos_reliable_)
      m_pub = this->create_publisher<PointCloud2>(m_output_topic, rclcpp::QoS(10).reliable());
    else
      m_pub = this->create_publisher<PointCloud2>(m_output_topic, rclcpp::SensorDataQoS());

    // 输入订阅: rslidar_sdk 发布端为默认 RELIABLE, 订阅端用默认 profile 即可匹配（§3.2）
    m_front_sub.subscribe(this, m_front_topic, rmw_qos_profile_default);
    m_rear_sub.subscribe(this, m_rear_topic, rmw_qos_profile_default);

    // 双雷达帧同步（ApproximateTime, slop 可配）
    // 注: policy 必须以右值传入——Synchronizer 存在 (policy, F0, F1) 与 (F0, F1, F2) 两个模板构造,
    //     传具名左值会歧义到后者并把 policy 误当作过滤器, 编译失败
    SyncPolicy policy(kSyncQueueSize);
    policy.setMaxIntervalDuration(rclcpp::Duration::from_seconds(m_sync_slop_sec));
    m_sync = std::make_shared<Sync>(std::move(policy), m_front_sub, m_rear_sub);
    m_sync->registerCallback(std::bind(&DualLidarFusionNode::onSync, this, std::placeholders::_1, std::placeholders::_2));

    // 逐条消息回调: 跟踪后雷达存活状态, 断流时降级为 front 单雷达输出（§3.3）
    m_front_sub.registerCallback(std::bind(&DualLidarFusionNode::onFrontAny, this, std::placeholders::_1));
    m_rear_sub.registerCallback(std::bind(&DualLidarFusionNode::onRearAny, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "dual_lidar_fusion_node started: front='%s' rear='%s' out='%s' frame='%s' "
                "sync_slop=%.0fms rear_timeout=%.1fs voxel=%.2fm output_reliable=%d",
                m_front_topic.c_str(), m_rear_topic.c_str(), m_output_topic.c_str(), m_frame_id.c_str(),
                m_sync_slop_sec * 1000.0, m_rear_timeout_sec, m_voxel_leaf_size, output_qos_reliable_ ? 1 : 0);
    RCLCPP_INFO(this->get_logger(),
                "self_filter_box (imu_link): x[%.2f, %.2f] y[%.2f, %.2f] z[%.2f, %.2f]",
                m_box.min_x, m_box.max_x, m_box.min_y, m_box.max_y, m_box.min_z, m_box.max_z);
    RCLCPP_INFO(this->get_logger(),
                "extrinsic_front_to_imu: t=[%.3f, %.3f, %.3f] q(xyzw)=[%.3f, %.3f, %.3f, %.3f] [占位示例, 按 §7 标定]",
                m_t_front.x(), m_t_front.y(), m_t_front.z(),
                m_q_front.x(), m_q_front.y(), m_q_front.z(), m_q_front.w());
    RCLCPP_INFO(this->get_logger(),
                "extrinsic_rear_to_imu:  t=[%.3f, %.3f, %.3f] q(xyzw)=[%.3f, %.3f, %.3f, %.3f] [占位示例, 按 §7 标定]",
                m_t_rear.x(), m_t_rear.y(), m_t_rear.z(),
                m_q_rear.x(), m_q_rear.y(), m_q_rear.z(), m_q_rear.w());
  }

private:
  static constexpr size_t kSyncQueueSize = 10;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  struct SelfFilterBox
  {
    double min_x, max_x, min_y, max_y, min_z, max_z;
  };

  // ---------------- 参数 ----------------
  void loadParameters()
  {
    m_front_topic = this->declare_parameter<std::string>("front_lidar_topic", "/rslidar_points");
    m_rear_topic = this->declare_parameter<std::string>("rear_lidar_topic", "/rslidar_rear_points");
    m_output_topic = this->declare_parameter<std::string>("output_topic", "/merged_points");
    m_frame_id = this->declare_parameter<std::string>("output_frame_id", "imu_link");
    m_sync_slop_sec = this->declare_parameter<double>("sync_slop_sec", 0.03);
    m_rear_timeout_sec = this->declare_parameter<double>("rear_timeout_sec", 2.0);
    m_voxel_leaf_size = this->declare_parameter<double>("voxel_leaf_size", 0.10);
    output_qos_reliable_ = this->declare_parameter<bool>("output_qos_reliable", true);

    m_box.min_x = this->declare_parameter<double>("self_filter_box.min_x", -0.65);
    m_box.max_x = this->declare_parameter<double>("self_filter_box.max_x", 0.65);
    m_box.min_y = this->declare_parameter<double>("self_filter_box.min_y", -0.40);
    m_box.max_y = this->declare_parameter<double>("self_filter_box.max_y", 0.40);
    m_box.min_z = this->declare_parameter<double>("self_filter_box.min_z", -0.60);
    m_box.max_z = this->declare_parameter<double>("self_filter_box.max_z", 0.30);

    m_t_front = loadTranslation("extrinsic_front_to_imu");
    m_q_front = loadQuaternion("extrinsic_front_to_imu");
    m_t_rear = loadTranslation("extrinsic_rear_to_imu");
    m_q_rear = loadQuaternion("extrinsic_rear_to_imu");

    if (m_voxel_leaf_size <= 0.0 || m_sync_slop_sec <= 0.0 || m_rear_timeout_sec <= 0.0)
      throw std::runtime_error("voxel_leaf_size / sync_slop_sec / rear_timeout_sec must be positive");
  }

  Eigen::Vector3d loadTranslation(const std::string & prefix)
  {
    const std::vector<double> t =
      this->declare_parameter<std::vector<double>>(prefix + ".translation", {0.0, 0.0, 0.0});
    if (t.size() != 3)
      throw std::runtime_error(prefix + ".translation must have 3 elements [x, y, z]");
    return Eigen::Vector3d(t[0], t[1], t[2]);
  }

  Eigen::Quaterniond loadQuaternion(const std::string & prefix)
  {
    // 参数顺序 [qx, qy, qz, qw]; Eigen 构造函数顺序为 (w, x, y, z)
    const std::vector<double> q =
      this->declare_parameter<std::vector<double>>(prefix + ".quaternion", {0.0, 0.0, 0.0, 1.0});
    if (q.size() != 4)
      throw std::runtime_error(prefix + ".quaternion must have 4 elements [qx, qy, qz, qw]");
    return Eigen::Quaterniond(q[3], q[0], q[1], q[2]).normalized();
  }

  // ---------------- 回调 ----------------
  void onSync(const PointCloud2::ConstSharedPtr & front, const PointCloud2::ConstSharedPtr & rear)
  {
    if (!m_first_pair_logged)
    {
      m_first_pair_logged = true;
      // 首对合并帧打印相位差, 供部署时核对 §6.3（实测相位差 > slop 会持续配对失败, 输出恒为空）
      const double phase_diff_ms =
        std::abs((rclcpp::Time(front->header.stamp) - rclcpp::Time(rear->header.stamp)).seconds()) * 1000.0;
      RCLCPP_INFO(this->get_logger(),
                  "first dual-lidar pair merged, header.stamp phase diff = %.1f ms (sync_slop_sec = %.0f ms)",
                  phase_diff_ms, m_sync_slop_sec * 1000.0);
    }
    mergeAndPublish(front, rear);
  }

  void onFrontAny(const PointCloud2::ConstSharedPtr & msg)
  {
    const rclcpp::Time stamp(msg->header.stamp);
    if (isRearTimedOut(stamp))
    {
      m_was_degraded = true;
      // 后雷达断流: 降级为 front 单雷达输出（仍执行外参/裁剪/降采样, 时间戳链路不变, §3.3）
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "rear lidar timed out (no data for > %.1fs), degraded to front-only output",
                           m_rear_timeout_sec);
      mergeAndPublish(msg, nullptr);
    }
  }

  void onRearAny(const PointCloud2::ConstSharedPtr & msg)
  {
    m_last_rear_stamp = rclcpp::Time(msg->header.stamp);
    if (m_was_degraded)
    {
      m_was_degraded = false;
      RCLCPP_INFO(this->get_logger(), "rear lidar recovered, back to dual-lidar merged output");
    }
  }

  // ---------------- 核心流水线 ----------------
  bool isRearTimedOut(const rclcpp::Time & front_stamp) const
  {
    return !m_last_rear_stamp.has_value() ||
           (front_stamp - *m_last_rear_stamp).seconds() > m_rear_timeout_sec;
  }

  // §3.1 运行时校验: 布局不符（典型事故: rslidar_sdk 忘改 POINT_TYPE=XYZIRT 时输入为 XYZI 布局）报错退出
  void validateLayout(const PointCloud2 & msg, const std::string & topic) const
  {
    std::string err;
    if (msg.point_step != kPointStep)
      err = "point_step=" + std::to_string(msg.point_step) + " != 26, not compact XYZIRT layout";
    else if (!checkField(msg, "x", PointField::FLOAT32) || !checkField(msg, "y", PointField::FLOAT32) ||
             !checkField(msg, "z", PointField::FLOAT32))
      err = "field x/y/z missing or not FLOAT32";
    else if (!checkField(msg, "intensity", PointField::FLOAT32))
      err = "field intensity missing or not FLOAT32";
    else if (!checkField(msg, "ring", PointField::UINT16))
      err = "field ring missing or not UINT16";
    else if (!checkField(msg, "timestamp", PointField::FLOAT64))
      err = "field timestamp missing or not FLOAT64 (absolute seconds)";

    if (!err.empty())
    {
      RCLCPP_FATAL(this->get_logger(),
                   "PointCloud2 on '%s' layout invalid: %s. "
                   "Check: rslidar_sdk built with POINT_TYPE=XYZIRT and dense_points/min/max settings (docs/3d_robosense.md §2).",
                   topic.c_str(), err.c_str());
      throw std::runtime_error("input layout validation failed on '" + topic + "': " + err);
    }
  }

  static bool checkField(const PointCloud2 & msg, const std::string & name, uint8_t datatype)
  {
    for (const auto & f : msg.fields)
      if (f.name == name)
        return f.datatype == datatype;
    return false;
  }

  void mergeAndPublish(const PointCloud2::ConstSharedPtr & front, const PointCloud2::ConstSharedPtr & rear)
  {
    validateLayout(*front, m_front_topic);
    if (front->width * front->height == 0)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "front cloud is empty, skip");
      return;
    }
    if (rear)
    {
      validateLayout(*rear, m_rear_topic);
      if (rear->width * rear->height == 0)
      {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "rear cloud is empty, skip");
        return;
      }
    }

    // 合并帧基准时刻 = 两帧 header.stamp 较早者（ts_first_point: true 即帧头 = 首点时刻, §3.2）
    const rclcpp::Time t_front(front->header.stamp);
    const rclcpp::Time t_rear(rear ? rear->header.stamp : front->header.stamp);
    const builtin_interfaces::msg::Time t_base =
      (rear && t_rear < t_front) ? rear->header.stamp : front->header.stamp;

    std::vector<uint8_t> data;
    const size_t front_points = static_cast<size_t>(front->width) * front->height;
    const size_t rear_points = rear ? static_cast<size_t>(rear->width) * rear->height : 0;
    data.reserve((front_points + rear_points) * kPointStep);

    // 外参变换 + 自遮挡裁剪 + 体素降采样（跨两帧共享同一张体素表, 重叠区自动去重）
    std::unordered_set<uint64_t> voxels;
    voxels.reserve((front_points + rear_points) / 4 + 1);
    const size_t kept_front = appendCloud(*front, m_t_front, m_q_front, voxels, data);
    const size_t kept_rear = rear ? appendCloud(*rear, m_t_rear, m_q_rear, voxels, data) : 0;

    if (data.empty())
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "all points filtered by self-filter box / voxel grid, skip");
      return;
    }
    RCLCPP_DEBUG(this->get_logger(), "merged frame: front %zu/%zu, rear %zu/%zu points kept",
                 kept_front, front_points, kept_rear, rear_points);

    // 输出与输入一致: x,y,z,intensity,ring,timestamp 紧凑布局 point_step=26,
    // timestamp 保持 FLOAT64 绝对秒原样透传（§3.2 步骤 3）
    PointCloud2 out;
    out.header.frame_id = m_frame_id;
    out.header.stamp = t_base;
    out.height = 1;
    out.width = static_cast<uint32_t>(data.size() / kPointStep);
    out.fields = makeFields();
    out.is_bigendian = false;
    out.point_step = kPointStep;
    out.row_step = static_cast<uint32_t>(data.size());
    out.data = std::move(data);
    out.is_dense = true;
    m_pub->publish(out);
  }

  // 对单帧点云执行: 外参变换到 imu_link + 包围盒自遮挡裁剪 + 体素降采样, 追加到输出缓冲。
  // 每点保留其原 timestamp 不变——点在"其采集时刻的 IMU 系"中的表达,
  // 正是 fastlio2 逐点补偿模型所要求的输入（§3.2 步骤 2）。
  size_t appendCloud(const PointCloud2 & msg, const Eigen::Vector3d & t_imu_lidar,
                     const Eigen::Quaterniond & q_imu_lidar, std::unordered_set<uint64_t> & voxels,
                     std::vector<uint8_t> & out)
  {
    sensor_msgs::PointCloud2ConstIterator<float> it_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> it_intensity(msg, "intensity");
    sensor_msgs::PointCloud2ConstIterator<uint16_t> it_ring(msg, "ring");
    sensor_msgs::PointCloud2ConstIterator<double> it_timestamp(msg, "timestamp");

    const size_t num = static_cast<size_t>(msg.width) * msg.height;
    const double inv_leaf = 1.0 / m_voxel_leaf_size;
    size_t kept = 0;
    for (size_t i = 0; i < num;
         ++i, ++it_x, ++it_y, ++it_z, ++it_intensity, ++it_ring, ++it_timestamp)
    {
      const float x = *it_x;
      const float y = *it_y;
      const float z = *it_z;
      // 双保险: 上游 dense_points=true 已丢弃 NaN, 此处再防穿透包围盒裁剪流入 SLAM
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
        continue;

      const Eigen::Vector3d pt_imu = q_imu_lidar *  // p_imu = R_imu<-lidar * p_lidar + t_imu<-lidar
        Eigen::Vector3d(static_cast<double>(x), static_cast<double>(y), static_cast<double>(z)) + t_imu_lidar;

      // 剔除机身包围盒内的自遮挡点（单位: 米, 相对于 imu_link, §3.2 步骤 2）
      if (pt_imu.x() > m_box.min_x && pt_imu.x() < m_box.max_x &&
          pt_imu.y() > m_box.min_y && pt_imu.y() < m_box.max_y &&
          pt_imu.z() > m_box.min_z && pt_imu.z() < m_box.max_z)
        continue;

      // 哈希表快速体素降采样: 每体素保留首个落入点（含其 timestamp/intensity, §3.2 步骤 3）
      if (!voxels.insert(voxelKey(pt_imu, inv_leaf)).second)
        continue;

      const float fx = static_cast<float>(pt_imu.x());
      const float fy = static_cast<float>(pt_imu.y());
      const float fz = static_cast<float>(pt_imu.z());
      const float intensity = *it_intensity;
      const uint16_t ring = *it_ring;
      const double timestamp = *it_timestamp;  // FLOAT64 绝对秒原样透传

      out.resize(out.size() + kPointStep);
      uint8_t * d = out.data() + out.size() - kPointStep;
      std::memcpy(d + 0, &fx, sizeof(float));        // x @0
      std::memcpy(d + 4, &fy, sizeof(float));        // y @4
      std::memcpy(d + 8, &fz, sizeof(float));        // z @8
      std::memcpy(d + 12, &intensity, sizeof(float));  // intensity @12
      std::memcpy(d + 16, &ring, sizeof(uint16_t));    // ring @16
      std::memcpy(d + 18, &timestamp, sizeof(double));  // timestamp @18 (F64, 未对齐按字节拷贝)
      ++kept;
    }
    return kept;
  }

  // 体素索引: 3 x 21bit 打包（0.1m 体素下覆盖 +/-1e6 m, 远超雷达量程）
  static uint64_t voxelKey(const Eigen::Vector3d & p, double inv_leaf)
  {
    const int64_t ix = static_cast<int64_t>(std::floor(p.x() * inv_leaf));
    const int64_t iy = static_cast<int64_t>(std::floor(p.y() * inv_leaf));
    const int64_t iz = static_cast<int64_t>(std::floor(p.z() * inv_leaf));
    return (static_cast<uint64_t>(ix & 0x1FFFFF) << 42) |
           (static_cast<uint64_t>(iy & 0x1FFFFF) << 21) |
           static_cast<uint64_t>(iz & 0x1FFFFF);
  }

  static std::vector<PointField> makeFields()
  {
    std::vector<PointField> fields(6);
    fields[0].name = "x";
    fields[0].offset = 0;
    fields[0].datatype = PointField::FLOAT32;
    fields[0].count = 1;
    fields[1].name = "y";
    fields[1].offset = 4;
    fields[1].datatype = PointField::FLOAT32;
    fields[1].count = 1;
    fields[2].name = "z";
    fields[2].offset = 8;
    fields[2].datatype = PointField::FLOAT32;
    fields[2].count = 1;
    fields[3].name = "intensity";
    fields[3].offset = 12;
    fields[3].datatype = PointField::FLOAT32;
    fields[3].count = 1;
    fields[4].name = "ring";
    fields[4].offset = 16;
    fields[4].datatype = PointField::UINT16;
    fields[4].count = 1;
    fields[5].name = "timestamp";
    fields[5].offset = 18;
    fields[5].datatype = PointField::FLOAT64;
    fields[5].count = 1;
    return fields;
  }

  // ---------------- 成员 ----------------
  std::string m_front_topic;
  std::string m_rear_topic;
  std::string m_output_topic;
  std::string m_frame_id;
  double m_sync_slop_sec = 0.03;
  double m_rear_timeout_sec = 2.0;
  double m_voxel_leaf_size = 0.10;
  bool output_qos_reliable_ = true;
  SelfFilterBox m_box{};

  Eigen::Vector3d m_t_front = Eigen::Vector3d::Zero();
  Eigen::Quaterniond m_q_front = Eigen::Quaterniond::Identity();
  Eigen::Vector3d m_t_rear = Eigen::Vector3d::Zero();
  Eigen::Quaterniond m_q_rear = Eigen::Quaterniond::Identity();

  message_filters::Subscriber<PointCloud2> m_front_sub;
  message_filters::Subscriber<PointCloud2> m_rear_sub;
  std::shared_ptr<Sync> m_sync;
  rclcpp::Publisher<PointCloud2>::SharedPtr m_pub;

  std::optional<rclcpp::Time> m_last_rear_stamp;
  bool m_was_degraded = false;
  bool m_first_pair_logged = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<DualLidarFusionNode>());
  }
  catch (const std::exception & e)
  {
    // §3.1: 输入布局校验失败等致命错误, 报错退出而非带病运行
    RCLCPP_FATAL(rclcpp::get_logger("dual_lidar_fusion"), "dual_lidar_fusion_node exited: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
