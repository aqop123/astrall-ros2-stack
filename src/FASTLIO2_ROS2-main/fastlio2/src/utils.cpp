#include "utils.h"
#include <algorithm>
#include <cmath>
#include <sensor_msgs/point_cloud2_iterator.hpp>

double Utils::getSec(std_msgs::msg::Header &header)
{
    return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1e-9;
}

builtin_interfaces::msg::Time Utils::getTime(const double &sec)
{
    builtin_interfaces::msg::Time time_msg;
    time_msg.sec = static_cast<int32_t>(sec);
    time_msg.nanosec = static_cast<uint32_t>((sec - time_msg.sec) * 1e9);
    return time_msg;
}

LidarTimeUnit Utils::parseTimeUnit(const std::string &unit)
{
    if (unit == "sec" || unit == "s" || unit == "second")
        return LidarTimeUnit::SEC;
    if (unit == "ms" || unit == "millisecond")
        return LidarTimeUnit::MS;
    if (unit == "us" || unit == "microsecond")
        return LidarTimeUnit::US;
    if (unit == "ns" || unit == "nanosecond")
        return LidarTimeUnit::NS;
    std::cerr << "Unknown lidar_time_unit '" << unit << "', fallback to 'sec'" << std::endl;
    return LidarTimeUnit::SEC;
}

namespace
{
double unitToSec(LidarTimeUnit unit)
{
    switch (unit)
    {
    case LidarTimeUnit::MS:
        return 1e-3;
    case LidarTimeUnit::US:
        return 1e-6;
    case LidarTimeUnit::NS:
        return 1e-9;
    case LidarTimeUnit::SEC:
    default:
        return 1.0;
    }
}

struct CloudConvertOptions
{
    int filter_num = 1;
    double min_range = 0.5;
    double max_range = 20.0;
    double unit_to_sec = 1.0;    // 单点时间字段数值 -> 秒 的换算系数
    bool absolute_time = true;   // true: 绝对时间戳; false: 已是相对扫描起始的偏移
    bool has_intensity = true;   // 点云是否带 intensity 字段
    bool has_time = true;        // 点云是否带 time/timestamp 字段
    std::string time_field = "time";
};

bool hasField(const sensor_msgs::msg::PointCloud2 &msg, const std::string &name)
{
    for (const auto &field : msg.fields)
        if (field.name == name)
            return true;
    return false;
}

// 单点时间统一读成 double(秒), 再换算为相对扫描起始的毫秒偏移写入 curvature
// 绝对时间戳场景下先减去基准时间(最早点的时刻), 避免 1.7e9 量级的秒值直接写入 float 丢失精度
template <typename TTime, typename TIntensity>
pcl::PointCloud<pcl::PointXYZINormal>::Ptr convertPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr &msg,
                                                             const CloudConvertOptions &opt,
                                                             double &scan_start_time)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    const size_t point_num = static_cast<size_t>(msg->width) * msg->height;

    // has_time/has_intensity 为 false 时用 "x"(float32) 字段做占位迭代器, 保证模板类型与字段类型匹配
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<TIntensity> iter_intensity(*msg, opt.has_intensity ? "intensity" : "x");
    sensor_msgs::PointCloud2ConstIterator<TTime> iter_time(*msg, opt.has_time ? opt.time_field : "x");

    std::vector<std::pair<pcl::PointXYZINormal, double>> points;
    points.reserve(point_num / opt.filter_num + 1);
    for (size_t i = 0; i < point_num; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_time)
    {
        if (i % static_cast<size_t>(opt.filter_num) != 0)
            continue;
        float x = *iter_x;
        float y = *iter_y;
        float z = *iter_z;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
            continue;
        if (x * x + y * y + z * z < opt.min_range * opt.min_range || x * x + y * y + z * z > opt.max_range * opt.max_range)
            continue;
        pcl::PointXYZINormal p;
        p.x = x;
        p.y = y;
        p.z = z;
        p.intensity = opt.has_intensity ? static_cast<float>(*iter_intensity) : 0.0f;
        p.curvature = 0.0f;
        double t = opt.has_time ? static_cast<double>(*iter_time) * opt.unit_to_sec : 0.0;
        points.emplace_back(p, t);
    }

    double base_time = 0.0;
    if (opt.absolute_time && !points.empty())
    {
        // 绝对时间戳: 以最早点时刻作为扫描起始时刻
        base_time = points.front().second;
        for (const auto &kp : points)
            base_time = std::min(base_time, kp.second);
        scan_start_time = base_time;
    }
    else
    {
        // 相对偏移: 单点时间已是相对扫描起始的偏移, 以 header 时间戳作为扫描起始时刻
        scan_start_time = Utils::getSec(msg->header);
    }

    cloud->reserve(points.size());
    for (auto &kp : points)
    {
        // 全流水线统一约定: curvature = 相对扫描起始的毫秒偏移
        // syncPackage 中 cloud_end_time = cloud_start_time + curvature/1000(秒)
        // undistort 中 curvature/1000 与 IMU 位姿插值的 offset(秒) 比较
        double offset_ms = (kp.second - base_time) * 1000.0;
        if (offset_ms < 0.0)
            offset_ms = 0.0;
        kp.first.curvature = static_cast<float>(offset_ms);
        cloud->push_back(kp.first);
    }
    return cloud;
}

template <typename TTime>
pcl::PointCloud<pcl::PointXYZINormal>::Ptr dispatchIntensity(const sensor_msgs::msg::PointCloud2::SharedPtr &msg,
                                                             const CloudConvertOptions &opt,
                                                             uint8_t intensity_dt,
                                                             double &scan_start_time)
{
    using sensor_msgs::msg::PointField;
    switch (intensity_dt)
    {
    case PointField::FLOAT32:
        return convertPointCloud<TTime, float>(msg, opt, scan_start_time);
    case PointField::FLOAT64:
        return convertPointCloud<TTime, double>(msg, opt, scan_start_time);
    case PointField::UINT32:
        return convertPointCloud<TTime, uint32_t>(msg, opt, scan_start_time);
    case PointField::INT32:
        return convertPointCloud<TTime, int32_t>(msg, opt, scan_start_time);
    case PointField::UINT16:
        return convertPointCloud<TTime, uint16_t>(msg, opt, scan_start_time);
    case PointField::INT16:
        return convertPointCloud<TTime, int16_t>(msg, opt, scan_start_time);
    case PointField::UINT8:
        return convertPointCloud<TTime, uint8_t>(msg, opt, scan_start_time);
    default:
        std::cerr << "Unsupported intensity field datatype: " << static_cast<int>(intensity_dt) << std::endl;
        return pcl::PointCloud<pcl::PointXYZINormal>::Ptr(new pcl::PointCloud<pcl::PointXYZINormal>);
    }
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr dispatchTime(const sensor_msgs::msg::PointCloud2::SharedPtr &msg,
                                                        const CloudConvertOptions &opt,
                                                        uint8_t time_dt,
                                                        uint8_t intensity_dt,
                                                        double &scan_start_time)
{
    using sensor_msgs::msg::PointField;
    switch (time_dt)
    {
    case PointField::FLOAT32:
        return dispatchIntensity<float>(msg, opt, intensity_dt, scan_start_time);
    case PointField::FLOAT64:
        return dispatchIntensity<double>(msg, opt, intensity_dt, scan_start_time);
    case PointField::UINT32:
        return dispatchIntensity<uint32_t>(msg, opt, intensity_dt, scan_start_time);
    case PointField::INT32:
        return dispatchIntensity<int32_t>(msg, opt, intensity_dt, scan_start_time);
    case PointField::UINT16:
        return dispatchIntensity<uint16_t>(msg, opt, intensity_dt, scan_start_time);
    case PointField::INT16:
        return dispatchIntensity<int16_t>(msg, opt, intensity_dt, scan_start_time);
    case PointField::UINT8:
        return dispatchIntensity<uint8_t>(msg, opt, intensity_dt, scan_start_time);
    default:
        std::cerr << "Unsupported time field datatype: " << static_cast<int>(time_dt) << std::endl;
        return pcl::PointCloud<pcl::PointXYZINormal>::Ptr(new pcl::PointCloud<pcl::PointXYZINormal>);
    }
}
} // namespace

pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::cloud2PCL(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
                                                            int filter_num,
                                                            double min_range,
                                                            double max_range,
                                                            LidarTimeUnit time_unit,
                                                            bool absolute_time,
                                                            double &scan_start_time)
{
    scan_start_time = 0.0;
    if (msg->width * msg->height == 0)
    {
        std::cerr << "Received empty PointCloud2" << std::endl;
        return pcl::PointCloud<pcl::PointXYZINormal>::Ptr(new pcl::PointCloud<pcl::PointXYZINormal>);
    }

    // x/y/z 必须为 float32, 否则无法用迭代器安全读取
    for (const auto &name : {"x", "y", "z"})
    {
        const sensor_msgs::msg::PointField *field = nullptr;
        for (const auto &f : msg->fields)
            if (f.name == name)
                field = &f;
        if (field == nullptr || field->datatype != sensor_msgs::msg::PointField::FLOAT32)
        {
            std::cerr << "PointCloud2 field '" << name << "' missing or not FLOAT32" << std::endl;
            return pcl::PointCloud<pcl::PointXYZINormal>::Ptr(new pcl::PointCloud<pcl::PointXYZINormal>);
        }
    }

    CloudConvertOptions opt;
    opt.filter_num = filter_num;
    opt.min_range = min_range;
    opt.max_range = max_range;
    opt.unit_to_sec = unitToSec(time_unit);
    opt.absolute_time = absolute_time;
    opt.has_intensity = hasField(*msg, "intensity");

    // 时间字段: 优先 "time", 其次 "timestamp"
    const sensor_msgs::msg::PointField *time_field = nullptr;
    for (const auto &f : msg->fields)
        if (f.name == "time" || f.name == "timestamp")
            time_field = &f;
    if (time_field != nullptr)
    {
        opt.has_time = true;
        opt.time_field = time_field->name;
        // float32 在 1.7e9 量级的分辨率约为 64~128 秒, 绝对秒级时间戳会全部坍缩到同一时刻, 去畸变失效
        if (absolute_time && time_field->datatype == sensor_msgs::msg::PointField::FLOAT32)
            std::cerr << "WARNING: time field is FLOAT32 with absolute time, precision lost (~100s granularity at 1.7e9). "
                         "Publish float64 'timestamp' or relative offsets instead!" << std::endl;
    }
    else
    {
        // 无单点时间字段: curvature 全 0, 退化为不做单点运动补偿
        opt.has_time = false;
        std::cerr << "PointCloud2 has no 'time'/'timestamp' field, per-point motion compensation disabled" << std::endl;
    }

    uint8_t time_dt = opt.has_time ? time_field->datatype : sensor_msgs::msg::PointField::FLOAT32;
    uint8_t intensity_dt = opt.has_intensity ? 0 : sensor_msgs::msg::PointField::FLOAT32;
    if (opt.has_intensity)
    {
        for (const auto &f : msg->fields)
            if (f.name == "intensity")
                intensity_dt = f.datatype;
    }

    return dispatchTime(msg, opt, time_dt, intensity_dt, scan_start_time);
}
