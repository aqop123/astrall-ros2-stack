#pragma once
#include <iomanip>
#include <iostream>
#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <builtin_interfaces/msg/time.hpp>

#define RESET "\033[0m"
#define BLACK "\033[30m"  /* Black */
#define RED "\033[31m"    /* Red */
#define GREEN "\033[32m"  /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m"   /* Blue */
#define PURPLE "\033[35m" /* Purple */
#define CYAN "\033[36m"   /* Cyan */
#define WHITE "\033[37m"  /* White */

// 单点时间字段(time/timestamp)的单位
// 速腾(rslidar_sdk)驱动默认输出秒级绝对时间戳
enum class LidarTimeUnit
{
    SEC,
    MS,
    US,
    NS
};

class Utils
{
public:
    static double getSec(std_msgs::msg::Header &header);
    static builtin_interfaces::msg::Time getTime(const double& sec);

    // 解析 lidar_time_unit 配置项(sec/ms/us/ns), 未知取值回退为 sec
    static LidarTimeUnit parseTimeUnit(const std::string &unit);

    // sensor_msgs::msg::PointCloud2(双雷达融合节点发布的 /merged_points) -> PCL 点云
    // 读取单点时间字段(time 优先, 其次 timestamp), 统一换算为"相对扫描起始的毫秒偏移"写入 curvature,
    // 与去畸变(IMUProcessor::undistort)及 syncPackage 中 curvature/1000 -> 秒 的约定保持一致
    // scan_start_time 输出扫描起始时刻(秒):
    //   absolute_time=true  -> 取单点绝对时间戳中的最小值(不依赖 header 语义)
    //   absolute_time=false -> 取 header 时间戳
    static pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud2PCL(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
                                                                int filter_num,
                                                                double min_range,
                                                                double max_range,
                                                                LidarTimeUnit time_unit,
                                                                bool absolute_time,
                                                                double &scan_start_time);
};
