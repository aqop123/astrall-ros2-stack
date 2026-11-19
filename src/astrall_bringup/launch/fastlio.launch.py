"""ASTRALL 感知栈一键启动（FAST-LIO2 前端模式）。

数据流：雷达驱动 -> IMU 桥接 -> 双雷达融合 -> FAST-LIO2 (+ RViz)，不启动 PGO。

纯前端诊断测试用本文件；含回环的建图模式用 slam.launch.py；定位用 localizer.launch.py。
三者不要同时运行，否则会重复启动 lio_node。

改配置只需编辑本包的 config/ 目录，然后重新 colcon build 本包。
每个配置路径都可通过 launch argument 临时覆盖，例如：
    ros2 launch astrall_bringup fastlio.launch.py lio_config:=/tmp/lio.yaml
    ros2 launch astrall_bringup fastlio.launch.py use_rviz:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PACKAGE = "astrall_bringup"


def config_path(*parts):
    """本包 config/ 目录下的文件路径。"""
    return PathJoinSubstitution([FindPackageShare(PACKAGE), "config", *parts])


def arg(name, default, description):
    return DeclareLaunchArgument(name, default_value=default, description=description)


def generate_launch_description():
    # ------------------------------------------------------------------
    # 1. 配置路径：全部集中在本包 config/ 下，命令行可覆盖
    # ------------------------------------------------------------------
    config_arguments = [
        arg("rslidar_config_dir", config_path("rslidar", "config.yaml"),
            "rslidar_sdk 配置文件（config.yaml）路径"),
        arg("sensors_params", config_path("sensors.yaml"),
            "astrall_sensor_bridge 参数文件"),
        arg("fusion_params", config_path("dual_lidar_fusion.yaml"),
            "dual_lidar_fusion 参数文件"),
        arg("lio_config", config_path("lio.yaml"),
            "FAST-LIO2 参数文件"),
        arg("pgo_config", config_path("pgo.yaml"),
            "PGO 回环检测参数文件"),
        arg("rviz_config", PathJoinSubstitution(
            [FindPackageShare(PACKAGE), "rviz", "pgo.rviz"]),
            "RViz 布局文件"),
        arg("use_rviz", "true", "是否启动 RViz（true / false）"),
    ]

    # ------------------------------------------------------------------
    # 2. 传感器驱动：发布 /rslidar_points、/rslidar_rear_points、/imu/data
    # ------------------------------------------------------------------
    rslidar_driver = Node(
        package="rslidar_sdk",
        executable="rslidar_sdk_node",
        namespace="rslidar_sdk",
        output="screen",
        parameters=[{"config_path": LaunchConfiguration("rslidar_config_dir")}],
    )

    sensor_bridge = Node(
        package="astrall_sensor_bridge",
        executable="astrall_sensor_node",
        name="astrall_sensor_node",
        output="screen",
        parameters=[LaunchConfiguration("sensors_params")],
    )

    # ------------------------------------------------------------------
    # 3. 双雷达融合：订阅两路点云，发布 /merged_points
    # ------------------------------------------------------------------
    lidar_fusion = Node(
        package="dual_lidar_fusion",
        executable="dual_lidar_fusion_node",
        name="dual_lidar_fusion_node",
        output="screen",
        parameters=[LaunchConfiguration("fusion_params")],
    )

    # ------------------------------------------------------------------
    # 4. FAST-LIO2 前端 + PGO 回环 + 可视化
    #    PGO 原始 launch 自带 lio_node，这里只保留这一个 lio_node
    # ------------------------------------------------------------------
    fastlio = Node(
        package="fastlio2",
        executable="lio_node",
        namespace="fastlio2",
        name="lio_node",
        output="screen",
        parameters=[{"config_path": LaunchConfiguration("lio_config")}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        namespace="pgo",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription([
        *config_arguments,
        rslidar_driver,
        sensor_bridge,
        lidar_fusion,
        fastlio,
        rviz,
    ])
