from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare("astrall_lidar_udp"), "config", "lidar_udp.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            Node(
                package="astrall_lidar_udp",
                executable="lidar_udp_node",
                name="lidar_udp_node",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
        ]
    )
