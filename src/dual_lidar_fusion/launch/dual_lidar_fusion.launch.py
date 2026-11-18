from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare("dual_lidar_fusion"), "config", "dual_lidar_fusion.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            Node(
                package="dual_lidar_fusion",
                executable="dual_lidar_fusion_node",
                name="dual_lidar_fusion_node",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
        ]
    )
