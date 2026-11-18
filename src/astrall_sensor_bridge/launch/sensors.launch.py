from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution(
        [FindPackageShare("astrall_sensor_bridge"), "config", "sensors.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            Node(
                package="astrall_sensor_bridge",
                executable="astrall_sensor_node",
                name="astrall_sensor_node",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
        ]
    )
