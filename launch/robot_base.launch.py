import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    rplidar_port = LaunchConfiguration("rplidar_port", default="/dev/ttyUSB0")

    return LaunchDescription([
        DeclareLaunchArgument(
            "rplidar_port",
            default_value=rplidar_port,
            description="port for rplidar sensor"
        ),
        # robot bridge node
        Node(
            package="robot",
            executable="robot_bridge",
            parameters=[],
            arguments=[],
            output="screen"
        ),
        # static transform from link to footprint
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '-0.0325', '0', '0', '0', 'base_link', 'base_footprint'],
            output='screen'
        )
    ])