import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Port for rplidar sensor'
        ),
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{'channel_type':'serial',
                         'serial_port': '/dev/ttyUSB0',
                         'serial_baudrate': 115200,
                         'frame_id': 'laser',
                         'inverted': False,
                         'angle_compensate': True,
                         'scan_mode': 'Boost'}],
            output='screen'),
    ])