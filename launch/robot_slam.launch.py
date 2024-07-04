# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    robot_prefix = get_package_share_directory('robot')
    return LaunchDescription([
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([robot_prefix, '/launch', '/rpi_robot_nostate.launch.py']),
        #     launch_arguments={}.items(),
        # ),
        # static transform for laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['-0.064', '0', '0.120', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),
        # static transform from link to footprint
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.0325', '0', '0', '0', 'base_footprint', 'base_link'],
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/cartographer.launch.py'])
        )
    ])