#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('smart_bringup')
    return LaunchDescription([
        DeclareLaunchArgument(
            'imu_filter_params_file',
            default_value=os.path.join(share, 'config', 'imu_filter.yaml')),
        Node(
            package='smart_bringup', executable='smart_imu_filter_node',
            name='smart_imu_filter', output='screen',
            parameters=[LaunchConfiguration('imu_filter_params_file')],
        ),
    ])
