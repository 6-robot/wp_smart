#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('smart_bringup')
    return LaunchDescription([
        DeclareLaunchArgument(
            'start_base', default_value='true',
            description='Start the serial chassis driver and model; false if already running'),
        DeclareLaunchArgument(
            'rviz_config', default_value=os.path.join(share, 'rviz', 'scan.rviz')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'bringup.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_base')),
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2_scan',
            arguments=['-d', LaunchConfiguration('rviz_config')], output='screen',
        ),
    ])
