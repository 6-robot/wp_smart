#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('smart_bringup')
    return LaunchDescription([
        DeclareLaunchArgument(
            'start_base', default_value='true',
            description='Start the serial chassis driver and model; false if already running'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('display_rate', default_value='5.0'),
        DeclareLaunchArgument(
            'calibration_seconds', default_value='2.0',
            description='Keep the IMU still for gyro bias estimation; 0 disables calibration'),
        DeclareLaunchArgument('correction_gain', default_value='2.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'bringup.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_base')),
        ),
        Node(
            package='smart_bringup', executable='imu_console', name='imu_console',
            parameters=[{
                'imu_topic': LaunchConfiguration('imu_topic'),
                'display_rate': ParameterValue(
                    LaunchConfiguration('display_rate'), value_type=float),
                'calibration_seconds': ParameterValue(
                    LaunchConfiguration('calibration_seconds'), value_type=float),
                'correction_gain': ParameterValue(
                    LaunchConfiguration('correction_gain'), value_type=float),
            }],
            output='screen', emulate_tty=True,
        ),
    ])
