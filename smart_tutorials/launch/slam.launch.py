#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('smart_tutorials')
    base_share = get_package_share_directory('smart_bringup')
    slam_share = get_package_share_directory('slam_toolbox')

    return LaunchDescription([
        DeclareLaunchArgument('start_base', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'base_params_file',
            default_value=os.path.join(base_share, 'config', 'smart_base.yaml'),
            description='Serial chassis parameters; port and baud_rate are inherited'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(package_share, 'config', 'slam_params.yaml')),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(package_share, 'rviz', 'slam.rviz')),
        # Scope the chassis params_file so it cannot replace the Nav2 parameter file.
        GroupAction(
            condition=IfCondition(LaunchConfiguration('start_base')),
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(base_share, 'launch', 'bringup.launch.py')),
                launch_arguments={
                    'params_file': LaunchConfiguration('base_params_file'),
                }.items(),
            )],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_share, 'launch', 'online_sync_launch.py')),
            launch_arguments={
                'use_sim_time': 'false',
                'slam_params_file': LaunchConfiguration('slam_params_file'),
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_slam',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': False}],
            output='screen',
        ),
    ])
