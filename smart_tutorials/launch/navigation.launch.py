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
    nav2_share = get_package_share_directory('nav2_bringup')

    return LaunchDescription([
        DeclareLaunchArgument('start_base', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'base_params_file',
            default_value=os.path.join(base_share, 'config', 'smart_base.yaml'),
            description='Serial chassis parameters; port and baud_rate are inherited'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(package_share, 'maps', 'map.yaml'),
            description='Map YAML; defaults to the map installed with smart_tutorials'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(package_share, 'config', 'nav2_params.yaml'),
            description='Nav2 parameters, separate from base_params_file'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(package_share, 'rviz', 'navi.rviz')),
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
                os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'slam': 'False',
                'map': LaunchConfiguration('map'),
                'use_sim_time': 'false',
                'params_file': LaunchConfiguration('params_file'),
                'autostart': LaunchConfiguration('autostart'),
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_navigation',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': False}],
            output='screen',
        ),
    ])
