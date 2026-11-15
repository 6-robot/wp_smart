#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('smart_bringup'), 'config', 'smart_base.yaml')
    model = os.path.join(
        get_package_share_directory('smart_description'), 'urdf', 'smart.urdf.xacro')
    # WCH UART bridge; override port for another board or a local udev alias.
    default_port = (
        '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5C85476222-if00')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('port', default_value=default_port),
        DeclareLaunchArgument('baud_rate', default_value='460800'),
        DeclareLaunchArgument(
            'enable_cmd_vel_timeout', default_value='false',
            description='Brake on stale cmd_vel; false holds velocity with 1 Hz keepalive'),
        DeclareLaunchArgument('publish_model', default_value='true'),
        DeclareLaunchArgument('imu_xyz', default_value='0 0 0'),
        DeclareLaunchArgument('imu_rpy', default_value='0 0 0'),
        DeclareLaunchArgument('use_imu_filter', default_value='true'),
        DeclareLaunchArgument(
            'imu_filter_params_file', default_value=os.path.join(
                get_package_share_directory('smart_bringup'), 'config', 'imu_filter.yaml')),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            condition=IfCondition(LaunchConfiguration('publish_model')),
            parameters=[{'robot_description': ParameterValue(Command([
                'xacro ', model,
                ' imu_xyz:="', LaunchConfiguration('imu_xyz'), '"',
                ' imu_rpy:="', LaunchConfiguration('imu_rpy'), '"',
            ]), value_type=str)}],
            output='screen',
        ),
        Node(
            package='smart_bringup',
            executable='smart_base_node',
            name='smart_base_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), {
                'port': LaunchConfiguration('port'),
                'baud_rate': ParameterValue(LaunchConfiguration('baud_rate'), value_type=int),
                'enable_cmd_vel_timeout': ParameterValue(
                    LaunchConfiguration('enable_cmd_vel_timeout'), value_type=bool),
            }],
        ),
        Node(
            package='smart_bringup', executable='smart_imu_filter_node',
            name='smart_imu_filter', output='screen',
            condition=IfCondition(LaunchConfiguration('use_imu_filter')),
            parameters=[LaunchConfiguration('imu_filter_params_file')],
        ),
    ])
