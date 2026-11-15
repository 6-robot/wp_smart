#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_camera(context):
    # usb_cam 0.8.1 handles relative by-id symlink targets incorrectly.
    # Resolve at launch time so camera_device overrides and USB reordering work.
    camera_device = os.path.realpath(LaunchConfiguration('camera_device').perform(context))
    return [Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        namespace=LaunchConfiguration('camera_namespace'),
        output='screen',
        parameters=[{
            'video_device': camera_device,
            'camera_name': 'smart_camera',
            'frame_id': ParameterValue(
                LaunchConfiguration('camera_frame_id'), value_type=str),
            'image_width': ParameterValue(
                LaunchConfiguration('camera_width'), value_type=int),
            'image_height': ParameterValue(
                LaunchConfiguration('camera_height'), value_type=int),
            'framerate': ParameterValue(
                LaunchConfiguration('camera_framerate'), value_type=float),
            'pixel_format': ParameterValue(
                LaunchConfiguration('camera_pixel_format'), value_type=str),
            'camera_info_url': ParameterValue(
                LaunchConfiguration('camera_info_url'), value_type=str),
            'io_method': 'mmap',
            'use_sim_time': False,
        }],
    )]


def generate_launch_description():
    package_share = get_package_share_directory('smart_bringup')
    camera_device = (
        '/dev/video0')

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_base', default_value='true',
            description='Start the serial chassis; false previews only the camera'),
        DeclareLaunchArgument(
            'camera_device', default_value=camera_device,
            description='V4L2 image capture device; index0 is the Realtek camera'),
        DeclareLaunchArgument('camera_namespace', default_value='camera'),
        DeclareLaunchArgument('camera_frame_id', default_value='camera_optical_frame'),
        DeclareLaunchArgument('camera_width', default_value='640'),
        DeclareLaunchArgument('camera_height', default_value='480'),
        DeclareLaunchArgument('camera_framerate', default_value='30.0'),
        DeclareLaunchArgument(
            'camera_pixel_format', default_value='mjpeg2rgb',
            description='usb_cam capture/conversion format, e.g. mjpeg2rgb or yuyv'),
        DeclareLaunchArgument(
            'camera_info_url', default_value='',
            description='Optional camera calibration YAML URL'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_share, 'launch', 'bringup.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_base')),
        ),
        OpaqueFunction(function=launch_camera),
    ])
