#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_dir = get_package_share_directory('insta360_ros_driver')
    extrinsics = os.path.join(pkg_dir, 'config', 'extrinsics_x2.json')
    intrinsics = os.path.join(pkg_dir, 'config', 'intrinsics.yaml')

    # Declare launch arguments
    undistort_arg = DeclareLaunchArgument('undistort', default_value='false', description='Enable undistortion')

    equirectangular_arg = DeclareLaunchArgument(
        'equirectangular',
        default_value='false',
        description='Enable equirectangular projection (disable for offline processing)',
    )

    config_arg = DeclareLaunchArgument('config',
                                       default_value='config.yaml',
                                       description='Path to the configuration file')

    exposure_mode_arg = DeclareLaunchArgument(
        'exposure_mode',
        default_value='auto',
        description='Camera exposure mode: "auto" (SDK default) or "manual" (applies iso/shutter_speed)',
    )

    iso_arg = DeclareLaunchArgument('iso',
                                    default_value='400',
                                    description='ISO value used when exposure_mode is "manual"')

    shutter_speed_arg = DeclareLaunchArgument(
        'shutter_speed',
        default_value='0.008',
        description='Shutter speed in seconds used when exposure_mode is "manual" (default 1/125s)',
    )

    video_resolution_arg = DeclareLaunchArgument(
        'video_resolution',
        default_value='3840x1920@20',
        description='Live stream resolution, one of: 1920x960@30, 2560x1280@30, 3840x1920@20, '
        '3840x1920@30 (4K is the effective ceiling on real X3 hardware)',
    )

    stream_retry_limit_arg = DeclareLaunchArgument(
        'stream_retry_limit',
        default_value='0',
        description='Max StartLiveStreaming retry attempts before giving up (0 = unlimited, default). '
        'Only set a finite value when explicitly testing an unsupported/untested video_resolution.',
    )

    # Define the bringup node with parameters
    bringup_node = Node(
        package='insta360_ros_driver',
        executable='insta360_ros_driver',
        name='insta360_bringup',
        parameters=[
            PathJoinSubstitution([FindPackageShare('insta360_ros_driver'), 'config',
                                  LaunchConfiguration('config')]),
            {
                'exposure_mode': LaunchConfiguration('exposure_mode'),
                'iso': LaunchConfiguration('iso'),
                'shutter_speed': LaunchConfiguration('shutter_speed'),
                'video_resolution': LaunchConfiguration('video_resolution'),
                'stream_retry_limit': LaunchConfiguration('stream_retry_limit'),
            },
        ],
        output='screen',
    )

    imu_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[PathJoinSubstitution([FindPackageShare('insta360_ros_driver'), 'config', 'imu_filter.yaml'])],
    )

    equirectangular_node = Node(
        package='insta360_ros_driver',
        executable='equirectangular.py',
        name='equirectangular_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('equirectangular')),
        arguments=['--gpu', '--calibration_file', extrinsics],
    )

    undistort_node = Node(
        package='insta360_ros_driver',
        executable='undistort.py',
        name='undistort_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('undistort')),
        parameters=[intrinsics],
    )

    ld = LaunchDescription()

    ld.add_action(undistort_arg)
    ld.add_action(config_arg)
    ld.add_action(equirectangular_arg)
    ld.add_action(exposure_mode_arg)
    ld.add_action(iso_arg)
    ld.add_action(shutter_speed_arg)
    ld.add_action(video_resolution_arg)
    ld.add_action(stream_retry_limit_arg)
    ld.add_action(bringup_node)
    ld.add_action(imu_node)
    ld.add_action(equirectangular_node)
    ld.add_action(undistort_node)

    return ld
