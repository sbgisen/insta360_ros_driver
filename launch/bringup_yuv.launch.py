#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('insta360_ros_driver')

    camera_info_node_left = Node(
        package='insta360_ros_driver',
        executable='camera_info_publisher',
        name='camera_info_publisher_left',
        parameters=[
            os.path.join(pkg_dir, 'config', 'intrinsics_left.yaml')  # YAMLファイルをパラメータとして読み込み
        ],
        remappings=[
            ('camera_info', '/left/camera_info')  # トピック名のremapping
        ],
        output='screen'
    )

    camera_info_node_right = Node(
        package='insta360_ros_driver',
        executable='camera_info_publisher',
        name='camera_info_publisher_right',
        parameters=[
            os.path.join(pkg_dir, 'config', 'intrinsics_right.yaml')  # YAMLファイルをパラメータとして読み込み
        ],
        remappings=[
            ('camera_info', '/right/camera_info')  # トピック名のremapping
        ],
        output='screen'
    )

    bringup_node = Node(
        package='insta360_ros_driver',
        executable='yuv_driver',
        name='insta360_bringup_yuv',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(camera_info_node_left)
    ld.add_action(camera_info_node_right)
    ld.add_action(bringup_node)

    return ld
