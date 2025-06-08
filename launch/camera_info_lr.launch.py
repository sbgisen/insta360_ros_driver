#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # パッケージのディレクトリを取得
    package_dir = get_package_share_directory('insta360_ros_driver')
    
    camera_info_node_left = Node(
        package='insta360_ros_driver',
        executable='camera_info_publisher',
        name='camera_info_publisher_left',
        parameters=[
            os.path.join(package_dir, 'config', 'intrinsics_left.yaml')  # YAMLファイルをパラメータとして読み込み
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
            os.path.join(package_dir, 'config', 'intrinsics_right.yaml')  # YAMLファイルをパラメータとして読み込み
        ],
        remappings=[
            ('camera_info', '/right/camera_info')  # トピック名のremapping
        ],
        output='screen'
    )
    
    return LaunchDescription([
        camera_info_node_left,
        camera_info_node_right
    ])