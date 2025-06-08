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
    
    # デフォルトのYAMLファイルパス
    default_yaml_path = os.path.join(package_dir, 'config', 'intrinsics_left.yaml')
    
    # Launch引数の宣言
    yaml_file_arg = DeclareLaunchArgument(
        'yaml_file_path',
        default_value=default_yaml_path,
        description='Path to the camera calibration YAML file'
    )
    
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='camera_info',
        description='Topic name for camera info messages (uses remapping)'
    )
    
    # Camera info publisherノード
    camera_info_node = Node(
        package='insta360_ros_driver',
        executable='camera_info_publisher',
        name='camera_info_publisher',
        parameters=[
            LaunchConfiguration('yaml_file_path')  # YAMLファイルをパラメータとして読み込み
        ],
        remappings=[
            ('camera_info', LaunchConfiguration('camera_info_topic'))  # トピック名のremapping
        ],
        output='screen'
    )
    
    return LaunchDescription([
        yaml_file_arg,
        camera_info_topic_arg,
        camera_info_node
    ])