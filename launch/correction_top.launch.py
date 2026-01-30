#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取包的共享目录
    pkg_share = get_package_share_directory('point_cloud_distortion_correction')
    
    # 配置文件路径
    config_file = os.path.join(pkg_share, 'config', 'correction_top_params.yaml')
    
    # 声明启动参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to config file'
    )
    
    # 创建节点
    correction_node = Node(
        package='point_cloud_distortion_correction',
        executable='point_cloud_distortion_correction_node',
        name='correction_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[],
    )
    
    return LaunchDescription([
        config_file_arg,
        correction_node,
    ])

