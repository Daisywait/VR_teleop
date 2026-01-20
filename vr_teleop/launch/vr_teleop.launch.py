#!/usr/bin/env python3
"""
VR to MoveIt Servo Launch File
启动VR数据接收和ROS2消息转换系统
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ========== 参数声明 ==========
    declared_arguments = [
        DeclareLaunchArgument(
            'config_file',
            default_value='teleop_params.yaml',
            description='Name of the configuration file'
        ),
        DeclareLaunchArgument(
            'update_rate',
            default_value='90.0',
            description='VR tracking update rate (Hz)'
        ),
        DeclareLaunchArgument(
            'linear_speed_scale',
            default_value='0.5',
            description='Linear velocity scaling factor'
        ),
        DeclareLaunchArgument(
            'angular_speed_scale',
            default_value='0.5',
            description='Angular velocity scaling factor'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='50.0',
            description='Command publish rate for moveit_servo (Hz)'
        ),
    ]

    # ========== 配置文件路径 ==========
    config_file_path = PathJoinSubstitution([
        FindPackageShare('vr_teleop'),
        'config',
        LaunchConfiguration('config_file')
    ])

    # ========== 节点定义 ==========

    # VR追踪节点 - 读取SteamVR/ALVR数据并发布到ROS2
    vr_tracker_node = Node(
        package='vr_teleop',
        executable='vr_tracker_node',
        name='vr_tracker_node',
        output='screen',
        parameters=[
            config_file_path,
            {'update_rate': LaunchConfiguration('update_rate')}
        ]
    )

    # VR消息转换节点 - 将VR数据转换为moveit_servo命令
    vr_converter_node = Node(
        package='vr_teleop',
        executable='vr_converter_node',
        name='vr_converter_node',
        output='screen',
        parameters=[
            config_file_path,
            {
                'linear_speed_scale': LaunchConfiguration('linear_speed_scale'),
                'angular_speed_scale': LaunchConfiguration('angular_speed_scale'),
                'publish_rate': LaunchConfiguration('publish_rate')
            }
        ]
    )

    # ========== 启动描述 ==========
    return LaunchDescription(
        declared_arguments + [
            LogInfo(msg='='*60),
            LogInfo(msg='VR to MoveIt Servo System Starting...'),
            LogInfo(msg='='*60),

            # 启动节点组
            GroupAction([
                vr_tracker_node,
                vr_converter_node,
            ]),

            LogInfo(msg='VR tracking and servo conversion nodes launched.'),
            LogInfo(msg='Waiting for VR connection and MoveIt Servo...'),
        ]
    )

