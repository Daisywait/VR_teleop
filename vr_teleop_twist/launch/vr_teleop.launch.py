#!/usr/bin/env python3
"""
VR to MoveIt Servo Launch File
启动VR数据接收和ROS2消息转换系统
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    # ========== 参数声明 ==========
    declared_arguments = [
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution(['src', 'vr_teleop_twist', 'config', 'teleop_params.yaml']),
            description='Path to the configuration file'
        ),
        DeclareLaunchArgument(
            'update_rate',
            default_value='90.0',
            description='VR tracking update rate (Hz)'
        ),
    ]

    # ========== 配置文件路径 ==========
    config_file_path = PathJoinSubstitution([
        LaunchConfiguration('config_file')
    ])

    # ========== 节点定义 ==========

    # VR追踪节点 - 读取SteamVR/ALVR数据并发布到ROS2
    vr_tracker_node = Node(
        package='vr_teleop_twist',
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
        package='vr_teleop_twist',
        executable='vr_converter_node',
        name='vr_converter_node',
        output='screen',
        parameters=[config_file_path]
    )

    def _make_gripper_tcp_tf(context):
        config_path = LaunchConfiguration('config_file').perform(context)
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
        except FileNotFoundError:
            data = {}

        params = (
            data.get('vr_converter_node', {})
            .get('ros__parameters', {})
        )
        xyz = params.get('gripper_tcp_xyz', [0.0, 0.0, 0.0])
        rpy = params.get('gripper_tcp_rpy', [0.0, 0.0, 0.0])
        xyz = [float(v) for v in (xyz + [0.0, 0.0, 0.0])[:3]]
        rpy = [float(v) for v in (rpy + [0.0, 0.0, 0.0])[:3]]

        return [
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='gripper_tcp_static_tf',
                output='screen',
                arguments=[
                    '--frame-id', 'robotiq_85_base_link',
                    '--child-frame-id', 'robotiq_85_tcp',
                    '--x', str(xyz[0]),
                    '--y', str(xyz[1]),
                    '--z', str(xyz[2]),
                    '--roll', str(rpy[0]),
                    '--pitch', str(rpy[1]),
                    '--yaw', str(rpy[2]),
                ],
            )
        ]

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
                OpaqueFunction(function=_make_gripper_tcp_tf),
            ]),

            LogInfo(msg='VR tracking and servo conversion nodes launched.'),
            LogInfo(msg='Waiting for VR connection and MoveIt Servo...'),
        ]
    )
