#!/usr/bin/env python3
"""
VR to MoveIt Servo Pose Launch File
启动VR数据接收和ROS2位姿目标转换系统
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
            default_value=PathJoinSubstitution(['src', 'vr_teleop_pose', 'config', 'teleop_params.yaml']),
            description='Path to the configuration file'
        ),
        DeclareLaunchArgument(
            'update_rate',
            default_value='90.0',
            description='VR tracking update rate (Hz)'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='50.0',
            description='Command publish rate for moveit_servo (Hz)'
        ),
    ]

    # ========== 配置文件路径 ==========
    config_file_path = PathJoinSubstitution([
        LaunchConfiguration('config_file')
    ])

    # ========== 节点定义 ==========
    vr_tracker_node = Node(
        package='vr_teleop_pose',
        executable='vr_tracker_node',
        name='vr_tracker_node',
        output='screen',
        parameters=[
            config_file_path,
            {'update_rate': LaunchConfiguration('update_rate')}
        ]
    )

    vr_pose_converter_node = Node(
        package='vr_teleop_pose',
        executable='vr_pose_converter_node',
        name='vr_converter_node',
        output='screen',
        parameters=[
            config_file_path,
            {
                'publish_rate': LaunchConfiguration('publish_rate')
            }
        ]
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

    return LaunchDescription(
        declared_arguments + [
            LogInfo(msg='='*60),
            LogInfo(msg='VR to MoveIt Servo Pose System Starting...'),
            LogInfo(msg='='*60),
            GroupAction([
                vr_tracker_node,
                vr_pose_converter_node,
                OpaqueFunction(function=_make_gripper_tcp_tf),
            ]),
            LogInfo(msg='VR tracking and pose conversion nodes launched.'),
            LogInfo(msg='Waiting for VR connection and MoveIt Servo...'),
        ]
    )
