#!/usr/bin/env python3
"""
VR to ROS2 Message Converter Node
接收VR控制器数据并转换为ROS2消息发送给moveit_servo

订阅:
- /vr/right_controller/pose (geometry_msgs/PoseStamped) - 右手控制器位姿
- /vr/right_controller/trigger (std_msgs/Float32) - 右手扳机值

发布:
- /servo_node/delta_twist_cmds (geometry_msgs/TwistStamped) - 末端速度命令给moveit_servo
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from std_msgs.msg import Float32
import numpy as np
from typing import Optional
import time


class VRMessageConverterNode(Node):
    """
    VR到ROS2消息转换节点
    
    从VR控制器接收数据，转换为ROS2消息格式，发送给moveit_servo
    """

    def __init__(self):
        super().__init__('vr_message_converter_node')

        # ========== 参数声明 ==========
        self.declare_parameter('linear_speed_scale', 0.5)      # 线速度缩放因子
        self.declare_parameter('angular_speed_scale', 0.5)     # 角速度缩放因子
        self.declare_parameter('max_linear_velocity', 0.5)     # 最大线速度 m/s
        self.declare_parameter('max_angular_velocity', 1.0)    # 最大角速度 rad/s
        self.declare_parameter('trigger_threshold', 0.5)       # 触发阈值
        self.declare_parameter('reference_frame', 'panda_link0')  # 参考坐标系
        self.declare_parameter('publish_rate', 50.0)           # 发布频率 Hz

        # ========== 获取参数 ==========
        self.linear_speed_scale = self.get_parameter('linear_speed_scale').value
        self.angular_speed_scale = self.get_parameter('angular_speed_scale').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.trigger_threshold = self.get_parameter('trigger_threshold').value
        self.reference_frame = self.get_parameter('reference_frame').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # ========== QoS配置 ==========
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ========== 订阅VR数据 ==========
        self.create_subscription(
            PoseStamped, '/vr/right_controller/pose',
            self._pose_callback, qos)

        self.create_subscription(
            Float32, '/vr/right_controller/trigger',
            self._trigger_callback, qos)

        # ========== 发布ROS2消息给moveit_servo ==========
        self.twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', qos)

        # ========== 状态变量 ==========
        self.current_pose: Optional[PoseStamped] = None
        self.trigger_value = 0.0
        self.enabled = False
        self.last_pose: Optional[PoseStamped] = None
        self.last_time = time.time()

        # ========== 定时器发布速度命令 ==========
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self.publish_velocity_command)

        self.get_logger().info('='*60)
        self.get_logger().info('VR Message Converter Node Started')
        self.get_logger().info(f'  Linear speed scale: {self.linear_speed_scale}')
        self.get_logger().info(f'  Angular speed scale: {self.angular_speed_scale}')
        self.get_logger().info(f'  Max linear velocity: {self.max_linear_vel} m/s')
        self.get_logger().info(f'  Max angular velocity: {self.max_angular_vel} rad/s')
        self.get_logger().info(f'  Publish rate: {self.publish_rate} Hz')
        self.get_logger().info(f'  Reference frame: {self.reference_frame}')
        self.get_logger().info('='*60)
        self.get_logger().info('Waiting for VR controller data...')

    def _pose_callback(self, msg: PoseStamped) -> None:
        """接收VR控制器位姿"""
        self.current_pose = msg

    def _trigger_callback(self, msg: Float32) -> None:
        """接收VR控制器扳机值"""
        old_enabled = self.enabled
        self.trigger_value = msg.data
        
        # 扳机值超过阈值则启用控制
        self.enabled = self.trigger_value >= self.trigger_threshold
        
        if self.enabled and not old_enabled:
            self.get_logger().info('✓ VR Control ENABLED')
        elif not self.enabled and old_enabled:
            self.get_logger().info('✗ VR Control DISABLED')

    def _calculate_velocity(self) -> Optional[Twist]:
        """
        根据VR手柄位置计算速度命令
        
        使用微分方法：v = Δp / Δt
        """
        if self.current_pose is None:
            return None

        current_time = time.time()
        dt = current_time - self.last_time

        if dt <= 0 or self.last_pose is None:
            self.last_pose = self.current_pose
            self.last_time = current_time
            return Twist()

        # 计算位置变化
        dp = np.array([
            self.current_pose.pose.position.x - self.last_pose.pose.position.x,
            self.current_pose.pose.position.y - self.last_pose.pose.position.y,
            self.current_pose.pose.position.z - self.last_pose.pose.position.z
        ])

        # 计算线速度
        linear_vel = dp / dt * self.linear_speed_scale

        # 速度限制
        linear_mag = np.linalg.norm(linear_vel)
        if linear_mag > self.max_linear_vel:
            linear_vel = linear_vel / linear_mag * self.max_linear_vel

        # 简化处理：角速度暂时设为0（可后续扩展）
        angular_vel = np.zeros(3)

        # 更新状态
        self.last_pose = self.current_pose
        self.last_time = current_time

        # 创建Twist消息
        twist = Twist()
        twist.linear.x = float(linear_vel[0])
        twist.linear.y = float(linear_vel[1])
        twist.linear.z = float(linear_vel[2])
        twist.angular.x = float(angular_vel[0])
        twist.angular.y = float(angular_vel[1])
        twist.angular.z = float(angular_vel[2])

        return twist

    def publish_velocity_command(self) -> None:
        """发布速度命令到moveit_servo"""
        if self.current_pose is None:
            return

        # 未启用时发送零速度命令
        if not self.enabled:
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = self.reference_frame
            self.twist_pub.publish(twist_msg)
            return

        # 计算速度
        twist = self._calculate_velocity()
        if twist is None:
            return

        # 创建TwistStamped消息
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self.reference_frame
        twist_msg.twist = twist

        # 发布到moveit_servo
        self.twist_pub.publish(twist_msg)

    def destroy_node(self) -> None:
        """清理资源"""
        self.get_logger().info('VR Message Converter Node shutdown')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = VRMessageConverterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
