#!/usr/bin/env python3
"""
VR ROS2 Node
将VR手柄数据发布到ROS2话题

发布的话题:
- /vr/right_controller/pose (geometry_msgs/PoseStamped)
- /vr/left_controller/pose (geometry_msgs/PoseStamped)
- /vr/right_controller/state (vr_teleop/ControllerState) [自定义消息或使用std_msgs]
- /vr/right_controller/trigger (std_msgs/Float32)
- /vr/right_controller/velocity (geometry_msgs/Twist)
- TF: vr_origin -> vr_controller_right/left
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped, Vector3
from std_msgs.msg import Float32, Bool, Header
from sensor_msgs.msg import Joy
from tf2_ros import TransformBroadcaster
import numpy as np

try:
    import transforms3d.quaternions as quat
    TRANSFORMS3D_AVAILABLE = True
except ImportError:
    TRANSFORMS3D_AVAILABLE = False
    print("[Warning] transforms3d not found, using fallback quaternion conversion")

from .vr_controller_reader import VRControllerReader, ControllerData


def rotation_matrix_to_quaternion(R: np.ndarray) -> tuple:
    """
    将3x3旋转矩阵转换为四元数 [w, x, y, z]
    如果transforms3d不可用则使用此备用实现
    """
    if TRANSFORMS3D_AVAILABLE:
        return quat.mat2quat(R)

    # 备用实现
    trace = np.trace(R)
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (w, x, y, z)


class VRRos2Node(Node):
    """
    VR数据ROS2发布节点

    参数:
        update_rate: 更新频率 (Hz), 默认90
        publish_tf: 是否发布TF变换, 默认True
        frame_id: 基准坐标系名称, 默认'vr_origin'
    """

    def __init__(self):
        super().__init__('vr_tracker_node')

        # 声明参数
        self.declare_parameter('update_rate', 90.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('frame_id', 'vr_origin')
        self.declare_parameter('enable_left_controller', True)
        self.declare_parameter('enable_right_controller', True)

        # 获取参数
        self.update_rate = self.get_parameter('update_rate').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.frame_id = self.get_parameter('frame_id').value
        self.enable_left = self.get_parameter('enable_left_controller').value
        self.enable_right = self.get_parameter('enable_right_controller').value

        # QoS设置 - 低延迟配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers - 右手
        if self.enable_right:
            self.right_pose_pub = self.create_publisher(
                PoseStamped, '/vr/right_controller/pose', qos_profile)
            self.right_trigger_pub = self.create_publisher(
                Float32, '/vr/right_controller/trigger', qos_profile)
            self.right_grip_pub = self.create_publisher(
                Bool, '/vr/right_controller/grip', qos_profile)
            self.right_velocity_pub = self.create_publisher(
                Twist, '/vr/right_controller/velocity', qos_profile)
            self.right_joy_pub = self.create_publisher(
                Joy, '/vr/right_controller/joy', qos_profile)

        # Publishers - 左手
        if self.enable_left:
            self.left_pose_pub = self.create_publisher(
                PoseStamped, '/vr/left_controller/pose', qos_profile)
            self.left_trigger_pub = self.create_publisher(
                Float32, '/vr/left_controller/trigger', qos_profile)
            self.left_grip_pub = self.create_publisher(
                Bool, '/vr/left_controller/grip', qos_profile)
            self.left_velocity_pub = self.create_publisher(
                Twist, '/vr/left_controller/velocity', qos_profile)
            self.left_joy_pub = self.create_publisher(
                Joy, '/vr/left_controller/joy', qos_profile)

        # TF广播器
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # 初始化VR读取器
        self.vr_reader = None
        self._init_vr()

        # 定时器
        period = 1.0 / self.update_rate
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(f'VR Tracker Node started at {self.update_rate} Hz')

    def _init_vr(self) -> bool:
        """初始化VR系统"""
        try:
            self.vr_reader = VRControllerReader()
            self.get_logger().info('VR system initialized successfully')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to initialize VR system: {e}')
            self.get_logger().error('Make sure SteamVR is running')
            return False

    def _create_pose_msg(self, data: ControllerData) -> PoseStamped:
        """从控制器数据创建PoseStamped消息"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # 位置
        msg.pose.position.x = float(data.position[0])
        msg.pose.position.y = float(data.position[1])
        msg.pose.position.z = float(data.position[2])

        # 旋转 (四元数)
        q = rotation_matrix_to_quaternion(data.rotation_matrix)
        msg.pose.orientation.w = float(q[0])
        msg.pose.orientation.x = float(q[1])
        msg.pose.orientation.y = float(q[2])
        msg.pose.orientation.z = float(q[3])

        return msg

    def _create_velocity_msg(self, data: ControllerData) -> Twist:
        """从控制器数据创建Twist消息"""
        msg = Twist()
        msg.linear.x = float(data.velocity[0])
        msg.linear.y = float(data.velocity[1])
        msg.linear.z = float(data.velocity[2])
        msg.angular.x = float(data.angular_velocity[0])
        msg.angular.y = float(data.angular_velocity[1])
        msg.angular.z = float(data.angular_velocity[2])
        return msg

    def _create_joy_msg(self, data: ControllerData) -> Joy:
        """从控制器数据创建Joy消息 (兼容标准游戏手柄格式)"""
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()

        # axes: [trackpad_x, trackpad_y, trigger, grip]
        msg.axes = [
            float(data.state.trackpad_x),
            float(data.state.trackpad_y),
            float(data.state.trigger),
            float(data.state.grip)
        ]

        # buttons: [trigger, grip, menu, trackpad, a, b]
        msg.buttons = [
            int(data.state.trigger_pressed),
            int(data.state.grip_pressed),
            int(data.state.menu_pressed),
            int(data.state.trackpad_pressed),
            int(data.state.a_pressed),
            int(data.state.b_pressed)
        ]

        return msg

    def _publish_tf(self, data: ControllerData, child_frame: str) -> None:
        """发布TF变换"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = child_frame

        t.transform.translation.x = float(data.position[0])
        t.transform.translation.y = float(data.position[1])
        t.transform.translation.z = float(data.position[2])

        q = rotation_matrix_to_quaternion(data.rotation_matrix)
        t.transform.rotation.w = float(q[0])
        t.transform.rotation.x = float(q[1])
        t.transform.rotation.y = float(q[2])
        t.transform.rotation.z = float(q[3])

        self.tf_broadcaster.sendTransform(t)

    def _publish_controller_data(self, data: ControllerData, hand: str) -> None:
        """发布单个控制器的所有数据"""
        if not data.is_valid:
            return

        if hand == 'right':
            # 位姿
            self.right_pose_pub.publish(self._create_pose_msg(data))
            # 扳机
            trigger_msg = Float32()
            trigger_msg.data = data.state.trigger
            self.right_trigger_pub.publish(trigger_msg)
            # 握持
            grip_msg = Bool()
            grip_msg.data = data.state.grip_pressed
            self.right_grip_pub.publish(grip_msg)
            # 速度
            self.right_velocity_pub.publish(self._create_velocity_msg(data))
            # Joy
            self.right_joy_pub.publish(self._create_joy_msg(data))
            # TF
            if self.publish_tf:
                self._publish_tf(data, 'vr_controller_right')

        elif hand == 'left':
            self.left_pose_pub.publish(self._create_pose_msg(data))
            trigger_msg = Float32()
            trigger_msg.data = data.state.trigger
            self.left_trigger_pub.publish(trigger_msg)
            grip_msg = Bool()
            grip_msg.data = data.state.grip_pressed
            self.left_grip_pub.publish(grip_msg)
            self.left_velocity_pub.publish(self._create_velocity_msg(data))
            self.left_joy_pub.publish(self._create_joy_msg(data))
            if self.publish_tf:
                self._publish_tf(data, 'vr_controller_left')

    def timer_callback(self) -> None:
        """定时器回调 - 读取并发布VR数据"""
        if self.vr_reader is None:
            # 尝试重新初始化
            if not self._init_vr():
                return

        try:
            # 读取右手控制器
            if self.enable_right:
                right_data = self.vr_reader.get_controller_data('right')
                if right_data:
                    self._publish_controller_data(right_data, 'right')

            # 读取左手控制器
            if self.enable_left:
                left_data = self.vr_reader.get_controller_data('left')
                if left_data:
                    self._publish_controller_data(left_data, 'left')

        except Exception as e:
            self.get_logger().error(f'Error reading VR data: {e}')

    def destroy_node(self) -> None:
        """清理资源"""
        if self.vr_reader:
            self.vr_reader.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = VRRos2Node()

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
