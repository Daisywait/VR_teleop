#!/usr/bin/env python3
"""
VR to MoveIt Servo Node (HMD-relative)
接收VR控制器/HMD位姿并转换为ROS2速度指令发送给moveit_servo

订阅:
- /vr/right_controller/pose_hmd (geometry_msgs/PoseStamped) - 右手相对头显位姿
- /vr/right_controller/trigger (std_msgs/Float32) - 右手扳机值
- /vr/right_controller/joystick_y (std_msgs/Float32) - 右手摇杆Y轴

发布:
- twist_topic 参数指定的话题 (geometry_msgs/TwistStamped) - 末端速度命令
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from std_msgs.msg import Float32
from control_msgs.action import GripperCommand
from moveit_msgs.srv import ServoCommandType
import tf2_ros
from tf2_ros import TransformException
import numpy as np
from typing import Optional


def _quat_normalize_xyzw(q: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q / norm


def _quat_inv_xyzw(q: np.ndarray) -> np.ndarray:
    q = _quat_normalize_xyzw(q)
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=float)


def _quat_mul_xyzw(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    ], dtype=float)


def _quat_to_rotvec_xyzw(q: np.ndarray) -> np.ndarray:
    q = _quat_normalize_xyzw(q)
    x, y, z, w = q
    w = float(np.clip(w, -1.0, 1.0))
    angle = 2.0 * np.arccos(w)
    s = np.sqrt(max(1.0 - w * w, 0.0))
    if s < 1e-6 or angle < 1e-6:
        return np.array([2.0 * x, 2.0 * y, 2.0 * z], dtype=float)
    axis = np.array([x, y, z], dtype=float) / s
    return axis * angle



def _quat_to_rotmat_xyzw(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)]
    ], dtype=float)


def _rotvec_to_quat_xyzw(rotvec: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(rotvec))
    if angle < 1e-9:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    axis = rotvec / angle
    half = 0.5 * angle
    s = np.sin(half)
    return np.array([axis[0] * s, axis[1] * s, axis[2] * s, np.cos(half)], dtype=float)


def _pose_compose(p1: np.ndarray, q1: np.ndarray, p2: np.ndarray, q2: np.ndarray) -> tuple:
    q1 = _quat_normalize_xyzw(q1)
    q2 = _quat_normalize_xyzw(q2)
    r1 = _quat_to_rotmat_xyzw(q1)
    p = p1 + r1 @ p2
    q = _quat_mul_xyzw(q1, q2)
    q = _quat_normalize_xyzw(q)
    return p, q


def _pose_inverse(p: np.ndarray, q: np.ndarray) -> tuple:
    q = _quat_normalize_xyzw(q)
    q_i = _quat_inv_xyzw(q)
    r_i = _quat_to_rotmat_xyzw(q_i)
    p_i = -r_i @ p
    return p_i, q_i


def _pose_error(p_cur: np.ndarray, q_cur: np.ndarray,
                p_des: np.ndarray, q_des: np.ndarray) -> tuple:
    p_ci, q_ci = _pose_inverse(p_cur, q_cur)
    p_err, q_err = _pose_compose(p_ci, q_ci, p_des, q_des)
    rotvec = _quat_to_rotvec_xyzw(q_err)
    return p_err, rotvec


class VRMessageConverterNode(Node):
    """
    VR到ROS2消息转换节点
    
    从VR控制器接收数据，转换为ROS2消息格式，发送给moveit_servo
    """

    def __init__(self):
        super().__init__('vr_message_converter_node')

        # ========== 参数声明 ==========
        self.declare_parameter('linear_scale', 2.1)       # 线速度缩放
        self.declare_parameter('angular_scale', 0.4)      # 角速度缩放
        self.declare_parameter('v_max', 0.15)             # 最大线速度 m/s
        self.declare_parameter('w_max', 1.0)              # 最大角速度 rad/s
        self.declare_parameter('smoothing_factor', 0.3)   # 低通滤波
        self.declare_parameter('deadzone_linear', 0.002)  # 线速度死区 m
        self.declare_parameter('deadzone_angular', 0.03)  # 角速度死区 rad
        self.declare_parameter('trigger_threshold', 0.5)  # 触发阈值
        self.declare_parameter('planning_frame', 'fr3_link0')  # 参考坐标系
        self.declare_parameter('ee_frame', 'robotiq_85_base_link')
        self.declare_parameter('publish_rate', 50.0)      # 发布频率 Hz
        # 固定旋转补偿已移除，依赖TF对齐
        self.declare_parameter('twist_topic', '/moveit_servo/delta_twist_cmds')
        self.declare_parameter('gripper_action', '/robotiq_gripper_controller/gripper_cmd')
        self.declare_parameter('gripper_open_pos', 0.0)
        self.declare_parameter('gripper_close_pos', 0.8)
        self.declare_parameter('gripper_force', 50.0)
        self.declare_parameter('gripper_speed', 0.8)
        self.declare_parameter('gripper_axis_deadzone', 0.08)
        self.declare_parameter('gripper_deadband', 0.01)
        self.declare_parameter('gripper_rate', 15.0)
        self.declare_parameter('servo_command_type_service', '/servo_node/switch_command_type')
        self.declare_parameter('servo_command_type', 1)

        # ========== 获取参数 ==========
        self.linear_scale = float(self.get_parameter('linear_scale').value)
        self.angular_scale = float(self.get_parameter('angular_scale').value)
        self.max_linear_vel = float(self.get_parameter('v_max').value)
        self.max_angular_vel = float(self.get_parameter('w_max').value)
        self.smoothing_factor = float(self.get_parameter('smoothing_factor').value)
        self.deadzone_linear = float(self.get_parameter('deadzone_linear').value)
        self.deadzone_angular = float(self.get_parameter('deadzone_angular').value)
        self.trigger_threshold = self.get_parameter('trigger_threshold').value
        self.reference_frame = self.get_parameter('planning_frame').value
        self.ee_frame = self.get_parameter('ee_frame').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.twist_topic = self.get_parameter('twist_topic').value
        self.gripper_action = self.get_parameter('gripper_action').value
        self.gripper_open_pos = float(self.get_parameter('gripper_open_pos').value)
        self.gripper_close_pos = float(self.get_parameter('gripper_close_pos').value)
        self.gripper_force = float(self.get_parameter('gripper_force').value)
        self.gripper_speed = float(self.get_parameter('gripper_speed').value)
        self.gripper_axis_deadzone = float(self.get_parameter('gripper_axis_deadzone').value)
        self.gripper_deadband = float(self.get_parameter('gripper_deadband').value)
        self.gripper_rate = float(self.get_parameter('gripper_rate').value)
        self.servo_command_type_service = str(
            self.get_parameter('servo_command_type_service').value
        )
        self.servo_command_type = int(self.get_parameter('servo_command_type').value)

        # 固定旋转补偿已移除，依赖TF对齐

        # ========== QoS配置 ==========
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ========== 订阅VR数据 ==========
        self.create_subscription(
            PoseStamped, '/vr/right_controller/pose_hmd',
            self._controller_pose_callback, qos_sub)

        self.create_subscription(
            Float32, '/vr/right_controller/trigger',
            self._trigger_callback, qos_sub)

        self.create_subscription(
            Float32, '/vr/right_controller/joystick_y',
            self._joystick_callback, qos_sub)

        # ========== 发布ROS2消息给moveit_servo ==========
        self.twist_pub = self.create_publisher(
            TwistStamped, self.twist_topic, qos_pub)

        # 夹爪 Action
        self.gripper_client = ActionClient(
            self, GripperCommand, self.gripper_action)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Servo command type client
        self.servo_command_type_client = self.create_client(
            ServoCommandType,
            self.servo_command_type_service,
        )
        self.servo_command_type_set = False
        self.servo_command_type_pending = False
        self.servo_command_type_timer = self.create_timer(1.0, self.try_set_command_type)

        # ========== 状态变量 ==========
        self.current_controller_pose: Optional[PoseStamped] = None
        self.trigger_value = 0.0
        self.enabled = False
        self.anchor_set = False
        self.ee_anchor_p: Optional[np.ndarray] = None
        self.ee_anchor_q: Optional[np.ndarray] = None
        self.vr_anchor_p: Optional[np.ndarray] = None
        self.vr_anchor_q: Optional[np.ndarray] = None
        self.target_twist = TwistStamped()
        self.joystick_y = 0.0
        self.gripper_target_pos = self.gripper_open_pos
        self.last_gripper_pos_sent = None

        # ========== 定时器发布速度命令 ==========
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self.publish_velocity_command)
        gripper_period = 1.0 / self.gripper_rate if self.gripper_rate > 0 else 0.1
        self.gripper_timer = self.create_timer(gripper_period, self.update_gripper_command)

        self.get_logger().info('='*60)
        self.get_logger().info('VR Message Converter Node Started')
        self.get_logger().info(f'  Linear scale: {self.linear_scale}')
        self.get_logger().info(f'  Angular scale: {self.angular_scale}')
        self.get_logger().info(f'  Max linear velocity: {self.max_linear_vel} m/s')
        self.get_logger().info(f'  Max angular velocity: {self.max_angular_vel} rad/s')
        self.get_logger().info(f'  Publish rate: {self.publish_rate} Hz')
        self.get_logger().info(f'  Reference frame: {self.reference_frame}')
        self.get_logger().info(f'  Twist topic: {self.twist_topic}')
        self.get_logger().info('='*60)
        self.get_logger().info('Waiting for VR controller data...')

    def try_set_command_type(self) -> None:
        """尝试设置 MoveIt Servo 命令类型（Twist/Speed Units）"""
        if self.servo_command_type_set or self.servo_command_type_pending:
            return
        if not self.servo_command_type_client.wait_for_service(timeout_sec=0.1):
            return
        req = ServoCommandType.Request()
        req.command_type = int(self.servo_command_type)
        future = self.servo_command_type_client.call_async(req)
        self.servo_command_type_pending = True
        future.add_done_callback(self._on_servo_command_type_response)

    def _on_servo_command_type_response(self, future) -> None:
        self.servo_command_type_pending = False
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().warn(f'Failed to set servo command type: {exc}')
            return
        if hasattr(resp, 'success') and not resp.success:
            self.get_logger().warn(
                f'Failed to set servo command type: {getattr(resp, "message", "")}'
            )
            return
        self.servo_command_type_set = True
        self.servo_command_type_timer.cancel()


    def _controller_pose_callback(self, msg: PoseStamped) -> None:
        """接收VR控制器相对头显位姿"""
        self.current_controller_pose = msg

    def _trigger_callback(self, msg: Float32) -> None:
        """接收VR控制器扳机值"""
        old_enabled = self.enabled
        self.trigger_value = msg.data
        
        # 扳机值超过阈值则启用控制
        self.enabled = self.trigger_value >= self.trigger_threshold
        
        if self.enabled and not old_enabled:
            self.get_logger().info('✓ VR Control ENABLED')
            self.anchor_set = False
        elif not self.enabled and old_enabled:
            self.get_logger().info('✗ VR Control DISABLED')

    def _joystick_callback(self, msg: Float32) -> None:
        """接收右手摇杆Y轴"""
        self.joystick_y = float(msg.data)

    def _get_current_ee_pose(self) -> tuple:
        """获取当前末端位姿"""
        try:
            trans = self.tf_buffer.lookup_transform(
                self.reference_frame, self.ee_frame, rclpy.time.Time())
            p = np.array([
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z
            ], dtype=float)
            q = np.array([
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w
            ], dtype=float)
            q = _quat_normalize_xyzw(q)
            return p, q
        except TransformException as exc:
            self.get_logger().warn(
                f'TF lookup failed ({self.reference_frame} -> {self.ee_frame}): {exc}')
            return None, None

    def _calculate_velocity(self) -> Optional[Twist]:
        """基于锚点的VR相对位姿 -> 末端位姿误差 -> 速度命令"""
        if self.current_controller_pose is None:
            return None

        ee_p, ee_q = self._get_current_ee_pose()
        if ee_p is None:
            return None

        vr_p_now = np.array([
            self.current_controller_pose.pose.position.x,
            self.current_controller_pose.pose.position.y,
            self.current_controller_pose.pose.position.z
        ], dtype=float)
        vr_q_now = _quat_normalize_xyzw(np.array([
            self.current_controller_pose.pose.orientation.x,
            self.current_controller_pose.pose.orientation.y,
            self.current_controller_pose.pose.orientation.z,
            self.current_controller_pose.pose.orientation.w
        ], dtype=float))

        if not self.anchor_set:
            self.ee_anchor_p = ee_p.copy()
            self.ee_anchor_q = ee_q.copy()
            self.vr_anchor_p = vr_p_now.copy()
            self.vr_anchor_q = vr_q_now.copy()
            self.anchor_set = True
            return Twist()

        vr_anchor_pi, vr_anchor_qi = _pose_inverse(self.vr_anchor_p, self.vr_anchor_q)
        dvr_p, dvr_q = _pose_compose(vr_anchor_pi, vr_anchor_qi, vr_p_now, vr_q_now)

        dp_robot = dvr_p
        rotvec_raw = _quat_to_rotvec_xyzw(dvr_q)
        dq_robot = _rotvec_to_quat_xyzw(rotvec_raw)

        p_des, q_des = _pose_compose(self.ee_anchor_p, self.ee_anchor_q, dp_robot, dq_robot)
        pos_err, rotvec_err = _pose_error(ee_p, ee_q, p_des, q_des)
        # 保留完整旋转（不限制轴）

        linear_vel = pos_err * self.linear_scale
        angular_vel = rotvec_err * self.angular_scale

        # 线速度死区
        if np.linalg.norm(linear_vel) < self.deadzone_linear:
            linear_vel[:] = 0.0
        # 角速度死区
        if np.linalg.norm(angular_vel) < self.deadzone_angular:
            angular_vel[:] = 0.0

        # 速度限制
        linear_mag = np.linalg.norm(linear_vel)
        if linear_mag > self.max_linear_vel:
            linear_vel = linear_vel / linear_mag * self.max_linear_vel

        angular_mag = np.linalg.norm(angular_vel)
        if angular_mag > self.max_angular_vel:
            angular_vel = angular_vel / angular_mag * self.max_angular_vel

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
        if self.current_controller_pose is None:
            return

        # 未启用时发送零速度命令
        if not self.enabled:
            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.header.frame_id = self.reference_frame
            self.twist_pub.publish(twist_msg)
            self.anchor_set = False
            return

        # 计算速度
        twist = self._calculate_velocity()
        if twist is None:
            return

        # 低通滤波
        twist.linear.x = (1.0 - self.smoothing_factor) * self.target_twist.twist.linear.x + \
            self.smoothing_factor * twist.linear.x
        twist.linear.y = (1.0 - self.smoothing_factor) * self.target_twist.twist.linear.y + \
            self.smoothing_factor * twist.linear.y
        twist.linear.z = (1.0 - self.smoothing_factor) * self.target_twist.twist.linear.z + \
            self.smoothing_factor * twist.linear.z
        twist.angular.x = (1.0 - self.smoothing_factor) * self.target_twist.twist.angular.x + \
            self.smoothing_factor * twist.angular.x
        twist.angular.y = (1.0 - self.smoothing_factor) * self.target_twist.twist.angular.y + \
            self.smoothing_factor * twist.angular.y
        twist.angular.z = (1.0 - self.smoothing_factor) * self.target_twist.twist.angular.z + \
            self.smoothing_factor * twist.angular.z

        self.target_twist.twist = twist

        # 创建TwistStamped消息
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self.reference_frame
        twist_msg.twist = self.target_twist.twist

        # 发布到moveit_servo
        self.twist_pub.publish(twist_msg)

    def update_gripper_command(self) -> None:
        """根据摇杆控制夹爪"""
        if self.gripper_rate <= 0.0:
            return

        if not self.gripper_client.wait_for_server(timeout_sec=0.05):
            return

        axis = float(self.joystick_y)
        if abs(axis) < self.gripper_axis_deadzone:
            axis = 0.0

        dt = 1.0 / self.gripper_rate
        delta = axis * self.gripper_speed * dt * (self.gripper_close_pos - self.gripper_open_pos)
        self.gripper_target_pos += delta
        self.gripper_target_pos = float(
            np.clip(self.gripper_target_pos,
                    min(self.gripper_open_pos, self.gripper_close_pos),
                    max(self.gripper_open_pos, self.gripper_close_pos))
        )

        if self.last_gripper_pos_sent is not None:
            if abs(self.gripper_target_pos - self.last_gripper_pos_sent) < self.gripper_deadband:
                return

        goal = GripperCommand.Goal()
        goal.command.position = self.gripper_target_pos
        goal.command.max_effort = self.gripper_force
        self.gripper_client.send_goal_async(goal)
        self.last_gripper_pos_sent = self.gripper_target_pos

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
