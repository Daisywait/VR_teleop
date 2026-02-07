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
from std_msgs.msg import Float32, Bool
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


def _twist_copy(src: Twist) -> Twist:
    t = Twist()
    t.linear.x = float(src.linear.x)
    t.linear.y = float(src.linear.y)
    t.linear.z = float(src.linear.z)
    t.angular.x = float(src.angular.x)
    t.angular.y = float(src.angular.y)
    t.angular.z = float(src.angular.z)
    return t


def _twist_scale(src: Twist, scale: float) -> Twist:
    t = Twist()
    t.linear.x = float(src.linear.x * scale)
    t.linear.y = float(src.linear.y * scale)
    t.linear.z = float(src.linear.z * scale)
    t.angular.x = float(src.angular.x * scale)
    t.angular.y = float(src.angular.y * scale)
    t.angular.z = float(src.angular.z * scale)
    return t


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
        self.declare_parameter('enable_threshold', 0.6)   # 触发迟滞 - 启用阈值
        self.declare_parameter('disable_threshold', 0.4)  # 触发迟滞 - 禁用阈值
        self.declare_parameter('use_anchor', False)       # 是否使用锚点逻辑
        self.declare_parameter('z_scale', 0.2)            # z 轴缩放
        self.declare_parameter('soft_stop_duration', 0.2) # 软停持续时间 s
        self.declare_parameter('linear_slew_rate', 0.5)   # 线速度变化率限制 m/s^2
        self.declare_parameter('angular_slew_rate', 1.0)  # 角速度变化率限制 rad/s^2
        self.declare_parameter('speed_mode_transition_time', 0.3)  # 速度模式切换时间 s
        self.declare_parameter('enable_gating', True)     # 连续门控
        self.declare_parameter('gating_bind_to_speed_mode', True)  # 门控绑定速度档位
        self.declare_parameter('gating_bind_use_tau', False)       # 绑定模式是否使用gating_tau低通
        self.declare_parameter('gating_v_ref', 0.25)      # 线速度归一化参考 m/s
        self.declare_parameter('gating_w_ref', 1.0)       # 角速度归一化参考 rad/s
        self.declare_parameter('gating_tau', 0.2)         # 门控alpha低通时间常数 s
        self.declare_parameter('gating_v_min_ratio', 0.25)  # alpha=1时线速度保留比例
        self.declare_parameter('gating_w_min_ratio', 0.25)  # alpha=0时角速度保留比例
        self.declare_parameter('planning_frame', 'fr3_link0')  # 参考坐标系
        self.declare_parameter('ee_frame', 'robotiq_85_base_link')
        self.declare_parameter('publish_rate', 90.0)      # 发布频率 Hz
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
        base_v_max = float(self.get_parameter('v_max').value)
        base_w_max = float(self.get_parameter('w_max').value)
        linear_slew_default = 0.5
        base_linear_slew = float(self.get_parameter('linear_slew_rate').value)
        if base_linear_slew <= 0.0:
            self.get_logger().warn(
                f'linear_slew_rate <= 0, fallback to {linear_slew_default}'
            )
            base_linear_slew = linear_slew_default
        angular_slew_default = 1.0
        base_angular_slew = float(self.get_parameter('angular_slew_rate').value)
        if base_angular_slew <= 0.0:
            self.get_logger().warn(
                f'angular_slew_rate <= 0, fallback to {angular_slew_default}'
            )
            base_angular_slew = angular_slew_default

        # 速度模式参数（默认与旧参数一致，兼容旧配置）
        self.declare_parameter('v_max_fast', base_v_max)
        self.declare_parameter('w_max_fast', base_w_max)
        self.declare_parameter('linear_slew_rate_fast', base_linear_slew)
        self.declare_parameter('angular_slew_rate_fast', base_angular_slew)
        self.declare_parameter('v_max_slow', base_v_max)
        self.declare_parameter('w_max_slow', base_w_max)
        self.declare_parameter('linear_slew_rate_slow', base_linear_slew)
        self.declare_parameter('angular_slew_rate_slow', base_angular_slew)

        self.linear_scale = float(self.get_parameter('linear_scale').value)
        self.angular_scale = float(self.get_parameter('angular_scale').value)
        self.max_linear_vel = base_v_max
        self.max_angular_vel = base_w_max
        self.smoothing_factor = float(self.get_parameter('smoothing_factor').value)
        self.deadzone_linear = float(self.get_parameter('deadzone_linear').value)
        self.deadzone_angular = float(self.get_parameter('deadzone_angular').value)
        self.enable_threshold = float(self.get_parameter('enable_threshold').value)
        self.disable_threshold = float(self.get_parameter('disable_threshold').value)
        self.use_anchor = bool(self.get_parameter('use_anchor').value)
        self.z_scale = float(self.get_parameter('z_scale').value)
        soft_stop_default = 0.2
        self.soft_stop_duration = float(self.get_parameter('soft_stop_duration').value)
        if self.soft_stop_duration <= 0.0:
            self.get_logger().warn(
                f'soft_stop_duration <= 0, fallback to {soft_stop_default}'
            )
            self.soft_stop_duration = soft_stop_default

        self.v_max_fast = float(self.get_parameter('v_max_fast').value)
        self.w_max_fast = float(self.get_parameter('w_max_fast').value)
        self.v_max_slow = float(self.get_parameter('v_max_slow').value)
        self.w_max_slow = float(self.get_parameter('w_max_slow').value)
        self.linear_slew_rate_fast = float(self.get_parameter('linear_slew_rate_fast').value)
        self.angular_slew_rate_fast = float(self.get_parameter('angular_slew_rate_fast').value)
        self.linear_slew_rate_slow = float(self.get_parameter('linear_slew_rate_slow').value)
        self.angular_slew_rate_slow = float(self.get_parameter('angular_slew_rate_slow').value)

        self.speed_mode_transition_time = float(
            self.get_parameter('speed_mode_transition_time').value
        )
        if self.speed_mode_transition_time <= 0.0:
            self.get_logger().warn(
                'speed_mode_transition_time <= 0, fallback to 0.3'
            )
            self.speed_mode_transition_time = 0.3
        self.speed_mode_ramp_rate = 1.0 / self.speed_mode_transition_time

        self.speed_mode = 'fast'
        self.speed_scale = 1.0
        self.speed_scale_target = 1.0
        self._update_speed_params()

        self.enable_gating = bool(self.get_parameter('enable_gating').value)
        self.gating_bind_to_speed_mode = bool(
            self.get_parameter('gating_bind_to_speed_mode').value
        )
        self.gating_bind_use_tau = bool(
            self.get_parameter('gating_bind_use_tau').value
        )
        self.gating_v_ref = float(self.get_parameter('gating_v_ref').value)
        self.gating_w_ref = float(self.get_parameter('gating_w_ref').value)
        self.gating_tau = float(self.get_parameter('gating_tau').value)
        self.gating_v_min_ratio = float(self.get_parameter('gating_v_min_ratio').value)
        self.gating_w_min_ratio = float(self.get_parameter('gating_w_min_ratio').value)

        if not self.gating_bind_to_speed_mode:
            if self.gating_v_ref <= 0.0:
                self.get_logger().warn('gating_v_ref <= 0, fallback to 0.25')
                self.gating_v_ref = 0.25
            if self.gating_w_ref <= 0.0:
                self.get_logger().warn('gating_w_ref <= 0, fallback to 1.0')
                self.gating_w_ref = 1.0
        if self.gating_tau < 0.0:
            self.get_logger().warn('gating_tau < 0, fallback to 0.2')
            self.gating_tau = 0.2
        self.gating_v_min_ratio = float(np.clip(self.gating_v_min_ratio, 0.0, 1.0))
        self.gating_w_min_ratio = float(np.clip(self.gating_w_min_ratio, 0.0, 1.0))
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

        self.create_subscription(
            Bool, '/vr/right_controller/grip_button',
            self._grip_button_callback, qos_sub)

        # ========== 发布ROS2消息给moveit_servo ==========
        self.twist_pub = self.create_publisher(
            TwistStamped, self.twist_topic, qos_pub)
        self.gating_alpha_pub = self.create_publisher(
            Float32, '/vr_teleop/gating_alpha', 10)
        self.gating_v_scale_pub = self.create_publisher(
            Float32, '/vr_teleop/gating_v_scale', 10)
        self.gating_w_scale_pub = self.create_publisher(
            Float32, '/vr_teleop/gating_w_scale', 10)
        self.speed_scale_pub = self.create_publisher(
            Float32, '/vr_teleop/speed_scale', 10)

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
        self.startup_active = False
        self.startup_start_time = None
        self.ee_anchor_p: Optional[np.ndarray] = None
        self.ee_anchor_q: Optional[np.ndarray] = None
        self.vr_anchor_p: Optional[np.ndarray] = None
        self.vr_anchor_q: Optional[np.ndarray] = None
        self.last_vr_p: Optional[np.ndarray] = None
        self.last_vr_q: Optional[np.ndarray] = None
        self.target_twist = TwistStamped()
        self.joystick_y = 0.0
        self.gripper_target_pos = self.gripper_open_pos
        self.last_gripper_pos_sent = None
        self.last_publish_time = None
        self.last_cmd_twist = Twist()
        self.soft_stop_active = False
        self.soft_stop_start_time = None
        self.soft_stop_start_twist = Twist()
        self.last_grip_button_state = False
        self.gating_alpha = 0.0
        self.gating_v_scale = 1.0
        self.gating_w_scale = 1.0

        # ========== 定时器发布速度命令 ==========
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self.publish_velocity_command)
        gripper_period = 1.0 / self.gripper_rate if self.gripper_rate > 0 else 0.1
        self.gripper_timer = self.create_timer(gripper_period, self.update_gripper_command)

        self.get_logger().info('='*60)
        self.get_logger().info('VR Message Converter Node Started')
        self.get_logger().info(f'  Linear scale: {self.linear_scale}')
        self.get_logger().info(f'  Angular scale: {self.angular_scale}')
        self.get_logger().info(
            f'  Max linear velocity: slow={self.v_max_slow} fast={self.v_max_fast} m/s'
        )
        self.get_logger().info(
            f'  Max angular velocity: slow={self.w_max_slow} fast={self.w_max_fast} rad/s'
        )
        self.get_logger().info(f'  Soft stop duration: {self.soft_stop_duration} s')
        self.get_logger().info(
            f'  Linear slew rate: slow={self.linear_slew_rate_slow} fast={self.linear_slew_rate_fast} m/s^2'
        )
        self.get_logger().info(
            f'  Angular slew rate: slow={self.angular_slew_rate_slow} fast={self.angular_slew_rate_fast} rad/s^2'
        )
        self.get_logger().info(
            f'  Speed mode: {self.speed_mode} (transition {self.speed_mode_transition_time}s)'
        )
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
        self.trigger_value = msg.data

        if (not self.enabled) and (self.trigger_value > self.enable_threshold):
            self.enabled = True
            self.anchor_set = False
            self.soft_stop_active = False
            self.soft_stop_start_time = None
            self.target_twist.twist = Twist()
            if not self.use_anchor:
                self._start_incremental_startup()
            self.get_logger().info('✓ VR Control ENABLED')
        elif self.enabled and (self.trigger_value < self.disable_threshold):
            self.enabled = False
            self._start_soft_stop()
            self.anchor_set = False
            self.startup_active = False
            self.startup_start_time = None
            self.last_vr_p = None
            self.last_vr_q = None
            self.get_logger().info('✗ VR Control DISABLED')

    def _joystick_callback(self, msg: Float32) -> None:
        """接收右手摇杆Y轴"""
        self.joystick_y = float(msg.data)

    def _grip_button_callback(self, msg: Bool) -> None:
        """Grip按下切换速度模式（上升沿触发）"""
        current = bool(msg.data)
        if current and (not self.last_grip_button_state):
            self._toggle_speed_mode()
        self.last_grip_button_state = current

    def _toggle_speed_mode(self) -> None:
        if self.speed_mode == 'fast':
            self.speed_mode = 'slow'
            self.speed_scale_target = 0.0
        else:
            self.speed_mode = 'fast'
            self.speed_scale_target = 1.0
        self.get_logger().info(f'Speed mode -> {self.speed_mode}')

    def _start_soft_stop(self) -> None:
        """开始软停过程"""
        self.soft_stop_active = True
        self.soft_stop_start_time = self.get_clock().now()
        self.soft_stop_start_twist = _twist_copy(self.last_cmd_twist)

    def _start_incremental_startup(self) -> None:
        self.startup_active = True
        self.startup_start_time = self.get_clock().now()
        if self.current_controller_pose is None:
            self.last_vr_p = None
            self.last_vr_q = None
            return
        self.last_vr_p, self.last_vr_q = self._get_vr_pose_arrays(self.current_controller_pose)

    def _compute_dt(self, now_time) -> float:
        if self.last_publish_time is None:
            return 1.0 / float(self.publish_rate)
        dt = (now_time - self.last_publish_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return 1.0 / float(self.publish_rate)
        return dt

    @staticmethod
    def _lerp(a: float, b: float, t: float) -> float:
        return a + (b - a) * t

    def _update_speed_scale(self, dt: float) -> None:
        if dt <= 0.0:
            return
        delta = self.speed_scale_target - self.speed_scale
        max_step = self.speed_mode_ramp_rate * dt
        if delta > max_step:
            self.speed_scale += max_step
        elif delta < -max_step:
            self.speed_scale -= max_step
        else:
            self.speed_scale = self.speed_scale_target
        self.speed_scale = float(min(max(self.speed_scale, 0.0), 1.0))

    def _update_speed_params(self) -> None:
        scale = self.speed_scale
        self.max_linear_vel = self._lerp(self.v_max_slow, self.v_max_fast, scale)
        self.max_angular_vel = self._lerp(self.w_max_slow, self.w_max_fast, scale)
        self.linear_slew_rate = self._lerp(
            self.linear_slew_rate_slow, self.linear_slew_rate_fast, scale
        )
        self.angular_slew_rate = self._lerp(
            self.angular_slew_rate_slow, self.angular_slew_rate_fast, scale
        )

    def _apply_slew_limit(self, target: Twist, dt: float) -> Twist:
        """限制Twist变化率（每轴）"""
        if dt <= 0.0:
            return _twist_copy(target)

        max_lin_delta = self.linear_slew_rate * dt
        max_ang_delta = self.angular_slew_rate * dt

        def clamp(prev: float, desired: float, max_delta: float) -> float:
            delta = desired - prev
            if delta > max_delta:
                return prev + max_delta
            if delta < -max_delta:
                return prev - max_delta
            return desired

        out = Twist()
        out.linear.x = clamp(self.last_cmd_twist.linear.x, target.linear.x, max_lin_delta)
        out.linear.y = clamp(self.last_cmd_twist.linear.y, target.linear.y, max_lin_delta)
        out.linear.z = clamp(self.last_cmd_twist.linear.z, target.linear.z, max_lin_delta)
        out.angular.x = clamp(self.last_cmd_twist.angular.x, target.angular.x, max_ang_delta)
        out.angular.y = clamp(self.last_cmd_twist.angular.y, target.angular.y, max_ang_delta)
        out.angular.z = clamp(self.last_cmd_twist.angular.z, target.angular.z, max_ang_delta)
        return out

    def _publish_twist(self, twist: Twist, now_time) -> None:
        twist_msg = TwistStamped()
        twist_msg.header.stamp = now_time.to_msg()
        twist_msg.header.frame_id = self.reference_frame
        twist_msg.twist = twist
        self.twist_pub.publish(twist_msg)
        self.last_cmd_twist = _twist_copy(twist)
        self.last_publish_time = now_time

    def _publish_gating_state(self) -> None:
        alpha_msg = Float32()
        alpha_msg.data = float(self.gating_alpha)
        self.gating_alpha_pub.publish(alpha_msg)
        v_msg = Float32()
        v_msg.data = float(self.gating_v_scale)
        self.gating_v_scale_pub.publish(v_msg)
        w_msg = Float32()
        w_msg.data = float(self.gating_w_scale)
        self.gating_w_scale_pub.publish(w_msg)

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

    @staticmethod
    def _get_vr_pose_arrays(pose_msg: PoseStamped) -> tuple:
        p = np.array([
            pose_msg.pose.position.x,
            pose_msg.pose.position.y,
            pose_msg.pose.position.z
        ], dtype=float)
        q = _quat_normalize_xyzw(np.array([
            pose_msg.pose.orientation.x,
            pose_msg.pose.orientation.y,
            pose_msg.pose.orientation.z,
            pose_msg.pose.orientation.w
        ], dtype=float))
        return p, q

    def _calculate_velocity_anchor(self) -> Optional[Twist]:
        """基于锚点的VR相对位姿 -> 末端位姿误差 -> 速度命令"""
        if self.current_controller_pose is None:
            return None

        ee_p, ee_q = self._get_current_ee_pose()
        if ee_p is None:
            return None

        vr_p_now, vr_q_now = self._get_vr_pose_arrays(self.current_controller_pose)

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
        linear_vel[2] *= self.z_scale
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

    def _calculate_velocity_incremental(self, dt: float) -> Optional[Twist]:
        """基于上一帧/当前帧的增量位姿计算速度命令"""
        if self.current_controller_pose is None:
            return None
        if dt < 1e-3:
            return Twist()

        vr_p_now, vr_q_now = self._get_vr_pose_arrays(self.current_controller_pose)

        if self.last_vr_p is None or self.last_vr_q is None:
            self.last_vr_p = vr_p_now.copy()
            self.last_vr_q = vr_q_now.copy()
            return Twist()

        vr_last_pi, vr_last_qi = _pose_inverse(self.last_vr_p, self.last_vr_q)
        dvr_p, dvr_q = _pose_compose(vr_last_pi, vr_last_qi, vr_p_now, vr_q_now)
        rotvec = _quat_to_rotvec_xyzw(dvr_q)

        linear_vel = (dvr_p / dt) * self.linear_scale
        linear_vel[2] *= self.z_scale
        angular_vel = (rotvec / dt) * self.angular_scale

        if np.linalg.norm(linear_vel) < self.deadzone_linear:
            linear_vel[:] = 0.0
        if np.linalg.norm(angular_vel) < self.deadzone_angular:
            angular_vel[:] = 0.0

        linear_mag = np.linalg.norm(linear_vel)
        if linear_mag > self.max_linear_vel:
            linear_vel = linear_vel / linear_mag * self.max_linear_vel

        angular_mag = np.linalg.norm(angular_vel)
        if angular_mag > self.max_angular_vel:
            angular_vel = angular_vel / angular_mag * self.max_angular_vel

        self.last_vr_p = vr_p_now.copy()
        self.last_vr_q = vr_q_now.copy()

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
        now_time = self.get_clock().now()
        dt = self._compute_dt(now_time)
        self._update_speed_scale(dt)
        self._update_speed_params()
        scale_msg = Float32()
        scale_msg.data = float(self.speed_scale)
        self.speed_scale_pub.publish(scale_msg)

        # 未启用时执行软停并最终发送零速度命令
        if not self.enabled:
            self._publish_gating_state()
            if self.soft_stop_active:
                if self.soft_stop_start_time is None:
                    self.soft_stop_start_time = now_time
                    self.soft_stop_start_twist = _twist_copy(self.last_cmd_twist)
                elapsed = (now_time - self.soft_stop_start_time).nanoseconds * 1e-9
                if elapsed < self.soft_stop_duration:
                    scale = max(0.0, 1.0 - (elapsed / self.soft_stop_duration))
                    twist_out = _twist_scale(self.soft_stop_start_twist, scale)
                    self._publish_twist(twist_out, now_time)
                    return
                self.soft_stop_active = False
                self.anchor_set = False
                self.target_twist.twist = Twist()
                self.last_cmd_twist = Twist()

            self.anchor_set = False
            if not self.use_anchor:
                self.last_vr_p = None
                self.last_vr_q = None
                self.startup_active = False
                self.startup_start_time = None
            self._publish_twist(Twist(), now_time)
            return

        if (not self.use_anchor) and self.startup_active:
            if self.current_controller_pose is not None:
                self.last_vr_p, self.last_vr_q = self._get_vr_pose_arrays(self.current_controller_pose)
            if self.startup_start_time is None:
                self.startup_start_time = now_time
            elapsed = (now_time - self.startup_start_time).nanoseconds * 1e-9
            if elapsed < self.soft_stop_duration:
                self._publish_gating_state()
                self._publish_twist(Twist(), now_time)
                return
            self.startup_active = False

        if dt < 1e-3:
            self._publish_gating_state()
            self._publish_twist(Twist(), now_time)
            return

        # 计算速度
        if self.use_anchor:
            twist = self._calculate_velocity_anchor()
        else:
            twist = self._calculate_velocity_incremental(dt)
        if twist is None:
            self._publish_gating_state()
            self._publish_twist(Twist(), now_time)
            return

        # 低通滤波
        filtered = Twist()
        filtered.linear.x = (1.0 - self.smoothing_factor) * self.target_twist.twist.linear.x + \
            self.smoothing_factor * twist.linear.x
        filtered.linear.y = (1.0 - self.smoothing_factor) * self.target_twist.twist.linear.y + \
            self.smoothing_factor * twist.linear.y
        filtered.linear.z = (1.0 - self.smoothing_factor) * self.target_twist.twist.linear.z + \
            self.smoothing_factor * twist.linear.z
        filtered.angular.x = (1.0 - self.smoothing_factor) * self.target_twist.twist.angular.x + \
            self.smoothing_factor * twist.angular.x
        filtered.angular.y = (1.0 - self.smoothing_factor) * self.target_twist.twist.angular.y + \
            self.smoothing_factor * twist.angular.y
        filtered.angular.z = (1.0 - self.smoothing_factor) * self.target_twist.twist.angular.z + \
            self.smoothing_factor * twist.angular.z

        self.target_twist.twist = filtered

        if self.enable_gating and (not self.use_anchor):
            if self.gating_bind_to_speed_mode:
                alpha_raw = 1.0 - self.speed_scale
                if self.gating_bind_use_tau and self.gating_tau > 1e-3:
                    k = min(1.0, dt / self.gating_tau)
                    self.gating_alpha += k * (alpha_raw - self.gating_alpha)
                else:
                    self.gating_alpha = alpha_raw
            else:
                v = np.array([
                    filtered.linear.x,
                    filtered.linear.y,
                    filtered.linear.z
                ], dtype=float)
                w = np.array([
                    filtered.angular.x,
                    filtered.angular.y,
                    filtered.angular.z
                ], dtype=float)
                v_mag = float(np.linalg.norm(v))
                w_mag = float(np.linalg.norm(w))
                u_v = v_mag / self.gating_v_ref
                u_w = w_mag / self.gating_w_ref
                alpha_raw = u_w / (u_v + u_w + 1e-6)

                if self.gating_tau > 1e-3:
                    k = min(1.0, dt / self.gating_tau)
                    self.gating_alpha += k * (alpha_raw - self.gating_alpha)
                else:
                    self.gating_alpha = alpha_raw

            v_scale = (1.0 - self.gating_alpha) + self.gating_alpha * self.gating_v_min_ratio
            w_scale = (1.0 - self.gating_alpha) * self.gating_w_min_ratio + self.gating_alpha
            self.gating_v_scale = v_scale
            self.gating_w_scale = w_scale

            filtered.linear.x *= v_scale
            filtered.linear.y *= v_scale
            filtered.linear.z *= v_scale
            filtered.angular.x *= w_scale
            filtered.angular.y *= w_scale
            filtered.angular.z *= w_scale
        else:
            self.gating_v_scale = 1.0
            self.gating_w_scale = 1.0

        self._publish_gating_state()

        # 变化率限制
        limited = self._apply_slew_limit(filtered, dt)

        # 发布到moveit_servo
        self._publish_twist(limited, now_time)

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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
