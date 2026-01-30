#!/usr/bin/env python3
"""
VR to Franka (UDP, 1 kHz libfranka backend)
接收VR控制器/HMD位姿，计算末端笛卡尔速度，通过UDP发送给实时环

订阅:
- /vr/right_controller/pose_hmd (geometry_msgs/PoseStamped)
- /vr/right_controller/trigger (std_msgs/Float32)
- /vr/right_controller/joystick_y (std_msgs/Float32)

UDP:
- 接收末端位姿 (O_T_EE 4x4 column-major) 来建立锚点
- 发送速度命令 (vx,vy,vz,wx,wy,wz)
"""

import socket
import struct
import threading
import time
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand


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


def _pose_compose(p1: np.ndarray, q1: np.ndarray, p2: np.ndarray, q2: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    q1 = _quat_normalize_xyzw(q1)
    q2 = _quat_normalize_xyzw(q2)
    r1 = _quat_to_rotmat_xyzw(q1)
    p = p1 + r1 @ p2
    q = _quat_mul_xyzw(q1, q2)
    q = _quat_normalize_xyzw(q)
    return p, q


def _pose_inverse(p: np.ndarray, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    q = _quat_normalize_xyzw(q)
    q_i = _quat_inv_xyzw(q)
    r_i = _quat_to_rotmat_xyzw(q_i)
    p_i = -r_i @ p
    return p_i, q_i


def _pose_error(p_cur: np.ndarray, q_cur: np.ndarray,
                p_des: np.ndarray, q_des: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    p_ci, q_ci = _pose_inverse(p_cur, q_cur)
    p_err, q_err = _pose_compose(p_ci, q_ci, p_des, q_des)
    rotvec = _quat_to_rotvec_xyzw(q_err)
    return p_err, rotvec


def _rotmat_to_quat_xyzw(r: np.ndarray) -> np.ndarray:
    trace = float(np.trace(r))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    return _quat_normalize_xyzw(np.array([x, y, z, w], dtype=float))


class VRMessageConverterUdpNode(Node):
    def __init__(self):
        super().__init__('vr_message_converter_udp_node')

        # ========== 参数声明 ==========
        self.declare_parameter('linear_scale', 2.1)
        self.declare_parameter('angular_scale', 0.4)
        self.declare_parameter('v_max', 0.15)
        self.declare_parameter('w_max', 1.0)
        self.declare_parameter('smoothing_factor', 0.3)
        self.declare_parameter('deadzone_linear', 0.002)
        self.declare_parameter('deadzone_angular', 0.03)
        self.declare_parameter('trigger_threshold', 0.5)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('gripper_action', '/robotiq_gripper_controller/gripper_cmd')
        self.declare_parameter('gripper_open_pos', 0.0)
        self.declare_parameter('gripper_close_pos', 0.8)
        self.declare_parameter('gripper_force', 50.0)
        self.declare_parameter('gripper_speed', 0.8)
        self.declare_parameter('gripper_axis_deadzone', 0.08)
        self.declare_parameter('gripper_deadband', 0.01)
        self.declare_parameter('gripper_rate', 15.0)

        # UDP 参数
        self.declare_parameter('udp_cmd_ip', '172.16.0.2')
        self.declare_parameter('udp_cmd_port', 5005)
        self.declare_parameter('udp_pose_bind', '0.0.0.0')
        self.declare_parameter('udp_pose_port', 5006)

        # ========== 获取参数 ==========
        self.linear_scale = float(self.get_parameter('linear_scale').value)
        self.angular_scale = float(self.get_parameter('angular_scale').value)
        self.max_linear_vel = float(self.get_parameter('v_max').value)
        self.max_angular_vel = float(self.get_parameter('w_max').value)
        self.smoothing_factor = float(self.get_parameter('smoothing_factor').value)
        self.deadzone_linear = float(self.get_parameter('deadzone_linear').value)
        self.deadzone_angular = float(self.get_parameter('deadzone_angular').value)
        self.trigger_threshold = self.get_parameter('trigger_threshold').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.gripper_action = self.get_parameter('gripper_action').value
        self.gripper_open_pos = float(self.get_parameter('gripper_open_pos').value)
        self.gripper_close_pos = float(self.get_parameter('gripper_close_pos').value)
        self.gripper_force = float(self.get_parameter('gripper_force').value)
        self.gripper_speed = float(self.get_parameter('gripper_speed').value)
        self.gripper_axis_deadzone = float(self.get_parameter('gripper_axis_deadzone').value)
        self.gripper_deadband = float(self.get_parameter('gripper_deadband').value)
        self.gripper_rate = float(self.get_parameter('gripper_rate').value)

        self.udp_cmd_ip = self.get_parameter('udp_cmd_ip').value
        self.udp_cmd_port = int(self.get_parameter('udp_cmd_port').value)
        self.udp_pose_bind = self.get_parameter('udp_pose_bind').value
        self.udp_pose_port = int(self.get_parameter('udp_pose_port').value)

        # ========== QoS配置 ==========
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
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

        # 夹爪 Action
        self.gripper_client = ActionClient(
            self, GripperCommand, self.gripper_action)

        # UDP sockets
        self._cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._cmd_target = (self.udp_cmd_ip, self.udp_cmd_port)
        self._pose_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._pose_sock.bind((self.udp_pose_bind, self.udp_pose_port))
        self._pose_sock.settimeout(0.05)
        self._pose_fmt = '<16dQ'
        self._pose_size = struct.calcsize(self._pose_fmt)

        # ========== 状态变量 ==========
        self.current_controller_pose: Optional[PoseStamped] = None
        self.trigger_value = 0.0
        self.enabled = False
        self.anchor_set = False
        self.ee_anchor_p: Optional[np.ndarray] = None
        self.ee_anchor_q: Optional[np.ndarray] = None
        self.vr_anchor_p: Optional[np.ndarray] = None
        self.vr_anchor_q: Optional[np.ndarray] = None
        self.target_twist = Twist()
        self.joystick_y = 0.0
        self.gripper_target_pos = self.gripper_open_pos
        self.last_gripper_pos_sent = None

        self._ee_lock = threading.Lock()
        self._ee_pose: Optional[Tuple[np.ndarray, np.ndarray]] = None

        # UDP接收线程（末端位姿）
        self._pose_thread = threading.Thread(target=self._pose_recv_loop, daemon=True)
        self._pose_thread.start()

        # ========== 定时器 ==========
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self.publish_velocity_command)
        gripper_period = 1.0 / self.gripper_rate if self.gripper_rate > 0 else 0.1
        self.gripper_timer = self.create_timer(gripper_period, self.update_gripper_command)

        self.get_logger().info('='*60)
        self.get_logger().info('VR Message Converter UDP Node Started')
        self.get_logger().info(f'  Linear scale: {self.linear_scale}')
        self.get_logger().info(f'  Angular scale: {self.angular_scale}')
        self.get_logger().info(f'  Max linear velocity: {self.max_linear_vel} m/s')
        self.get_logger().info(f'  Max angular velocity: {self.max_angular_vel} rad/s')
        self.get_logger().info(f'  Publish rate: {self.publish_rate} Hz')
        self.get_logger().info(f'  UDP cmd target: {self.udp_cmd_ip}:{self.udp_cmd_port}')
        self.get_logger().info(f'  UDP pose bind: {self.udp_pose_bind}:{self.udp_pose_port}')
        self.get_logger().info('='*60)
        self.get_logger().info('Waiting for VR controller data and EE pose...')

    def _pose_recv_loop(self) -> None:
        while True:
            try:
                data, _ = self._pose_sock.recvfrom(2048)
            except socket.timeout:
                continue
            if len(data) < self._pose_size:
                continue
            vals = struct.unpack(self._pose_fmt, data[:self._pose_size])
            T = np.array(vals[:16], dtype=float).reshape((4, 4), order='F')
            r = T[:3, :3]
            p = T[:3, 3]
            q = _rotmat_to_quat_xyzw(r)
            with self._ee_lock:
                self._ee_pose = (p, q)

    def _controller_pose_callback(self, msg: PoseStamped) -> None:
        self.current_controller_pose = msg

    def _trigger_callback(self, msg: Float32) -> None:
        old_enabled = self.enabled
        self.trigger_value = msg.data
        self.enabled = self.trigger_value >= self.trigger_threshold
        if self.enabled and not old_enabled:
            self.get_logger().info('✓ VR Control ENABLED')
            self.anchor_set = False
        elif not self.enabled and old_enabled:
            self.get_logger().info('✗ VR Control DISABLED')

    def _joystick_callback(self, msg: Float32) -> None:
        self.joystick_y = float(msg.data)

    def _get_current_ee_pose(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        with self._ee_lock:
            if self._ee_pose is None:
                return None, None
            p, q = self._ee_pose
            return p.copy(), q.copy()

    def _calculate_velocity(self) -> Optional[Twist]:
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

        linear_vel = pos_err * self.linear_scale
        angular_vel = rotvec_err * self.angular_scale

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

        twist = Twist()
        twist.linear.x = float(linear_vel[0])
        twist.linear.y = float(linear_vel[1])
        twist.linear.z = float(linear_vel[2])
        twist.angular.x = float(angular_vel[0])
        twist.angular.y = float(angular_vel[1])
        twist.angular.z = float(angular_vel[2])
        return twist

    def publish_velocity_command(self) -> None:
        if self.current_controller_pose is None:
            return

        if not self.enabled:
            self.anchor_set = False
            self._send_udp_cmd(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            return

        twist = self._calculate_velocity()
        if twist is None:
            return

        twist.linear.x = (1.0 - self.smoothing_factor) * self.target_twist.linear.x + \
            self.smoothing_factor * twist.linear.x
        twist.linear.y = (1.0 - self.smoothing_factor) * self.target_twist.linear.y + \
            self.smoothing_factor * twist.linear.y
        twist.linear.z = (1.0 - self.smoothing_factor) * self.target_twist.linear.z + \
            self.smoothing_factor * twist.linear.z
        twist.angular.x = (1.0 - self.smoothing_factor) * self.target_twist.angular.x + \
            self.smoothing_factor * twist.angular.x
        twist.angular.y = (1.0 - self.smoothing_factor) * self.target_twist.angular.y + \
            self.smoothing_factor * twist.angular.y
        twist.angular.z = (1.0 - self.smoothing_factor) * self.target_twist.angular.z + \
            self.smoothing_factor * twist.angular.z

        self.target_twist = twist
        self._send_udp_cmd(twist.linear.x, twist.linear.y, twist.linear.z,
                           twist.angular.x, twist.angular.y, twist.angular.z)

    def _send_udp_cmd(self, vx: float, vy: float, vz: float, wx: float, wy: float, wz: float) -> None:
        t_ns = time.monotonic_ns()
        payload = struct.pack('<6dQ', vx, vy, vz, wx, wy, wz, t_ns)
        self._cmd_sock.sendto(payload, self._cmd_target)

    def update_gripper_command(self) -> None:
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
        self.get_logger().info('VR Message Converter UDP Node shutdown')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VRMessageConverterUdpNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
