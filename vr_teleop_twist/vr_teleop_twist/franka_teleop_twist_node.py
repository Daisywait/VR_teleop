#!/usr/bin/env python3
"""
VR to MoveIt Servo Node (HMD-relative)
接收VR控制器/HMD位姿并转换为ROS2速度指令发送给moveit_servo

订阅:
- /vr/right_controller/pose_hmd (geometry_msgs/PoseStamped)
- /vr/right_controller/trigger (std_msgs/Float32)
- /vr/right_controller/joystick_y (std_msgs/Float32)
- /vr/right_controller/grip_button (std_msgs/Bool)

发布:
- twist_topic 参数指定的话题 (geometry_msgs/TwistStamped)
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
from scipy.spatial.transform import Rotation
from typing import Optional
from dataclasses import dataclass, field


@dataclass
class TeleopParams:
    linear_scale: float = 2.1
    angular_scale: float = 0.4
    smoothing_factor: float = 0.3
    deadzone_linear: float = 0.002
    deadzone_angular: float = 0.03
    enable_threshold: float = 0.6
    disable_threshold: float = 0.4
    use_anchor: bool = False
    z_scale: float = 0.2
    soft_stop_duration: float = 0.2
    publish_rate: float = 90.0
    planning_frame: str = 'fr3_link0'
    ee_frame: str = 'robotiq_85_base_link'
    twist_topic: str = '/moveit_servo/delta_twist_cmds'
    servo_command_type_service: str = '/servo_node/switch_command_type'
    servo_command_type: int = 1
    # 速度档位
    v_max_fast: float = 0.15
    w_max_fast: float = 1.0
    linear_slew_rate_fast: float = 0.5
    angular_slew_rate_fast: float = 1.0
    v_max_slow: float = 0.15
    w_max_slow: float = 1.0
    linear_slew_rate_slow: float = 0.5
    angular_slew_rate_slow: float = 1.0
    speed_mode_transition_time: float = 0.3
    # 门控
    enable_gating: bool = True
    gating_bind_to_speed_mode: bool = True
    gating_bind_use_tau: bool = False
    gating_v_ref: float = 0.25
    gating_w_ref: float = 1.0
    gating_tau: float = 0.2
    gating_v_min_ratio: float = 0.25
    gating_w_min_ratio: float = 0.25
    # 夹爪
    gripper_action: str = '/robotiq_gripper_controller/gripper_cmd'
    gripper_open_pos: float = 0.0
    gripper_close_pos: float = 0.8
    gripper_force: float = 50.0
    gripper_speed: float = 0.8
    gripper_axis_deadzone: float = 0.08
    gripper_deadband: float = 0.01
    gripper_rate: float = 15.0


def _vec_to_twist(linear: np.ndarray, angular: np.ndarray) -> Twist:
    t = Twist()
    t.linear.x, t.linear.y, t.linear.z = float(linear[0]), float(linear[1]), float(linear[2])
    t.angular.x, t.angular.y, t.angular.z = float(angular[0]), float(angular[1]), float(angular[2])
    return t


def _clamp_vel(vel: np.ndarray, v_max: float) -> np.ndarray:
    mag = np.linalg.norm(vel)
    return vel / mag * v_max if mag > v_max else vel


def _pose_from_msg(msg: PoseStamped):
    p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    q = np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                  msg.pose.orientation.z, msg.pose.orientation.w])
    return p, Rotation.from_quat(q)


def _pose_delta(p_prev, r_prev: Rotation, p_now, r_now: Rotation):
    dp = r_prev.inv().apply(p_now - p_prev)
    dr = r_prev.inv() * r_now
    return dp, dr


class VRMessageConverterNode(Node):

    def __init__(self):
        super().__init__('vr_message_converter_node')
        self.p = self._load_params()

        qos_sub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_pub = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=10)

        self.create_subscription(PoseStamped, '/vr/right_controller/pose_hmd',
                                 self._pose_cb, qos_sub)
        self.create_subscription(Float32, '/vr/right_controller/trigger',
                                 self._trigger_cb, qos_sub)
        self.create_subscription(Float32, '/vr/right_controller/joystick_y',
                                 self._joystick_cb, qos_sub)
        self.create_subscription(Bool, '/vr/right_controller/grip_button',
                                 self._grip_cb, qos_sub)

        self.twist_pub = self.create_publisher(TwistStamped, self.p.twist_topic, qos_pub)
        self.gating_alpha_pub = self.create_publisher(Float32, '/vr_teleop/gating_alpha', 10)
        self.gating_v_scale_pub = self.create_publisher(Float32, '/vr_teleop/gating_v_scale', 10)
        self.gating_w_scale_pub = self.create_publisher(Float32, '/vr_teleop/gating_w_scale', 10)
        self.speed_scale_pub = self.create_publisher(Float32, '/vr_teleop/speed_scale', 10)

        self.gripper_client = ActionClient(self, GripperCommand, self.p.gripper_action)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.servo_cmd_client = self.create_client(ServoCommandType, self.p.servo_command_type_service)
        self.servo_cmd_set = False
        self.servo_cmd_pending = False
        self.create_timer(1.0, self._try_set_servo_cmd_type)

        # 状态
        self.current_pose: Optional[PoseStamped] = None
        self.trigger_value = 0.0
        self.joystick_y = 0.0
        self.enabled = False
        self.last_grip_state = False

        self.last_vr_p: Optional[np.ndarray] = None
        self.last_vr_r: Optional[Rotation] = None
        self.anchor_set = False
        self.ee_anchor_p: Optional[np.ndarray] = None
        self.ee_anchor_r: Optional[Rotation] = None
        self.vr_anchor_p: Optional[np.ndarray] = None
        self.vr_anchor_r: Optional[Rotation] = None

        self.target_lin = np.zeros(3)
        self.target_ang = np.zeros(3)
        self.last_cmd_lin = np.zeros(3)
        self.last_cmd_ang = np.zeros(3)

        self.soft_stop_active = False
        self.soft_stop_start_time = None
        self.soft_stop_start_lin = np.zeros(3)
        self.soft_stop_start_ang = np.zeros(3)

        self.startup_active = False
        self.startup_start_time = None

        self.speed_mode = 'fast'
        self.speed_scale = 1.0
        self.speed_scale_target = 1.0
        self.speed_mode_ramp_rate = 1.0 / max(self.p.speed_mode_transition_time, 1e-3)
        self._update_speed_params()

        self.gating_alpha = 0.0
        self.gating_v_scale = 1.0
        self.gating_w_scale = 1.0

        self.gripper_target_pos = self.p.gripper_open_pos
        self.last_gripper_pos_sent: Optional[float] = None
        self.last_publish_time = None

        self.create_timer(1.0 / self.p.publish_rate, self._publish_cmd)
        gripper_period = 1.0 / self.p.gripper_rate if self.p.gripper_rate > 0 else 0.1
        self.create_timer(gripper_period, self._update_gripper)

        self.get_logger().info(
            f'VR Teleop started | frame={self.p.planning_frame} '
            f'v_max fast={self.p.v_max_fast} slow={self.p.v_max_slow} | '
            f'topic={self.p.twist_topic}'
        )

    def _load_params(self) -> TeleopParams:
        defaults = TeleopParams()
        fields = defaults.__dataclass_fields__

        def gp(name):
            default = getattr(defaults, name)
            self.declare_parameter(name, default)
            return type(default)(self.get_parameter(name).value)

        return TeleopParams(**{f: gp(f) for f in fields})

    # ── callbacks ──────────────────────────────────────────────────────────

    def _pose_cb(self, msg: PoseStamped):
        self.current_pose = msg

    def _trigger_cb(self, msg: Float32):
        self.trigger_value = msg.data
        if not self.enabled and self.trigger_value > self.p.enable_threshold:
            self.enabled = True
            self.anchor_set = False
            self.soft_stop_active = False
            self.target_lin[:] = 0
            self.target_ang[:] = 0
            if not self.p.use_anchor:
                self._start_startup()
            self.get_logger().info('VR Control ENABLED')
        elif self.enabled and self.trigger_value < self.p.disable_threshold:
            self.enabled = False
            self._start_soft_stop()
            self.anchor_set = False
            self.startup_active = False
            self.last_vr_p = None
            self.last_vr_r = None
            self.get_logger().info('VR Control DISABLED')

    def _joystick_cb(self, msg: Float32):
        self.joystick_y = float(msg.data)

    def _grip_cb(self, msg: Bool):
        current = bool(msg.data)
        if current and not self.last_grip_state:
            self.speed_mode = 'slow' if self.speed_mode == 'fast' else 'fast'
            self.speed_scale_target = 0.0 if self.speed_mode == 'slow' else 1.0
            self.get_logger().info(f'Speed mode -> {self.speed_mode}')
        self.last_grip_state = current

    # ── servo command type ─────────────────────────────────────────────────

    def _try_set_servo_cmd_type(self):
        if self.servo_cmd_set or self.servo_cmd_pending:
            return
        if not self.servo_cmd_client.wait_for_service(timeout_sec=0.1):
            return
        req = ServoCommandType.Request()
        req.command_type = self.p.servo_command_type
        future = self.servo_cmd_client.call_async(req)
        self.servo_cmd_pending = True
        future.add_done_callback(self._on_servo_cmd_response)

    def _on_servo_cmd_response(self, future):
        self.servo_cmd_pending = False
        try:
            resp = future.result()
            if hasattr(resp, 'success') and not resp.success:
                self.get_logger().warn('Failed to set servo command type')
                return
            self.servo_cmd_set = True
        except Exception as e:
            self.get_logger().warn(f'Servo command type error: {e}')

    # ── speed / gating ─────────────────────────────────────────────────────

    def _update_speed_params(self):
        s = self.speed_scale
        self.max_linear_vel = self.p.v_max_slow + s * (self.p.v_max_fast - self.p.v_max_slow)
        self.max_angular_vel = self.p.w_max_slow + s * (self.p.w_max_fast - self.p.w_max_slow)
        self.linear_slew_rate = (self.p.linear_slew_rate_slow +
                                 s * (self.p.linear_slew_rate_fast - self.p.linear_slew_rate_slow))
        self.angular_slew_rate = (self.p.angular_slew_rate_slow +
                                  s * (self.p.angular_slew_rate_fast - self.p.angular_slew_rate_slow))

    def _update_speed_scale(self, dt: float):
        delta = self.speed_scale_target - self.speed_scale
        step = self.speed_mode_ramp_rate * dt
        self.speed_scale = float(np.clip(self.speed_scale + np.clip(delta, -step, step), 0.0, 1.0))

    def _apply_gating(self, lin: np.ndarray, ang: np.ndarray, dt: float):
        if not self.p.enable_gating or self.p.use_anchor:
            self.gating_v_scale = 1.0
            self.gating_w_scale = 1.0
            return lin, ang

        if self.p.gating_bind_to_speed_mode:
            alpha_raw = 1.0 - self.speed_scale
            if self.p.gating_bind_use_tau and self.p.gating_tau > 1e-3:
                k = min(1.0, dt / self.p.gating_tau)
                self.gating_alpha += k * (alpha_raw - self.gating_alpha)
            else:
                self.gating_alpha = alpha_raw
        else:
            u_v = np.linalg.norm(lin) / max(self.p.gating_v_ref, 1e-6)
            u_w = np.linalg.norm(ang) / max(self.p.gating_w_ref, 1e-6)
            alpha_raw = u_w / (u_v + u_w + 1e-6)
            if self.p.gating_tau > 1e-3:
                k = min(1.0, dt / self.p.gating_tau)
                self.gating_alpha += k * (alpha_raw - self.gating_alpha)
            else:
                self.gating_alpha = alpha_raw

        v_scale = (1.0 - self.gating_alpha) + self.gating_alpha * self.p.gating_v_min_ratio
        w_scale = (1.0 - self.gating_alpha) * self.p.gating_w_min_ratio + self.gating_alpha
        self.gating_v_scale = v_scale
        self.gating_w_scale = w_scale
        return lin * v_scale, ang * w_scale

    def _apply_slew(self, lin: np.ndarray, ang: np.ndarray, dt: float):
        max_l = self.linear_slew_rate * dt
        max_a = self.angular_slew_rate * dt
        lin_out = self.last_cmd_lin + np.clip(lin - self.last_cmd_lin, -max_l, max_l)
        ang_out = self.last_cmd_ang + np.clip(ang - self.last_cmd_ang, -max_a, max_a)
        return lin_out, ang_out

    # ── soft stop / startup ────────────────────────────────────────────────

    def _start_soft_stop(self):
        self.soft_stop_active = True
        self.soft_stop_start_time = self.get_clock().now()
        self.soft_stop_start_lin = self.last_cmd_lin.copy()
        self.soft_stop_start_ang = self.last_cmd_ang.copy()

    def _start_startup(self):
        self.startup_active = True
        self.startup_start_time = self.get_clock().now()
        if self.current_pose is not None:
            self.last_vr_p, self.last_vr_r = _pose_from_msg(self.current_pose)

    # ── velocity calculation ───────────────────────────────────────────────

    def _get_ee_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.p.planning_frame, self.p.ee_frame,
                                                rclpy.time.Time())
            p = np.array([t.transform.translation.x, t.transform.translation.y,
                          t.transform.translation.z])
            q = np.array([t.transform.rotation.x, t.transform.rotation.y,
                          t.transform.rotation.z, t.transform.rotation.w])
            return p, Rotation.from_quat(q)
        except TransformException as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None, None

    def _calc_incremental(self, dt: float):
        if self.current_pose is None or dt < 1e-3:
            return np.zeros(3), np.zeros(3)

        p_now, r_now = _pose_from_msg(self.current_pose)
        if self.last_vr_p is None:
            self.last_vr_p, self.last_vr_r = p_now.copy(), r_now
            return np.zeros(3), np.zeros(3)

        dp, dr = _pose_delta(self.last_vr_p, self.last_vr_r, p_now, r_now)
        self.last_vr_p, self.last_vr_r = p_now.copy(), r_now

        lin = dp / dt * self.p.linear_scale
        lin[2] *= self.p.z_scale
        ang = dr.as_rotvec() / dt * self.p.angular_scale

        if np.linalg.norm(lin) < self.p.deadzone_linear:
            lin[:] = 0
        if np.linalg.norm(ang) < self.p.deadzone_angular:
            ang[:] = 0

        return _clamp_vel(lin, self.max_linear_vel), _clamp_vel(ang, self.max_angular_vel)

    def _calc_anchor(self):
        if self.current_pose is None:
            return np.zeros(3), np.zeros(3)

        ee_p, ee_r = self._get_ee_pose()
        if ee_p is None:
            return np.zeros(3), np.zeros(3)

        vr_p, vr_r = _pose_from_msg(self.current_pose)

        if not self.anchor_set:
            self.ee_anchor_p, self.ee_anchor_r = ee_p.copy(), ee_r
            self.vr_anchor_p, self.vr_anchor_r = vr_p.copy(), vr_r
            self.anchor_set = True
            return np.zeros(3), np.zeros(3)

        dp_vr, dr_vr = _pose_delta(self.vr_anchor_p, self.vr_anchor_r, vr_p, vr_r)
        p_des = self.ee_anchor_p + self.ee_anchor_r.apply(dp_vr)
        r_des = self.ee_anchor_r * dr_vr

        pos_err = ee_r.inv().apply(p_des - ee_p)
        rot_err = (ee_r.inv() * r_des).as_rotvec()

        lin = pos_err * self.p.linear_scale
        lin[2] *= self.p.z_scale
        ang = rot_err * self.p.angular_scale

        if np.linalg.norm(lin) < self.p.deadzone_linear:
            lin[:] = 0
        if np.linalg.norm(ang) < self.p.deadzone_angular:
            ang[:] = 0

        return _clamp_vel(lin, self.max_linear_vel), _clamp_vel(ang, self.max_angular_vel)

    # ── main publish loop ──────────────────────────────────────────────────

    def _publish_twist_vecs(self, lin: np.ndarray, ang: np.ndarray, now):
        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.p.planning_frame
        msg.twist = _vec_to_twist(lin, ang)
        self.twist_pub.publish(msg)
        self.last_cmd_lin = lin.copy()
        self.last_cmd_ang = ang.copy()
        self.last_publish_time = now

    def _publish_gating_state(self):
        for pub, val in [(self.gating_alpha_pub, self.gating_alpha),
                         (self.gating_v_scale_pub, self.gating_v_scale),
                         (self.gating_w_scale_pub, self.gating_w_scale)]:
            m = Float32()
            m.data = float(val)
            pub.publish(m)

    def _publish_cmd(self):
        now = self.get_clock().now()
        dt = ((now - self.last_publish_time).nanoseconds * 1e-9
              if self.last_publish_time else 1.0 / self.p.publish_rate)

        self._update_speed_scale(dt)
        self._update_speed_params()
        m = Float32()
        m.data = float(self.speed_scale)
        self.speed_scale_pub.publish(m)

        zero = np.zeros(3)

        if not self.enabled:
            self._publish_gating_state()
            if self.soft_stop_active:
                elapsed = (now - self.soft_stop_start_time).nanoseconds * 1e-9
                if elapsed < self.p.soft_stop_duration:
                    scale = max(0.0, 1.0 - elapsed / self.p.soft_stop_duration)
                    self._publish_twist_vecs(self.soft_stop_start_lin * scale,
                                             self.soft_stop_start_ang * scale, now)
                    return
                self.soft_stop_active = False
                self.last_cmd_lin[:] = 0
                self.last_cmd_ang[:] = 0
            if not self.p.use_anchor:
                self.last_vr_p = None
                self.last_vr_r = None
                self.startup_active = False
            self._publish_twist_vecs(zero, zero, now)
            return

        if not self.p.use_anchor and self.startup_active:
            if self.current_pose is not None:
                self.last_vr_p, self.last_vr_r = _pose_from_msg(self.current_pose)
            elapsed = (now - self.startup_start_time).nanoseconds * 1e-9
            if elapsed < self.p.soft_stop_duration:
                self._publish_gating_state()
                self._publish_twist_vecs(zero, zero, now)
                return
            self.startup_active = False

        lin, ang = (self._calc_anchor() if self.p.use_anchor
                    else self._calc_incremental(dt))

        # 低通滤波
        a = self.p.smoothing_factor
        self.target_lin = (1 - a) * self.target_lin + a * lin
        self.target_ang = (1 - a) * self.target_ang + a * ang

        lin_g, ang_g = self._apply_gating(self.target_lin.copy(), self.target_ang.copy(), dt)
        self._publish_gating_state()

        lin_out, ang_out = self._apply_slew(lin_g, ang_g, dt)
        self._publish_twist_vecs(lin_out, ang_out, now)

    # ── gripper ────────────────────────────────────────────────────────────

    def _update_gripper(self):
        if not self.gripper_client.wait_for_server(timeout_sec=0.05):
            return
        axis = self.joystick_y if abs(self.joystick_y) >= self.p.gripper_axis_deadzone else 0.0
        dt = 1.0 / self.p.gripper_rate
        delta = axis * self.p.gripper_speed * dt * (self.p.gripper_close_pos - self.p.gripper_open_pos)
        self.gripper_target_pos = float(np.clip(
            self.gripper_target_pos + delta,
            min(self.p.gripper_open_pos, self.p.gripper_close_pos),
            max(self.p.gripper_open_pos, self.p.gripper_close_pos)
        ))
        if (self.last_gripper_pos_sent is not None and
                abs(self.gripper_target_pos - self.last_gripper_pos_sent) < self.p.gripper_deadband):
            return
        goal = GripperCommand.Goal()
        goal.command.position = self.gripper_target_pos
        goal.command.max_effort = self.p.gripper_force
        self.gripper_client.send_goal_async(goal)
        self.last_gripper_pos_sent = self.gripper_target_pos

    def destroy_node(self):
        self.get_logger().info('VR Teleop Node shutdown')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VRMessageConverterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
