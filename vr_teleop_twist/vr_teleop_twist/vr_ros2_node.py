#!/usr/bin/env python3
"""
VR ROS2 Node
将VR手柄数据发布到ROS2话题

发布的话题:
- /vr/right_controller/pose_hmd (geometry_msgs/PoseStamped)
- /vr/right_controller/trigger (std_msgs/Float32)
- /vr/right_controller/joystick_y (std_msgs/Float32)
- /vr/right_controller/a_button (std_msgs/Bool)
- /vr/right_controller/grip_button (std_msgs/Bool)
- TF: vr_room -> vr_hmd_ros -> vr_controller_right
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Float32, Bool
from tf2_ros import TransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation
import openvr
from dataclasses import dataclass
from typing import Optional, Dict


@dataclass
class ControllerState:
    trigger: float
    grip: float
    joystick_x: float
    joystick_y: float
    trigger_pressed: bool
    grip_pressed: bool
    menu_pressed: bool
    joystick_pressed: bool
    a_pressed: bool
    b_pressed: bool


@dataclass
class ControllerData:
    position: np.ndarray
    rotation: Rotation
    velocity: np.ndarray
    angular_velocity: np.ndarray
    state: ControllerState
    is_connected: bool
    is_valid: bool


# HMD坐标系(Y-up, Z朝后) → ROS坐标系(Z-up, X朝前)
HMD_TO_ROS = np.array([
    [0.0,  0.0, -1.0],
    [-1.0, 0.0,  0.0],
    [0.0,  1.0,  0.0],
], dtype=float)
HMD_TO_ROS_ROT = Rotation.from_matrix(HMD_TO_ROS)


def _openvr_matrix_to_pose(m34):
    """从OpenVR 3x4矩阵提取位置和旋转"""
    mat = np.array([[m34[i][j] for j in range(4)] for i in range(3)])
    pos = mat[:, 3]
    rot = Rotation.from_matrix(mat[:, :3])
    return pos, rot


def _make_pose_msg(pos: np.ndarray, rot: Rotation, frame_id: str) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = float(pos[0]), float(pos[1]), float(pos[2])
    q = rot.as_quat()  # xyzw
    msg.pose.orientation.x, msg.pose.orientation.y = float(q[0]), float(q[1])
    msg.pose.orientation.z, msg.pose.orientation.w = float(q[2]), float(q[3])
    return msg


def _make_tf_msg(pos: np.ndarray, rot: Rotation, parent: str, child: str) -> TransformStamped:
    msg = TransformStamped()
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = float(pos[0]), float(pos[1]), float(pos[2])
    q = rot.as_quat()
    msg.transform.rotation.x, msg.transform.rotation.y = float(q[0]), float(q[1])
    msg.transform.rotation.z, msg.transform.rotation.w = float(q[2]), float(q[3])
    return msg


class VRRos2Node(Node):

    def __init__(self):
        super().__init__('vr_tracker_node')

        self.declare_parameter('update_rate', 90.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('frame_id', 'vr_room')
        self.declare_parameter('hmd_frame_id', 'vr_hmd_ros')
        self.declare_parameter('enable_right_controller', True)
        self.declare_parameter('haptic_on_trigger', True)
        self.declare_parameter('haptic_duration_us', 3000)
        self.declare_parameter('reinit_interval', 5.0)

        self.update_rate = self.get_parameter('update_rate').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.frame_id = self.get_parameter('frame_id').value
        self.hmd_frame_id = self.get_parameter('hmd_frame_id').value
        self.enable_right = self.get_parameter('enable_right_controller').value
        self.haptic_on_trigger = self.get_parameter('haptic_on_trigger').value
        self.haptic_duration_us = int(self.get_parameter('haptic_duration_us').value)
        self.reinit_interval = float(self.get_parameter('reinit_interval').value)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        if self.enable_right:
            self.right_pose_pub = self.create_publisher(PoseStamped, '/vr/right_controller/pose_hmd', qos)
            self.right_trigger_pub = self.create_publisher(Float32, '/vr/right_controller/trigger', qos)
            self.right_joystick_y_pub = self.create_publisher(Float32, '/vr/right_controller/joystick_y', qos)
            self.right_a_pub = self.create_publisher(Bool, '/vr/right_controller/a_button', qos)
            self.right_grip_pub = self.create_publisher(Bool, '/vr/right_controller/grip_button', qos)

        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        self.vr_system = None
        self.last_init_attempt = 0.0
        self.last_trigger_pressed = False
        self._controller_indices: Dict[str, Optional[int]] = {}
        self._init_vr()

        self.create_timer(1.0 / self.update_rate, self._timer_cb)
        self.get_logger().info(f'VR Tracker Node started at {self.update_rate} Hz')

    def _init_vr(self) -> bool:
        try:
            self.vr_system = openvr.init(openvr.VRApplication_Background)
            self._update_controller_indices()
            self.get_logger().info('VR system initialized successfully')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to initialize VR system: {e}. Make sure SteamVR is running.')
            return False

    def _update_controller_indices(self) -> None:
        self._controller_indices = {'left': None, 'right': None}
        for i in range(openvr.k_unMaxTrackedDeviceCount):
            if self.vr_system.getTrackedDeviceClass(i) != openvr.TrackedDeviceClass_Controller:
                continue
            role = self.vr_system.getControllerRoleForTrackedDeviceIndex(i)
            if role == openvr.TrackedControllerRole_LeftHand:
                self._controller_indices['left'] = i
            elif role == openvr.TrackedControllerRole_RightHand:
                self._controller_indices['right'] = i

    def _parse_buttons(self, state) -> ControllerState:
        pressed = state.ulButtonPressed
        trigger = state.rAxis[1].x if len(state.rAxis) > 1 else 0.0
        grip = state.rAxis[2].x if len(state.rAxis) > 2 else 0.0
        joystick_x = state.rAxis[0].x if len(state.rAxis) > 0 else 0.0
        joystick_y = state.rAxis[0].y if len(state.rAxis) > 0 else 0.0
        return ControllerState(
            trigger=trigger, grip=grip,
            joystick_x=joystick_x, joystick_y=joystick_y,
            trigger_pressed=bool(pressed & (1 << openvr.k_EButton_SteamVR_Trigger)),
            grip_pressed=bool(pressed & (1 << openvr.k_EButton_Grip)),
            menu_pressed=bool(pressed & (1 << openvr.k_EButton_ApplicationMenu)),
            joystick_pressed=bool(pressed & (1 << openvr.k_EButton_SteamVR_Touchpad))
                if hasattr(openvr, 'k_EButton_SteamVR_Touchpad') else False,
            a_pressed=bool(pressed & (1 << openvr.k_EButton_A))
                if hasattr(openvr, 'k_EButton_A') else False,
            b_pressed=False,
        )

    def _get_controller_data(self, side: str) -> Optional[ControllerData]:
        idx = self._controller_indices.get(side)
        if idx is None:
            self._update_controller_indices()
            return None

        poses = self.vr_system.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding, 0.0, openvr.k_unMaxTrackedDeviceCount)
        pose = poses[idx]
        if not pose.bDeviceIsConnected or not pose.bPoseIsValid:
            return None

        pos, rot = _openvr_matrix_to_pose(pose.mDeviceToAbsoluteTracking)
        vel = np.array([pose.vVelocity.v[0], pose.vVelocity.v[1], pose.vVelocity.v[2]])
        ang_vel = np.array([pose.vAngularVelocity.v[0], pose.vAngularVelocity.v[1], pose.vAngularVelocity.v[2]])

        result, state = self.vr_system.getControllerState(idx)
        if not result:
            return None

        return ControllerData(position=pos, rotation=rot, velocity=vel,
                              angular_velocity=ang_vel, state=self._parse_buttons(state),
                              is_connected=pose.bDeviceIsConnected, is_valid=pose.bPoseIsValid)

    def _get_hmd_pose(self) -> Optional[tuple]:
        poses = self.vr_system.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding, 0.0, openvr.k_unMaxTrackedDeviceCount)
        hmd = poses[openvr.k_unTrackedDeviceIndex_Hmd]
        if not hmd.bPoseIsValid:
            return None
        return _openvr_matrix_to_pose(hmd.mDeviceToAbsoluteTracking)

    def _relative_pose_hmd_frame(self, ctrl_pos, ctrl_rot, hmd_pos, hmd_rot):
        """控制器相对头显位姿，转换到ROS坐标系"""
        rel_pos_vr = hmd_rot.inv().apply(ctrl_pos - hmd_pos)
        rel_rot_vr = hmd_rot.inv() * ctrl_rot
        # 坐标系变换: VR(Y-up) → ROS(Z-up)
        rel_pos_ros = HMD_TO_ROS @ rel_pos_vr
        rel_rot_ros = HMD_TO_ROS_ROT * rel_rot_vr * HMD_TO_ROS_ROT.inv()
        return rel_pos_ros, rel_rot_ros

    def _publish_controller(self, data: ControllerData, side: str) -> None:
        stamp = self.get_clock().now().to_msg()
        state = data.state

        # 位姿发布在 timer_callback 里处理（需要HMD）
        # 按键/轴
        if side == 'right':
            msg_f = Float32()
            msg_f.data = float(state.trigger)
            self.right_trigger_pub.publish(msg_f)

            msg_f2 = Float32()
            msg_f2.data = float(state.joystick_y)
            self.right_joystick_y_pub.publish(msg_f2)

            msg_b = Bool()
            msg_b.data = state.a_pressed
            self.right_a_pub.publish(msg_b)

            msg_b2 = Bool()
            msg_b2.data = state.grip_pressed
            self.right_grip_pub.publish(msg_b2)

        # 触觉反馈
        if self.haptic_on_trigger:
            idx = self._controller_indices.get(side)
            if idx is not None:
                trigger_pressed = state.trigger_pressed
                if trigger_pressed and not self.last_trigger_pressed:
                    duration = max(0, min(self.haptic_duration_us, 3999))
                    self.vr_system.triggerHapticPulse(idx, 0, duration)
                self.last_trigger_pressed = trigger_pressed

    def _timer_cb(self) -> None:
        if self.vr_system is None:
            now = self.get_clock().now().nanoseconds * 1e-9
            if (now - self.last_init_attempt) < self.reinit_interval:
                return
            self.last_init_attempt = now
            if not self._init_vr():
                return

        try:
            hmd_result = self._get_hmd_pose()
            if hmd_result is None:
                return
            hmd_pos, hmd_rot = hmd_result

            if self.publish_tf:
                # HMD TF: vr_room → vr_hmd_ros
                hmd_pos_ros = HMD_TO_ROS @ hmd_pos
                hmd_rot_ros = HMD_TO_ROS_ROT * hmd_rot * HMD_TO_ROS_ROT.inv()
                tf_msg = _make_tf_msg(hmd_pos_ros, hmd_rot_ros, self.frame_id, self.hmd_frame_id)
                tf_msg.header.stamp = self.get_clock().now().to_msg()
                self.tf_broadcaster.sendTransform(tf_msg)

            if self.enable_right:
                data = self._get_controller_data('right')
                if data:
                    self._publish_controller(data, 'right')
                    rel_pos, rel_rot = self._relative_pose_hmd_frame(
                        data.position, data.rotation, hmd_pos, hmd_rot)
                    stamp = self.get_clock().now().to_msg()
                    pose_msg = _make_pose_msg(rel_pos, rel_rot, self.hmd_frame_id)
                    pose_msg.header.stamp = stamp
                    self.right_pose_pub.publish(pose_msg)

                    if self.publish_tf:
                        tf_ctrl = _make_tf_msg(rel_pos, rel_rot, self.hmd_frame_id, 'vr_controller_right')
                        tf_ctrl.header.stamp = stamp
                        self.tf_broadcaster.sendTransform(tf_ctrl)

        except Exception as e:
            self.get_logger().error(f'Error reading VR data: {e}')

    def destroy_node(self) -> None:
        if self.vr_system:
            openvr.shutdown()
            self.vr_system = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VRRos2Node()
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
