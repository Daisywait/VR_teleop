#!/usr/bin/env python3
"""
VR ROS2 Node
将VR手柄数据发布到ROS2话题

发布的话题:
- /vr/right_controller/pose_hmd (geometry_msgs/PoseStamped)
- /vr/right_controller/trigger (std_msgs/Float32)
- /vr/right_controller/joystick_y (std_msgs/Float32)
- TF: vr_room -> vr_hmd_ros -> vr_controller_right
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
import numpy as np
import openvr
from dataclasses import dataclass
from typing import Optional, Dict

try:
    import transforms3d.quaternions as quat
    TRANSFORMS3D_AVAILABLE = True
except ImportError:
    TRANSFORMS3D_AVAILABLE = False
    print("[Warning] transforms3d not found, using fallback quaternion conversion")


@dataclass
class ControllerState:
    """控制器按键/轴状态"""
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
    """控制器数据"""
    position: np.ndarray
    rotation_matrix: np.ndarray
    velocity: np.ndarray
    angular_velocity: np.ndarray
    state: ControllerState
    is_connected: bool
    is_valid: bool


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
        frame_id: 基准坐标系名称, 默认'vr_room'
        hmd_frame_id: 头显坐标系名称, 默认'vr_hmd_ros'
    """

    def __init__(self):
        super().__init__('vr_tracker_node')

        # 声明参数
        self.declare_parameter('update_rate', 90.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('frame_id', 'vr_room')
        self.declare_parameter('hmd_frame_id', 'vr_hmd_ros')
        self.declare_parameter('enable_right_controller', True)

        # 获取参数
        self.update_rate = self.get_parameter('update_rate').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.frame_id = self.get_parameter('frame_id').value
        self.hmd_frame_id = self.get_parameter('hmd_frame_id').value
        self.enable_right = self.get_parameter('enable_right_controller').value
        self.hmd_to_ros_rot = np.array([
            [0.0, 0.0, -1.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0]
        ], dtype=float)

        # QoS设置 - 低延迟配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers - 右手
        if self.enable_right:
            self.right_pose_hmd_pub = self.create_publisher(
                PoseStamped, '/vr/right_controller/pose_hmd', qos_profile)
            self.right_trigger_pub = self.create_publisher(
                Float32, '/vr/right_controller/trigger', qos_profile)
            self.right_joystick_y_pub = self.create_publisher(
                Float32, '/vr/right_controller/joystick_y', qos_profile)

        # TF广播器
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # 初始化VR系统
        self.vr_system = None
        self._controller_indices: Dict[str, Optional[int]] = {}
        self._init_vr()

        # 定时器
        period = 1.0 / self.update_rate
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(f'VR Tracker Node started at {self.update_rate} Hz')

    def _init_vr(self) -> bool:
        """初始化VR系统"""
        try:
            self.vr_system = openvr.init(openvr.VRApplication_Other)
            self._update_controller_indices()
            self.get_logger().info('VR system initialized successfully')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to initialize VR system: {e}')
            self.get_logger().error('Make sure SteamVR is running')
            return False

    def _update_controller_indices(self) -> None:
        """更新控制器设备索引映射"""
        self._controller_indices = {'left': None, 'right': None}
        for i in range(openvr.k_unMaxTrackedDeviceCount):
            device_class = self.vr_system.getTrackedDeviceClass(i)
            if device_class == openvr.TrackedDeviceClass_Controller:
                role = self.vr_system.getControllerRoleForTrackedDeviceIndex(i)
                if role == openvr.TrackedControllerRole_LeftHand:
                    self._controller_indices['left'] = i
                elif role == openvr.TrackedControllerRole_RightHand:
                    self._controller_indices['right'] = i

    def _get_device_poses(self):
        """获取所有设备位姿"""
        return self.vr_system.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding,
            0.0,
            openvr.k_unMaxTrackedDeviceCount
        )

    def _parse_controller_buttons(self, state) -> ControllerState:
        """解析控制器按钮状态"""
        pressed = state.ulButtonPressed

        trigger = state.rAxis[1].x if len(state.rAxis) > 1 else 0.0
        grip = state.rAxis[2].x if len(state.rAxis) > 2 else 0.0
        joystick_x = state.rAxis[0].x if len(state.rAxis) > 0 else 0.0
        joystick_y = state.rAxis[0].y if len(state.rAxis) > 0 else 0.0

        trigger_pressed = bool(pressed & (1 << openvr.k_EButton_SteamVR_Trigger))
        grip_pressed = bool(pressed & (1 << openvr.k_EButton_Grip))
        menu_pressed = bool(pressed & (1 << openvr.k_EButton_ApplicationMenu))

        if hasattr(openvr, 'k_EButton_SteamVR_Touchpad'):
            joystick_pressed = bool(pressed & (1 << openvr.k_EButton_SteamVR_Touchpad))
        else:
            joystick_pressed = False

        if hasattr(openvr, 'k_EButton_A'):
            a_pressed = bool(pressed & (1 << openvr.k_EButton_A))
        else:
            a_pressed = False

        b_pressed = False

        return ControllerState(
            trigger=trigger,
            grip=grip,
            joystick_x=joystick_x,
            joystick_y=joystick_y,
            trigger_pressed=trigger_pressed,
            grip_pressed=grip_pressed,
            menu_pressed=menu_pressed,
            joystick_pressed=joystick_pressed,
            a_pressed=a_pressed,
            b_pressed=b_pressed
        )

    def _get_controller_data(self, hand: str) -> Optional[ControllerData]:
        """获取原始控制器数据"""
        if hand not in self._controller_indices:
            return None

        device_index = self._controller_indices.get(hand)
        if device_index is None:
            self._update_controller_indices()
            device_index = self._controller_indices.get(hand)
            if device_index is None:
                return None

        poses = self._get_device_poses()
        pose = poses[device_index]

        is_connected = pose.bDeviceIsConnected
        is_valid = pose.bPoseIsValid

        if not is_connected or not is_valid:
            zero = np.zeros(3)
            state = ControllerState(
                trigger=0.0,
                grip=0.0,
                joystick_x=0.0,
                joystick_y=0.0,
                trigger_pressed=False,
                grip_pressed=False,
                menu_pressed=False,
                joystick_pressed=False,
                a_pressed=False,
                b_pressed=False
            )
            return ControllerData(
                position=zero,
                rotation_matrix=np.eye(3),
                velocity=zero,
                angular_velocity=zero,
                state=state,
                is_connected=is_connected,
                is_valid=is_valid
            )

        position = np.array([
            pose.mDeviceToAbsoluteTracking[0][3],
            pose.mDeviceToAbsoluteTracking[1][3],
            pose.mDeviceToAbsoluteTracking[2][3]
        ])

        rotation_matrix = np.array([
            [pose.mDeviceToAbsoluteTracking[0][0], pose.mDeviceToAbsoluteTracking[0][1], pose.mDeviceToAbsoluteTracking[0][2]],
            [pose.mDeviceToAbsoluteTracking[1][0], pose.mDeviceToAbsoluteTracking[1][1], pose.mDeviceToAbsoluteTracking[1][2]],
            [pose.mDeviceToAbsoluteTracking[2][0], pose.mDeviceToAbsoluteTracking[2][1], pose.mDeviceToAbsoluteTracking[2][2]]
        ])

        velocity = np.array([pose.vVelocity.v[0], pose.vVelocity.v[1], pose.vVelocity.v[2]])
        angular_velocity = np.array([pose.vAngularVelocity.v[0], pose.vAngularVelocity.v[1], pose.vAngularVelocity.v[2]])

        result, state = self.vr_system.getControllerState(device_index)
        if result:
            parsed_state = self._parse_controller_buttons(state)
        else:
            parsed_state = ControllerState(
                trigger=0.0,
                grip=0.0,
                joystick_x=0.0,
                joystick_y=0.0,
                trigger_pressed=False,
                grip_pressed=False,
                menu_pressed=False,
                joystick_pressed=False,
                a_pressed=False,
                b_pressed=False
            )

        return ControllerData(
            position=position,
            rotation_matrix=rotation_matrix,
            velocity=velocity,
            angular_velocity=angular_velocity,
            state=parsed_state,
            is_connected=is_connected,
            is_valid=is_valid
        )

    def _create_pose_msg_from_arrays_frame(self, position: np.ndarray, quaternion: tuple,
                                           frame_id: str) -> PoseStamped:
        """从位置和四元数创建PoseStamped消息（指定frame）"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])
        msg.pose.orientation.w = float(quaternion[0])
        msg.pose.orientation.x = float(quaternion[1])
        msg.pose.orientation.y = float(quaternion[2])
        msg.pose.orientation.z = float(quaternion[3])
        return msg

    def _openvr_matrix_to_quaternion(self, matrix) -> tuple:
        """将OpenVR 3x4矩阵的旋转部分转换为四元数 (w, x, y, z)"""
        m = np.array([
            [matrix[0][0], matrix[0][1], matrix[0][2]],
            [matrix[1][0], matrix[1][1], matrix[1][2]],
            [matrix[2][0], matrix[2][1], matrix[2][2]]
        ])
        return rotation_matrix_to_quaternion(m)

    def _quat_wxyz_to_rotmat(self, q: tuple) -> np.ndarray:
        """四元数(w,x,y,z)转旋转矩阵"""
        w, x, y, z = q
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z
        return np.array([
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)]
        ], dtype=float)

    def _get_hmd_pose(self) -> Optional[tuple]:
        """获取头显位姿 (position, quaternion)"""
        poses = self._get_device_poses()
        hmd_pose = poses[openvr.k_unTrackedDeviceIndex_Hmd]

        if not hmd_pose.bPoseIsValid:
            return None

        position = np.array([
            hmd_pose.mDeviceToAbsoluteTracking[0][3],
            hmd_pose.mDeviceToAbsoluteTracking[1][3],
            hmd_pose.mDeviceToAbsoluteTracking[2][3]
        ])
        quaternion = self._openvr_matrix_to_quaternion(hmd_pose.mDeviceToAbsoluteTracking)
        return (position, quaternion)

    def _map_pose_hmd_to_ros(self, pos_hmd: np.ndarray, quat_hmd: tuple) -> tuple:
        """将HMD坐标系下的位姿转换为ROS坐标系表示"""
        rot_hmd = self._quat_wxyz_to_rotmat(quat_hmd)
        pos_ros = self.hmd_to_ros_rot @ pos_hmd
        rot_ros = self.hmd_to_ros_rot @ rot_hmd @ self.hmd_to_ros_rot.T
        quat_ros = rotation_matrix_to_quaternion(rot_ros)
        return pos_ros, quat_ros

    def _publish_hmd_tf(self, hmd_pose: tuple) -> None:
        """发布vr_room -> vr_hmd_ros的TF"""
        position, quaternion = hmd_pose
        rot_room_hmd = self._quat_wxyz_to_rotmat(quaternion)
        rot_room_hmd_ros = rot_room_hmd @ self.hmd_to_ros_rot.T
        quaternion = rotation_matrix_to_quaternion(rot_room_hmd_ros)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.hmd_frame_id
        t.transform.translation.x = float(position[0])
        t.transform.translation.y = float(position[1])
        t.transform.translation.z = float(position[2])
        t.transform.rotation.w = float(quaternion[0])
        t.transform.rotation.x = float(quaternion[1])
        t.transform.rotation.y = float(quaternion[2])
        t.transform.rotation.z = float(quaternion[3])
        self.tf_broadcaster.sendTransform(t)

    def _publish_controller_tf(self, rel_pos: np.ndarray, rel_quat: tuple) -> None:
        """发布vr_hmd_ros -> vr_controller_right的TF"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.hmd_frame_id
        t.child_frame_id = 'vr_controller_right'
        t.transform.translation.x = float(rel_pos[0])
        t.transform.translation.y = float(rel_pos[1])
        t.transform.translation.z = float(rel_pos[2])
        t.transform.rotation.w = float(rel_quat[0])
        t.transform.rotation.x = float(rel_quat[1])
        t.transform.rotation.y = float(rel_quat[2])
        t.transform.rotation.z = float(rel_quat[3])
        self.tf_broadcaster.sendTransform(t)

    def _get_relative_pose(self, controller_pose: tuple, hmd_pose: tuple) -> tuple:
        """计算控制器相对头显位姿 (position, quaternion)"""
        p_ctrl, q_ctrl = controller_pose
        p_hmd, q_hmd = hmd_pose
        q_hmd = np.array([q_hmd[1], q_hmd[2], q_hmd[3], q_hmd[0]], dtype=float)
        q_ctrl = np.array([q_ctrl[1], q_ctrl[2], q_ctrl[3], q_ctrl[0]], dtype=float)
        q_hmd = q_hmd / max(np.linalg.norm(q_hmd), 1e-12)
        q_ctrl = q_ctrl / max(np.linalg.norm(q_ctrl), 1e-12)

        r_hmd = self._quat_to_rotmat_xyzw(q_hmd)
        rel_pos = r_hmd.T @ (p_ctrl - p_hmd)
        rel_quat = self._quat_mul_xyzw(self._quat_inv_xyzw(q_hmd), q_ctrl)
        rel_quat_wxyz = (float(rel_quat[3]), float(rel_quat[0]), float(rel_quat[1]), float(rel_quat[2]))
        return rel_pos, rel_quat_wxyz

    def _quat_to_rotmat_xyzw(self, q: np.ndarray) -> np.ndarray:
        """四元数(x,y,z,w)转旋转矩阵"""
        x, y, z, w = q
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z
        return np.array([
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)]
        ], dtype=float)

    def _quat_inv_xyzw(self, q: np.ndarray) -> np.ndarray:
        """四元数(x,y,z,w)逆"""
        return np.array([-q[0], -q[1], -q[2], q[3]], dtype=float)

    def _quat_mul_xyzw(self, q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """四元数乘法(x,y,z,w)"""
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return np.array([
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        ], dtype=float)

    def _publish_controller_data(self, data: ControllerData, hand: str) -> None:
        """发布单个控制器的必要数据"""
        if not data.is_valid:
            return

        if hand == 'right':
            # 扳机
            trigger_msg = Float32()
            trigger_msg.data = data.state.trigger
            self.right_trigger_pub.publish(trigger_msg)
            joystick_msg = Float32()
            joystick_msg.data = data.state.joystick_y
            self.right_joystick_y_pub.publish(joystick_msg)

    def timer_callback(self) -> None:
        """定时器回调 - 读取并发布VR数据"""
        if self.vr_system is None:
            # 尝试重新初始化
            if not self._init_vr():
                return

        try:
            right_data = None
            # 读取右手控制器
            if self.enable_right:
                right_data = self._get_controller_data('right')
                if right_data:
                    self._publish_controller_data(right_data, 'right')

            # 获取 HMD 位姿
            hmd_pose = self._get_hmd_pose()
            if hmd_pose is not None and self.publish_tf:
                self._publish_hmd_tf(hmd_pose)

            # 发布控制器相对头显位姿
            if hmd_pose is not None:
                if self.enable_right and right_data:
                    ctrl_pose = (right_data.position,
                                 rotation_matrix_to_quaternion(right_data.rotation_matrix))
                    rel_pos, rel_quat = self._get_relative_pose(ctrl_pose, hmd_pose)
                    rel_pos, rel_quat = self._map_pose_hmd_to_ros(rel_pos, rel_quat)
                    self.right_pose_hmd_pub.publish(
                        self._create_pose_msg_from_arrays_frame(
                            rel_pos, rel_quat, self.hmd_frame_id
                        )
                    )
                    if self.publish_tf:
                        self._publish_controller_tf(rel_pos, rel_quat)

        except Exception as e:
            self.get_logger().error(f'Error reading VR data: {e}')

    def destroy_node(self) -> None:
        """清理资源"""
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
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
