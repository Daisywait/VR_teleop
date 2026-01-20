#!/usr/bin/env python3
"""
VR Relative Controller Reader - 头显坐标系轴向测试工具

用途：
- 验证控制器在【头显局部坐标系】下的相对位移方向
- 通过物理移动手柄，验证轴向映射是否符合实测坐标语义

房间坐标系定义（实测，以本系统为准）：
  右手系
  +X：左
  +Y：上
  +Z：前

注意：
- 本定义来源于 SteamVR Standing Space + ALVR 实测结果
- 后续所有“前 / 后 / 左 / 右”判断均严格基于坐标轴正负
"""

import openvr
import numpy as np
import time
import os
import csv
from typing import Optional, Dict, List, Tuple
from dataclasses import dataclass
from datetime import datetime


@dataclass
class RawControllerData:
    """原始控制器数据 - 仅位姿和按钮"""
    position: np.ndarray        # [x, y, z] 位置 (米)
    quaternion: tuple           # (w, x, y, z) 四元数
    velocity: np.ndarray        # [vx, vy, vz] 线速度 (m/s)
    angular_velocity: np.ndarray # [wx, wy, wz] 角速度 (rad/s)
    trigger: float              # 扳机值 0-1
    grip: float                 # 握持键值 0-1
    thumbstick_x: float         # 摇杆X -1到1
    thumbstick_y: float         # 摇杆Y -1到1
    trigger_pressed: bool       # 扳机是否按下
    grip_pressed: bool          # 握持键是否按下
    menu_pressed: bool          # 菜单键是否按下
    is_connected: bool          # 设备是否连接
    is_valid: bool              # 位姿是否有效


@dataclass
class RelativeControllerData:
    """相对于头显的控制器数据"""
    # 相对于头显的数据
    relative_position: np.ndarray        # [x, y, z] 相对位置 (米)
    relative_quaternion: tuple           # (w, x, y, z) 相对四元数
    relative_velocity: np.ndarray        # [vx, vy, vz] 相对线速度 (m/s)
    relative_angular_velocity: np.ndarray # [wx, wy, wz] 相对角速度 (rad/s)
    
    # 原始数据（绝对坐标系）
    absolute_position: np.ndarray
    absolute_quaternion: tuple
    
    # 按钮状态
    trigger: float
    grip: float
    thumbstick_x: float
    thumbstick_y: float
    trigger_pressed: bool
    grip_pressed: bool
    menu_pressed: bool
    
    # 状态标志
    is_connected: bool
    is_valid: bool
    hmd_valid: bool  # 头显位姿是否有效


class VRRelativeControllerReader:
    """
    VR控制器相对坐标系读取器
    
    提供相对于头显的控制器位姿数据
    坐标系：以头显为原点的局部坐标系
    """

    def __init__(self, app_type: int = openvr.VRApplication_Other):
        """初始化VR系统"""
        self.vr_system = None
        self._initialized = False

        try:
            self.vr_system = openvr.init(app_type)
            self._initialized = True
            print("[VRRelativeControllerReader] OpenVR initialized successfully")
        except openvr.OpenVRError as e:
            print(f"[VRRelativeControllerReader] Failed to initialize OpenVR: {e}")
            raise

        # 缓存控制器索引
        self._controller_indices: Dict[str, int] = {}
        self._update_controller_indices()

    def _update_controller_indices(self) -> None:
        """更新控制器设备索引映射"""
        self._controller_indices = {'left': None, 'right': None}

        for i in range(openvr.k_unMaxTrackedDeviceCount):
            device_class = self.vr_system.getTrackedDeviceClass(i)

            if device_class == openvr.TrackedDeviceClass_Controller:
                role = self.vr_system.getControllerRoleForTrackedDeviceIndex(i)

                # 获取设备信息
                model = self._get_string_property(i, openvr.Prop_ModelNumber_String)

                if role == openvr.TrackedControllerRole_LeftHand:
                    self._controller_indices['left'] = i
                    print(f"[VRRelativeControllerReader] Left controller found: index={i}, model={model}")
                elif role == openvr.TrackedControllerRole_RightHand:
                    self._controller_indices['right'] = i
                    print(f"[VRRelativeControllerReader] Right controller found: index={i}, model={model}")

    def _get_string_property(self, device_index: int, prop: int) -> str:
        """获取设备字符串属性"""
        try:
            return self.vr_system.getStringTrackedDeviceProperty(device_index, prop)
        except openvr.OpenVRError:
            return "Unknown"

    def get_device_poses(self) -> List:
        """获取所有设备位姿"""
        poses = self.vr_system.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding,
            0.0,
            openvr.k_unMaxTrackedDeviceCount
        )
        return poses

    def _matrix_to_quaternion(self, matrix) -> tuple:
        """将3x3旋转矩阵转换为四元数 (w, x, y, z)"""
        # 提取3x3旋转部分
        m = np.array([
            [matrix[0][0], matrix[0][1], matrix[0][2]],
            [matrix[1][0], matrix[1][1], matrix[1][2]],
            [matrix[2][0], matrix[2][1], matrix[2][2]]
        ])
        
        trace = m[0, 0] + m[1, 1] + m[2, 2]
        
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (m[2, 1] - m[1, 2]) * s
            y = (m[0, 2] - m[2, 0]) * s
            z = (m[1, 0] - m[0, 1]) * s
        elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
            s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
            w = (m[2, 1] - m[1, 2]) / s
            x = 0.25 * s
            y = (m[0, 1] + m[1, 0]) / s
            z = (m[0, 2] + m[2, 0]) / s
        elif m[1, 1] > m[2, 2]:
            s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
            w = (m[0, 2] - m[2, 0]) / s
            x = (m[0, 1] + m[1, 0]) / s
            y = 0.25 * s
            z = (m[1, 2] + m[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
            w = (m[1, 0] - m[0, 1]) / s
            x = (m[0, 2] + m[2, 0]) / s
            y = (m[1, 2] + m[2, 1]) / s
            z = 0.25 * s
        
        return (w, x, y, z)

    def _quaternion_inverse(self, quat: tuple) -> tuple:
        """计算四元数的逆 (w, x, y, z)"""
        w, x, y, z = quat
        norm_sq = w*w + x*x + y*y + z*z
        if norm_sq < 1e-10:
            return (1.0, 0.0, 0.0, 0.0)
        return (w/norm_sq, -x/norm_sq, -y/norm_sq, -z/norm_sq)

    def _quaternion_multiply(self, q1: tuple, q2: tuple) -> tuple:
        """四元数乘法 q1 * q2"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return (
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        )

    def _rotate_vector_by_quaternion(self, vec: np.ndarray, quat: tuple) -> np.ndarray:
        """使用四元数旋转向量"""
        # 将向量转换为四元数 (0, x, y, z)
        vec_quat = (0.0, vec[0], vec[1], vec[2])
        
        # q * v * q^-1
        quat_inv = self._quaternion_inverse(quat)
        temp = self._quaternion_multiply(quat, vec_quat)
        result_quat = self._quaternion_multiply(temp, quat_inv)
        
        return np.array([result_quat[1], result_quat[2], result_quat[3]])

    def _parse_controller_buttons(self, state) -> tuple:
        """解析控制器按钮状态"""
        pressed = state.ulButtonPressed

        trigger = state.rAxis[1].x if len(state.rAxis) > 1 else 0.0
        grip = state.rAxis[2].x if len(state.rAxis) > 2 else 0.0
        trackpad_x = state.rAxis[0].x if len(state.rAxis) > 0 else 0.0
        trackpad_y = state.rAxis[0].y if len(state.rAxis) > 0 else 0.0

        trigger_pressed = bool(pressed & (1 << openvr.k_EButton_SteamVR_Trigger))
        grip_pressed = bool(pressed & (1 << openvr.k_EButton_Grip))
        menu_pressed = bool(pressed & (1 << openvr.k_EButton_ApplicationMenu))

        return trigger, grip, trackpad_x, trackpad_y, trigger_pressed, grip_pressed, menu_pressed

    def get_hmd_pose(self) -> Optional[Tuple[np.ndarray, tuple]]:
        """获取头显原始位姿 (position, quaternion)"""
        poses = self.get_device_poses()
        hmd_pose = poses[openvr.k_unTrackedDeviceIndex_Hmd]

        if not hmd_pose.bPoseIsValid:
            return None

        position = np.array([
            hmd_pose.mDeviceToAbsoluteTracking[0][3],
            hmd_pose.mDeviceToAbsoluteTracking[1][3],
            hmd_pose.mDeviceToAbsoluteTracking[2][3]
        ])
        quaternion = self._matrix_to_quaternion(hmd_pose.mDeviceToAbsoluteTracking)

        return (position, quaternion)

    def get_controller_data_absolute(self, hand: str = 'right') -> Optional[RawControllerData]:
        """
        获取原始控制器数据（绝对坐标系）

        Args:
            hand: 'left' 或 'right'

        Returns:
            RawControllerData 或 None
        """
        if hand not in self._controller_indices:
            return None

        device_index = self._controller_indices.get(hand)
        if device_index is None:
            self._update_controller_indices()
            device_index = self._controller_indices.get(hand)
            if device_index is None:
                return None

        poses = self.get_device_poses()
        pose = poses[device_index]

        is_connected = pose.bDeviceIsConnected
        is_valid = pose.bPoseIsValid

        if not is_connected or not is_valid:
            return RawControllerData(
                position=np.zeros(3),
                quaternion=(1.0, 0.0, 0.0, 0.0),
                velocity=np.zeros(3),
                angular_velocity=np.zeros(3),
                trigger=0.0,
                grip=0.0,
                thumbstick_x=0.0,
                thumbstick_y=0.0,
                trigger_pressed=False,
                grip_pressed=False,
                menu_pressed=False,
                is_connected=is_connected,
                is_valid=is_valid
            )

        position = np.array([
            pose.mDeviceToAbsoluteTracking[0][3],
            pose.mDeviceToAbsoluteTracking[1][3],
            pose.mDeviceToAbsoluteTracking[2][3]
        ])

        quaternion = self._matrix_to_quaternion(pose.mDeviceToAbsoluteTracking)

        velocity = np.array([
            pose.vVelocity.v[0],
            pose.vVelocity.v[1],
            pose.vVelocity.v[2]
        ])

        angular_velocity = np.array([
            pose.vAngularVelocity.v[0],
            pose.vAngularVelocity.v[1],
            pose.vAngularVelocity.v[2]
        ])

        result, state = self.vr_system.getControllerState(device_index)
        if result:
            trigger, grip, trackpad_x, trackpad_y, trigger_pressed, grip_pressed, menu_pressed = \
                self._parse_controller_buttons(state)
        else:
            trigger = grip = trackpad_x = trackpad_y = 0.0
            trigger_pressed = grip_pressed = menu_pressed = False

        return RawControllerData(
            position=position,
            quaternion=quaternion,
            velocity=velocity,
            angular_velocity=angular_velocity,
            trigger=trigger,
            grip=grip,
            thumbstick_x=trackpad_x,
            thumbstick_y=trackpad_y,
            trigger_pressed=trigger_pressed,
            grip_pressed=grip_pressed,
            menu_pressed=menu_pressed,
            is_connected=is_connected,
            is_valid=is_valid
        )

    def get_controller_data_relative(self, hand: str = 'right') -> Optional[RelativeControllerData]:
        """
        获取相对于头显的控制器数据

        Args:
            hand: 'left' 或 'right'

        Returns:
            RelativeControllerData 或 None
        """
        # 获取控制器绝对位姿
        controller_abs = self.get_controller_data_absolute(hand)
        if controller_abs is None:
            return None

        # 获取头显位姿
        hmd_pose = self.get_hmd_pose()
        if hmd_pose is None:
            # 头显位姿无效，返回绝对数据
            return RelativeControllerData(
                relative_position=controller_abs.position,
                relative_quaternion=controller_abs.quaternion,
                relative_velocity=controller_abs.velocity,
                relative_angular_velocity=controller_abs.angular_velocity,
                absolute_position=controller_abs.position,
                absolute_quaternion=controller_abs.quaternion,
                trigger=controller_abs.trigger,
                grip=controller_abs.grip,
                thumbstick_x=controller_abs.thumbstick_x,
                thumbstick_y=controller_abs.thumbstick_y,
                trigger_pressed=controller_abs.trigger_pressed,
                grip_pressed=controller_abs.grip_pressed,
                menu_pressed=controller_abs.menu_pressed,
                is_connected=controller_abs.is_connected,
                is_valid=controller_abs.is_valid,
                hmd_valid=False
            )

        hmd_position, hmd_quaternion = hmd_pose

        # 计算相对位置：将控制器位置转换到头显局部坐标系
        # relative_pos = R_hmd^T * (controller_pos - hmd_pos)
        position_diff = controller_abs.position - hmd_position
        hmd_quat_inv = self._quaternion_inverse(hmd_quaternion)
        relative_position = self._rotate_vector_by_quaternion(position_diff, hmd_quat_inv)

        # 计算相对旋转：relative_quat = hmd_quat^-1 * controller_quat
        relative_quaternion = self._quaternion_multiply(hmd_quat_inv, controller_abs.quaternion)

        # 计算相对速度：转换到头显局部坐标系
        relative_velocity = self._rotate_vector_by_quaternion(controller_abs.velocity, hmd_quat_inv)

        # 计算相对角速度：转换到头显局部坐标系
        relative_angular_velocity = self._rotate_vector_by_quaternion(
            controller_abs.angular_velocity, hmd_quat_inv
        )

        return RelativeControllerData(
            relative_position=relative_position,
            relative_quaternion=relative_quaternion,
            relative_velocity=relative_velocity,
            relative_angular_velocity=relative_angular_velocity,
            absolute_position=controller_abs.position,
            absolute_quaternion=controller_abs.quaternion,
            trigger=controller_abs.trigger,
            grip=controller_abs.grip,
            thumbstick_x=controller_abs.thumbstick_x,
            thumbstick_y=controller_abs.thumbstick_y,
            trigger_pressed=controller_abs.trigger_pressed,
            grip_pressed=controller_abs.grip_pressed,
            menu_pressed=controller_abs.menu_pressed,
            is_connected=controller_abs.is_connected,
            is_valid=controller_abs.is_valid,
            hmd_valid=True
        )

    def trigger_haptic_pulse(self, hand: str = 'right',
                             duration_microseconds: int = 3000) -> bool:
        """
        触发手柄震动

        Args:
            hand: 'left' 或 'right'
            duration_microseconds: 震动时长(微秒), 最大3999

        Returns:
            是否成功
        """
        device_index = self._controller_indices.get(hand)
        if device_index is None:
            return False

        self.vr_system.triggerHapticPulse(device_index, 0,
                                          min(duration_microseconds, 3999))
        return True

    def is_initialized(self) -> bool:
        """检查VR系统是否已初始化"""
        return self._initialized

    def shutdown(self) -> None:
        """关闭VR系统"""
        if self._initialized:
            openvr.shutdown()
            self._initialized = False
            print("[VRRelativeControllerReader] OpenVR shutdown")


def main():
    """头显坐标系轴向测试工具 - 验证相对坐标转换"""
    import argparse

    parser = argparse.ArgumentParser(description='VR 头显坐标系轴向测试工具')
    args = parser.parse_args()

    # 创建日志目录
    log_dir = os.path.expanduser("~/vr_teleop_logs")
    os.makedirs(log_dir, exist_ok=True)

    # 时间戳
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = os.path.join(log_dir, f"vr_relative_axis_test_{timestamp}.csv")
    analysis_log_file = os.path.join(log_dir, f"vr_relative_analysis_{timestamp}.txt")

    print("=" * 70)
    print("VR 头显坐标系轴向测试工具")
    print("=" * 70)
    print("测试方法:")
    print("  1. 保持头显朝向不动")
    print("  2. 按 Grip 按键切换到下一个测试方向")
    print("  3. 按下扳机开始记录基准位置")
    print("  4. 相对于头显方向移动手柄:")
    print("     - 前: 沿头显朝向前方")
    print("     - 后: 沿头显朝向后方")
    print("     - 左: 头显视角的左边")
    print("     - 右: 头显视角的右边")
    print("     - 上: 头显视角的上方")
    print("     - 下: 头显视角的下方")
    print("  5. 松开扳机，系统分析相对位移和轴向")
    print("  6. 按 Ctrl+C 退出并查看总结")
    print(f"数据文件: {log_file}")
    print(f"分析日志: {analysis_log_file}")
    print("坐标系: 头显局部坐标系（右手系）(+X 左, +Y 上, +Z 前)")
    print("=" * 70)

    try:
        reader = VRRelativeControllerReader()
    except Exception as e:
        print(f"初始化失败: {e}")
        print("\n请确保:")
        print("1. SteamVR 正在运行")
        print("2. 手柄已连接")
        print("3. 已安装 openvr 库: pip install openvr")
        return

    # 检查控制器
    right_controller = reader._controller_indices.get('right')
    if right_controller is None:
        print("\n❌ 未检测到右手控制器！")
        reader.shutdown()
        return

    print(f"\n✅ 检测到右手控制器 (索引: {right_controller})")
    print("\n开始读取数据...\n")

    # 打开CSV文件
    csv_file = open(log_file, 'w', newline='')
    csv_writer = csv.writer(csv_file)

    # 打开分析日志
    analysis_file = open(analysis_log_file, 'w', encoding='utf-8')
    analysis_file.write("=" * 70 + "\n")
    analysis_file.write("VR 头显坐标系轴向测试分析日志\n")
    analysis_file.write(f"测试时间: {timestamp}\n")
    analysis_file.write("坐标系: 头显局部坐标系（右手系）(+X 左, +Y 上, +Z 前)\n")
    analysis_file.write("=" * 70 + "\n\n")

    # 写入设备信息
    try:
        hmd_index = openvr.k_unTrackedDeviceIndex_Hmd
        hmd_model = reader._get_string_property(hmd_index, openvr.Prop_ModelNumber_String)
        hmd_serial = reader._get_string_property(hmd_index, openvr.Prop_SerialNumber_String)

        right_index = reader._controller_indices.get('right')
        if right_index is not None:
            right_model = reader._get_string_property(right_index, openvr.Prop_ModelNumber_String)
            right_serial = reader._get_string_property(right_index, openvr.Prop_SerialNumber_String)
        else:
            right_model = right_serial = 'N/A'

        analysis_file.write("设备信息:\n")
        analysis_file.write(f"  HMD: {hmd_model} (序列号: {hmd_serial})\n")
        analysis_file.write(f"  右手控制器: {right_model} (序列号: {right_serial})\n")
        analysis_file.write("\n" + "=" * 70 + "\n\n")

        print("\n设备信息:")
        print(f"  HMD: {hmd_model} (序列号: {hmd_serial})")
        print(f"  右手控制器: {right_model} (序列号: {right_serial})")
        print()

    except Exception as e:
        analysis_file.write(f"设备信息获取失败: {e}\n\n")

    analysis_file.flush()

    # 写入CSV表头
    csv_header = [
        '时间戳(秒)', '帧号',
        '右手_扳机值', '右手_Grip值', '右手_扳机按下',
        '测试方向', '期望轴向',
        '相对位移_X(m)', '相对位移_Y(m)', '相对位移_Z(m)',
        '头显有效'
    ]
    csv_writer.writerow(["坐标系: 头显局部坐标系（右手系）(+X 左, +Y 上, +Z 前)"])
    csv_writer.writerow(csv_header)

    # 测试统计
    test_results = []
    test_count = 0

    # 定义测试方向 (头显坐标系)
    directions = [
        {'name': '前', 'axis': 'Z', 'sign': -1, 'description': '向头显前方移动手柄'},
        {'name': '后', 'axis': 'Z', 'sign': 1, 'description': '向头显后方移动手柄'},
        {'name': '左', 'axis': 'X', 'sign': -1, 'description': '向头显左边移动手柄'},
        {'name': '右', 'axis': 'X', 'sign': 1, 'description': '向头显右边移动手柄'},
        {'name': '上', 'axis': 'Y', 'sign': 1, 'description': '向头显上方移动手柄'},
        {'name': '下', 'axis': 'Y', 'sign': -1, 'description': '向头显下方移动手柄'}
    ]
    current_direction_index = 0

    try:
        frame = 0
        start_time = time.time()
        last_trigger_state = False
        last_grip_state = False
        trigger_press_position = None  # 扳机按下时的相对位置
        trigger_press_time = None
        trigger_press_frame = None
        current_direction = directions[current_direction_index]
        
        # 采样率设置（Hz）
        sampling_rate_hz = 20
        loop_delay = 1.0 / float(sampling_rate_hz)
        
        while True:
            current_time = time.time() - start_time
            rel_data = reader.get_controller_data_relative('right')

            # 检测 Grip 按键变化来切换测试方向
            if rel_data and rel_data.grip_pressed and not last_grip_state:
                current_direction_index = (current_direction_index + 1) % len(directions)
                current_direction = directions[current_direction_index]
                print(f"\n🔄 切换到: {current_direction['name']} - {current_direction['description']}\n")
            
            last_grip_state = rel_data.grip_pressed if rel_data else False

            # 检查状态
            if rel_data is None or not rel_data.is_connected:
                if frame % (sampling_rate_hz * 2) == 0:
                    print("\n⚠️  警告: 右手控制器未连接或追踪丢失！\n")
            elif not rel_data.is_valid:
                if frame % (sampling_rate_hz * 2) == 0:
                    print("\n⚠️  警告: 控制器位姿无效！\n")
            elif not rel_data.hmd_valid:
                if frame % (sampling_rate_hz * 2) == 0:
                    print("\n⚠️  警告: 头显位姿无效！相对坐标可能不准确！\n")

            # 检测扳机按下/释放
            if rel_data and rel_data.trigger_pressed and not last_trigger_state:
                # 扳机刚按下，记录当前相对位置作为基准
                if rel_data.hmd_valid:
                    trigger_press_position = rel_data.relative_position.copy()
                    trigger_press_time = current_time
                    trigger_press_frame = frame
                else:
                    print("⚠️  头显追踪无效，无法开始测试")

            elif rel_data and not rel_data.trigger_pressed and last_trigger_state:
                # 扳机释放，分析这次测试
                if trigger_press_position is not None and rel_data.is_valid and rel_data.hmd_valid:
                    test_count += 1
                    total_delta = rel_data.relative_position - trigger_press_position
                    dx, dy, dz = float(total_delta[0]), float(total_delta[1]), float(total_delta[2])

                    # 计算绝对位移
                    abs_dx, abs_dy, abs_dz = abs(dx), abs(dy), abs(dz)
                    displacements = {'X': abs_dx, 'Y': abs_dy, 'Z': abs_dz}
                    
                    # 找出主要变化的轴向
                    max_axis = max(displacements, key=displacements.get)
                    max_displacement = displacements[max_axis]

                    # 获取实际位移值（带符号）
                    actual_displacements = {'X': dx, 'Y': dy, 'Z': dz}
                    actual_value = actual_displacements[max_axis]

                    # 计算期望轴向的位移
                    expected_axis = current_direction['axis']
                    expected_sign = current_direction['sign']
                    expected_value = actual_displacements[expected_axis]

                    # 计算差距
                    # 计算差距
                    if expected_axis == max_axis:
                        # 主要轴正确，计算符号差距
                        expected_signed_value = expected_sign * abs(expected_value) if expected_value != 0 else 0
                        sign_difference = abs(actual_value - expected_signed_value)
                        axis_correct = True
                    else:
                        # 轴向错误
                        sign_difference = float('inf')  # 无限大差距
                        axis_correct = False

                    # 记录测试结果
                    test_result = {
                        'test_num': test_count,
                        'direction': current_direction['name'],
                        'expected_axis': expected_axis,
                        'expected_sign': expected_sign,
                        'actual_axis': max_axis,
                        'actual_value': actual_value,
                        'expected_value': expected_value,
                        'axis_correct': axis_correct,
                        'sign_difference': sign_difference,
                        'dx': dx,
                        'dy': dy,
                        'dz': dz,
                        'duration': current_time - trigger_press_time,
                        'frames': frame - trigger_press_frame
                    }
                    test_results.append(test_result)

                    # 写入分析日志
                    analysis_file.write(f"【测试 #{test_count}】 - {current_direction['name']}方向 (头显坐标系)\n")
                    analysis_file.write(f"  测试方向: {current_direction['description']}\n")
                    analysis_file.write(f"  期望轴向: {expected_axis}轴 ({'正' if expected_sign > 0 else '负'}方向)\n")
                    analysis_file.write(f"  时间段: {trigger_press_time:.2f}s - {current_time:.2f}s (持续 {test_result['duration']:.2f}s)\n")
                    analysis_file.write(f"  帧数: {trigger_press_frame} - {frame} (共 {test_result['frames']} 帧)\n")
                    analysis_file.write(f"  相对位移:\n")
                    analysis_file.write(f"    X轴: {dx:+.6f} m\n")
                    analysis_file.write(f"    Y轴: {dy:+.6f} m\n")
                    analysis_file.write(f"    Z轴: {dz:+.6f} m\n")
                    analysis_file.write(f"  主要变化轴: {max_axis}轴 (位移: {actual_value:+.6f} m)\n")
                    analysis_file.write(f"  期望轴位移: {expected_value:+.6f} m\n")

                    if axis_correct:
                        analysis_file.write(f"  ✅ 轴向正确: {max_axis}轴为主变化轴\n")
                        analysis_file.write(f"  符号差距: {sign_difference:.6f} m\n")
                    else:
                        analysis_file.write(f"  ❌ 轴向错误: 期望{expected_axis}轴，实际{max_axis}轴为主变化\n")
                        analysis_file.write(f"  差距: 轴向不匹配\n")

                    analysis_file.write("\n")
                    analysis_file.flush()

                    # 在终端显示测试结果
                    print("\n" + "=" * 70)
                    print(f"【测试 #{test_count} 完成】 - {current_direction['name']}方向 (头显坐标系)")
                    print(f"  期望轴: {expected_axis}轴 ({'正' if expected_sign > 0 else '负'})")
                    print(f"  实际轴: {max_axis}轴 (位移: {actual_value:+.6f} m)")
                    print(f"  相对位移: X={dx:+.6f} Y={dy:+.6f} Z={dz:+.6f} (m)")
                    if axis_correct:
                        result_mark = "✅ 轴向正确"
                        print(f"  结果: {result_mark} | 符号差距: {sign_difference:.6f} m")
                    else:
                        result_mark = "❌ 轴向错误"
                        print(f"  结果: {result_mark}")
                    print("=" * 70 + "\n")

                # 扳机释放，清除基准位置
                trigger_press_position = None
                trigger_press_time = None
                trigger_press_frame = None

            last_trigger_state = rel_data.trigger_pressed if rel_data else False

            # 计算相对位移
            relative_displacement = [0.0, 0.0, 0.0]
            if trigger_press_position is not None and rel_data and rel_data.is_valid and rel_data.hmd_valid:
                delta = rel_data.relative_position - trigger_press_position
                relative_displacement = [float(d) for d in delta]

            # 记录数据到CSV
            row_data = [f"{current_time:.3f}", frame]

            # 右手触发/握持值
            if rel_data:
                row_data.append(f"{rel_data.trigger:.4f}")
                row_data.append(f"{rel_data.grip:.4f}")
                row_data.append(int(rel_data.trigger_pressed))
            else:
                row_data.extend(['', '', 0])

            # 添加当前测试方向和期望轴向
            row_data.append(current_direction['name'])
            row_data.append(f"{current_direction['axis']}{'+' if current_direction['sign'] > 0 else '-'}")

            # 添加相对位移
            row_data.extend([f"{d:.6f}" for d in relative_displacement])

            # 添加头显有效性
            row_data.append(int(rel_data.hmd_valid) if rel_data else 0)

            # 写入CSV
            csv_writer.writerow(row_data)
            csv_file.flush()

            # 每sampling_rate_hz显示一次到终端
            if frame % sampling_rate_hz == 0:
                print("\033[H\033[J", end="")  # ANSI清屏
                print("=" * 70)
                print(f"帧号: {frame:6d}  |  时间: {current_time:.2f}s  |  日志: {log_file}")
                print("坐标系: 头显局部坐标系（右手系）(+X 左, +Y 上, +Z 前)")
                print("=" * 70)

                # 显示当前测试方向
                print(f"\n🎯 当前测试方向: {current_direction['name']} - {current_direction['description']}")
                print(f"   期望轴向: {current_direction['axis']}轴 ({'正' if current_direction['sign'] > 0 else '负'}方向)")

                # 右手数据
                if rel_data:
                    print(f"\n【右手控制器 - 头显相对坐标系】")
                    if rel_data.is_connected:
                        if rel_data.is_valid:
                            if rel_data.hmd_valid:
                                print(f"  扳机: {rel_data.trigger:.2f}  {'[按下]' if rel_data.trigger_pressed else '[松开]'}")
                                if trigger_press_position is not None:
                                    print(f"  相对位移: X={relative_displacement[0]:+.6f}  Y={relative_displacement[1]:+.6f}  Z={relative_displacement[2]:+.6f} (m)")
                                else:
                                    print("  相对位移: 未记录（请按下扳机以开始记录基准位置）")
                                print(f"  当前相对位置: X={rel_data.relative_position[0]:+.4f}  Y={rel_data.relative_position[1]:+.4f}  Z={rel_data.relative_position[2]:+.4f} (m)")
                            else:
                                print("  ⚠️  头显位姿无效，相对坐标不可用")
                        else:
                            print("  ⚠️  位姿无效")
                    else:
                        print("  ❌ 设备未连接")
                else:
                    print("\n【右手控制器】 未找到")

                print("\n" + "=" * 70)
                print("按 Ctrl+C 退出 | 按 Grip 切换测试方向 | 按扳机开始测试")

            frame += 1
            time.sleep(loop_delay)

    except KeyboardInterrupt:
        print("\n\n退出中...")

        # 生成总结分析
        if test_results:
            print("\n" + "=" * 70)
            print("头显坐标系测试总结")
            print("=" * 70)

            # 写入总结到分析日志
            analysis_file.write("\n" + "=" * 70 + "\n")
            analysis_file.write("头显坐标系测试总结\n")
            analysis_file.write("=" * 70 + "\n\n")

            # 按方向分组统计
            direction_stats = {}
            for result in test_results:
                direction = result['direction']
                if direction not in direction_stats:
                    direction_stats[direction] = []
                direction_stats[direction].append(result)

            # 分析每个方向的测试结果
            for direction_name, tests in direction_stats.items():
                if tests:
                    total_count = len(tests)
                    axis_correct_count = sum(1 for t in tests if t['axis_correct'])
                    axis_accuracy = (axis_correct_count / total_count * 100) if total_count > 0 else 0

                    # 计算平均符号差距（只对轴向正确的测试）
                    correct_tests = [t for t in tests if t['axis_correct']]
                    if correct_tests:
                        avg_sign_diff = sum(t['sign_difference'] for t in correct_tests) / len(correct_tests)
                    else:
                        avg_sign_diff = float('inf')

                    # 计算平均位移
                    avg_dx = sum(t['dx'] for t in tests) / total_count
                    avg_dy = sum(t['dy'] for t in tests) / total_count
                    avg_dz = sum(t['dz'] for t in tests) / total_count

                    # 终端输出
                    print(f"\n【{direction_name}方向测试】")
                    print(f"  测试次数: {total_count}")
                    print(f"  轴向正确次数: {axis_correct_count}")
                    print(f"  轴向准确率: {axis_accuracy:.1f}%")
                    if correct_tests:
                        print(f"  平均符号差距: {avg_sign_diff:.6f} m")
                    print(f"  平均相对位移: X={avg_dx:.6f} Y={avg_dy:.6f} Z={avg_dz:.6f} (m)")

                    # 写入日志
                    analysis_file.write(f"【{direction_name}方向测试】\n")
                    analysis_file.write(f"  测试次数: {total_count}\n")
                    analysis_file.write(f"  轴向正确次数: {axis_correct_count}\n")
                    analysis_file.write(f"  轴向准确率: {axis_accuracy:.1f}%\n")
                    if correct_tests:
                        analysis_file.write(f"  平均符号差距: {avg_sign_diff:.6f} m\n")
                    analysis_file.write(f"  平均相对位移:\n")
                    analysis_file.write(f"    X轴: {avg_dx:.6f} m\n")
                    analysis_file.write(f"    Y轴: {avg_dy:.6f} m\n")
                    analysis_file.write(f"    Z轴: {avg_dz:.6f} m\n")

                    # 判断结果
                    if axis_accuracy >= 80:
                        result_str = "  ✅ 结果: 良好"
                    elif axis_accuracy >= 50:
                        result_str = "  ⚠️  结果: 需要调整"
                    else:
                        result_str = "  ❌ 结果: 严重问题"

                    print(result_str)
                    analysis_file.write(result_str + "\n\n")

            # 总体统计
            total_tests = len(test_results)
            total_axis_correct = sum(1 for t in test_results if t['axis_correct'])
            overall_axis_accuracy = (total_axis_correct / total_tests * 100) if total_tests > 0 else 0

            # 计算总体平均符号差距
            correct_tests = [t for t in test_results if t['axis_correct']]
            if correct_tests:
                overall_avg_sign_diff = sum(t['sign_difference'] for t in correct_tests) / len(correct_tests)
            else:
                overall_avg_sign_diff = float('inf')

            print(f"\n【总体统计】")
            print(f"  总测试次数: {total_tests}")
            print(f"  轴向正确次数: {total_axis_correct}")
            print(f"  总体轴向准确率: {overall_axis_accuracy:.1f}%")
            if correct_tests:
                print(f"  总体平均符号差距: {overall_avg_sign_diff:.6f} m")

            analysis_file.write(f"【总体统计】\n")
            analysis_file.write(f"  总测试次数: {total_tests}\n")
            analysis_file.write(f"  轴向正确次数: {total_axis_correct}\n")
            analysis_file.write(f"  总体轴向准确率: {overall_axis_accuracy:.1f}%\n")
            if correct_tests:
                analysis_file.write(f"  总体平均符号差距: {overall_avg_sign_diff:.6f} m\n\n")

            # 结论和建议
            print("\n【分析结论】")
            analysis_file.write("【分析结论】\n")

            if overall_axis_accuracy >= 90 and (not correct_tests or overall_avg_sign_diff < 0.01):
                conclusion = "✅ 头显坐标系转换正确！"
                analysis_file.write(f"  {conclusion}\n")
                analysis_file.write("  相对坐标系转换工作正常，可以用于实际应用。\n")
                print(f"  {conclusion}")
                print("  相对坐标系转换工作正常，可以用于实际应用。")
            elif overall_axis_accuracy >= 70:
                conclusion = "⚠️  头显坐标系转换基本正确，但需要微调"
                analysis_file.write(f"  {conclusion}\n")
                print(f"  {conclusion}")
            else:
                conclusion = "❌ 头显坐标系转换存在问题"
                analysis_file.write(f"  {conclusion}\n")
                analysis_file.write("  建议检查:\n")
                analysis_file.write("  1. 四元数转换逻辑\n")
                analysis_file.write("  2. 坐标系定义（右手系 vs 左手系）\n")
                analysis_file.write("  3. 旋转矩阵到四元数的转换\n")
                print(f"  {conclusion}")
                print("  建议检查:")
                print("  1. 四元数转换逻辑")
                print("  2. 坐标系定义（右手系 vs 左手系）")
                print("  3. 旋转矩阵到四元数的转换")

            # 详细建议
            if overall_axis_accuracy < 100:
                print("\n  详细建议:")
                analysis_file.write("\n  详细建议:\n")
                for direction_name, tests in direction_stats.items():
                    axis_correct_count = sum(1 for t in tests if t['axis_correct'])
                    if axis_correct_count < len(tests):
                        msg = f"    - {direction_name}方向: 轴向映射需要检查"
                        print(msg)
                        analysis_file.write(msg + "\n")

            print("=" * 70)
            analysis_file.write("\n" + "=" * 70 + "\n")
            analysis_file.flush()

        print(f"\n✅ 数据已保存到: {log_file}")
        print(f"✅ 分析已保存到: {analysis_log_file}")

    finally:
        csv_file.close()
        analysis_file.close()
        reader.shutdown()


if __name__ == '__main__':
    main()

