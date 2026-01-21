#!/usr/bin/env python3
"""
VR Controller Reader - 调试工具
读取SteamVR/ALVR手柄的原始位姿数据，记录到日志文件
"""

import openvr
import numpy as np
import time
import os
import csv
from typing import Optional, Dict, List, Any
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


class VRControllerReader:
    """
    VR控制器原始数据读取器 - 调试工具
    
    只读取原始位姿数据，不做任何转换
    """

    def __init__(self, app_type: int = openvr.VRApplication_Other):
        """初始化VR系统"""
        self.vr_system = None
        self._initialized = False

        try:
            self.vr_system = openvr.init(app_type)
            self._initialized = True
            print("[VRControllerReader] OpenVR initialized successfully")
        except openvr.OpenVRError as e:
            print(f"[VRControllerReader] Failed to initialize OpenVR: {e}")
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
                    print(f"[VRControllerReader] Left controller found: index={i}, model={model}")
                elif role == openvr.TrackedControllerRole_RightHand:
                    self._controller_indices['right'] = i
                    print(f"[VRControllerReader] Right controller found: index={i}, model={model}")

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

    def _parse_controller_buttons(self, state) -> tuple:
        """解析控制器按钮状态"""
        pressed = state.ulButtonPressed

        trigger = state.rAxis[1].x if len(state.rAxis) > 1 else 0.0
        grip = state.rAxis[2].x if len(state.rAxis) > 2 else 0.0
        trackpad_x = state.rAxis[0].x if len(state.rAxis) > 0 else 0.0
        trackpad_y = state.rAxis[0].y if len(state.rAxis) > 0 else 0.0

        # 使用位移操作创建按钮掩码: 1 << button_id
        trigger_pressed = bool(pressed & (1 << openvr.k_EButton_SteamVR_Trigger))
        grip_pressed = bool(pressed & (1 << openvr.k_EButton_Grip))
        menu_pressed = bool(pressed & (1 << openvr.k_EButton_ApplicationMenu))

        return trigger, grip, trackpad_x, trackpad_y, trigger_pressed, grip_pressed, menu_pressed

    def get_controller_data(self, hand: str = 'right') -> Optional[RawControllerData]:
        """
        获取原始控制器数据（仅位姿和按钮，不做任何转换）

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

        # 获取原始位姿
        poses = self.get_device_poses()
        pose = poses[device_index]

        # 检查连接和有效性
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

        # 提取位置 (原始数据，不转换)
        position = np.array([
            pose.mDeviceToAbsoluteTracking[0][3],
            pose.mDeviceToAbsoluteTracking[1][3],
            pose.mDeviceToAbsoluteTracking[2][3]
        ])

        # 提取旋转矩阵并转换为四元数
        quaternion = self._matrix_to_quaternion(pose.mDeviceToAbsoluteTracking)

        # 速度 (原始数据，不转换)
        velocity = np.array([
            pose.vVelocity.v[0],
            pose.vVelocity.v[1],
            pose.vVelocity.v[2]
        ])

        # 角速度 (原始数据，不转换)
        angular_velocity = np.array([
            pose.vAngularVelocity.v[0],
            pose.vAngularVelocity.v[1],
            pose.vAngularVelocity.v[2]
        ])

        # 获取按钮状态
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

    def get_hmd_pose(self) -> Optional[tuple]:
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

        # 轴0通常是震动轴
        self.vr_system.triggerHapticPulse(device_index, 0,
                                          min(duration_microseconds, 3999))
        return True

    def is_initialized(self) -> bool:
        """检查VR系统是否已初始化"""
        return self._initialized

    def get_tracking_space_origin(self) -> str:
        """获取当前追踪空间类型"""
        # 返回追踪空间类型描述
        return "Standing"  # 我们使用Standing模式

    def shutdown(self) -> None:
        """关闭VR系统"""
        if self._initialized:
            openvr.shutdown()
            self._initialized = False
            print("[VRControllerReader] OpenVR shutdown")


def main():
    """调试工具主函数 - 输出原始位姿数据并记录到文件，支持轴向测试"""
    import argparse

    parser = argparse.ArgumentParser(description='VR 控制器调试工具')
    args = parser.parse_args()

    # 原有的轴向测试模式
    # 创建日志目录
    log_dir = "/home/enine/VR_debug/vr_teleop_logs"
    os.makedirs(log_dir, exist_ok=True)

    # 时间戳
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = os.path.join(log_dir, f"vr_raw_data_{timestamp}.csv")
    analysis_log_file = os.path.join(log_dir, f"vr_analysis_{timestamp}.txt")

    print("=" * 70)
    print("VR 手柄轴向测试工具")
    print("=" * 70)
    print("测试方法:")
    print("  1. 系统会提示你往特定方向移动手柄")
    print("  2. 按 Grip 按键切换到下一个测试方向")
    print("  3. 按下 扳机开始记录基准位置")
    print("  4. 沿提示方向移动手柄")
    print("  5. 松开 扳机，系统分析位移和轴向差距")
    print("  6. 按 Ctrl+C 退出并查看总结")
    print(f"数据文件: {log_file}")
    print(f"分析日志: {analysis_log_file}")
    print("坐标系: 右手系 (X 右, Y 上, Z 前)")
    print("=" * 70)

    try:
        reader = VRControllerReader()
    except Exception as e:
        print(f"初始化失败: {e}")
        print("\n请确保:")
        print("1. SteamVR 正在运行")
        print("2. 手柄已连接")
        print("3. 已安装 openvr 库: pip install openvr")
        return

    # 检查控制器是否被检测到
    right_controller = reader._controller_indices.get('right')
    if right_controller is None:
        print("\n❌ 未检测到右手控制器！")
        print("\n请检查:")
        print("1. 右手柄是否正确连接到电脑")
        print("2. SteamVR 是否正在运行")
        print("3. 手柄是否在追踪范围内")
        print("4. 尝试重新连接手柄或重启SteamVR")
        print("\n程序退出。")
        reader.shutdown()
        return

    print(f"\n✅ 检测到右手控制器 (索引: {right_controller})")
    print("\n开始读取数据...\n")

    # 打开CSV文件写入
    csv_file = open(log_file, 'w', newline='')
    csv_writer = csv.writer(csv_file)

    # 打开分析日志文件
    analysis_file = open(analysis_log_file, 'w', encoding='utf-8')
    analysis_file.write("=" * 70 + "\n")
    analysis_file.write("VR 轴向测试分析日志\n")
    analysis_file.write(f"测试时间: {timestamp}\n")
    analysis_file.write("坐标系: 右手系 (X 右, Y 上, Z 后)\n")
    analysis_file.write("=" * 70 + "\n\n")

    # 收集并写入设备信息
    try:
        hmd_index = openvr.k_unTrackedDeviceIndex_Hmd
        hmd_model = reader._get_string_property(hmd_index, openvr.Prop_ModelNumber_String)
        hmd_serial = reader._get_string_property(hmd_index, openvr.Prop_SerialNumber_String)
        hmd_manu = reader._get_string_property(hmd_index, openvr.Prop_ManufacturerName_String)
        hmd_tracking = reader._get_string_property(hmd_index, openvr.Prop_TrackingSystemName_String)

        right_index = reader._controller_indices.get('right')
        if right_index is not None:
            right_model = reader._get_string_property(right_index, openvr.Prop_ModelNumber_String)
            right_serial = reader._get_string_property(right_index, openvr.Prop_SerialNumber_String)
            right_manu = reader._get_string_property(right_index, openvr.Prop_ManufacturerName_String)
            right_tracking = reader._get_string_property(right_index, openvr.Prop_TrackingSystemName_String)
        else:
            right_model = right_serial = right_manu = right_tracking = 'N/A'

        # 写入设备信息到分析日志
        analysis_file.write("设备信息:\n")
        analysis_file.write(f"  HMD (索引={hmd_index}):\n")
        analysis_file.write(f"    型号: {hmd_model}\n")
        analysis_file.write(f"    序列号: {hmd_serial}\n")
        analysis_file.write(f"    制造商: {hmd_manu}\n")
        analysis_file.write(f"    追踪系统: {hmd_tracking}\n\n")

        analysis_file.write(f"  右手控制器 (索引={right_index if right_index is not None else 'N/A'}):\n")
        analysis_file.write(f"    型号: {right_model}\n")
        analysis_file.write(f"    序列号: {right_serial}\n")
        analysis_file.write(f"    制造商: {right_manu}\n")
        analysis_file.write(f"    追踪系统: {right_tracking}\n")
        analysis_file.write("\n" + "=" * 70 + "\n\n")

        # 在终端输出设备信息
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
        '相对位移_X(m)', '相对位移_Y(m)', '相对位移_Z(m)'
    ]
    csv_writer.writerow(["坐标系: 右手系 (X 右, Y 上, Z 前)"])
    csv_writer.writerow(csv_header)

    # 测试统计
    test_results = []  # 存储每次测试的结果
    test_count = 0

    # 定义测试方向
    directions = [
        {'name': '前', 'axis': 'Z', 'sign': 1, 'description': '向前移动手柄'},
        {'name': '后', 'axis': 'Z', 'sign': -1, 'description': '向后移动手柄'},
        {'name': '左', 'axis': 'X', 'sign': 1, 'description': '向左移动手柄'},
        {'name': '右', 'axis': 'X', 'sign': -1, 'description': '向右移动手柄'},
        {'name': '上', 'axis': 'Y', 'sign': 1, 'description': '向上移动手柄'},
        {'name': '下', 'axis': 'Y', 'sign': -1, 'description': '向下移动手柄'}
    ]
    current_direction_index = 0

    try:
        frame = 0
        start_time = time.time()
        last_trigger_state = False
        last_grip_state = False  # 添加 Grip 状态跟踪
        trigger_press_position = None  # 扳机按下时的位置
        trigger_press_time = None
        trigger_press_frame = None
        current_direction = directions[current_direction_index]  # 当前测试方向
        # 采样率设置（Hz）
        sampling_rate_hz = 20
        loop_delay = 1.0 / float(sampling_rate_hz)
        
        while True:
            current_time = time.time() - start_time
            right = reader.get_controller_data('right')

            # 检测 Grip 按键变化来切换测试方向
            if right and right.grip_pressed and not last_grip_state:
                # Grip 刚按下，切换到下一个方向
                current_direction_index = (current_direction_index + 1) % len(directions)
                current_direction = directions[current_direction_index]
                print(f"\n🔄 切换到: {current_direction['name']} - {current_direction['description']}\n")
            
            last_grip_state = right.grip_pressed if right else False

            # 检查控制器追踪状态
            if right is None or not right.is_connected:
                if frame % (sampling_rate_hz * 2) == 0:  # 每2秒显示一次警告
                    print("\n⚠️  警告: 右手控制器未连接或追踪丢失！")
                    print("   请检查控制器连接和追踪状态。")
                    print("   测试可能产生无效数据。\n")
            elif not right.is_valid:
                if frame % (sampling_rate_hz * 2) == 0:  # 每2秒显示一次警告
                    print("\n⚠️  警告: 控制器位姿无效！")
                    print("   追踪质量差，可能影响测试准确性。\n")

            # 检测扳机按下/释放
            if right and right.trigger_pressed and not last_trigger_state:
                # 扳机刚按下，记录当前位置作为基准
                trigger_press_position = right.position.copy() if right.position is not None else None
                trigger_press_time = current_time
                trigger_press_frame = frame

            elif right and not right.trigger_pressed and last_trigger_state:
                # 扳机释放，分析这次测试
                if trigger_press_position is not None and right and right.is_valid and right.position is not None:
                    test_count += 1
                    total_delta = right.position - trigger_press_position
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
                    analysis_file.write(f"【测试 #{test_count}】 - {current_direction['name']}方向\n")
                    analysis_file.write(f"  测试方向: {current_direction['description']}\n")
                    analysis_file.write(f"  期望轴向: {expected_axis}轴 ({'正' if expected_sign > 0 else '负'}方向)\n")
                    analysis_file.write(f"  时间段: {trigger_press_time:.2f}s - {current_time:.2f}s (持续 {test_result['duration']:.2f}s)\n")
                    analysis_file.write(f"  帧数: {trigger_press_frame} - {frame} (共 {test_result['frames']} 帧)\n")
                    analysis_file.write(f"  总位移:\n")
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
                    print(f"【测试 #{test_count} 完成】 - {current_direction['name']}方向")
                    print(f"  期望轴: {expected_axis}轴 ({'正' if expected_sign > 0 else '负'})")
                    print(f"  实际轴: {max_axis}轴 (位移: {actual_value:+.6f} m)")
                    print(f"  位移: X={dx:+.6f} Y={dy:+.6f} Z={dz:+.6f} (m)")
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

            last_trigger_state = right.trigger_pressed if right else False

            # 计算相对位移
            relative_displacement = [0.0, 0.0, 0.0]
            if trigger_press_position is not None and right and right.is_valid:
                delta = right.position - trigger_press_position
                relative_displacement = [float(d) for d in delta]

            # 记录数据到CSV
            row_data = [f"{current_time:.3f}", frame]

            # 右手触发/握持值
            if right:
                row_data.append(f"{right.trigger:.4f}")
                row_data.append(f"{right.grip:.4f}")
                row_data.append(int(right.trigger_pressed))
            else:
                row_data.extend(['', '', 0])

            # 添加当前测试方向和期望轴向
            row_data.append(current_direction['name'])
            row_data.append(f"{current_direction['axis']}{'+' if current_direction['sign'] > 0 else '-'}")

            # 添加相对位移
            row_data.extend([f"{d:.6f}" for d in relative_displacement])

            # 写入CSV
            csv_writer.writerow(row_data)
            csv_file.flush()

            # 每sampling_rate_hz显示一次到终端
            if frame % sampling_rate_hz == 0:
                print("\033[H\033[J", end="")  # ANSI清屏
                print("=" * 70)
                print(f"帧号: {frame:6d}  |  时间: {current_time:.2f}s  |  日志: {log_file}")
                print("坐标系: 右手系 (X 右, Y 上, Z 前)")
                print("=" * 70)

                # 显示当前测试方向
                print(f"\n🎯 当前测试方向: {current_direction['name']} - {current_direction['description']}")
                print(f"   期望轴向: {current_direction['axis']}轴 ({'正' if current_direction['sign'] > 0 else '负'}方向)")

                # 右手数据
                if right:
                    print(f"\n【右手控制器】")
                    if right.is_connected:
                        if right.is_valid:
                            print(f"  扳机: {right.trigger:.2f}  {'[按下]' if right.trigger_pressed else '[松开]'}")
                            if trigger_press_position is not None:
                                print(f"  相对位移: X={relative_displacement[0]:+.6f}  Y={relative_displacement[1]:+.6f}  Z={relative_displacement[2]:+.6f} (m)")
                            else:
                                print("  相对位移: 未记录（请按下扳机以开始记录基准位置）")
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
            print("测试总结")
            print("=" * 70)

            # 写入总结到分析日志
            analysis_file.write("\n" + "=" * 70 + "\n")
            analysis_file.write("测试总结\n")
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
                    print(f"  平均位移: X={avg_dx:.6f} Y={avg_dy:.6f} Z={avg_dz:.6f} (m)")

                    # 写入日志
                    analysis_file.write(f"【{direction_name}方向测试】\n")
                    analysis_file.write(f"  测试次数: {total_count}\n")
                    analysis_file.write(f"  轴向正确次数: {axis_correct_count}\n")
                    analysis_file.write(f"  轴向准确率: {axis_accuracy:.1f}%\n")
                    if correct_tests:
                        analysis_file.write(f"  平均符号差距: {avg_sign_diff:.6f} m\n")
                    analysis_file.write(f"  平均位移:\n")
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
                conclusion = "追踪精度良好 ✅"
            elif overall_axis_accuracy >= 70:
                conclusion = "追踪精度一般，需要微调 ⚠️"
            else:
                conclusion = "追踪精度存在问题 ❌"

            print(f"  {conclusion}")
            analysis_file.write(f"  {conclusion}\n")

            # 详细建议
            if overall_axis_accuracy < 100:
                print("  建议:")
                analysis_file.write("  建议:\n")
                for direction_name, tests in direction_stats.items():
                    axis_correct_count = sum(1 for t in tests if t['axis_correct'])
                    if axis_correct_count < len(tests):
                        msg = f"    - {direction_name}方向轴向映射需要检查"
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
