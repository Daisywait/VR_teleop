#!/usr/bin/env python3
"""
VR ROS2 Monitor Node
订阅VR话题并在终端清屏显示，同时记录到日志文件

订阅:
- /vr/right_controller/pose_hmd (geometry_msgs/PoseStamped)
- /vr/right_controller/joystick_y (std_msgs/Float32)
- /vr/right_controller/trigger (std_msgs/Float32)
"""

import os
import csv
from datetime import datetime
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32


class VRMonitorNode(Node):
    """VR话题显示与日志记录节点"""

    def __init__(self):
        super().__init__('vr_monitor_node')

        self.declare_parameter('display_rate', 20.0)
        self.declare_parameter('log_dir', '/home/enine/VR_debug/vr_teleop_logs')
        self.display_rate = float(self.get_parameter('display_rate').value)
        self.log_dir = str(self.get_parameter('log_dir').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.pose_msg: Optional[PoseStamped] = None
        self.joystick_y = 0.0
        self.trigger = 0.0

        self.create_subscription(
            PoseStamped, '/vr/right_controller/pose_hmd',
            self._pose_callback, qos)
        self.create_subscription(
            Float32, '/vr/right_controller/joystick_y',
            self._joystick_callback, qos)
        self.create_subscription(
            Float32, '/vr/right_controller/trigger',
            self._trigger_callback, qos)

        self._init_log()

        period = 1.0 / self.display_rate if self.display_rate > 0.0 else 0.05
        self.timer = self.create_timer(period, self._display_and_log)

        self.get_logger().info('VR Monitor Node Started')

    def _init_log(self) -> None:
        os.makedirs(self.log_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_file_path = os.path.join(self.log_dir, f'vr_monitor_{timestamp}.csv')
        self.log_file = open(self.log_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow([
            'stamp_sec',
            'pos_x', 'pos_y', 'pos_z',
            'quat_w', 'quat_x', 'quat_y', 'quat_z',
            'joystick_y', 'trigger'
        ])
        self.log_file.flush()

    def _pose_callback(self, msg: PoseStamped) -> None:
        self.pose_msg = msg

    def _joystick_callback(self, msg: Float32) -> None:
        self.joystick_y = float(msg.data)

    def _trigger_callback(self, msg: Float32) -> None:
        self.trigger = float(msg.data)

    def _display_and_log(self) -> None:
        if self.pose_msg is None:
            return

        p = self.pose_msg.pose.position
        q = self.pose_msg.pose.orientation
        stamp = self.pose_msg.header.stamp
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9

        # 清屏并显示
        print("\033[H\033[J", end="")
        print("=" * 60)
        print("VR Monitor (pose_hmd / joystick_y / trigger)")
        print("=" * 60)
        print(f"stamp: {stamp_sec:.6f}")
        print(f"position: x={p.x:+.4f} y={p.y:+.4f} z={p.z:+.4f}")
        print(f"quaternion: w={q.w:+.4f} x={q.x:+.4f} y={q.y:+.4f} z={q.z:+.4f}")
        print(f"joystick_y: {self.joystick_y:+.3f}")
        print(f"trigger:    {self.trigger:+.3f}")
        print("=" * 60)
        print(f"log: {self.log_file_path}")

        # 写日志
        self.csv_writer.writerow([
            f"{stamp_sec:.6f}",
            f"{p.x:.6f}", f"{p.y:.6f}", f"{p.z:.6f}",
            f"{q.w:.6f}", f"{q.x:.6f}", f"{q.y:.6f}", f"{q.z:.6f}",
            f"{self.joystick_y:.6f}",
            f"{self.trigger:.6f}"
        ])
        self.log_file.flush()

    def destroy_node(self) -> None:
        if hasattr(self, 'log_file'):
            self.log_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VRMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
