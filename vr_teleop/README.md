VR Teleop 功能包说明
=====================

本功能包用于在 HTC Vive Focus Vision  OpenVR 设备上获取手柄/头显数据，并将手柄相对头显的运动转换为 ROS2 话题与 TF，进一步驱动 Franka FR3 的 MoveIt Servo 速度控制，同时支持摇杆控制 Robotiq 夹爪。

功能概览
--------
- 读取 OpenVR 设备位姿（手柄、头显）
- 计算手柄相对头显位姿（HMD 局部坐标系）
- 将 HMD 局部坐标系转换为 ROS 右手系（Z 轴向上）
- 发布 TF：`vr_room -> vr_hmd_ros -> vr_controller_right`
- 发布遥操所需话题（手柄相对位姿、扳机、摇杆）
- 提供终端清屏显示与日志记录的监视节点

节点与话题
----------
1) vr_tracker_node (`vr_teleop/vr_ros2_node.py`)
   - 发布：
     - `/vr/right_controller/pose_hmd` (`geometry_msgs/PoseStamped`)
     - `/vr/right_controller/trigger` (`std_msgs/Float32`)
     - `/vr/right_controller/joystick_y` (`std_msgs/Float32`)
   - TF：
     - `vr_room -> vr_hmd_ros -> vr_controller_right`

2) vr_converter_node (`vr_teleop/franka_teleop_node.py`)
   - 订阅：
     - `/vr/right_controller/pose_hmd`
     - `/vr/right_controller/trigger`
     - `/vr/right_controller/joystick_y`
   - 发布：
     - MoveIt Servo 速度指令（默认 `/moveit_servo/delta_twist_cmds`）
   - 夹爪控制：
     - 使用 `GripperCommand` action（默认 `/robotiq_gripper_controller/gripper_cmd`）
     - `joystick_y > 0` 夹紧，`joystick_y < 0` 张开

3) vr_monitor_node (`vr_teleop_debug/vr_monitor_node.py`)
   - 订阅：
     - `/vr/right_controller/pose_hmd`
     - `/vr/right_controller/joystick_y`
     - `/vr/right_controller/trigger`
   - 功能：
     - 终端清屏显示数据
     - CSV 记录到 `~/vr_teleop_logs/vr_monitor_*.csv`

启动方式
--------
1) 启动 VR 采集与遥操转换：
```
ros2 launch vr_teleop vr_teleop.launch.py
```

2) 单独启动监视节点：
```
ros2 run vr_teleop_debug vr_monitor_node
```

配置文件
--------
主要参数在 `src/vr_teleop/config/teleop_params.yaml`：
- `vr_tracker_node.ros__parameters`：
  - `update_rate`：VR 数据发布频率 (Hz)，通常 90Hz。
  - `publish_tf`：是否发布 TF。
  - `frame_id`：VR 原点坐标系名（如 `vr_room`）。
  - `hmd_frame_id`：头显坐标系名（如 `vr_hmd_ros`）。
  - `enable_right_controller`：是否启用右手控制器。
- `vr_converter_node.ros__parameters`（`franka_teleop_node.py` 使用）：
  - `linear_scale`：线速度缩放系数。控制器平移 -> 末端线速度的比例增益。
  - `angular_scale`：角速度缩放系数。控制器转动 -> 末端角速度的比例增益。
  - `v_max`：线速度上限 (m/s)。用于裁剪线速度，过大将更敏感。
  - `w_max`：角速度上限 (rad/s)。用于裁剪角速度。
  - `smoothing_factor`：速度低通滤波系数 (0~1)。越大越平滑但响应更慢。
  - `deadzone_linear`：线速度死区阈值 (m)。小于该值的线速度被置零。
  - `deadzone_angular`：角速度死区阈值 (rad)。小于该值的角速度被置零。
  - `trigger_threshold`：扳机触发阈值。>= 该值时启用控制。
  - `planning_frame`：MoveIt/机器人基坐标系（如 `fr3_link0`）。
  - `ee_frame`：末端执行器 TF 坐标系（如 `robotiq_85_base_link`）。
  - `publish_rate`：速度指令发布频率 (Hz)。
  - `vr_to_robot_rotation`：VR 坐标到机器人坐标的旋转 (roll, pitch, yaw, deg)。
  - `gripper_tcp_xyz`：TCP 相对 `robotiq_85_base_link` 的平移 (m)。
  - `gripper_tcp_rpy`：TCP 相对 `robotiq_85_base_link` 的旋转 (deg)。
  - `twist_topic`：MoveIt Servo 速度指令话题。
  - `gripper_action`：夹爪 `GripperCommand` action 名称。
  - `gripper_open_pos`：夹爪张开目标位置。
  - `gripper_close_pos`：夹爪闭合目标位置。
  - `gripper_force`：夹爪最大力。
  - `gripper_speed`：夹爪速度（与位置增量相关）。
  - `gripper_axis_deadzone`：摇杆死区阈值。
  - `gripper_deadband`：夹爪位置变化小于该值时不发送新命令。
  - `gripper_rate`：夹爪指令发送频率 (Hz)。

坐标系说明
----------
- `vr_room`：VR 空间坐标系
- `vr_hmd_ros`：头显局部坐标系（已转换为 ROS 右手系，Z 向上）
- `vr_controller_right`：右手控制器相对头显坐标系

日志
----
监视节点会将数据记录到：
- `~/VR_debug/vr_teleop_logs/vr_monitor_YYYYmmdd_HHMMSS.csv`

备注
----
- 目前遥操采用速度控制方式：扳机按下时发送速度，松开停止。
- 若需要“按下建立锚点、松开结束”的增量位移模式，可在 `franka_teleop_node.py` 中扩展。
