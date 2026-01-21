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
  - `frame_id`: `vr_room`
  - `hmd_frame_id`: `vr_hmd_ros`
- `franka_teleop_node.ros__parameters`：
  - 速度缩放与限幅参数
  - `vr_to_robot_rotation`（HMD 坐标到机器人坐标的旋转）
  - 夹爪 action 与控制参数

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
