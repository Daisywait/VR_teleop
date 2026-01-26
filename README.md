VR_debug 工作区
==============

本工作区包含一套遥操作功能包，共用同一套 VR 追踪输入，输出给 MoveIt Servo 的指令类型为 Twist：

- vr_teleop_twist：笛卡尔速度（Twist）遥操

两套功能包都包含 VR 追踪节点，读取 OpenVR/ALVR，并发布：
- /vr/right_controller/pose_hmd (geometry_msgs/PoseStamped)
- /vr/right_controller/trigger (std_msgs/Float32)
- /vr/right_controller/joystick_y (std_msgs/Float32)
- TF: vr_room -> vr_hmd_ros -> vr_controller_right

功能包说明
----------

1) vr_teleop_twist（速度控制）
   - 节点：vr_converter_node
     - 文件：vr_teleop_twist/franka_teleop_twist_node.py
     - 发布：TwistStamped 到 /moveit_servo/delta_twist_cmds（可配置）
   - 启动：
     - ros2 launch vr_teleop_twist vr_teleop.launch.py
   - 配置：
     - vr_teleop_twist/config/teleop_params.yaml

调试/监视
---------
- 功能包：vr_teleop_debug
- 节点：vr_monitor_node
  - ros2 run vr_teleop_debug vr_monitor_node
  - 日志输出：~/VR_debug/vr_teleop_logs/vr_monitor_YYYYmmdd_HHMMSS.csv

参数说明
--------

A) VR 追踪参数（两包通用，vr_tracker_node.ros__parameters）
- update_rate：VR 数据发布频率 (Hz)，通常 90Hz。
- publish_tf：是否发布 TF。
- frame_id：VR 原点坐标系名（如 vr_room）。
- hmd_frame_id：头显坐标系名（如 vr_hmd_ros）。
- enable_right_controller：是否启用右手控制器。

B) 速度遥操参数（vr_teleop_twist，vr_converter_node.ros__parameters）
- linear_scale：线速度缩放系数。
- angular_scale：角速度缩放系数。
- v_max：线速度上限 (m/s)。
- w_max：角速度上限 (rad/s)。
- smoothing_factor：速度低通滤波系数 (0~1)。
- deadzone_linear：线速度死区阈值 (m)。
- deadzone_angular：角速度死区阈值 (rad)。
- trigger_threshold：扳机触发阈值。
- planning_frame：MoveIt/机器人基坐标系（如 fr3_link0）。
- ee_frame：末端执行器 TF 坐标系（如 robotiq_85_base_link）。
- publish_rate：速度指令发布频率 (Hz)。
- gripper_tcp_xyz：TCP 相对 base_link 的平移 (m)。
- gripper_tcp_rpy：TCP 相对 base_link 的旋转 (deg)。
- twist_topic：MoveIt Servo 速度指令话题。
- gripper_action：夹爪 GripperCommand action 名称。
- gripper_open_pos：夹爪张开目标位置。
- gripper_close_pos：夹爪闭合目标位置。
- gripper_force：夹爪最大力。
- gripper_speed：夹爪速度（与位置增量相关）。
- gripper_axis_deadzone：摇杆死区阈值。
- gripper_deadband：夹爪位置变化小于该值时不发送新命令。
- gripper_rate：夹爪指令发送频率 (Hz)。

备注
----
- 遥操使用“扳机触发 + 锚点相对位姿”的逻辑，并支持夹爪控制。
- 发布频率、缩放系数、话题名等参数请在 YAML 中调整。

控制链路图（ASCII）
------------------

Twist 速度遥操链路：

VR 设备
  └─ vr_teleop_twist/vr_tracker_node
       ├─ /vr/right_controller/pose_hmd
       ├─ /vr/right_controller/trigger
       └─ /vr/right_controller/joystick_y
               │
               ▼
  vr_teleop_twist/vr_converter_node
       ├─ 输出：/moveit_servo/delta_twist_cmds (TwistStamped)
       └─ 夹爪：/robotiq_gripper_controller/gripper_cmd (GripperCommand)
               │
               ▼
          MoveIt Servo
               │
               ▼
        ROS2 控制器 / 机械臂

话题 + 节点表格
--------------

节点 | 所属包 | 订阅 | 发布 | 说明
----|--------|------|------|-----
vr_tracker_node | vr_teleop_twist | (无) | /vr/right_controller/pose_hmd<br>/vr/right_controller/trigger<br>/vr/right_controller/joystick_y<br>TF: vr_room -> vr_hmd_ros -> vr_controller_right | 读取 OpenVR/ALVR 并发布 VR 数据与 TF
vr_converter_node | vr_teleop_twist | /vr/right_controller/pose_hmd<br>/vr/right_controller/trigger<br>/vr/right_controller/joystick_y | /moveit_servo/delta_twist_cmds (TwistStamped)<br>/robotiq_gripper_controller/gripper_cmd (GripperCommand) | 速度遥操（扳机触发 + 锚点相对位姿）
moveit_servo | MoveIt Servo | /moveit_servo/delta_twist_cmds | 控制器输出话题 | 将 Twist 转换为关节控制指令
vr_monitor_node | vr_teleop_debug | /vr/right_controller/pose_hmd<br>/vr/right_controller/trigger<br>/vr/right_controller/joystick_y | (终端输出 / CSV 日志) | 监视与记录 VR 数据
