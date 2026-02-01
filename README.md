VR_Teleop 工作区
==============

本工作区包含两套功能包：

- vr_teleop_twist：VR 追踪 + 速度遥操（Twist）
- vr_teleop_debug：VR 数据监视与日志

说明
----
- 详细使用说明请见各包内 README：
  - vr_teleop_twist/README.md
  - vr_teleop_debug/README.md
- 启动（vr_teleop_twist）：
  ```
  ros2 launch vr_teleop_twist vr_teleop.launch.py
  ```
- 启动（vr_teleop_debug）：
  ```
  ros2 launch vr_teleop_debug vr_teleop_debug.launch.py
  ```