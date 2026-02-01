vr_teleop_debug
===============

VR 遥操数据的调试/监视包，订阅 VR 追踪话题并输出日志。

节点说明
--------

1) vr_monitor_node
   - 订阅：/vr/right_controller/pose_hmd, /vr/right_controller/trigger, /vr/right_controller/joystick_y
   - 输出：终端打印与 CSV 日志

启动
----
- ros2 run vr_teleop_debug vr_monitor_node

日志输出
--------
- ~/VR_debug/vr_teleop_logs/vr_monitor_YYYYmmdd_HHMMSS.csv

说明
----
- VR 追踪话题由 vr_teleop_twist 包内的 vr_tracker_node 发布。
- 若需要查看 VR 追踪参数，请参考 vr_teleop_twist/README.md。
