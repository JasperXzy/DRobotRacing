# test/dev_ws 91 点测试流程

## 1. 编译 test 的 91 点节点

第一次使用、清理过 test 工作区后、或者改了 `src/*.cpp` / `CMakeLists.txt` / `package.xml` 之后需要编译：

```bash
source /root/D_Robot/dev_ws/install/setup.bash
cd /root/D_Robot/test/dev_ws
colcon build --symlink-install --packages-select origincar_system
```

之后如果只改 `smooth_path_follower.launch.xml` 里的参数，不需要重新编译，重启第 3 步即可。

## 2. 启动前置系统

```bash
source /root/D_Robot/dev_ws/install/setup.bash
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 launch /root/D_Robot/test/dev_ws/src/origincar_system/launch/test_bringup.launch.py \
  start_foxglove:=true
```

这个启动会拉起主线硬件、雷达、相机、EKF、地图、AMCL、识别和图生文。

识别分工：

- `line_detector` 发布 `/racing_track_center_detection`，供 `visual_control` 巡线。
- `perception_detector` 发布 `/racing_obstacle_detection`，里面包含障碍物、停车标志和二维码目标框。
- `qr_detector_node` 订阅 `/camera/qr_roi/image`，解码后发布 `/qr_code`，供 `smooth_path_follower` 切换到巡点。

现在默认不会自动启动 `visual_control`。等前置系统稳定后，单独启动 `visual_control`，小车会直接开始巡线。

等待前置系统启动完成后，确认定位正常：

```bash
source /root/D_Robot/dev_ws/install/setup.bash
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 run tf2_ros tf2_echo map base_link
```

如果定位不准，在 RViz/Foxglove 里重新给初始位姿。

## 3. 启动 test 的点路径跟随

新终端启动：

```bash
source /root/D_Robot/dev_ws/install/setup.bash
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 launch /root/D_Robot/test/dev_ws/src/origincar_system/launch/smooth_path_follower.launch.xml
```

`smooth_path_follower` 默认先等二维码；扫到二维码后会挂起 `visual_control`，并切到 `waypoints_ysvv_replaced.yaml` 跑点。

`waypoints_ysvv_replaced.yaml` 里已经去掉顺时针末尾的 `point_91` 和逆时针末尾的 `point_01`。追点不再负责最终停车；到达环线末尾后，`smooth_path_follower` 会发布 `/sign4return=6` 恢复 `visual_control`，由 `visual_control` 继续完成停车。

## 4. 单独启动 visual_control

新终端启动，车会直接开始执行主线巡线：

```bash
source /root/D_Robot/dev_ws/install/setup.bash
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 launch /root/D_Robot/test/dev_ws/src/origincar_system/launch/visual_control_test.launch.py avoid_angular_ratio:=-0.005 avoid_linear_speed:=0.5 recovering_linear_speed:=0.85 recovering_angular_ratio:=1.25 follow_angular_ratio:=-3.0 follow_linear_speed:=0.85 bottom_threshold_caution:=295 bottom_threshold_avoid:=305 obstacle_y_jump_threshold:=50 parking_y_threshold:=405 line_confidence_threshold:=0.2 obstacle_confidence_threshold:=0.5 parking_sign_confidence_threshold:=0.6 qr_code_confidence_threshold:=0.6 qr_code_hight_follow_threshold:=20 qr_code_height_threshold:=54 line_center_threshold_x:=280 qr_code_speed_modifier:=0.2 follow_qr_code_ratio:=-4.0 follow_parking_sign_ratio:=-4.5 delta_x_ratio:=0.0 debug_control_logs:=false
```

`visual_control_test.launch.py` 会启动主线 `visual_control`，速度直接发布到 `/cmd_vel`。扫到二维码后，`smooth_path_follower` 会向 `/sign4return` 发布 5，挂起 `visual_control`，然后切到 `waypoints_ysvv_replaced.yaml` 接管 `/cmd_vel` 巡点。追点结束后，`smooth_path_follower` 会向 `/sign4return` 发布 6，恢复 `visual_control` 接管停车。

test 启动文件默认给 `visual_control` 打开了两个边界行为：

- 从 `/sign4return=6` 恢复时清空旧的线/目标缓存，等待新的识别结果。
- 刚启动或刚恢复时，如果还没有第一帧线消息，先用 `follow_linear_speed` 直走；收到线后进入原来的巡线/避障/回线状态机。

确认 `visual_control` 是否启动：

```bash
ros2 node list | grep visual_control
ros2 topic info /cmd_vel -v
```

巡线阶段，`/cmd_vel` 的发布者应该是 `/visual_control`。扫到二维码切换后，`/visual_control` 会被挂起，`/cmd_vel` 由 `/smooth_path_follower` 发布。

需要手动挂起 `visual_control` 时：

```bash
ros2 topic pub --once /sign4return std_msgs/msg/Int32 "{data: 5}"
```
