# lidar_location 定位方案：直接启动

本方案使用 `test_bringup_lidar_location.launch.py` 启动 `lidar_location`。
原有的 `test_bringup.launch.py` 保持不变，因此 `lidar_location` 与 AMCL
两套方案可以在同一个工作空间中按需切换。

> 两套总系统只能选择一套启动，不要同时运行。`lidar_location` 和 AMCL
> 都会发布 `map -> odom` TF，同时运行会造成 TF 冲突。

## 1. 编译

以下命令同时编译 `lidar_location`、AMCL 两套方案共用的功能包：

```bash
cd /root/D_Robot/test/dev_ws
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /opt/tros/humble/setup.bash
colcon build --symlink-install --packages-select \
  origincar_msg camera_driver chassis_driver lidar_driver lidar_location \
  line_detector perception_detector qr_detector_cpp visual_control \
  vlm_tts_demo origincar_system
```

## 2. 终端一：启动 lidar_location 总系统

```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /opt/tros/humble/setup.bash
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 launch origincar_system test_bringup_lidar_location.launch.py \
  start_foxglove:=true
```

`lidar_location` 默认从
`src/origincar_system/config/localization/lidar_location.yaml` 中的
`initial_pose` 启动，目前为 `x=0.0, y=0.0, yaw=0.0`。如果车辆实际起点不同，
请在 Foxglove/RViz 中通过 `/initialpose` 发布正确的初始位姿。

`/amcl_pose` 是为兼容原有下游节点而保留的位姿话题名称，此方案不会启动 AMCL。
可使用以下命令观察定位结果和扫描匹配分数：

```bash
ros2 topic echo /amcl_pose
ros2 topic echo /lidar_location/match_score
```

## 3. 终端二：启动 waypoint_shift 追点

```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 launch origincar_system visual_lidar_path_follower.launch.xml
```

> 必须在车辆扫描二维码前启动本终端，否则追点节点可能错过二维码触发消息，
> `visual_control` 暂停后车辆将无法进入巡点逻辑。

## 4. 终端三：启动 visual_control

```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 launch origincar_system visual_control_test.launch.py \
  avoid_angular_ratio:=-0.005 avoid_linear_speed:=0.45 \
  recovering_linear_speed:=0.75 recovering_angular_ratio:=1.3 \
  follow_angular_ratio:=-4.0 follow_linear_speed:=0.85 \
  bottom_threshold_caution:=295 bottom_threshold_avoid:=305 \
  obstacle_y_jump_threshold:=50 parking_y_threshold:=405 \
  line_confidence_threshold:=0.2 obstacle_confidence_threshold:=0.5 \
  parking_sign_confidence_threshold:=0.6 qr_code_confidence_threshold:=0.6 \
  qr_code_hight_follow_threshold:=20 qr_code_height_threshold:=40 \
  line_center_threshold_x:=280 qr_code_speed_modifier:=0.2 \
  follow_qr_code_ratio:=-4.0 follow_parking_sign_ratio:=-4.5 \
  delta_x_ratio:=0.0 debug_control_logs:=false
```

## 5. lidar_location 定位单独测试（不要与总系统同时启动）

```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 launch origincar_system odom_test_lidar_location.launch.py \
  start_foxglove:=true
```

## 6. 两套定位方案切换入口

| 定位方案 | 总系统 launch | 定位单独测试 launch | 使用说明 |
|---|---|---|---|
| AMCL | `test_bringup.launch.py` | `odom_test.launch.py` | `New2README.md` |
| `lidar_location` | `test_bringup_lidar_location.launch.py` | `odom_test_lidar_location.launch.py` | 本文档 |

切换前先用 `Ctrl+C` 完整停止当前总系统，再启动另一套方案。如果雷达驱动没有正常退出，可执行：

```bash
pkill -INT -f lidar_driver_node
sleep 2
```
