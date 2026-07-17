
## 1. 编译

```bash
cd /root/D_Robot/test/dev_ws
source /opt/tros/humble/setup.bash
colcon build --symlink-install --packages-select origincar_system
source install/setup.bash


cd /root/D_Robot/test/dev_ws
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /opt/tros/humble/setup.bash
colcon build --symlink-install --packages-select \
  origincar_msg camera_driver chassis_driver lidar_driver lidar_location \
  line_detector perception_detector qr_detector_cpp visual_control \
  vlm_tts_demo origincar_system
```
## 2. 启动顺序

### 终端一：启动硬件、EKF、地图和 AMCL

```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /opt/tros/humble/setup.bash
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 launch origincar_system test_bringup.launch.py \
  start_foxglove:=true
```


### 终端二：启动 visual_lidar_path_follower_szc
```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /opt/tros/humble/setup.bash
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 launch origincar_system visual_lidar_path_follower_szc.launch.xml
```

启动后会同时运行 SZC 专用路线图渲染节点，但运行中只在内存里缓存路径坐标，不加载地图、不调用 Pillow 绘图，也不写 PNG。环线追点完成后，控制节点通过 `/visual_lidar_path_follower_szc/route_finished` 通知渲染节点；渲染节点以较低的进程优先级一次性生成：

```text
/root/D_Robot/test/dev_ws/src/origincar_system/path/szc/latest_final_shifted_route.png
/root/D_Robot/test/dev_ws/src/origincar_system/path/szc/<启动时间>/01_original_route.png
/root/D_Robot/test/dev_ws/src/origincar_system/path/szc/<启动时间>/02_final_shifted_route.png
```

`02_final_shifted_route.png` 会保留每个 waypoint 最后一次确认过的非零偏移。即使逻辑1在车辆通过后释放了当前偏移，最终图片仍会显示实际确认过的偏移方向和偏移量。Foxglove 的实时路径话题不受影响，仍会随逻辑1动态更新。

### 终端三：需要完整视觉巡线流程时启动 visual_control

```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /opt/tros/humble/setup.bash
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
  delta_x_ratio:=0.0 debug_control_logs:=true
```

```bash
pkill -INT -f lidar_driver_node
sleep 2
```
