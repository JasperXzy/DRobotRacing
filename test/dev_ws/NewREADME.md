# lidar_location 替代 AMCL：直接启动

## 1. 编译
```bash
cd /root/D_Robot/test/dev_ws
source /opt/tros/humble/setup.bash
source /root/D_Robot/dev_ws/install/setup.bash
colcon build --symlink-install --packages-select \
  chassis_driver lidar_driver lidar_location perception_detector origincar_system \
  --allow-overriding chassis_driver lidar_driver perception_detector
```
## 2. 终端一：启动总系统
```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 launch origincar_system test_bringup_lidar_location.launch.py \
  start_foxglove:=true
```

# 可用 ros2 topic echo /lidar_location/state 查看质量诊断；该状态只用于观察，不干预车速。

## 3. 终端二：启动 waypoint_shift 追点
```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 launch origincar_system visual_lidar_path_follower.launch.xml
```
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
  qr_code_hight_follow_threshold:=20 qr_code_height_threshold:=54 \
  line_center_threshold_x:=280 qr_code_speed_modifier:=0.2 \
  follow_qr_code_ratio:=-4.0 follow_parking_sign_ratio:=-4.5 \
  delta_x_ratio:=0.0 debug_control_logs:=false
```
## 5. 定位单独测试（不要与总系统同时启动）
```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 launch origincar_system odom_test_lidar_location.launch.py \
  start_foxglove:=false
```
## 杀死雷达
```bash
pkill -INT -f lidar_driver_node
sleep 2
```
