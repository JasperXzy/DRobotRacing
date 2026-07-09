# test/dev_ws 启动流程

目标：硬件、地图定位、二维码、图生文全部使用主线 `/root/D_Robot/dev_ws`；这里只使用 `test/dev_ws` 里的 91 点路径跟随逻辑。

不要启动主线 `waypoint_cruiser`，也不要启动 test 的 `start_qr_navigation.sh`。


source /root/D_Robot/dev_ws/install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml

## 1. 编译 test 的 91 点节点

```bash
source /root/D_Robot/dev_ws/install/setup.bash
cd /root/D_Robot/test/dev_ws
colcon build --symlink-install --packages-select origincar_system
```

## 2. 启动主线硬件

新终端：

```bash
source /root/D_Robot/dev_ws/install/setup.bash && ros2 launch driver_bringup drivers.launch.py
```

## 3. 启动主线 EKF

新终端：

```bash
source /root/D_Robot/dev_ws/install/setup.bash && ros2 launch state_estimation ekf.launch.py
```

## 4. 启动主线地图和 AMCL

新终端启动 `map_server`：

```bash
source /root/D_Robot/dev_ws/install/setup.bash
ros2 run nav2_map_server map_server --ros-args \
  --params-file "/root/D_Robot/dev_ws/config/nav2/nav2(6.13).yaml"
```

新终端启动 `amcl`：

```bash
source /root/D_Robot/dev_ws/install/setup.bash
ros2 run nav2_amcl amcl --ros-args \
  --params-file "/root/D_Robot/dev_ws/config/nav2/nav2(6.13).yaml"
```

新终端激活生命周期节点：

```bash
source /root/D_Robot/dev_ws/install/setup.bash
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -p use_sim_time:=false \
  -p autostart:=true \
  -p node_names:="['map_server','amcl']"
```

确认 TF 正常：

```bash
ros2 run tf2_ros tf2_echo map base_link
```

如果定位不准，在 RViz 里用 `2D Pose Estimate` 手动给初始位姿。

## 5. 启动主线二维码

新终端启动主线目标检测，用来发布二维码 ROI：

```bash
source /root/D_Robot/dev_ws/install/setup.bash && ros2 launch perception_detector perception_detector.launch.py
```

新终端启动主线二维码识别：

```bash
source /root/D_Robot/dev_ws/install/setup.bash
ros2 run qr_detector_cpp qr_detector_node --ros-args \
  --params-file /root/D_Robot/dev_ws/config/perception/qr_detector.yaml
```

主线二维码会发布：

```text
/qr_code      std_msgs/String，二维码原始数字
/sign_switch  origincar_msg/Sign，奇数=3 顺时针，偶数=4 逆时针
```

test 的 91 点节点使用 `/qr_code`。

## 6. 启动主线图生文

新终端：

```bash
source /root/D_Robot/dev_ws/install/setup.bash
ros2 launch vlm_tts_demo vlm_tts_demo.launch.py \
  launch_camera:=false \
  camera_topic:=/camera/image \
  trigger_value:=8 \
  infer_on_capture:=true
```

这里 `trigger_value:=8` 表示收到 `/sign4return=8` 时拍照并立即推理。

如果希望换到别的航点触发图生文，后面启动 test 91 点节点时，`image_analysis_waypoint` 也要改成同一个数字。

## 7. 启动 test 的 91 点路径跟随

新终端：

```bash
source /root/D_Robot/dev_ws/install/setup.bash
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 launch origincar_system smooth_path_follower.launch.xml \
  waypoints_file:=/root/D_Robot/test/dev_ws/src/origincar/origincar_system/origincar_system/config/waypoints.yaml \
  route_direction:=auto \
  enable_qr_direction_select:=true \
  qr_topic:=/qr_code \
  use_qr_parity_direction:=true \
  start_image_analyzer:=false \
  image_analysis_trigger_topic:=/sign4return \
  image_analysis_waypoint:=8
```

这个节点会：

```text
订阅 /scan
读取 map -> odom -> base_link
订阅 /qr_code 决定顺时针/逆时针
到第 8 个航点时发布 /sign4return=8，触发主线图生文
发布 /cmd_vel，由主线底盘驱动执行
```

启动前确认没有其他节点在发布 `/cmd_vel`：

```bash
ros2 topic info /cmd_vel -v
```
