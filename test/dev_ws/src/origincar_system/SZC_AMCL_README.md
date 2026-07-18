# SZC：AMCL定位、进场避障与逻辑1

## 1. 编译

```bash
cd /root/D_Robot/test/dev_ws
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /opt/tros/humble/setup.bash
colcon build --symlink-install --packages-select \
  origincar_msg camera_driver chassis_driver lidar_driver lidar_location \
  line_detector perception_detector qr_detector_cpp visual_control \
  vlm_tts_demo origincar_system
source install/setup.bash
```

## 2. 启动顺序

### 终端一：启动硬件、地图、AMCL和识别节点

```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /opt/tros/humble/setup.bash
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 launch origincar_system test_bringup.launch.py start_foxglove:=true
```

该 launch 会分批启动底盘、雷达、相机、EKF、地图、AMCL、YOLO、line detector、QR detector、VLM/TTS和 Foxglove。QR detector 大约在启动命令执行9秒后拉起。

AMCL 默认初始位姿为 `x=0, y=0, yaw=0`。车辆实际起点不一致时，先在 Foxglove 重新设置初始位姿。

### 终端二：启动 SZC 专用 visual_control

```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /opt/tros/humble/setup.bash
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 launch origincar_system visual_control_szc.launch.py \
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

该节点默认 `start_suspended:=true`，启动后只接收检测数据和输出日志，不发送非零速度。不要同时启动共享版 `visual_control`。

### 终端三：启动 SZC

```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /opt/tros/humble/setup.bash
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 launch origincar_system visual_lidar_path_follower_szc.launch.xml
```

SZC launch 启动控制节点和后台 PNG 路径渲染节点。不要同时启动原版 `visual_lidar_path_follower`。SZC 默认 `wait_for_start_signal: true`，启动后持续发布零速度并等待人工发车。

当前默认参数：

```yaml
approach_control_mode: waypoint_lidar
```

### 终端四：确认日志后发车

```bash
source /opt/tros/humble/setup.bash
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 service call /visual_lidar_path_follower_szc/enable \
  std_srvs/srv/SetBool "{data: true}"
```

发车时会检查：已经收到雷达、两张必需地图和有效 `map -> base_link` TF；`visual`模式还会确认 `visual_control_szc`门控订阅已经连接。未就绪时服务返回 `success=false`，底盘继续保持停车；检查日志后重新调用即可。

运行中需要人工停车时：

```bash
source /opt/tros/humble/setup.bash
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 service call /visual_lidar_path_follower_szc/enable \
  std_srvs/srv/SetBool "{data: false}"
```

`false`会同时关闭SZC速度输出和 `visual_control_szc`专用门控，但不停止YOLO、二维码、line detector等识别节点；再次发送 `true`即可继续。

- `approach_control_mode: visual`：发车后只开启 `visual_control_szc`专用门控；SZC等待二维码，扫码后再接管channel路线。
- `approach_control_mode: waypoint_lidar`：发车后由SZC直接巡点；`visual_control_szc`保持挂起，环线结束后仍可按原流程恢复。
- 四个终端各自保留独立日志，便于分别查看基础节点、视觉控制、SZC状态和服务调用结果。

## 3. `waypoint_lidar`流程

```text
全部节点启动
  -> SZC和visual_control_szc保持零速度，YOLO/二维码识别继续运行
人工发车成功
  -> visual_control_szc继续挂起
  -> AMCL巡waypoints_qr + 直接雷达避障
有效二维码
  -> 选择channel并立即暂停YOLO
  -> 原有一次性二维码倒车（最高优先级）
  -> 追channel航点 + 同一套直接雷达避障
进入channel
  -> 关闭进场雷达避障，启用逻辑1 waypoint_shift
环线结束
  -> 恢复完整视觉识别和visual_control
```

直接雷达避障只覆盖当周期速度，不修改 waypoint。每帧新雷达数据都以 `base_link` 车头中心线为基准重新判断：左侧障碍向右避，右侧障碍向左避；障碍簇越靠近中心线，转向越接近 `approach_lidar_avoid_angular_speed`，越靠近 `approach_lidar_half_width` 边界则线性降到 0。不锁定方向、不设最小转向；当前帧有障碍立即避障，无障碍立即解除。

二维码倒车期间不执行直接雷达避障；进入 channel 后它也不会与逻辑1同时工作。发车服务只检查雷达和地图是否已经收到，不增加雷达超时、clear scan或方向锁定维护。

## 4. `visual`流程

```text
全部节点启动
  -> SZC和visual_control_szc保持零速度
人工发车成功
  -> 只恢复visual_control_szc的扫码前控制
  -> visual_control负责扫码前视觉巡线
有效二维码
  -> 挂起visual_control，选择channel
  -> 原有一次性二维码倒车
  -> 追channel航点 + 旧视觉障碍物覆盖
进入channel
  -> 关闭进场视觉覆盖，启用逻辑1 waypoint_shift
环线结束
  -> 恢复完整视觉识别和visual_control
```

## 5. 最终路径 PNG

运行中只缓存路径，不加载地图、不调用 Pillow、不写 PNG；环线完成后由低优先级渲染进程生成：

```text
/root/D_Robot/test/dev_ws/src/origincar_system/path/szc/latest_final_shifted_route.png
/root/D_Robot/test/dev_ws/src/origincar_system/path/szc/<启动时间>/01_original_route.png
/root/D_Robot/test/dev_ws/src/origincar_system/path/szc/<启动时间>/02_final_shifted_route.png
```

`02_final_shifted_route.png` 保留每个 waypoint 最后一次确认过的非零偏移；Foxglove实时路径仍随逻辑1动态更新。
