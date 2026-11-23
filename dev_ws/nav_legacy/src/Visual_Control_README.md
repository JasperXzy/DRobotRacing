# Visual Control 启动流程

## 编译包
```bash
cd ~/D_Robot/dev_ws/nav_legacy
source /opt/tros/humble/setup.bash
colcon build --packages-select visual_control line_detector perception_detector qr_detector_cpp
source install/setup.bash
```

## 进入工作区
```bash
cd ~/D_Robot/dev_ws/nav_legacy
source /opt/tros/humble/setup.bash
source install/setup.bash
```

## test完整启动流程（雷达版和视觉版看这里）

雷达避障版和视觉避障版的前两个终端完全相同，只有终端三不同。

每个终端都先执行：

```bash
cd /root/D_Robot/test/dev_ws
source /opt/tros/humble/setup.bash
source /root/D_Robot/dev_ws/nav_legacy/install/setup.bash
source /root/D_Robot/test/dev_ws/install/setup.bash
```

### 终端一：启动公共前置系统

```bash
ros2 launch origincar_system test_bringup.launch.py
```

该命令统一启动底盘、相机、雷达、机器人模型、定位、地图、线识别、障碍物
识别和二维码识别。视觉避障版也需要保持雷达运行，因为 AMCL 定位仍然使用
雷达，只是不使用雷达做追点避障。

### 终端二：启动前后两段visual_control

```bash
ros2 launch origincar_system visual_control_test.launch.py \
  avoid_angular_ratio:=-0.005 avoid_linear_speed:=0.5 \
  recovering_linear_speed:=0.85 recovering_angular_ratio:=1.25 \
  follow_angular_ratio:=-3.0 follow_linear_speed:=0.85 \
  bottom_threshold_caution:=295 bottom_threshold_avoid:=305 \
  obstacle_y_jump_threshold:=50 parking_y_threshold:=405 \
  line_confidence_threshold:=0.2 obstacle_confidence_threshold:=0.5 \
  parking_sign_confidence_threshold:=0.6 qr_code_confidence_threshold:=0.6 \
  qr_code_hight_follow_threshold:=20 qr_code_height_threshold:=54 \
  line_center_threshold_x:=280 qr_code_speed_modifier:=0.2 \
  follow_qr_code_ratio:=-4.0 follow_parking_sign_ratio:=-4.5 \
  delta_x_ratio:=0.0 debug_control_logs:=true
```

这个节点不会启动两次，而是一直运行并自动切换阶段：

```text
第一次visual_control：赛道线 + 二维码 + 障碍物，忽略停车位
追点结束后恢复：赛道线 + 停车位 + 障碍物，忽略二维码
```

### 终端三：视觉触发、雷达辅助方向决策的追点链路

```bash
ros2 launch origincar_system visual_lidar_path_follower.launch.xml
```

组合节点的运行参数统一放在：

```text
/root/D_Robot/test/dev_ws/src/origincar_system/config/visual_lidar_path_follower.yaml
```

正常调参只修改这一份 YAML。该命令启动一个 `visual_lidar_path_follower` 节点，
节点内部完成路径追踪、视觉触发和雷达点云簇辅助方向决策，并直接输出 `/cmd_vel`。

## 单节点启动参考

以下命令用于单独调试硬件或识别节点。正常跑上述 test 完整流程时，不需要再
逐个启动这些节点。

## 启动底盘
```bash
ros2 launch chassis_driver chassis.launch.py
```

## 启动相机
```bash
ros2 launch camera_driver camera.launch.py
```

### 线识别
```bash
ros2 launch line_detector line_detector.launch.py
```

### 障碍物 / 停车位识别
推荐使用配置文件启动：
```bash
ros2 launch perception_detector perception_detector.launch.py
```

### 二维码识别
```bash
ros2 run qr_detector_cpp qr_detector_node
```

## 参数参考：中速_offset线性关系         //提升0.85线速度
```bash
ros2 launch visual_control visual_control.launch.py avoid_angular_ratio:=-0.005 avoid_linear_speed:=0.5 recovering_linear_speed:=0.85 recovering_angular_ratio:=1.25 follow_angular_ratio:=-3.0 follow_linear_speed:=0.85 bottom_threshold_caution:=295 bottom_threshold_avoid:=305 obstacle_y_jump_threshold:=50 parking_y_threshold:=405 line_confidence_threshold:=0.2 obstacle_confidence_threshold:=0.5 parking_sign_confidence_threshold:=0.6 qr_code_confidence_threshold:=0.6 qr_code_hight_follow_threshold:=20 qr_code_height_threshold:=54 line_center_threshold_x:=280 qr_code_speed_modifier:=0.2 follow_qr_code_ratio:=-4.0 follow_parking_sign_ratio:=-4.5 delta_x_ratio:=0.0 debug_control_logs:=true
```

## 视觉追点避障实现说明

实验流程：

```text
第一次visual_control（线+二维码+障碍物）
  -> 航点追踪+视觉避障（只处理障碍物）
  -> 第二次visual_control（线+停车位+障碍物）
```

`visual_lidar_path_follower` 在同一个控制循环中先计算巡点速度，再根据
`/racing_obstacle_detection` 的锥桶检测结果决定是否覆盖为避障速度，最后直接
发布到 `/cmd_vel`。

- `/sign4return=5`：挂起 `visual_control`，启用视觉追点避障。
- `/sign4return=6`：停止视觉追点避障输出，恢复 `visual_control`。
- 第一次 `visual_control` 忽略停车位，避免在扫码前误停车。
- 第二次 `visual_control` 忽略二维码，只寻找赛道线、障碍物和停车位。
- `visual_lidar_path_follower` 只处理 `construction_cone`，不会跟随二维码或停车位。
- 避障方向由路径方向参考点与最近非墙点云簇的相对角度决定。
- 障碍物消失后直接恢复追点，不执行回线操作。
- 缺少路径方向目标、雷达或有效点云簇时停车等待，不回退到纯视觉方向。
- 前视相机无法观察车后方，倒车航点不启用视觉避障。

编译：

```bash
cd /root/D_Robot/test/dev_ws
source /opt/tros/humble/setup.bash
source /root/D_Robot/dev_ws/nav_legacy/install/setup.bash
colcon build --packages-select origincar_system
source install/setup.bash
```

启动现有的底盘、相机、定位、线识别、障碍物识别、二维码识别和
`visual_control_test.launch.py` 后，启动实验追点链路。该 test launch 默认设置：

```text
enable_parking_before_path=false
enable_qr_after_path=false
```

然后启动实验追点链路：

```bash
cd /root/D_Robot/test/dev_ws
source /opt/tros/humble/setup.bash
source /root/D_Robot/dev_ws/nav_legacy/install/setup.bash
source install/setup.bash
ros2 launch origincar_system visual_lidar_path_follower.launch.xml
```

上面的命令只启动一个组合节点，不需要额外运行追点器或避障过滤器。

实验版默认沿用上面的中速参数：

```text
linear_speed=0.85
avoid_linear_speed=0.5
avoid_angular_ratio=-0.005
bottom_threshold_caution=295
bottom_threshold_avoid=305
obstacle_confidence_threshold=0.5
delta_x_ratio=0.0
```


## 高速_offset线性关系         //提升0.95速度
```bash
ros2 launch visual_control visual_control.launch.py avoid_angular_ratio:=-0.005 avoid_linear_speed:=0.5 recovering_linear_speed:=0.95 recovering_angular_ratio:=1.45 follow_angular_ratio:=-3.5 follow_linear_speed:=0.95 bottom_threshold_caution:=275 bottom_threshold_avoid:=280 obstacle_y_jump_threshold:=50 parking_y_threshold:=405 line_confidence_threshold:=0.4 obstacle_confidence_threshold:=0.3 parking_sign_confidence_threshold:=0.3 qr_code_confidence_threshold:=0.6 qr_code_hight_follow_threshold:=30 qr_code_height_threshold:=55 line_center_threshold_x:=280 qr_code_speed_modifier:=0.2 follow_qr_code_ratio:=-4.0 follow_parking_sign_ratio:=-4.5 delta_x_ratio:=0.0
```

## 光速_offset线性关系         //提升1.15速度
```bash
ros2 launch visual_control visual_control.launch.py avoid_angular_ratio:=-0.005 avoid_linear_speed:=0.5 recovering_linear_speed:=1.15 recovering_angular_ratio:=1.45 follow_angular_ratio:=-3.5 follow_linear_speed:=0.95 bottom_threshold_caution:=275 bottom_threshold_avoid:=280 obstacle_y_jump_threshold:=50 parking_y_threshold:=415 line_confidence_threshold:=0.4 obstacle_confidence_threshold:=0.3 parking_sign_confidence_threshold:=0.3 qr_code_confidence_threshold:=0.6 qr_code_hight_follow_threshold:=30 qr_code_height_threshold:=54 line_center_threshold_x:=280 qr_code_speed_modifier:=0.2 follow_qr_code_ratio:=-4.0 follow_parking_sign_ratio:=-4.5 delta_x_ratio:=0.15
```


## 性能监测
```bash
sudo hrut_somstatus
```


## 专门用来test、debug
```bash
ros2 launch visual_control visual_control.launch.py avoid_angular_ratio:=-0.0055 avoid_linear_speed:=0.55 recovering_linear_speed:=0.95 recovering_angular_ratio:=1.45 follow_angular_ratio:=-3.5 follow_linear_speed:=0.95 bottom_threshold_caution:=275 bottom_threshold_avoid:=280 obstacle_y_jump_threshold:=50 parking_y_threshold:=405 line_confidence_threshold:=0.4 obstacle_confidence_threshold:=0.3 parking_sign_confidence_threshold:=0.3 qr_code_confidence_threshold:=0.6 qr_code_hight_follow_threshold:=30 qr_code_height_threshold:=55 line_center_threshold_x:=280 qr_code_speed_modifier:=0.2 follow_qr_code_ratio:=-4.0 follow_parking_sign_ratio:=-4.5 delta_x_ratio:=0.0
```












## test巡线
```bash
ros2 launch visual_control visual_control.launch.py avoid_angular_ratio:=-0.005 avoid_linear_speed:=0.5 recovering_linear_speed:=0.85 recovering_angular_ratio:=1.25 follow_angular_ratio:=-3.0 follow_linear_speed:=0.85 bottom_threshold_caution:=295 bottom_threshold_avoid:=305 obstacle_y_jump_threshold:=50 parking_y_threshold:=405 line_confidence_threshold:=0.2 obstacle_confidence_threshold:=0.5 parking_sign_confidence_threshold:=1.1 qr_code_confidence_threshold:=1.1 qr_code_hight_follow_threshold:=20 qr_code_height_threshold:=54 line_center_threshold_x:=320 qr_code_speed_modifier:=0.2 follow_qr_code_ratio:=-3.0 follow_parking_sign_ratio:=-4.5
```























## 中速         //完赛参数
```bash
ros2 launch racing_control racing_control.launch.py avoid_angular_ratio:=-0.5 avoid_linear_speed:=0.3 recovering_linear_speed:=0.8 recovering_angular_ratio:=0.95 follow_angular_ratio:=-4.5 follow_linear_speed:=0.85 bottom_threshold_caution:=275 bottom_threshold_avoid:=314 parking_y_threshold:=395 line_confidence_threshold:=0.7 obstacle_confidence_threshold:=0.7 parking_sign_confidence_threshold:=0.3
```
