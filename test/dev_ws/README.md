# test/dev_ws 测试流程

当前支持两种追点避障方式：

- 雷达避障：`smooth_path_follower` 使用 `/scan` 避障。
- 视觉避障：`visual_path_avoidance` 使用 `/racing_obstacle_detection` 中的锥桶避障。

两种方式的前置系统和 `visual_control` 完全相同，只在第 3 步二选一。

## 1. 编译

修改过主线识别或 `visual_control` 后，编译主工作区：

```bash
cd /root/D_Robot/dev_ws
source /opt/tros/humble/setup.bash
colcon build --symlink-install --packages-select \
  visual_control line_detector perception_detector qr_detector_cpp
```

编译 test 工作区的雷达版和视觉版：

```bash
cd /root/D_Robot/test/dev_ws
source /opt/tros/humble/setup.bash
source /root/D_Robot/dev_ws/install/setup.bash
colcon build --symlink-install --packages-select \
  origincar_system visual_path_avoidance
```

只修改 launch 参数或 YAML 航点时不需要重新编译，重启对应 launch 即可。

## 2. 启动公共前置系统

新终端执行：

```bash
source /opt/tros/humble/setup.bash
source /root/D_Robot/dev_ws/install/setup.bash
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 launch origincar_system test_bringup.launch.py start_foxglove:=true
```

该命令会启动底盘、相机、雷达、机器人模型、EKF、地图、AMCL、线识别、
障碍物/停车位识别、二维码识别和图生文。

视觉避障版也要保持雷达运行，因为 AMCL 定位仍然使用雷达；视觉版只是不用
雷达做追点阶段的避障。

前置系统稳定后确认定位：

```bash
ros2 run tf2_ros tf2_echo map base_link
```

持续有坐标输出表示 TF 定位链路已经建立。如果位置不准，在 RViz/Foxglove 中
重新设置初始位姿。

## 3. 选择追点避障版本（二选一）

先启动选定的追点版本，让它等待二维码，再启动第 4 步的 `visual_control`。

### 方案 A：雷达避障版本

新终端执行：

```bash
source /opt/tros/humble/setup.bash
source /root/D_Robot/dev_ws/install/setup.bash
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 launch origincar_system smooth_path_follower.launch.xml
```

扫码后的控制关系：

```text
smooth_path_follower -> /cmd_vel
/scan -> 雷达避障
```

### 方案 B：视觉避障版本

新终端执行：

```bash
source /opt/tros/humble/setup.bash
source /root/D_Robot/dev_ws/install/setup.bash
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 launch visual_path_avoidance visual_path_follower.launch.xml
```

这个 launch 会同时启动 `smooth_path_follower` 和 `visual_path_avoidance`，不需要
再单独启动 `smooth_path_follower.launch.xml`。

扫码后的控制关系：

```text
smooth_path_follower -> /visual_path_cmd_vel
visual_path_avoidance -> /cmd_vel
/racing_obstacle_detection -> 只处理 construction_cone 障碍物
```

视觉版会自动关闭 `smooth_path_follower` 自身的雷达避障。避障方向优先采用当前
航点的追踪方向；障碍消失后直接恢复追点，不执行回线动作。

> 雷达版和视觉版禁止同时启动，否则会争抢 `/cmd_vel`。

## 4. 启动 visual_control

确认第 2、3 步已经正常启动后，新终端执行下面命令。此命令启动后车辆会开始
巡线：

```bash
source /opt/tros/humble/setup.bash
source /root/D_Robot/dev_ws/install/setup.bash
source /root/D_Robot/test/dev_ws/install/setup.bash
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
  delta_x_ratio:=0.0 debug_control_logs:=false
```

完整状态切换：

```text
第一次 visual_control
  赛道线 + 二维码 + 障碍物，忽略停车位
          |
          | 扫到二维码，/sign4return=5
          v
雷达追点避障 或 视觉追点避障
          |
          | 追点完成，/sign4return=6
          v
第二次 visual_control
  赛道线 + 停车位 + 障碍物，忽略二维码
```

## 5. 检查当前控制者

```bash
ros2 node list
ros2 topic info /cmd_vel -v
```

正常情况下 `/cmd_vel` 发布者应为：

| 阶段 | 雷达版 | 视觉版 |
| --- | --- | --- |
| 扫码前巡线 | `/visual_control` | `/visual_control` |
| 扫码后追点 | `/smooth_path_follower` | `/visual_path_avoidance` |
| 追点完成后 | `/visual_control` | `/visual_control` |

视觉版追点时，`/smooth_path_follower` 发布的是内部话题
`/visual_path_cmd_vel`，所以不会直接出现在 `/cmd_vel` 发布者列表里。

## 6. 切换雷达版和视觉版

目前不支持运行中热切换。切换方式：

1. 先停车。
2. 关闭第 3 步当前避障版本的终端。
3. 关闭第 4 步的 `visual_control`，准备重置阶段状态。
4. 启动第 3 步中的另一个避障版本。
5. 重新启动第 4 步的 `visual_control` 开始下一轮。

第 2 步公共前置系统可以保持运行。

需要手动恢复 `visual_control` 时：

```bash
ros2 topic pub --once /sign4return std_msgs/msg/Int32 "{data: 6}"
```
