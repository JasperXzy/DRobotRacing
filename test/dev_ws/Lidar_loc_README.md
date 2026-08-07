# lidar_location 定位方案：直接启动

本方案使用 `test_bringup_lidar_location.launch.py` 启动 `lidar_location`。
原有的 `test_bringup.launch.py` 保持不变，因此 `lidar_location` 与 AMCL
两套方案可以在同一个工作空间中按需切换。

> 两套总系统只能选择一套启动，不要同时运行。`lidar_location` 和 AMCL
> 都会发布 `map -> odom` TF，同时运行会造成 TF 冲突。

## 日志显示
```bash
source /opt/ros/humble/setup.bash
ros2 run plotjuggler plotjuggler
```

## 1. 编译

以下命令同时编译 `lidar_location`、AMCL 两套方案共用的功能包：

```bash
cd /root/D_Robot/test/dev_ws
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /opt/tros/humble/setup.bash
colcon build --symlink-install --packages-select \
  origincar_msg camera_driver chassis_driver lidar_driver lidar_location \
  perception_detector qr_detector_cpp \
  vlm_tts_demo origincar_system
```

## 2. 终端一：启动 lidar_location 总系统

```bash
tmux new -s drobot
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /opt/tros/humble/setup.bash
source /root/D_Robot/test/dev_ws/install/setup.bash
fastdds shm clean
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 launch origincar_system test_bringup_lidar_location.launch.py \
  start_foxglove:=false
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

终端一不再周期输出 `match_score`、YOLO FPS/Timing 和 Foxglove 连接信息；
只在雷达、YOLO、定位或控制的健康状态发生变化时输出一行。
每次启动的性能记录位于：

```text
/root/D_Robot/test/dev_ws/debug_log/runtime_performance/<启动时间>/
```

- `metrics.csv`：每秒1次的雷达/YOLO/控制 FPS、控制周期 P95/P99/最大值和
  match score 汇总。
- `match_score.csv`：match score 最高5 Hz。
- `cmd_vel.csv`：`/cmd_vel` 最高20 Hz。
- `events.jsonl`：健康状态、四阶段任务/控制状态和路线快照事件。

### SSH 终端直接查看（推荐）

无需图形界面和额外 Python 依赖，在 SSH 中运行：

```bash
source /opt/tros/humble/setup.bash
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 run origincar_system runtime_metrics_viewer.py
```

默认每秒1次刷新最新批次，以表格显示当前值、本次平均值、
本次最小/最大值，以及最新 `/cmd_vel` 和状态变化。退出按 `Ctrl-C`。
只查看一次而不持续刷新：

```bash
ros2 run origincar_system runtime_metrics_viewer.py --once
```

查看指定批次：

```bash
ros2 run origincar_system runtime_metrics_viewer.py \
  --session '2026.08.07_21:30:00'
```

该仪表盘只读取低频 CSV/JSONL，不订阅ROS话题、不发布控制消息。
比赛中需要排查时可短时打开，赛后查看对小车运行没有影响。

### PlotJuggler 赛后详细分析

这些 CSV 是给赛后 PlotJuggler 绘图使用的，不建议直接阅读文本。
车上首次使用前安装（只需执行一次）：

```bash
sudo apt-get install -y ros-humble-plotjuggler-ros
```

车辆停稳后启动 PlotJuggler：

```bash
source /opt/ros/humble/setup.bash
ros2 run plotjuggler plotjuggler
```

在 PlotJuggler 中选择 `File -> Load Data`，打开本次目录的
`metrics.csv`，CSV 分隔符选逗号，时间列选 `elapsed_seconds`。
建议建立三块曲线图：

1. 帧率：`lidar_fps`、`yolo_fps`、`control_fps`。
2. 定位：`match_score_latest`、`match_score_mean`。
3. 控制抖动：`control_period_mean_ms`、`control_period_p95_ms`、
   `control_period_p99_ms`、`control_period_max_ms`。

如需查看速度指令，再加载 `cmd_vel.csv`，绘制 `linear_x`和
`angular_z`。调整好后使用 `File -> Save Layout`保存布局，
下次可直接复用。PlotJuggler 只在赛后运行，不会影响比赛控制。

## 3. 终端二：启动四阶段静态+动态追点

#### 1.50（简单摆法）
```bash
tmux new -s drobot2
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /opt/tros/humble/setup.bash
source /root/D_Robot/test/dev_ws/install/setup.bash
fastdds shm clean
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 launch origincar_system visual_lidar_path_follower_fast.launch.xml
```

Task1 当前 map 控制点为
`src/origincar_system/scripts/generate_task1_waypoints.py` 后可重新生成：
#### Task1任务路径生成脚本
```bash
python3 /root/D_Robot/test/dev_ws/src/origincar_system/scripts/generate_task1_waypoints.py
```

> 必须在打开物理底盘开关前启动本终端，使绿色静态候选先形成READY快照。
> Task1、二维码过渡、Task2和Task3均由该节点控制，不再启动第三个
> `visual_control` 终端。

## 4. 停车后手动绘制路线图

比赛过程中仅在本次时间批次目录中记录路线快照 JSON，不读取地图、
不绘图、不压缩 PNG。车辆停稳后，绘制最新一批路线图：

```bash
source /opt/tros/humble/setup.bash
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 run origincar_system route_snapshot_renderer.py \
  --render-session latest
```

绘制指定时间批次：

```bash
ros2 run origincar_system route_snapshot_renderer.py \
  --render-session '2026.08.07_18:23:13'
```

JSON 和生成的 PNG 都位于：

```text
/root/D_Robot/test/dev_ws/src/origincar_system/path/<本次时间批次>/
```

`latest` 只会选择含有路线快照 JSON 的最新批次。旧批次如果只有 PNG、
没有 JSON，则无法使用该命令重新绘制。

## 5. lidar_location 定位单独测试（不要与总系统同时启动）

```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 launch origincar_system odom_test_lidar_location.launch.py \
  start_foxglove:=true
```

## 6. 杀死雷达
```bash
pkill -INT -f lidar_driver_node
sleep 2
```


## 7. 调整音量
```bash
amixer -c 0 sset PCM 50% unmute
```

#### 终端1
```bash 
source /opt/tros/humble/setup.bash
source /root/D_Robot/test/dev_ws/install/setup.bash

export XDG_RUNTIME_DIR=/run/user/1000
export PULSE_SERVER=unix:/run/user/1000/pulse/native
export PULSE_COOKIE=/home/sunrise/.config/pulse/cookie

ros2 run hobot_tts hobot_tts --ros-args \
  -p topic_sub:=/tts_text \
  -p playback_device:=pulse
```

#### 终端2
```bash 
source /opt/tros/humble/setup.bash
source /root/D_Robot/test/dev_ws/install/setup.bash

ros2 topic pub --once /tts_text std_msgs/msg/String \
  "{data: '测试'}"
```
