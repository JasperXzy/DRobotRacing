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
  start_foxglove:=true
```

`lidar_location` 默认从
`src/origincar_system/config/localization/lidar_location.yaml` 中的
`initial_pose` 启动，目前为 `x=0.0, y=0.0, yaw=0.7853981634 rad（45°）`。如果车辆实际起点不同，
请在 Foxglove/RViz 中通过 `/initialpose` 发布正确的初始位姿。

`/amcl_pose` 是为兼容原有下游节点而保留的位姿话题名称，此方案不会启动 AMCL。
可使用以下命令观察定位结果和扫描匹配分数：

```bash
ros2 topic echo /amcl_pose
ros2 topic echo /lidar_location/match_score
```

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
`(0,0) → (0.60,0.60) → (3.30,0.65) → (4.10,3.60)`。修改
`src/origincar_system/scripts/generate_task1_waypoints.py` 后可重新生成：
#### Task1任务路径生成脚本
```bash
python3 /root/D_Robot/test/dev_ws/src/origincar_system/scripts/generate_task1_waypoints.py
```

> 必须在打开物理底盘开关前启动本终端，使绿色静态候选先形成READY快照。
> Task1、二维码过渡、Task2和Task3均由该节点控制，不再启动第三个
> `visual_control` 终端。

## 4. lidar_location 定位单独测试（不要与总系统同时启动）

```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 launch origincar_system odom_test_lidar_location.launch.py \
  start_foxglove:=true
```

## 5. 杀死雷达
```bash
pkill -INT -f lidar_driver_node
sleep 2
```


## 6. 调整音量
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