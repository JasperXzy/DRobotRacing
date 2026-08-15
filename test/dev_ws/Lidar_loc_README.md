## 1. 编译

以下命令同时编译 `lidar_location`、AMCL 两套方案共用的功能包：

```bash
cd /root/D_Robot/test/dev_ws
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /opt/tros/humble/setup.bash
colcon build --symlink-install --packages-select \
  origincar_msg camera_driver chassis_driver bmi088_driver lidar_driver lidar_location \
  perception_detector qr_detector_cpp \
  dual_rate_tts vlm_tts_demo origincar_system
```

## 2. 终端一：启动 lidar_location 总系统（ BMI088）
```bash
tmux new -s drobot
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /opt/tros/humble/setup.bash
source /root/D_Robot/test/dev_ws/install/setup.bash
fastdds shm clean
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 launch origincar_system test_bringup_lidar_location.launch.py \
  start_foxglove:=false \
  imu_topic:=/bmi088/imu/data
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
ros2 launch origincar_system visual_lidar_path_follower_fast.launch.xml \
  task1_profile:=low \
  task3_profile:=fast
```

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
amixer -c 0 sset PCM 75% unmute
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
