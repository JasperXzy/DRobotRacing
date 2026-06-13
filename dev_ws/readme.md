## source
```bash
export ROS_DOMAIN_ID=1
export ROS_DOMAIN_ID=1 && cd ~/D_Robot/dev_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
```
## 编译
```bash
colcon build --packages-select driver_bringup   
colcon build --packages-select origincar_nav2
source install/setup.bash
```
# B站阿杰的跟随launch
```bash
ros2 launch jie_deamon start.launch.py
```
Web界面: http://192.168.10.143:8080
```bash
# 终端1：一键启动全部
ros2 launch driver_bringup competition_bringup.launch.py
# 等待约10秒，所有节点分阶段就绪

# 终端2：发令！
ros2 run origincar_nav2 trigger_cruise
```
## 调参
```bash
ros2 run rqt_reconfigure rqt_reconfigure

```
# 以下是全开launch
```bash
ros2 launch driver_bringup drivers.launch.py
ros2 launch slam slam.launch.py
ros2 launch state_estimation ekf.launch.py
ros2 launch origincar_nav2 origincar_nav2.launch.py
ros2 run origincar_nav2 waypoint_cruiser

```
## 激活/map
```bash
ros2 lifecycle set /map_server configure 
ros2 lifecycle set /map_server activate
```

## 键盘控制节点
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel
```

## 建图
```bash
./scripts/switch_mode.sh mapping   # 建图模式
ros2 launch driver_bringup drivers.launch.py
ros2 launch state_estimation ekf.launch.py
ros2 launch slam slam.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel #键盘控制

mkdir -p ~/D_Robot/dev_ws/src/origincar_nav2/map/serialized

## 序列化地图
 ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/root/D_Robot/dev_ws/src/origincar_nav2/map/serialized/my_map'}"

## 栅格地图
ros2 run nav2_map_server map_saver_cli -f ~/D_Robot/dev_ws/src/origincar_nav2/map/my_map  



./scripts/switch_mode.sh localization  # 定位模式
```

## 定位模式运行
定位模式下需要启动 Nav2 进行导航：
```bash
# 终端 1: 驱动
ros2 launch driver_bringup drivers.launch.py

# 终端 2: EKF
ros2 launch state_estimation ekf.launch.py

# 终端 3: Nav2（内部已集成 SLAM Toolbox 定位 + map_server + 规划器 + 控制器）
ros2 launch origincar_nav2 origincar_nav2.launch.py

```
在 RViz 中使用 "2D Pose Estimate" 工具设置初始位姿，SLAM Toolbox 即开始 scan-matching 定位。




## 录制路径
```bash
ros2 run origincar_nav2 waypoint_recorder
```
## 巡航
```bash
ros2 run origincar_nav2 waypoint_cruiser
```
## 叠加图片
```bash
ros2 run origincar_nav2 map_overlay_publisher 

```
# 默认: 绿色 + 白色 → 禁行区
python png_to_pgm.py

# 所有彩色都是禁行区 (旧行为)
python png_to_pgm.py --color-filter all

# 仅绿色
python png_to_pgm.py --color-filter green

# 仅白色
python png_to_pgm.py --color-filter white