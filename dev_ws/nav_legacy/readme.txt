export ROS_DOMAIN_ID=1
cd ~/D_Robot/dev_ws/nav_legacy
source /opt/ros/humble/setup.bash
source install/setup.bash
rviz2
gazebo
colcon build --packages-select driver_bringup
colcon build --packages-select origincar_nav2
source install/setup.bash

#以下是全开launch
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch driver_bringup drivers.launch.py
ros2 launch slam slam.launch.py
ros2 launch state_estimation ekf.launch.py

#以下是导航
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch origincar_nav2 origincar_nav2.launch.py
# 1. 配置（此时读取 yaml 文件）
ros2 lifecycle set /map_server configure 
# 2. 激活（开始发布 /map）
ros2 lifecycle set /map_server activate

# 键盘控制节点
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/chassis/cmd_vel

#####################一下是xzt的.launch#####################

ros2 run origincar_nav2 map_overlay_publisher

ros2 launch chassis_driver chassis.launch.py 

# 录制路径
ros2 run origincar_nav2 waypoint_recorder
# 巡航
ros2 run origincar_nav2 waypoint_cruiser
# 叠加图片
ros2 run origincar_nav2 map_overlay_publisher 

