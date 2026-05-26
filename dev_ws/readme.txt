export ROS_DOMAIN_ID=1
cd ~/D_Robot/dev_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
rviz2
gazebo
colcon build --packages-select driver_bringup
colcon build --packages-select origincar_nav2
source install/setup.bash

ros2 launch driver_bringup drivers.launch.py
ros2 launch slam slam.launch.py
ros2 launch state_estimation ekf.launch.py


ros2 launch origincar_nav2 origincar_nav2.launch.py
export ROS_DOMAIN_ID=1

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/chassis/cmd_vel



