#!/usr/bin/env bash
# 终端一：启动 lidar_location 总系统
# 对应 Lidar_loc_README.md 第 28-34 行，不使用 tmux，直接前台运行。
set -e

unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /opt/tros/humble/setup.bash
source /root/D_Robot/test/dev_ws/install/setup.bash
fastdds shm clean
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

ros2 launch origincar_system test_bringup_lidar_location.launch.py \
  start_foxglove:=true
