#!/bin/bash
# ============================================================
# 终端一：启动 lidar_location 总系统
# 对应 Lidar_loc_README.md 第 28-34 行命令
#
# 默认：在当前终端前台运行（等价于手动输入 README 命令）
# 日志始终输出到 /tmp/term1.log
# ============================================================
set -e

WS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${WS_DIR}/log/local_runtime"

if [ "$#" -ne 0 ]; then
  echo "[start_lidar_location] Foreground only; run without --tmux."
  exit 2
fi

run_command() {
  mkdir -p "$LOG_DIR"
  unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
  source /opt/tros/humble/setup.bash
  source "$WS_DIR/install/setup.bash"
  fastdds shm clean
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/tros/humble/lib/hobot_shm/config/shm_fastdds.xml
  export RMW_FASTRTPS_USE_QOS_FROM_XML=1
  export ROS_DISABLE_LOANED_MESSAGES=1
  cd "$WS_DIR"
  ros2 launch origincar_system test_bringup_lidar_location.launch.py \
    start_foxglove:=true 2>&1 | tee "$LOG_DIR/term1.log"
}

echo "[start_lidar_location] Starting in foreground; workspace: $WS_DIR; log: $LOG_DIR/term1.log (Ctrl-C stops)."
run_command
