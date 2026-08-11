#!/bin/bash
# ============================================================
# 终端二：四阶段静态+动态追点
# 对应 Lidar_loc_README.md 第 55-60 行命令
# 注意：须在打开物理底盘开关前启动，使绿色静态候选先形成 READY 快照
#
# 默认：在当前终端前台运行（等价于手动输入 README 命令）
# 日志始终输出到 /tmp/term2.log
# ============================================================
set -e

WS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${WS_DIR}/log/local_runtime"

if [ "$#" -ne 0 ]; then
  echo "[start_path_follower] Foreground only; run without --tmux."
  exit 2
fi

run_command() {
  mkdir -p "$LOG_DIR"
  unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
  source /opt/tros/humble/setup.bash
  source "$WS_DIR/install/setup.bash"
  fastdds shm clean
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  cd "$WS_DIR"
  ros2 launch origincar_system visual_lidar_path_follower_fast.launch.xml \
    2>&1 | tee "$LOG_DIR/term2.log"
}

echo "[start_path_follower] Starting in foreground; workspace: $WS_DIR; log: $LOG_DIR/term2.log (Ctrl-C stops)."
run_command
