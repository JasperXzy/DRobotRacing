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

prepare_environment() {
  mkdir -p "$LOG_DIR"
  unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
  source /opt/tros/humble/setup.bash
  source "$WS_DIR/install/setup.bash"
  fastdds shm clean
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  cd "$WS_DIR"
}

send_manual_route_freeze() {
  local value="$1"
  local action="$2"
  echo
  echo "==> 请求${action}…"
  if ! timeout 5s ros2 service call \
    /visual_lidar_path_follower/manual_route_freeze std_srvs/srv/SetBool \
    "{data: ${value}}"; then
    echo "==> ${action}请求失败：主控服务尚未就绪或未响应。"
  fi
}

keyboard_route_freeze_loop() {
  echo
  echo "=============================================================="
  echo "静态路线人工冻结：看到 READY=1 且静态障碍稳定时按 [1] 冻结；按 [2] 解冻。"
  echo "若按1时 READY=0，主控会等待下一次完整 READY=1 快照后自动冻结。"
  echo "发车后主控将锁定路线并拒绝按2解冻，避免行驶中误操作。"
  echo "=============================================================="
  while kill -0 "$LAUNCH_PID" 2>/dev/null; do
    local key=""
    if ! IFS= read -r -s -n 1 key < /dev/tty; then
      break
    fi
    case "$key" in
      1) send_manual_route_freeze true "人工冻结静态路线" ;;
      2) send_manual_route_freeze false "人工解冻静态路线" ;;
      *) ;;
    esac
  done
}

cleanup_path_follower() {
  local status=$?
  trap - EXIT INT TERM
  if [ -n "${KEY_LISTENER_PID:-}" ] && kill -0 "$KEY_LISTENER_PID" 2>/dev/null; then
    kill "$KEY_LISTENER_PID" 2>/dev/null || true
    wait "$KEY_LISTENER_PID" 2>/dev/null || true
  fi
  if [ -n "${LAUNCH_PID:-}" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
  return "$status"
}

prepare_environment
echo "[start_path_follower] Starting in foreground; workspace: $WS_DIR; log: $LOG_DIR/term2.log (Ctrl-C stops)."
ros2 launch origincar_system visual_lidar_path_follower_fast.launch.xml \
  > >(tee "$LOG_DIR/term2.log") 2>&1 &
LAUNCH_PID=$!
KEY_LISTENER_PID=""
trap cleanup_path_follower EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

keyboard_route_freeze_loop &
KEY_LISTENER_PID=$!

if wait "$LAUNCH_PID"; then
  LAUNCH_STATUS=0
else
  LAUNCH_STATUS=$?
fi
exit "$LAUNCH_STATUS"
