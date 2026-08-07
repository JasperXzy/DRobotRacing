#!/usr/bin/env bash
# 竞赛系统一键强力收尾。
#
# 仅处理本工作空间拉起的竞赛节点、辅助脚本和 Fast DDS 临时资源；不会删除
# 路线快照、运行监控报告、地图、配置或 captures 中的比赛资料。
# 用法：/root/D_Robot/test/dev_ws/stop_all.sh
set -uo pipefail

WORKSPACE="/root/D_Robot/test/dev_ws"

# 既覆盖 launch 父进程，也覆盖从 /opt/tros 启动、命令行不含工作空间路径的
# 驱动节点。不要使用笼统的 "ros2" 或 "python"，以免影响其他系统服务。
PATTERNS=(
  "[r]os2 launch origincar_system"
  "[l]idar_driver_node"
  "[c]hassis_driver_node"
  "[c]md_vel_start_gate"
  "[s]tatic_transform_publisher"
  "[r]obot_state_publisher"
  "[h]obot_usb_cam"
  "[h]obot_codec_republish"
  "[h]obot_codec"
  "[h]obot_tts"
  "[f]oxglove_bridge"
  "[e]kf_node"
  "[i]mu_filter_madgwick"
  "[m]ap_server"
  "[l]idar_location_node"
  "[l]ifecycle_manager"
  "[p]erception_detector_node"
  "[q]r_detector_node"
  "[v]lm_tts_demo_node"
  "[v]isual_lidar_path_follower"
  "[r]oute_snapshot_renderer"
  "[c]ompetition_run_monitor.py"
  "[m]anual_qr_pub.py"
)

declare -A OWN_ANCESTORS=()
own_pid=$$
while [[ "$own_pid" =~ ^[0-9]+$ ]] && (( own_pid > 1 )); do
  OWN_ANCESTORS["$own_pid"]=1
  own_pid=$(ps -o ppid= -p "$own_pid" 2>/dev/null | tr -d '[:space:]' || true)
done

collect_matching_pids() {
  local pattern pid
  local -A seen=()
  for pattern in "${PATTERNS[@]}"; do
    while IFS= read -r pid; do
      [[ "$pid" =~ ^[0-9]+$ ]] || continue
      # 不终止本清理脚本、其 ssh/终端祖先进程，也不重复计数。
      [[ -n "${OWN_ANCESTORS[$pid]:-}" ]] && continue
      [[ -n "${seen[$pid]:-}" ]] && continue
      seen["$pid"]=1
      printf '%s\n' "$pid"
    done < <(pgrep -f -- "$pattern" 2>/dev/null || true)
  done
}

read_matching_pids() {
  mapfile -t MATCHING_PIDS < <(collect_matching_pids)
}

print_matching_processes() {
  local pid
  for pid in "${MATCHING_PIDS[@]:-}"; do
    ps -p "$pid" -o pid=,ppid=,stat=,args= 2>/dev/null || true
  done
}

signal_matching_processes() {
  local signal="$1"
  read_matching_pids
  if (( ${#MATCHING_PIDS[@]} == 0 )); then
    echo "==> 未发现残留竞赛进程。"
    return
  fi

  echo "==> 向 ${#MATCHING_PIDS[@]} 个竞赛进程发送 SIG${signal}："
  print_matching_processes
  kill "-$signal" "${MATCHING_PIDS[@]}" 2>/dev/null || true
}

wait_for_exit() {
  local attempts="$1"
  local delay="$2"
  local i
  for ((i = 0; i < attempts; ++i)); do
    read_matching_pids
    (( ${#MATCHING_PIDS[@]} == 0 )) && return 0
    sleep "$delay"
  done
  return 1
}

publish_zero_velocity() {
  # 在节点仍在线时先尝试发布一次零速度；即使 ROS 图尚未就绪也继续清理。
  if command -v ros2 >/dev/null 2>&1 && command -v timeout >/dev/null 2>&1; then
    echo "==> 尝试发布一次 /cmd_vel 零速度 ..."
    timeout 3s ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
      '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
      >/dev/null 2>&1 || true
  fi
}

clean_transport_state() {
  echo "==> 清理 Fast DDS 共享内存与本次启动的临时文件 ..."
  command -v fastdds >/dev/null 2>&1 && \
    timeout 5s fastdds shm clean >/dev/null 2>&1 || true

  # 只匹配 Fast DDS 已知前缀；不会触及其他程序的 /dev/shm 对象。
  rm -f /dev/shm/fastrtps_* /dev/shm/fastdds_* \
    /dev/shm/sem.fastrtps_* /dev/shm/sem.fastdds_* 2>/dev/null || true
  rm -f /tmp/vlpf_*.yaml /tmp/competition_run_monitor_*.log /tmp/manual_qr_pub.log \
    2>/dev/null || true

  # ROS 2 daemon 只缓存本机图信息；停止它能避免下一次启动看到陈旧节点。
  command -v ros2 >/dev/null 2>&1 && timeout 5s ros2 daemon stop >/dev/null 2>&1 || true
}

echo "=============================================================="
echo "竞赛系统强力收尾：$WORKSPACE"
echo "保留比赛资料，只清理节点、辅助进程、DDS 与临时状态。"
echo "=============================================================="

read_matching_pids
if (( ${#MATCHING_PIDS[@]} > 0 )); then
  publish_zero_velocity
else
  echo "==> 当前没有竞赛节点，跳过 /cmd_vel 零速度发布。"
fi
signal_matching_processes TERM

if ! wait_for_exit 30 0.1; then
  echo "==> 3 秒后仍有 ${#MATCHING_PIDS[@]} 个竞赛进程，发送 SIGKILL："
  print_matching_processes
  kill -KILL "${MATCHING_PIDS[@]}" 2>/dev/null || true
  wait_for_exit 10 0.1 || true
fi

clean_transport_state
read_matching_pids

if (( ${#MATCHING_PIDS[@]} == 0 )); then
  echo "==> 清理完成：未发现竞赛残留进程，可重新干净启动。"
else
  echo "==> 清理完成但仍发现 ${#MATCHING_PIDS[@]} 个匹配进程（请检查）："
  print_matching_processes
fi
