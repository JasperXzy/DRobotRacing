#!/usr/bin/env bash
# 清理全部竞赛进程（终端一 + 终端二的所有节点）
# 使用 [x] 正则技巧避免 pkill 模式匹配到本脚本自身命令行。
set -e

PATTERNS=(
  "[r]os2 launch origincar_system"
  "[l]idar_driver_node"
  "[c]hassis_driver_node"
  "[s]tatic_transform_publisher"
  "[h]obot_usb_cam"
  "[h]obot_codec_republish"
  "[h]obot_tts"
  "[f]oxglove_bridge"
  "[e]kf_node"
  "[m]ap_server"
  "[l]idar_location_node"
  "[l]ifecycle_manager"
  "[p]erception_detector_node"
  "[q]r_detector_node"
  "[v]lm_tts_demo_node"
  "[v]isual_lidar_path_follower"
  "[r]oute_snapshot_renderer"
)

CHECK_PATTERN='[l]idar_driver_node|[c]hassis_driver_node|[s]tatic_transform_publisher|[h]obot_usb_cam|[h]obot_codec|[h]obot_tts|[f]oxglove_bridge|[e]kf_node|[m]ap_server|[l]idar_location_node|[l]ifecycle_manager|[p]erception_detector_node|[q]r_detector_node|[v]lm_tts_demo|[v]isual_lidar_path_follower|[r]oute_snapshot|[r]os2 launch'

echo "==> 发送 SIGTERM 停止所有竞赛进程 ..."
for pat in "${PATTERNS[@]}"; do
  pkill -TERM -f "$pat" 2>/dev/null || true
done
sleep 3

REMAIN=$(ps aux | grep -cE "$CHECK_PATTERN" || true)
if [ "$REMAIN" -gt 0 ]; then
  echo "==> 仍有 $REMAIN 个进程未退出，发送 SIGKILL 强制终止 ..."
  for pat in "${PATTERNS[@]}"; do
    pkill -9 -f "$pat" 2>/dev/null || true
  done
  sleep 1
fi

echo "==> 清理共享内存 ..."
fastdds shm clean 2>/dev/null || true

REMAIN=$(ps aux | grep -cE "$CHECK_PATTERN" || true)
echo "==> 清理完成，剩余竞赛进程数: $REMAIN"
