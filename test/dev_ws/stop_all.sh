#!/bin/bash
# ============================================================
# 停止并清理竞赛系统所有进程（终端一 + 终端二）
# 策略：SIGTERM 优雅停止 -> 等待 -> SIGKILL 兜底
#       正则 [x] 写法防止 pkill 匹配到本脚本自身命令行
#       最后清理 FastDDS 共享内存与 tmux 会话
# ============================================================

# 需要清理的进程模式（[x] 防自匹配）
PATTERNS=(
  "ros2 [l]aunch"
  "test_bringup_lidar_locatio[n]"
  "visual_lidar_path_followe[r]"
  "lidar_driver_nod[e]"
  "lidar_location_nod[e]"
  "chassis_driver_nod[e]"
  "hobot_usb_ca[m]"
  "hobot_code[c]"
  "hobot_tt[s]"
  "static_transform_publishe[r]"
  "perception_detector_nod[e]"
  "qr_detector_nod[e]"
  "vlm_tts_demo_nod[e]"
  "ekf_nod[e]"
  "map_serve[r]"
  "lifecycle_manage[r]"
  "foxglove_bridg[e]"
)

# 第一轮：SIGTERM
for p in "${PATTERNS[@]}"; do
  pkill -TERM -f "$p" 2>/dev/null
done

# 等待进程退出（最多 8 秒）
for i in $(seq 1 8); do
  REMAIN=0
  for p in "${PATTERNS[@]}"; do
    N=$(pgrep -f "$p" | wc -l)
    REMAIN=$((REMAIN + N))
  done
  if [ "$REMAIN" -eq 0 ]; then
    break
  fi
  sleep 1
done

# 第二轮：SIGKILL 兜底
for p in "${PATTERNS[@]}"; do
  pkill -KILL -f "$p" 2>/dev/null
done
sleep 1

# 清理 tmux 会话
tmux kill-session -t drobot 2>/dev/null
tmux kill-session -t drobot2 2>/dev/null

# 清理 FastDDS 共享内存
if command -v fastdds >/dev/null 2>&1; then
  fastdds shm clean 2>/dev/null
fi

# 验证清理结果
REMAIN=0
for p in "${PATTERNS[@]}"; do
  N=$(pgrep -f "$p" | wc -l)
  REMAIN=$((REMAIN + N))
done
echo "[stop_all] 清理完成，残留进程数: $REMAIN"
[ "$REMAIN" -eq 0 ]
