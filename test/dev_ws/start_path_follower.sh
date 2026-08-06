#!/usr/bin/env bash
# 终端二：启动四阶段静态+动态追点
# 对应 Lidar_loc_README.md 第 55-60 行，不使用 tmux，直接前台运行。
# 支持命令行参数覆盖关键配置（不修改yaml原文件，临时生效）：
#   --variant default|backup_45deg   选择Task1路线（默认沿用yaml配置）
#   --static on|off                  静态避障开关（静态绿色快照+路径偏移）
#   --lidar  on|off                  实时雷达避障开关（扇形/走廊动态避障）
#   --zyr                            使用ZYR自定义路线（waypoints_task1_ZYR.yaml）
#   --qr <数字>                      手动注入二维码，跳过扫码点选路并播报
# 示例：
#   ./start_path_follower.sh --variant backup_45deg --static off
#   ./start_path_follower.sh --zyr --qr 4399   # ZYR路线+手动二维码
#   ./start_path_follower.sh --lidar off --static off   # 纯航点追线
# 运行后可直接在本终端按键：
#   1：用当前最后一份READY静态雷达快照手动冻结路线
#   2：仅在尚未检测到发车运动时，解除人工冻结并恢复静态探测
set -e

BASE_PARAMS=/root/D_Robot/test/dev_ws/src/origincar_system/config/visual_lidar_path_follower_fast.yaml
PARAMS_FILE="$BASE_PARAMS"
VARIANT=""
STATIC=""
LIDAR=""
ZYR=""
QR_NUMBER=""

while [ $# -gt 0 ]; do
  case "$1" in
    --variant)
      [ $# -ge 2 ] || { echo "错误：--variant 需要一个参数（default|backup_45deg）"; exit 1; }
      VARIANT="$2"; shift 2 ;;
    --static)
      [ $# -ge 2 ] || { echo "错误：--static 需要一个参数（on|off）"; exit 1; }
      STATIC="$2"; shift 2 ;;
    --lidar)
      [ $# -ge 2 ] || { echo "错误：--lidar 需要一个参数（on|off）"; exit 1; }
      LIDAR="$2"; shift 2 ;;
    --zyr)
      ZYR="true"; shift ;;
    --qr)
      [ $# -ge 2 ] || { echo "错误：--qr 需要一个数字参数"; exit 1; }
      QR_NUMBER="$2"; shift 2 ;;
    -h|--help)
      echo "用法：$0 [--variant default|backup_45deg] [--static on|off] [--lidar on|off] [--zyr] [--qr 数字]"
      exit 0 ;;
    *)
      echo "未知参数：$1（可用 --help 查看用法）"; exit 1 ;;
  esac
done

# --qr 输入校验：仅接受纯数字（与主控 std::stoi 解析一致）
if [ -n "$QR_NUMBER" ]; then
  case "$QR_NUMBER" in
    ''|*[!0-9]*) echo "错误：--qr 仅接受纯数字，收到“$QR_NUMBER”"; exit 1 ;;
  esac
  QR_DIRECTION=$([ $((10#$QR_NUMBER % 2)) -eq 1 ] && echo "顺时针(奇数)" || echo "逆时针(偶数)")
  echo "==> 手动二维码模式：数字 $QR_NUMBER（$QR_DIRECTION），等待主控订阅后自动注入 /qr_code"
fi
if [ -n "$ZYR" ]; then
  echo "==> ZYR路线模式：Task1将使用 waypoints_task1_ZYR.yaml"
fi

# 任一参数提供时，基于fast.yaml生成临时覆盖文件（退出自动清理）
if [ -n "$VARIANT" ] || [ -n "$STATIC" ] || [ -n "$LIDAR" ] || [ -n "$ZYR" ]; then
  # 输入校验：variant 只允许 default/backup_45deg；开关统一转成 yaml 布尔值
  if [ -n "$VARIANT" ]; then
    case "$VARIANT" in
      default|original) VARIANT="default" ;;
      backup|backup_45deg) VARIANT="backup_45deg" ;;
      *) echo "错误：--variant 仅支持 default|backup_45deg，收到“$VARIANT”"; exit 1 ;;
    esac
  fi
  for name in STATIC LIDAR; do
    name_lower=$(echo "$name" | tr 'A-Z' 'a-z')
    value=$(eval echo \$$name)
    if [ -n "$value" ]; then
      case "$value" in
        on|ON|true|True) eval "$name=true" ;;
        off|OFF|false|False) eval "$name=false" ;;
        *) echo "错误：--$name_lower 仅支持 on|off，收到“$value”"; exit 1 ;;
      esac
    fi
  done
  TMP_FILE=$(mktemp /tmp/vlpf_XXXXXX.yaml)
  trap 'rm -f "$TMP_FILE"' EXIT
  cp "$BASE_PARAMS" "$TMP_FILE"
  if [ -n "$VARIANT" ]; then
    sed -i "s/^\([[:space:]]*task1_route_variant:\).*/\1 $VARIANT/" "$TMP_FILE"
  fi
  if [ -n "$STATIC" ]; then
    sed -i "s/^\([[:space:]]*enable_static_avoidance:\).*/\1 $STATIC/" "$TMP_FILE"
  fi
  if [ -n "$LIDAR" ]; then
    sed -i "s/^\([[:space:]]*enable_lidar_avoidance:\).*/\1 $LIDAR/" "$TMP_FILE"
  fi
  if [ -n "$ZYR" ]; then
    sed -i "s/^\([[:space:]]*task1_route_variant:\).*/\1 backup_45deg/" "$TMP_FILE"
    sed -i "s#^\([[:space:]]*task1_waypoints_backup_file:\).*#\1 /root/D_Robot/test/dev_ws/src/origincar_system/config/waypoints_task1_ZYR.yaml#" "$TMP_FILE"
  fi
  PARAMS_FILE="$TMP_FILE"
  echo "==> 使用临时参数覆盖文件：$TMP_FILE（variant=${VARIANT:-沿用} static=${STATIC:-沿用} lidar=${LIDAR:-沿用} zyr=${ZYR:-否}）"
fi

unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /opt/tros/humble/setup.bash
source /root/D_Robot/test/dev_ws/install/setup.bash
fastdds shm clean
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# 手动二维码模式：后台启动注入工具（等待主控订阅 /qr_code 后自动发布并退出）
if [ -n "$QR_NUMBER" ]; then
  nohup python3 /root/D_Robot/test/dev_ws/src/origincar_system/scripts/manual_qr_pub.py "$QR_NUMBER" \
    > /tmp/manual_qr_pub.log 2>&1 &
fi

# 让 launch 在后台保持输出，前台脚本则可以持续读取本终端的单键输入。
# 不通过修改运行时参数实现冻结，而是调用主控服务，确保按键与自动发车
# 使用同一份 READY 静态快照和同一套路线重建逻辑。
ros2 launch origincar_system visual_lidar_path_follower_fast.launch.xml params_file:="$PARAMS_FILE" &
LAUNCH_PID=$!
KEY_LISTENER_PID=""

cleanup_path_follower() {
  local status=$?
  if [ -n "${KEY_LISTENER_PID:-}" ] && kill -0 "$KEY_LISTENER_PID" 2>/dev/null; then
    kill "$KEY_LISTENER_PID" 2>/dev/null || true
    wait "$KEY_LISTENER_PID" 2>/dev/null || true
  fi
  if [ -n "${LAUNCH_PID:-}" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
  [ -z "${TMP_FILE:-}" ] || rm -f "$TMP_FILE"
  return "$status"
}
trap cleanup_path_follower EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

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
  echo "静态路线人工冻结：看到READY静态障碍输出稳定时按 [1] 冻结；按 [2] 解冻。"
  echo "人工冻结后直接开启电机即可按该路线发车；未按1则保留发车自动冻结。"
  echo "一旦检测到小车运动，系统会拒绝解冻，防止行驶中误操作。"
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

keyboard_route_freeze_loop &
KEY_LISTENER_PID=$!

if wait "$LAUNCH_PID"; then
  LAUNCH_STATUS=0
else
  LAUNCH_STATUS=$?
fi
exit "$LAUNCH_STATUS"
