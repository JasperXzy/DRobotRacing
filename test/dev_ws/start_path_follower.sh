#!/usr/bin/env bash
# 终端二：启动四阶段静态+动态追点
# 对应 Lidar_loc_README.md 第 55-60 行，不使用 tmux，直接前台运行。
# 支持命令行参数覆盖关键配置（不修改yaml原文件，临时生效）：
#   --variant default|backup_45deg   选择Task1路线（默认沿用yaml配置）
#   --static on|off                  静态避障开关（静态绿色快照+路径偏移）
#   --lidar  on|off                  实时雷达避障开关（扇形/走廊动态避障）
# 示例：
#   ./start_path_follower.sh --variant backup_45deg --static off
#   ./start_path_follower.sh --lidar off --static off   # 纯航点追线
set -e

BASE_PARAMS=/root/D_Robot/test/dev_ws/src/origincar_system/config/visual_lidar_path_follower_fast.yaml
PARAMS_FILE="$BASE_PARAMS"
VARIANT=""
STATIC=""
LIDAR=""

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
    -h|--help)
      echo "用法：$0 [--variant default|backup_45deg] [--static on|off] [--lidar on|off]"
      exit 0 ;;
    *)
      echo "未知参数：$1（可用 --help 查看用法）"; exit 1 ;;
  esac
done

# 任一参数提供时，基于fast.yaml生成临时覆盖文件（退出自动清理）
if [ -n "$VARIANT" ] || [ -n "$STATIC" ] || [ -n "$LIDAR" ]; then
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
  PARAMS_FILE="$TMP_FILE"
  echo "==> 使用临时参数覆盖文件：$TMP_FILE（variant=${VARIANT:-沿用} static=${STATIC:-沿用} lidar=${LIDAR:-沿用}）"
fi

unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /opt/tros/humble/setup.bash
source /root/D_Robot/test/dev_ws/install/setup.bash
fastdds shm clean
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

ros2 launch origincar_system visual_lidar_path_follower_fast.launch.xml params_file:="$PARAMS_FILE"
