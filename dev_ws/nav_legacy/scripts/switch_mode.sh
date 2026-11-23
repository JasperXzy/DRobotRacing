#!/bin/bash
# 用法: ./switch_mode.sh mapping   # 建图模式
#      ./switch_mode.sh localization  # 定位模式

MODE=${1:-localization}
BASE=~/D_Robot/dev_ws/nav_legacy/config

# 1. slam_toolbox mode
sed -i "s/^    mode: .*/    mode: $MODE/" \
    "$BASE/localization/slam_toolbox.yaml"

# 2. nav2 allow_unknown
if [ "$MODE" = "mapping" ]; then
    sed -i 's/^      allow_unknown: .*/      allow_unknown: true/' \
        "$BASE/nav2/nav2.yaml"
else
    sed -i 's/^      allow_unknown: .*/      allow_unknown: false/' \
        "$BASE/nav2/nav2.yaml"
fi

echo "✅ 已切换为 $MODE 模式"
echo "  slam_toolbox.yaml → mode: $MODE"
[ "$MODE" = "mapping" ] && echo "  nav2.yaml         → allow_unknown: true" \
    || echo "  nav2.yaml         → allow_unknown: false"