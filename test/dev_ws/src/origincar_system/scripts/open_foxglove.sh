#!/usr/bin/env bash
set -u

host="${1:-localhost}"
url="https://app.foxglove.dev/~/view?ds=foxglove-websocket&ds.url=ws%3A%2F%2F${host}%3A8765&openIn=desktop"

echo "Foxglove data source: ws://${host}:8765"
echo "Layout file: $(ros2 pkg prefix origincar_system)/share/origincar_system/foxglove/visual_lidar_path_follower.json"
echo "Open URL: ${url}"

if command -v xdg-open >/dev/null 2>&1; then
  xdg-open "${url}" >/dev/null 2>&1 &
fi
