#!/usr/bin/env bash
source /opt/tros/humble/setup.bash
source /root/D_Robot/test/dev_ws/install/setup.bash
set -uo pipefail
export ROS_LOG_DIR=/tmp/ros_log_full_task_20260724

python3 /tmp/full_task_sim.py > /tmp/full_task_terminal1.log 2>&1 &
simulator_pid=$!
ros2 launch origincar_system visual_lidar_path_follower.launch.xml \
  params_file:=/root/D_Robot/test/dev_ws/src/origincar_system/config/visual_lidar_path_follower_fast.yaml \
  > /tmp/full_task_terminal2.log 2>&1 &
follower_pid=$!

cleanup() {
  kill -INT "${follower_pid}" "${simulator_pid}" 2>/dev/null || true
  wait "${follower_pid}" 2>/dev/null || true
  wait "${simulator_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

deadline=$((SECONDS + 120))
while kill -0 "${follower_pid}" 2>/dev/null; do
  if grep -q "SNAPSHOT_FINAL_DYNAMIC" /tmp/full_task_terminal1.log; then
    echo "SIMULATION_FINISHED"
    exit 0
  fi
  if ((SECONDS >= deadline)); then
    echo "SIMULATION_TIMEOUT"
    exit 124
  fi
  sleep 0.5
done

sleep 1
echo "SIMULATION_FINISHED"
