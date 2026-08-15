# AMCL 定位方案：直接启动

本方案使用 `test_bringup.launch.py` 启动 AMCL。原有的
`test_bringup_lidar_location.launch.py` 保持不变，因此 AMCL 与
`lidar_location` 两套方案可以在同一个工作空间中按需切换。

> 两套总系统只能选择一套启动，不要同时运行。AMCL 和
> `lidar_location` 都会发布 `map -> odom` TF，同时运行会造成 TF 冲突。

## 1. 编译

以下命令同时编译 AMCL、`lidar_location` 两套方案共用的功能包；保留
`lidar_location` 在编译列表中，可以继续直接使用 `Lidar_loc_README.md` 中的方案。

```bash
cd /root/D_Robot/test/dev_ws
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /opt/tros/humble/setup.bash
colcon build --symlink-install --packages-select \
  origincar_msg camera_driver chassis_driver lidar_driver lidar_location \
  perception_detector qr_detector_cpp \
  vlm_tts_demo origincar_system
```

## 2. 终端一：启动 AMCL 总系统

```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /opt/tros/humble/setup.bash
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 launch origincar_system test_bringup.launch.py \
  start_foxglove:=false
```

AMCL 默认从
`src/origincar_system/config/nav2/nav2.yaml` 中的 `initial_pose` 启动，目前为
`x=0.0, y=0.0, yaw=0.0`。如果车辆实际起点不同，请在 Foxglove/RViz 中通过
`/initialpose` 发布正确的初始位姿。

可使用以下命令观察 AMCL 位姿输出：

```bash
ros2 topic echo /amcl_pose
```

## 3. 终端二：启动四阶段静态+动态追点

```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /opt/tros/humble/setup.bash
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 launch origincar_system visual_lidar_path_follower.launch.xml \
  task1_profile:=low \
  task3_profile:=slow
```

`task1_profile` 可选 `low` 或 `high`，`task3_profile` 可选
`fast` 或 `slow`，两者彼此独立，共有四种组合。例如 Task1 选
`low`、Task3 选 `fast`：

```bash
ros2 launch origincar_system visual_lidar_path_follower.launch.xml \
  task1_profile:=low \
  task3_profile:=fast
```

顺/逆时针 Task2 通道几何仍共用，`task3_profile` 只切换通道后的 Task3
返程路线。

两个 Task1 和两个 Task3 分别使用独立生成脚本：

```bash
python3 /root/D_Robot/test/dev_ws/src/origincar_system/scripts/generate_task1_low.py
python3 /root/D_Robot/test/dev_ws/src/origincar_system/scripts/generate_task1_high.py
python3 /root/D_Robot/test/dev_ws/src/origincar_system/scripts/generate_task3_fast.py
python3 /root/D_Robot/test/dev_ws/src/origincar_system/scripts/generate_task3_slow.py
```

Task1 脚本只改 `TASK1_CONTROL_POINTS`；Task3 脚本只改
`TASK3_CONTROL_POINTS`。`low` 和 `slow` 是当前路线；`high` 暂时复制
`low` 等待调整；`fast` 从通道出口直接连回发车起点。

必须在打开物理底盘开关前启动本终端，使绿色静态候选先形成READY快照。
Task1、二维码过渡、Task2和Task3均由该节点控制，不再启动第三个
`visual_control` 终端。

## 4. AMCL 定位单独测试（不要与总系统同时启动）

```bash
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 launch origincar_system odom_test.launch.py \
  start_foxglove:=true
```

## 5. 两套定位方案切换入口

| 定位方案 | 总系统 launch | 定位单独测试 launch | 使用说明 |
|---|---|---|---|
| AMCL | `test_bringup.launch.py` | `odom_test.launch.py` | 本文档 |
| `lidar_location` | `test_bringup_lidar_location.launch.py` | `odom_test_lidar_location.launch.py` | `Lidar_loc_README.md` |

切换前先用 `Ctrl+C` 完整停止当前总系统，再启动另一套方案。如果雷达驱动没有正常退出，可执行：

```bash
pkill -INT -f lidar_driver_node
sleep 2
```
