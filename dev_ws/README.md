# 比赛工作空间

`dev_ws/src` 是基于 `d41f8ea` 的比赛工作空间。原版在 [`test/dev_ws`](../test/dev_ws)，可以独立编译和启动，用于 A/B 对照和回退。

## 主控结构

| 模块 | 职责 |
| --- | --- |
| `visual_lidar_path_follower.cpp`、`control_loop.cpp` | 装配 ROS 接口并按原版顺序执行控制周期。 |
| `race_mission.cpp`、`channel_transition.cpp`、`control_output.cpp` | 管理任务阶段、扫码进场、速度和制动。 |
| `route_tracking.cpp`、`follower_localization.cpp` | 计算跟踪指令，处理 TF、轮速里程计和定位诊断。 |
| `static_route_planning.cpp`、`route_shaping.cpp` | 管理原始路线、候选路线、冻结路线和已确认的航点偏移。 |
| `circle_envelope.cpp`、`circle_envelope_candidate.cpp`、`waypoint_shift_confirmation.cpp` | 提供独立于 ROS 的静态绕障、候选曲线和偏移确认计算。 |
| `lidar_avoidance.cpp`、`obstacle_tracking.cpp`、`route_diagnostics.cpp` | 处理实时雷达、障碍状态与路线快照。 |

AMCL、激光定位和各自的里程计调试入口共用 `origincar_system/bringup.py`。控制节点使用原版的单线程执行顺序，静态重规划在调用时同步完成。

编译 `origincar_system` 后，可以从本工作空间运行核心单测。

```bash
ctest --test-dir build/origincar_system -R '^(circle_envelope|waypoint_shift_confirmation)$' --output-on-failure
```

## S100 编译与启动

板端模型和本地 VLM 配置由原版工作空间提供。先在仓库根目录执行下列命令，随后按比赛实际使用的定位后端编译和启动。

```bash
python3 dev_ws/scripts/stage_runtime_assets.py
```

脚本从相邻的 `test/dev_ws` 复制比赛感知模型，以及原版存在的巡线模型和 `vlm_api.env`。若目标文件已经存在，脚本校验内容一致。原版以环境变量提供 VLM 配置时，迁移版沿用同一环境配置。VLM 配置在编译前准备，供包的安装步骤读取。

- [激光定位与 BMI088 比赛入口](Lidar_loc_README.md)
- [AMCL 比赛入口](Amcl_loc_README.md)
- [手动驾驶与定位调试](手动操作.md)

跟随器、二维码和 VLM 共用 `/root/D_Robot/dev_ws/src/origincar_system/path`。该目录保存同一发车批次的路线图、截图和播报缓存。

## A/B 完赛验收

1. 用原版工作空间在 S100 跑出基线。记录 TROS 版本、编译选项、固件、起点、障碍物布置、二维码方向和路线配置。
2. 每轮采集底盘里程计、雷达、TF、定位诊断、二维码、阶段触发和速度。把 bag 与路线快照、实际启动命令及参数放在同一批次。

   ```bash
   ros2 bag record -o race_baseline \
     /scan /chassis/odom /odometry/filtered /tf /tf_static /initialpose \
     /map /restriction_map /logic3_static_restriction_map /logic3_runtime_restriction_map \
     /lidar_location/match_score /lidar_location/match_diagnostics \
     /qr_code /racing_obstacle_detection /sign4return /tts_text \
     /cmd_vel /visual_lidar_path_follower/route_snapshot
   ```

3. 按同一种定位、相同参数和布置启动 `dev_ws` 版。核对发车、选路、条件倒车、拍照、返程和终点制动。对照轨迹、最小净距、停车位置、完赛时间和指令间隔，以原版重复跑的波动范围作为验收依据。
4. 顺逆时针和对应赛况通过后，从 `dev_ws/install/setup.bash` 切换正式入口。回退时用新的终端加载原版 `test/dev_ws/install/setup.bash`。

## Nav2 和手动驾驶

旧的工作空间位于 [`nav_legacy`](nav_legacy/readme.md)，保有自己的包、配置和安装目录。从该目录执行 `colcon build --base-paths src`，并在专用终端加载它的 `install/setup.bash`。外层比赛工作空间通过 `COLCON_IGNORE` 隔离这套包。
