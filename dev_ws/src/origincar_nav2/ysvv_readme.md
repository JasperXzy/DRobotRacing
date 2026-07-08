# YSVV 路径录制与导航使用说明

本文档说明新增的 YSVV 路径录制和路径点导航功能怎么使用。

相关文件：

- 录制脚本：`origincar_nav2/record_ysvv_waypoints.py`
- 路径点文件：`origincar_nav2/ysvv_waypoints.yaml`
- 导航节点：`origincar_nav2/ysvv_waypoint_navigator.py`
- 导航启动文件：`launch/nav2_ysvv_waypoints.launch.py`

## 1. 编译并加载环境

每次修改脚本或 launch 文件后，先编译 `origincar_nav2` 包：

```bash
cd /root/D_Robot/dev_ws
colcon build --packages-select origincar_nav2 --symlink-install --allow-overriding origincar_nav2
source /root/D_Robot/dev_ws/install/setup.bash
```

如果只是重新录制 `ysvv_waypoints.yaml`，通常不需要重新编译。

## 2. 路径录制

录制脚本启动后，会用键盘发布 `/cmd_vel` 控制小车运动，同时读取 TF 里的
`map -> base_link`，自动记录地图坐标 `x`、`y` 和朝向 `yaw`。

启动前请确认：

- 底盘能响应 `/cmd_vel`
- 定位已经正常工作
- TF 中能查到 `map -> base_link`

启动录制：

```bash
source /root/D_Robot/dev_ws/install/setup.bash
ros2 run origincar_nav2 record_ysvv_waypoints
```

键盘控制：

```text
w       前进
s       后退
a       前进左转
d       前进右转
q       慢速左转
e       慢速右转
空格/x  停车
p       立即保存
z       撤销最后一个点
Ctrl-C  保存并退出
```

默认保存位置：

```text
/root/D_Robot/dev_ws/src/origincar_nav2/origincar_nav2/ysvv_waypoints.yaml
```

保存格式示例：

```yaml
waypoints:
- [1.900, 1.900, 1.571]
- [1.900, 2.400, 1.571]
- [2.400, 2.910, 0.000]
```

如果终端一直提示等待 TF，说明还没有 `map -> base_link`，需要先启动定位/地图相关节点。

## 3. 路径点导航

导航启动文件会读取：

```text
/root/D_Robot/dev_ws/config/nav2/nav2(6.13).yaml
```

并按顺序读取：

```text
/root/D_Robot/dev_ws/src/origincar_nav2/origincar_nav2/ysvv_waypoints.yaml
```

启动导航：

```bash
source /root/D_Robot/dev_ws/install/setup.bash
ros2 launch origincar_nav2 nav2_ysvv_waypoints.launch.py
```

默认流程：

```text
t=4s   启动 EKF
t=6s   启动 map_server
t=8s   启动 AMCL 和 lifecycle manager
t=12s  启动 Nav2
t=24s  发送 ysvv_waypoints.yaml 中的全部路径点
```

路径点发送使用 Nav2 的 `NavigateThroughPoses`，所以会一次性把 YAML 中的点按顺序发送出去。

## 4. 常用参数

录制时指定保存文件：

```bash
ros2 run origincar_nav2 record_ysvv_waypoints --ros-args \
  -p output_file:=/root/D_Robot/dev_ws/src/origincar_nav2/origincar_nav2/ysvv_waypoints.yaml
```

录制时调整点位密度：

```bash
ros2 run origincar_nav2 record_ysvv_waypoints --ros-args \
  -p min_distance:=0.05 \
  -p min_yaw_delta:=0.10
```

导航时指定路径点文件：

```bash
ros2 launch origincar_nav2 nav2_ysvv_waypoints.launch.py \
  waypoints_file:=/root/D_Robot/dev_ws/src/origincar_nav2/origincar_nav2/ysvv_waypoints.yaml
```

导航时延后发送路径点：

```bash
ros2 launch origincar_nav2 nav2_ysvv_waypoints.launch.py navigator_delay:=30.0
```

不自动发布初始位姿：

```bash
ros2 launch origincar_nav2 nav2_ysvv_waypoints.launch.py publish_initial_pose:=false
```

## 5. 推荐使用顺序

1. 启动底盘、雷达、定位等基础节点，让 `/cmd_vel` 和 TF 都正常。
2. 运行 `record_ysvv_waypoints`，用键盘开车走一遍路线。
3. 按 `p` 或 `Ctrl-C` 保存 `ysvv_waypoints.yaml`。
4. 检查 YAML 中是否有有效的 `[x, y, yaw]` 点。
5. 运行 `nav2_ysvv_waypoints.launch.py`，让 Nav2 按顺序巡航。

## 6. 快速排查

如果录制没有点：

- 看终端是否提示等待 `map -> base_link`
- 确认 AMCL/定位节点已经启动
- 确认机器人在地图坐标系下有位姿

如果小车不动：

- 确认底盘驱动正在运行
- 确认 `/cmd_vel` 没有被其他节点覆盖
- 录制时先按空格停车，再按 `w/a/d` 重新发速度

如果导航不开始：

- 确认 `ysvv_waypoints.yaml` 不是 `waypoints: []`
- 确认 `navigate_through_poses` action server 已启动
- 确认 `nav2(6.13).yaml` 中的 BT XML 路径能被 launch 动态修正

如果路径点太密：

- 录制时调大 `min_distance`
- 例如：`-p min_distance:=0.12`

如果路径点太稀：

- 录制时调小 `min_distance`
- 例如：`-p min_distance:=0.05`
