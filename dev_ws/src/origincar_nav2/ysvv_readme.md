# YSVV 使用说明

## 编译

```bash
cd /root/D_Robot/dev_ws
colcon build --packages-select origincar_nav2 --symlink-install --allow-overriding origincar_nav2
source /root/D_Robot/dev_ws/install/setup.bash
export ROS_DOMAIN_ID=1
```

## 录制路径

### 方案 A：teleop 键盘控制

终端 1：启动底盘、雷达、EKF、map_server、AMCL。

```bash
source /root/D_Robot/dev_ws/install/setup.bash
export ROS_DOMAIN_ID=1
ros2 launch origincar_nav2 record_ysvv_waypoints.launch.py
```

终端 2：启动键盘控制节点。

```bash
source /root/D_Robot/dev_ws/install/setup.bash
export ROS_DOMAIN_ID=1
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel
```

终端 3：只录制路径，不发布 `/cmd_vel`。

```bash
source /root/D_Robot/dev_ws/install/setup.bash
export ROS_DOMAIN_ID=1
ros2 run origincar_nav2 record_ysvv_waypoints --ros-args -p record_only:=true
```

录制节点可用键：

```text
p       保存
z       撤销最后一个点
Ctrl-C  保存并退出
```

不用自己的键盘节点时，也可以直接运行 `ros2 run origincar_nav2 record_ysvv_waypoints`，它会自带键盘控制并录制。

### 方案 B：jie_deamon 控制

终端 1：启动 `jie_deamon`，它会启动底盘、雷达，并发布 `/cmd_vel` 控车。

```bash
source /root/D_Robot/dev_ws/install/setup.bash
export ROS_DOMAIN_ID=1
ros2 launch jie_deamon start.launch.py
```

终端 2：只启动定位链，不重复启动底盘和雷达。

```bash
source /root/D_Robot/dev_ws/install/setup.bash
export ROS_DOMAIN_ID=1
ros2 launch origincar_nav2 record_ysvv_waypoints.launch.py start_chassis_driver:=false start_lidar_driver:=false
```

终端 3：只录制路径，不发布 `/cmd_vel`。

```bash
source /root/D_Robot/dev_ws/install/setup.bash
export ROS_DOMAIN_ID=1
ros2 run origincar_nav2 record_ysvv_waypoints --ros-args -p record_only:=true
```

方案 B 不要再开 `teleop_twist_keyboard`，也不要运行不带 `record_only:=true` 的录制节点。

路径保存到：

```text
/root/D_Robot/dev_ws/src/origincar_nav2/origincar_nav2/ysvv_waypoints.yaml
```

## 导航路径

先 Ctrl-C 停掉录制节点和键盘控制节点，避免继续占用 `/cmd_vel`。
把小车放回录制起点附近，再启动导航。

```bash
source /root/D_Robot/dev_ws/install/setup.bash
export ROS_DOMAIN_ID=1
ros2 launch origincar_nav2 nav2_ysvv_waypoints.launch.py
```

该 launch 会启动：底盘、雷达、EKF、map_server、AMCL、Nav2，并自动发送 `ysvv_waypoints.yaml`。
闭环路线会默认按 60 个点一批发送，避免终点贴近起点时提前判定完成。

默认读取：

```text
/root/D_Robot/dev_ws/config/nav2/nav2(6.13).yaml
/root/D_Robot/dev_ws/src/origincar_nav2/origincar_nav2/ysvv_waypoints.yaml
```

## 快速检查

确认底盘订阅 `/cmd_vel`：

```bash
ros2 topic info /cmd_vel -v
```

正常应看到：

```text
Subscription count: 1
Node name: chassis_driver_node
```

如果提示 `waiting for TF map->base_link`：

```text
小车仍可被键盘控制，但不会记录地图坐标；检查雷达、AMCL、map_server 是否正常。
```

如果小车不动：

```text
1. 确认底盘 launch 已启动
2. 确认看到 chassis_driver serial port opened
3. 确认 /cmd_vel 有 chassis_driver_node 订阅者
4. 确认两个终端 ROS_DOMAIN_ID 都是 1
```
