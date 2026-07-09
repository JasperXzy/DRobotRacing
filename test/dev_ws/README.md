# test/dev_ws 91 点测试流程

## 1. 编译 test 的 91 点节点

第一次使用、清理过 test 工作区后、或者改了 `smooth_path_follower.cpp` / `CMakeLists.txt` / `package.xml` 之后需要编译：

```bash
source /root/D_Robot/dev_ws/install/setup.bash
cd /root/D_Robot/test/dev_ws
colcon build --symlink-install --packages-select origincar_system
```

之后如果只改 `smooth_path_follower.launch.xml` 里的参数，不需要重新编译，重启第 3 步即可。

## 2. 一键启动前置系统

```bash
source /root/D_Robot/dev_ws/install/setup.bash && ros2 launch /root/D_Robot/test/dev_ws/src/origincar_system/launch/test_bringup.launch.py \
  start_foxglove:=true
```

等待前置系统启动完成后，确认定位正常：

```bash
source /root/D_Robot/dev_ws/install/setup.bash && ros2 run tf2_ros tf2_echo map base_link
```

如果定位不准，在 RViz/Foxglove 里重新给初始位姿。

## 3. 启动 test 的 91 点路径跟随

启动前确认没有其他节点也在发布 `/cmd_vel`：

```bash
source /root/D_Robot/dev_ws/install/setup.bash && ros2 topic info /cmd_vel -v
```

新终端启动：

```bash
source /root/D_Robot/dev_ws/install/setup.bash && source /root/D_Robot/test/dev_ws/install/setup.bash && ros2 launch /root/D_Robot/test/dev_ws/src/origincar_system/launch/smooth_path_follower.launch.xml
```
