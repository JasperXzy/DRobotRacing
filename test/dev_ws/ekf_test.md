# EKF 单独验证方案

## 1. 验证目标

本测试只启动底盘驱动、EKF 和键盘控制，不启动 AMCL、`lidar_location`、地图服务器或其他定位节点，用于独立验证：

1. IMU 实际发布频率是否符合姿态解算中固定 `20 Hz` 的假设。
2. IMU 陀螺仪 Z 轴、编码器角速度和 EKF 航向的比例及方向是否正确。
3. EKF 的直线误差、静止漂移和一圈闭环误差。

键盘控制不是位置真值。测试时必须使用地面标线、卷尺和车头方向标记作为外部参考。

## 2. 测试前准备

- 将车辆放在平整地面。
- 标记车辆中心的起始位置和车头方向。
- 准备卷尺或已知长度的直线标线。
- 测试开始前保持车辆静止 10 秒。
- 确认没有启动 AMCL、`lidar_location` 或 SLAM：

```bash
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 node list | rg 'amcl|lidar_location|slam_toolbox'
```

没有输出表示定位节点未运行。

## 3. 启动系统

### 终端 1：启动底盘驱动

```bash
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 launch chassis_driver chassis.launch.py \
  chassis_config_file:=/root/D_Robot/test/dev_ws/src/origincar_system/config/drivers/chassis_driver.yaml
```

### 终端 2：启动 EKF

```bash
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 launch origincar_system ekf_test.launch.py \
  ekf_config_file:=/root/D_Robot/test/dev_ws/src/origincar_system/config/localization/ekf.yaml
```

### 终端 3：启动键盘控制

```bash
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel
```

### 终端 4：记录测试数据

每次测试使用不同的输出目录名，避免覆盖已有 rosbag：

```bash
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 bag record -o /tmp/ekf_only_test \
  /cmd_vel \
  /chassis/odom \
  /chassis/imu/data_raw \
  /odometry/filtered \
  /tf \
  /tf_static
```

推荐分别记录为 `/tmp/ekf_static`、`/tmp/ekf_yaw`、`/tmp/ekf_straight` 和 `/tmp/ekf_loop`。

## 4. 测试一：IMU 频率和静止漂移

车辆保持静止 60 至 120 秒，分别检查话题频率：

```bash
ros2 topic hz /chassis/imu/data_raw
ros2 topic hz /chassis/odom
ros2 topic hz /odometry/filtered
```

同时观察 EKF 位姿：

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

检查内容：

- IMU 平均频率是否接近并稳定在 `20 Hz`。
- 静止时 `odom -> base_link` 的 X、Y 和 yaw 是否持续变化。
- `/chassis/imu/data_raw` 的 `angular_velocity.z` 静止均值是否明显偏离零。
- IMU、里程计和 EKF 话题是否有明显掉频或长时间停顿。

姿态解算固定按 `20 Hz` 积分。若实测频率为 `f`，仅由采样周期假设造成的航向比例误差约为：

```text
航向估计比例 = f / 20
```

例如实际频率为 `25 Hz`，却按 `20 Hz` 积分，IMU 姿态累计量理论上会放大约 `1.25` 倍。

## 5. 测试二：陀螺仪 yaw 比例

使用地面方向线作为真值，低速完成以下动作：

1. 左转约 `90 deg`。
2. 回到初始方向。
3. 右转约 `90 deg`。
4. 条件允许时再测试 `180 deg`。

车辆不能原地旋转时，可以低速走圆弧，但必须测量最终车头方向。不要只用一次 `360 deg` 转向后的四元数首尾值判断，因为 yaw 会绕回零；应分析展开后的累计角度。

需要比较：

- 实际物理转角。
- 按消息真实时间戳积分的 IMU `angular_velocity.z`。
- 按消息真实时间戳积分的 `/chassis/odom.twist.twist.angular.z`。
- `/odometry/filtered` 或 `odom -> base_link` 的 yaw 变化。

判定方法：

- IMU 角度错误、编码器角度正确：检查 IMU 比例、零偏、坐标轴方向和固定 `20 Hz` 假设。
- 编码器角度错误、IMU 角度正确：检查底盘角速度换算和编码器标定。
- 两者方向相反：检查 IMU 安装方向、静态 TF 或角速度符号。
- 两个原始量合理，但 EKF 更差：检查 EKF 观测选择、协方差和重复融合。

## 6. 测试三：直线往返

沿地面直线低速前进 `1 至 2 m`，停车后再沿原路线倒回起点。尽量让车辆中心和车头方向都对准初始标记。

检查内容：

- 前进距离是否存在固定比例误差。
- 直线过程中是否出现持续横向漂移。
- 返回起点后 EKF 的 X、Y 和 yaw 残差。
- `/chassis/odom` 正常时，`/odometry/filtered` 是否反而出现更大误差。

判定方法：

- 位置误差明显但 yaw 基本正确：优先检查编码器线速度比例。
- 横向误差随距离增大：优先检查航向漂移或左右轮速度比例。
- 加减速时 EKF 明显摆动：重点检查 IMU 线加速度及其协方差。

## 7. 测试四：EKF 闭环误差

让车辆走矩形、圆形或完整巡航路线一圈，最终回到地面起点，并对准初始车头方向。

在运动前和运动后分别记录：

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

计算起点到终点的差值：

```text
位置闭环误差 = sqrt((X_end - X_start)^2 + (Y_end - Y_start)^2)
航向闭环误差 = normalize(yaw_end - yaw_start)
```

由于本测试没有 AMCL 或 `lidar_location`，该差值就是编码器、IMU 和 EKF 链路共同产生的闭环误差，不包含 `map -> odom` 补偿。

## 8. 结果判断

| 测试现象 | 主要怀疑方向 |
| --- | --- |
| IMU 频率明显不是 20 Hz，航向误差比例接近 `f/20` | IMU 姿态解算固定采样频率 |
| 静止时 IMU yaw 和 EKF yaw 持续漂移 | 陀螺仪零偏、IMU yaw 过度可信 |
| 编码器 yaw 正确，IMU yaw 错误，EKF 跟随 IMU | IMU 比例或 EKF 协方差 |
| IMU yaw 正确，编码器 yaw 错误 | 编码器角速度换算或底盘标定 |
| 原始传感器合理，EKF 输出更差 | EKF 配置、协方差或相关观测重复融合 |
| 直线距离成比例偏大或偏小 | 编码器线速度比例 |
| 单独运行 EKF 时一圈闭环误差已经很大 | 后续定位产生较大的 `map -> odom` 多半属于正常补偿 |

## 9. 测试记录模板

```text
日期：
地面条件：
电池电压：

IMU 平均频率：
IMU 最小/最大频率：
里程计平均频率：
EKF 平均频率：

静止 120 秒：
  X 漂移：
  Y 漂移：
  yaw 漂移：
  静止 gyro.z 均值：

90 度转向：
  物理角度：
  IMU 积分角度：
  编码器积分角度：
  EKF 角度：

直线往返：
  实际距离：
  EKF 最大横向偏差：
  返回位置误差：
  返回航向误差：

闭环巡航：
  路线长度：
  位置闭环误差：
  航向闭环误差：
  rosbag 路径：
```

## 10. 完成后的下一步

先根据本测试确定 EKF 自身闭环误差。之后使用同一条路线分别启动 AMCL 和 `lidar_location`：

- 如果两种定位都产生幅度接近的 `map -> odom` 修正，优先处理 EKF 漂移。
- 如果 EKF 闭环较好，但某一种定位的 `map -> base_link` 跳变或偏离真实起点，重点检查该定位器的激光匹配和修正门控。
