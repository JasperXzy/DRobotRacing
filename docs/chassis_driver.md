# Chassis Driver (Ackerman)

## 启动方式
```bash
ros2 launch chassis_driver chassis.launch.py
```

## udev 规则配置
为了防止串口名称漂移，运行以下脚本创建固定别名 `/dev/chassis`：
```bash
bash src/drivers/chassis_driver/scripts/udev.sh
```

## 配置参数 (`config/chassis_driver.yaml`)

| 参数名 | 类型 | 默认值 | 说明 |
| :--- | :--- | :--- | :--- |
| `usart_port_name` | string | `/dev/chassis` | 串口设备路径 |
| `serial_baud_rate` | int | `115200` | 串口波特率 |
| `odom_frame_id` | string | `odom` | 里程计坐标系 |
| `robot_frame_id` | string | `base_link` | 机器人基座坐标系 |
| `gyro_frame_id` | string | `gyro_link` | IMU 坐标系 |
| `cmd_vel` | string | `chassis/cmd_vel` | 订阅的速度控制话题 |
| `akm_cmd_vel` | string | `chassis/ackermann_cmd` | 订阅的阿克曼控制话题 |

## 发布话题

| 话题名称 | 类型 | 说明 |
| :--- | :--- | :--- |
| `/chassis/odom` | `nav_msgs/msg/Odometry` | 轮式里程计数据 (位置和速度) |
| `/chassis/imu/data_raw` | `sensor_msgs/msg/Imu` | 原始 IMU 数据 (加速度计和陀螺仪) |
| `/chassis/power_voltage` | `std_msgs/msg/Float32` | 电池电压 |
| `/chassis/robot_pose` | `geometry_msgs/msg/Vector3` | 机器人位姿 (x, y, yaw) - 调试用 |
| `/chassis/robot_vel` | `geometry_msgs/msg/Vector3` | 机器人速度 (vx, vy, w) - 调试用 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 发布 `odom` -> `base_link` 的坐标变换 |

## 订阅话题

| 话题名称 | 类型 | 说明 |
| :--- | :--- | :--- |
| `/chassis/cmd_vel` | `geometry_msgs/msg/Twist` | 通用速度指令 (线速度 x, 角速度 z) |
| `/chassis/ackermann_cmd` | `ackermann_msgs/msg/AckermannDriveStamped` | 阿克曼专用控制指令 (速度 speed, 转向角 steering_angle) |

## 通信协议
驱动内置了特定的串口协议解析逻辑：
*   **帧头**: `0x7B`
*   **帧尾**: `0x7D`
*   **校验**: 异或校验 (XOR)
