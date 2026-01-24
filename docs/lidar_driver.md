# Lidar Driver (N10)

## 启动方式
```bash
ros2 launch lidar_driver lidar.launch.py
```

## 配置参数 (`config/lidar_driver.yaml`)

| 参数名 | 类型 | 默认值 | 说明 |
| :--- | :--- | :--- | :--- |
| `frame_id` | string | `laser` | 激光雷达坐标系名称 |
| `scan_topic` | string | `/lidar/scan` | 发布 LaserScan 消息的话题名称 |
| `pointcloud_topic` | string | `/lidar/point_cloud` | 发布 PointCloud2 消息的话题名称 |
| `min_range` | double | `0.2` | 最小有效距离 (米) |
| `max_range` | double | `200.0` | 最大有效距离 (米) |
| `angle_disable_min` | double | `0.0` | 屏蔽角度起始值 (度) |
| `angle_disable_max` | double | `0.0` | 屏蔽角度结束值 (度) |
| `use_gps_ts` | bool | `false` | 是否使用 GPS 时间戳 |
| `interface_selection` | string | `serial` | 接口类型 (`serial` 或 `net`) |
| `serial_port_` | string | `/dev/lidar` | 串口设备路径 |
| `pubScan` | bool | `true` | 是否发布 LaserScan |
| `pubPointCloud2` | bool | `false` | 是否发布 PointCloud2 |

## 发布话题

| 话题名称 | 类型 | 说明 |
| :--- | :--- | :--- |
| `/lidar/scan` | `sensor_msgs/msg/LaserScan` | 2D 激光扫描数据 |
| `/lidar/point_cloud` | `sensor_msgs/msg/PointCloud2` | 3D 点云数据 (默认关闭) |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 驱动健康状态及频率信息 |

## 订阅话题
*   `/lidar/order`: `std_msgs/msg/Int8` - 控制雷达启停或模式切换
