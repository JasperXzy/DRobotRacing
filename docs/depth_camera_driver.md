# Depth Camera Driver (Aurora 930)

## 启动方式
```bash
ros2 launch depth_camera_driver depth_camera.launch.py
```

## 环境变量
*   `DEBUG_SHOW`: 设置为 `true` 时，开启点云发布 (`/depth_camera/points2`)，默认为 `false`

## 配置参数 (`config/depth_camera_driver.yaml`)

| 参数名 | 类型 | 默认值 | 说明 |
| :--- | :--- | :--- | :--- |
| `rgb_enable` | bool | `true` | 开启 RGB 彩色图像流 |
| `ir_enable` | bool | `false` | 开启 IR 红外灰度图像流 |
| `depth_enable` | bool | `true` | 开启深度图像流 |
| `point_cloud_enable` | bool | `false` | 开启 3D 点云发布 (CPU 密集型，仅在调试时开启) |
| `rgbd_enable` | bool | `false` | 使用 RGBD 混合流模式 (使用独立流更稳定) |
| `align_mode` | bool | `true` | **关键参数**，开启 RGB 与 Depth 的空间对齐 (Registration) |
| `depth_correction` | bool | `true` | 开启深度图畸变校正 |
| `resolution_mode_index` | int | `2` | 分辨率模式索引 (具体对应关系见源码) |
| `laser_power` | float | `1.0` | 红外激光发射器功率 |

## 发布话题

| 话题名称 | 类型 | 说明 |
| :--- | :--- | :--- |
| `/depth_camera/rgb/image_raw` | `sensor_msgs/msg/Image` | 彩色图像 |
| `/depth_camera/rgb/camera_info` | `sensor_msgs/msg/CameraInfo` | 彩色相机内参 |
| `/depth_camera/depth/image_raw` | `sensor_msgs/msg/Image` | 深度图像 (16-bit, 单位: mm) |
| `/depth_camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 深度相机内参 (与 RGB 对齐后与 RGB 内参一致) |
| `/depth_camera/points2` | `sensor_msgs/msg/PointCloud2` | 3D 点云 (仅在 `DEBUG_SHOW=true` 时发布) |
