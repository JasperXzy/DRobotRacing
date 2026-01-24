# Driver Bringup

## 启动方式

### 普通模式
仅启动必要的驱动和话题，关闭可视化调试数据以节省 CPU
```bash
ros2 launch driver_bringup drivers.launch.py
```

### 调试模式
开启所有可选的可视化话题
```bash
export DEBUG_SHOW=true
ros2 launch driver_bringup drivers.launch.py
```

## 包含的模块

1.  **`lidar_driver`**: Leishen N10 雷达
2.  **`camera_driver`**: Hobot USB 摄像头
3.  **`depth_camera_driver`**: Aurora 930 深度相机
4.  **`chassis_driver`**: OriginCar 底盘驱动

## 话题总览

启动后，系统将提供以下核心话题供上层算法使用：

*   **感知**:
    *   `/lidar/scan` (2D 激光)
    *   `/camera/image` (NV12 图像, 零拷贝)
    *   `/depth_camera/rgb/image_raw` (RGB 图像)
    *   `/depth_camera/depth/image_raw` (深度图像)
    *   `/chassis/imu/data_raw` (IMU)
*   **控制与状态**:
    *   `/chassis/odom` (里程计)
    *   `/chassis/cmd_vel` (速度控制)
    *   `/tf` (坐标变换)
