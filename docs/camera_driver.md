# Camera Driver (Hobot USB Cam)

## 启动方式
```bash
ros2 launch camera_driver camera.launch.py
```

## 环境变量
*   `DEBUG_SHOW`: 设置为 `true` 时，启动 JPEG 编码节点以支持远程预览 (`/image_raw/compressed`)，默认为 `false`

## 配置参数 (`config/camera_driver.yaml`)

| 参数名 | 类型 | 默认值 | 说明 |
| :--- | :--- | :--- | :--- |
| `video_device` | string | `/dev/video0` | 摄像头设备路径 |
| `image_width` | int | `1920` | 图像宽度 |
| `image_height` | int | `1080` | 图像高度 |
| `pixel_format` | string | `mjpeg` | 摄像头输出格式 (推荐 `mjpeg`) |
| `io_method` | string | `mmap` | IO 模式 (`mmap`, `read`, `userptr`) |
| `frame_id` | string | `camera_link` | 相机坐标系名称 |

## 发布话题

| 话题名称 | 类型 | 说明 |
| :--- | :--- | :--- |
| `/camera/image` | `hbm_img_msgs/msg/HbmMsg1080P` | **核心数据流**，基于共享内存的 NV12 图像 |
| `/camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | JPEG 压缩图像，仅在 `DEBUG_SHOW=true` 时发布，用于远程调试 |
| `/camera_info` | `sensor_msgs/msg/CameraInfo` | 相机内参信息 |

## 架构说明
驱动包含两个主要节点：
1.  **`usb_cam_node`**: 从 USB 读取数据
2.  **`usb_cam_nv12_codec_node`**: 利用硬件 VPU 将 MJPEG 解码为 NV12，发布 `/camera/image`
3.  **(可选) `usb_cam_preview_codec_node`**: 将 NV12 编码为 JPEG，仅用于调试
