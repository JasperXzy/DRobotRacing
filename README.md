# README

```bash
cd ~/ros2_ws
colcon build --packages-select <功能包名>//只编译一个包会快一点
```

```bash
ifconfig
sudo nmcli dev dis wlan0
```


```bash
# hdu203:
192.168.10.168
(教室wifi)
```

```bash
# 启动rosbridge端⼝  --> 上位机costudio
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

```bash
# 启动底盘
ros2 launch origincar_base origincar_bringup.launch.py
```

```bash
# 启动相机   在studio中更改节点为   /image_compressed
ros2 launch origincar_bringup camera.launch.py
```

```bash
# 检测中线
cd racing_ws && source install/setup.bash
ros2 launch racing_track_detection_resnet racing_track_detection_resnet.launch.py
```

```bash
# 障碍物检测
ros2 launch racing_obstacle_detection obstacle_detection.launch.py
```

```bash
# 巡线控制
ros2 launch racing_control racing_control.launch.py avoid_angular_ratio:=0.2 avoid_linear_speed:=0.5 follow_angular_ratio:=-1.0 follow_linear_speed:=0.5
```

```bash
# 扫二维码
ros2 run qr_detector qr_detector_node
```

```bash
# 图生文
ros2 run qwen_pic_to_text qwen_pic_to_text
```
