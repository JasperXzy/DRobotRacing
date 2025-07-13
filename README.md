# README

## 编译
```bash
cd ~/racing_ws
colcon build --packages-select <packages-name>
```
## HDU-203 WLAN
```bash
ifconfig
sudo nmcli dev dis wlan0
```

```bash
192.168.10.12
```

## 启动rosbridge端⼝  --> 上位机costudio
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

## 启动底盘
```bash
ros2 launch origincar_base origincar_bringup.launch.py
```

## 启动相机   在studio中更改节点为   /image
```bash
ros2 launch origincar_bringup camera.launch.py
```

## 检测中线
```bash
cd racing_ws && source install/setup.bash
ros2 launch racing_track_detection_resnet racing_track_detection_resnet.launch.py
```

## 障碍物检测
```bash
ros2 launch racing_obstacle_detection obstacle_detection.launch.py
```

## 巡线控制
```bash
ros2 launch racing_control racing_control.launch.py avoid_angular_ratio:=0.2 avoid_linear_speed:=0.5 follow_angular_ratio:=-1.0 follow_linear_speed:=0.5
```

## 扫二维码
```bash
ros2 run qr_detector qr_detector_node
```

## 图生文
```bash
ros2 run qwen_pic_to_text qwen_pic_to_text
```
