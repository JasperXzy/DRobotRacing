# README

```bash
cd ~/ros2_ws
colcon build --packages-select <package-name>
```

```bash
ifconfig
sudo nmcli dev dis wlan0
```

```bash
cd ../userdata
cd ../userdata && cd racing_ws && source install/setup.bash
```

## HDU-203
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
ros2 launch racing_track_detection racing_track_detection.launch.py
```

## 障碍物检测
```bash
ros2 launch racing_obstacle_detection obstacle_detection.launch.py
```

## 扫二维码
```bash
ros2 run qr_detector qr_detector_node
```

## 图生文
```bash
ros2 run qwen_pic_to_text qwen_pic_to_text
```

## 中速         //完赛参数

## 巡线控制
```bash
ros2 launch racing_control racing_control.launch.py avoid_angular_ratio:=-0.42 avoid_linear_speed:=0.54 recovering_linear_speed:=0.7 recovering_angular_ratio:=1.0 follow_angular_ratio:=-3.8 follow_linear_speed:=0.85 bottom_threshold_caution:=280 bottom_threshold_avoid:=313 parking_y_threshold:=400 line_confidence_threshold:=0.7 obstacle_confidence_threshold:=0.75 parking_sign_confidence_threshold:=0.4
```

## 龟速稳定的一版
```bash
ros2 launch racing_control racing_control.launch.py avoid_angular_ratio:=-0.46 avoid_linear_speed:=0.3 recovering_linear_speed:=0.75 recovering_angular_ratio:=0.8 follow_angular_ratio:=-4.2 follow_linear_speed:=0.75 bottom_threshold_caution:=275 bottom_threshold_avoid:=313 parking_y_threshold:=400 line_confidence_threshold:=0.7 obstacle_confidence_threshold:=0.7 parking_sign_confidence_threshold:=0.3
```

## 中速         //车身右视图摄像头直立，比较稳定的一版
```bash
ros2 launch racing_control racing_control.launch.py avoid_angular_ratio:=-0.5 avoid_linear_speed:=0.3 recovering_linear_speed:=0.8 recovering_angular_ratio:=0.95 follow_angular_ratio:=-4.5 follow_linear_speed:=0.85 bottom_threshold_caution:=275 bottom_threshold_avoid:=314 parking_y_threshold:=395 line_confidence_threshold:=0.7 obstacle_confidence_threshold:=0.7 parking_sign_confidence_threshold:=0.3
```

## 中速         //提升0.85线速度
```bash
ros2 launch racing_control racing_control.launch.py avoid_angular_ratio:=-0.53 avoid_linear_speed:=0.55 recovering_linear_speed:=0.85 recovering_angular_ratio:=1.1 follow_angular_ratio:=-4.5 follow_linear_speed:=0.85 bottom_threshold_caution:=305 bottom_threshold_avoid:=314 parking_y_threshold:=395 line_confidence_threshold:=0.7 obstacle_confidence_threshold:=0.7 parking_sign_confidence_threshold:=0.3
```

## 中速         // test
```bash
ros2 launch racing_control racing_control.launch.py avoid_angular_ratio:=-0.53 avoid_linear_speed:=0.55 recovering_linear_speed:=0.6 recovering_angular_ratio:=1.1 follow_angular_ratio:=-4.5 follow_linear_speed:=0.6 bottom_threshold_caution:=305 bottom_threshold_avoid:=314 parking_y_threshold:=395 line_confidence_threshold:=0.7 obstacle_confidence_threshold:=0.7 parking_sign_confidence_threshold:=0.3
```

## 中速、更低的避障速度，更近的减速距离
```bash
ros2 launch racing_control racing_control.launch.py avoid_angular_ratio:=-0.5 avoid_linear_speed:=0.2 recovering_linear_speed:=0.8 recovering_angular_ratio:=0.95 follow_angular_ratio:=-4.5 follow_linear_speed:=0.85 bottom_threshold_caution:=280 bottom_threshold_avoid:=313 parking_y_threshold:=395 line_confidence_threshold:=0.7 obstacle_confidence_threshold:=0.7 parking_sign_confidence_threshold:=0.3
```

## 高速         //车身右视图摄像头直立，比较稳定的一版
```bash
ros2 launch racing_control racing_control.launch.py avoid_angular_ratio:=-0.5 avoid_linear_speed:=0.2 recovering_linear_speed:=0.95 recovering_angular_ratio:=1.1 follow_angular_ratio:=-4.5 follow_linear_speed:=0.95 bottom_threshold_caution:=280 bottom_threshold_avoid:=300 parking_y_threshold:=395 line_confidence_threshold:=0.7 obstacle_confidence_threshold:=0.7 parking_sign_confidence_threshold:=0.3
```

## 性能监测
```bash
sudo hrut_somstatus
```
