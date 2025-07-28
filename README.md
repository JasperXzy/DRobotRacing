
# build包
```bash
cd ~/ros2_ws
colcon build --packages-select <package-name>
```

# MobaXterm启动
```bash
cd ../userdata
cd ../userdata && cd racing_ws && source install/setup.bash
```

## HDU-203
```bash
192.168.1.107
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
export WEB_SHOW=True
ros2 launch racing_obstacle_detection obstacle_detection.launch.py
```

## 扫二维码
```bash
ros2 run qr_detector qr_detector_node
```
## 测试用二维码
```bash
ros2 run qr_detector qr_test
```
## 图生文
```bash
ros2 run qwen_pic_to_text qwen_pic_to_text
```


## 中速_offset线性关系         //提升0.85线速度
```bash
ros2 launch racing_control racing_control.launch.py avoid_angular_ratio:=-0.0045 avoid_linear_speed:=0.5 recovering_linear_speed:=0.85 recovering_angular_ratio:=1.25 follow_angular_ratio:=-3.0 follow_linear_speed:=0.85 bottom_threshold_caution:=300 bottom_threshold_avoid:=305 obstacle_y_jump_threshold:=50 parking_y_threshold:=405 line_confidence_threshold:=0.7 obstacle_confidence_threshold:=0.5 parking_sign_confidence_threshold:=0.3 qr_code_confidence_threshold:=1.1 qr_code_area_threshold:=500
```


## 中速_offset反比例         //提升0.85线速度
```bash
ros2 launch racing_control racing_control.launch.py avoid_angular_ratio:=-80.0 avoid_linear_speed:=0.5 recovering_linear_speed:=0.85 recovering_angular_ratio:=1.3 follow_angular_ratio:=-4.0 follow_linear_speed:=0.85 bottom_threshold_caution:=300 bottom_threshold_avoid:=305 obstacle_y_jump_threshold:=50 parking_y_threshold:=405 line_confidence_threshold:=0.7 obstacle_confidence_threshold:=0.5 parking_sign_confidence_threshold:=0.3 qr_code_confidence_threshold:=1.1 qr_code_area_threshold:=500
```

## 高速         //提升0.95线速度
```bash
ros2 launch racing_control racing_control.launch.py avoid_angular_ratio:=-0.55 avoid_linear_speed:=0.5 recovering_linear_speed:=0.95 recovering_angular_ratio:=1.25 follow_angular_ratio:=-4.0 follow_linear_speed:=0.95 bottom_threshold_caution:=295 bottom_threshold_avoid:=305 obstacle_y_jump_threshold:=50 parking_y_threshold:=395 line_confidence_threshold:=0.9 obstacle_confidence_threshold:=0.7 parking_sign_confidence_threshold:=0.3 qr_code_confidence_threshold:=1.1 qr_code_area_threshold:=500
```
## 性能监测
```bash
sudo hrut_somstatus
```


## test
```bash
ros2 launch racing_control racing_control.launch.py avoid_angular_ratio:=-0.1 avoid_linear_speed:=0.1 recovering_linear_speed:=0.1 recovering_angular_ratio:=1.3 follow_angular_ratio:=-4.0 follow_linear_speed:=0.1 bottom_threshold_caution:=300 bottom_threshold_avoid:=305 obstacle_y_jump_threshold:=50 parking_y_threshold:=405 line_confidence_threshold:=0.7 obstacle_confidence_threshold:=0.5 parking_sign_confidence_threshold:=0.3 qr_code_confidence_threshold:=1.1 qr_code_area_threshold:=500
```



























## 中速         //完赛参数
```bash
ros2 launch racing_control racing_control.launch.py avoid_angular_ratio:=-0.5 avoid_linear_speed:=0.3 recovering_linear_speed:=0.8 recovering_angular_ratio:=0.95 follow_angular_ratio:=-4.5 follow_linear_speed:=0.85 bottom_threshold_caution:=275 bottom_threshold_avoid:=314 parking_y_threshold:=395 line_confidence_threshold:=0.7 obstacle_confidence_threshold:=0.7 parking_sign_confidence_threshold:=0.3
```
