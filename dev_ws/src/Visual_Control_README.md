# Visual Control 启动流程

## 编译包
```bash
cd ~/D_Robot/dev_ws
source /opt/tros/humble/setup.bash
colcon build --packages-select visual_control line_detector perception_detector qr_detector_cpp
source install/setup.bash
```

## 进入工作区
```bash
cd ~/D_Robot/dev_ws
source /opt/tros/humble/setup.bash
source install/setup.bash
```

## 启动底盘
```bash
ros2 launch chassis_driver chassis.launch.py
```

## 启动相机
```bash
ros2 launch camera_driver camera.launch.py
```

检查相机 hbmem 图像：
```bash
ros2 topic hz /camera/image_hbmem
```

## 启动 perception 视觉识别
`perception` 是当前视觉识别部分的总目录，包含：

- 线识别：`line_detector`
- 二维码识别：`qr_detector_cpp`
- 障碍物和停车位识别：`perception_detector`

### 线识别
```bash
ros2 launch line_detector line_detector.launch.py
```

检查输出：
```bash
ros2 topic hz /visual_track_center_detection
ros2 topic echo /visual_track_center_detection --once
```

### 障碍物 / 停车位识别
推荐使用配置文件启动：
```bash
ros2 launch perception_detector perception_detector.launch.py
```

如需临时覆盖参数，可直接运行节点：
```bash
ros2 run perception_detector perception_detector_node --ros-args \
  -p input_topic:=/camera/image_hbmem \
  -p detection_topic:=/visual_obstacle_detection \
  -p model_path:=/root/D_Robot/model/Visual_Control/yolo26_det_bpu_nashe_640x640_nv12.hbm \
  -p class_num:=2 \
  -p class_names:="['construction_cone', 'parking_sign']" \
  -p qr_class_name:=parking_sign \
  -p qr_class_id:=1
```

检查输出：
```bash
ros2 topic hz /visual_obstacle_detection
ros2 topic echo /visual_obstacle_detection --once
```

### 二维码识别
```bash
ros2 run qr_detector_cpp qr_detector_node
```

## 中速_offset线性关系         //提升0.85线速度
```bash
ros2 launch visual_control visual_control.launch.py avoid_angular_ratio:=-0.005 avoid_linear_speed:=0.5 recovering_linear_speed:=0.85 recovering_angular_ratio:=1.25 follow_angular_ratio:=-3.0 follow_linear_speed:=0.85 bottom_threshold_caution:=295 bottom_threshold_avoid:=305 obstacle_y_jump_threshold:=50 parking_y_threshold:=405 line_confidence_threshold:=0.2 obstacle_confidence_threshold:=0.5 parking_sign_confidence_threshold:=0.6 qr_code_confidence_threshold:=0.6 qr_code_hight_follow_threshold:=20 qr_code_height_threshold:=54 line_center_threshold_x:=280 qr_code_speed_modifier:=0.2 follow_qr_code_ratio:=-4.0 follow_parking_sign_ratio:=-4.5 delta_x_ratio:=0.0
```


## 高速_offset线性关系         //提升0.95速度
```bash
ros2 launch visual_control visual_control.launch.py avoid_angular_ratio:=-0.0055 avoid_linear_speed:=0.55 recovering_linear_speed:=0.95 recovering_angular_ratio:=1.45 follow_angular_ratio:=-3.5 follow_linear_speed:=0.95 bottom_threshold_caution:=275 bottom_threshold_avoid:=280 obstacle_y_jump_threshold:=50 parking_y_threshold:=405 line_confidence_threshold:=0.4 obstacle_confidence_threshold:=0.3 parking_sign_confidence_threshold:=0.3 qr_code_confidence_threshold:=0.6 qr_code_hight_follow_threshold:=30 qr_code_height_threshold:=55 line_center_threshold_x:=280 qr_code_speed_modifier:=0.2 follow_qr_code_ratio:=-4.0 follow_parking_sign_ratio:=-4.5 delta_x_ratio:=0.17
```

## 光速_offset线性关系         //提升1.15速度
```bash
ros2 launch visual_control visual_control.launch.py avoid_angular_ratio:=-0.005 avoid_linear_speed:=0.5 recovering_linear_speed:=1.15 recovering_angular_ratio:=1.45 follow_angular_ratio:=-3.5 follow_linear_speed:=0.95 bottom_threshold_caution:=275 bottom_threshold_avoid:=280 obstacle_y_jump_threshold:=50 parking_y_threshold:=415 line_confidence_threshold:=0.4 obstacle_confidence_threshold:=0.3 parking_sign_confidence_threshold:=0.3 qr_code_confidence_threshold:=0.6 qr_code_hight_follow_threshold:=30 qr_code_height_threshold:=54 line_center_threshold_x:=280 qr_code_speed_modifier:=0.2 follow_qr_code_ratio:=-4.0 follow_parking_sign_ratio:=-4.5 delta_x_ratio:=0.15
```


## 性能监测
```bash
sudo hrut_somstatus
```


## 专门用来test、debug
```bash
ros2 launch visual_control visual_control.launch.py avoid_angular_ratio:=-0.0055 avoid_linear_speed:=0.55 recovering_linear_speed:=0.95 recovering_angular_ratio:=1.45 follow_angular_ratio:=-3.5 follow_linear_speed:=0.95 bottom_threshold_caution:=275 bottom_threshold_avoid:=280 obstacle_y_jump_threshold:=50 parking_y_threshold:=405 line_confidence_threshold:=0.4 obstacle_confidence_threshold:=0.3 parking_sign_confidence_threshold:=0.3 qr_code_confidence_threshold:=0.6 qr_code_hight_follow_threshold:=30 qr_code_height_threshold:=55 line_center_threshold_x:=280 qr_code_speed_modifier:=0.2 follow_qr_code_ratio:=-4.0 follow_parking_sign_ratio:=-4.5 delta_x_ratio:=0.0
```












## test巡线
```bash
ros2 launch visual_control visual_control.launch.py avoid_angular_ratio:=-0.005 avoid_linear_speed:=0.5 recovering_linear_speed:=0.85 recovering_angular_ratio:=1.25 follow_angular_ratio:=-3.0 follow_linear_speed:=0.85 bottom_threshold_caution:=295 bottom_threshold_avoid:=305 obstacle_y_jump_threshold:=50 parking_y_threshold:=405 line_confidence_threshold:=0.2 obstacle_confidence_threshold:=0.5 parking_sign_confidence_threshold:=1.1 qr_code_confidence_threshold:=1.1 qr_code_hight_follow_threshold:=20 qr_code_height_threshold:=54 line_center_threshold_x:=320 qr_code_speed_modifier:=0.2 follow_qr_code_ratio:=-3.0 follow_parking_sign_ratio:=-4.5
```























## 中速         //完赛参数
```bash
ros2 launch racing_control racing_control.launch.py avoid_angular_ratio:=-0.5 avoid_linear_speed:=0.3 recovering_linear_speed:=0.8 recovering_angular_ratio:=0.95 follow_angular_ratio:=-4.5 follow_linear_speed:=0.85 bottom_threshold_caution:=275 bottom_threshold_avoid:=314 parking_y_threshold:=395 line_confidence_threshold:=0.7 obstacle_confidence_threshold:=0.7 parking_sign_confidence_threshold:=0.3
```
