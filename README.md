cd ~/ros2_ws
colcon build --packages-select <功能包名>//只编译一个包会快一点

# 关闭板端wifi，板端wifi一般为wlan0，可以通过ifconfig查看

sudo nmcli dev dis wlan0


#localhost：
172.20.10.13 (XJW)
192.168.249.191 (SZC)
192.168.249.76 (网卡)

#启动rosbridge端⼝  --> 上位机costudio
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
#启动底盘
ros2 launch origincar_base origincar_bringup.launch.py

#启动相机   在studio中更改节点为   /image_compressed
ros2 launch origincar_bringup camera.launch.py

#检测中线
ros2 launch racing_track_detection_resnet racing_track_detection_resnet.launch.py

#巡线控制
ros2 launch racing_control racing_control.launch.py avoid_angular_ratio:=0.2 avoid_linear_speed:=0.1 follow_angular_ratio:=-1.0 follow_linear_speed:=0.5

#障碍物检测
ros2 launch racing_obstacle_detection_yolo racing_obstacle_detection_yolo.launch.py

#扫二维码
ros2 run qr_detector qr_detector_node

#图生文
ros2 run qwen_pic_to_text qwen_pic_to_textwe