# lidar_location（ROS 2）

这是 ROS1 jie_ware/lidar_loc 栅格激光匹配算法的 ROS2 迁移版，用于在
test/dev_ws 中替代 AMCL。

接口：

- 订阅 /map、/scan、/initialpose
- 发布 /amcl_pose、/lidar_location/match_score
- 发布 map -> odom TF
- 依赖已有的 odom -> base_link 和 base_link -> laser TF

节点先使用里程计预测本次位姿，再在占据栅格生成的似然场中做局部爬山匹配。
它是局部连续定位器，不具备 AMCL 的全局粒子重定位能力，因此 /initialpose
必须与真实位姿足够接近。

单独启动：

    ros2 launch lidar_location lidar_location.launch.py

总项目使用 origincar_system/launch/test_bringup_lidar_location.launch.py 启动。
原始 GPL-2.0 ROS1 源码和许可证保留在
src/ros1_lidar_location/jie_ware-main。
