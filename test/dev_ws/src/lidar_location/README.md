# lidar_location（ROS 2）

这是 ROS1 jie_ware/lidar_loc 栅格激光匹配算法的 ROS2 迁移版，用于在
test/dev_ws 中替代 AMCL。

接口：

- 订阅 /map、/scan、/initialpose
- 发布 /amcl_pose、/lidar_location/match_score
- 发布 /lidar_location/state（LOCKED、DEGRADED、LOST）
- 发布 /lidar_location/robust_score、/lidar_location/inlier_ratio
- 发布 /lidar_location/correction_distance、/lidar_location/correction_yaw
- 发布 /lidar_location/valid_beams、/lidar_location/iterations
- 发布 /lidar_location/coarse_search_used
- 发布 map -> odom TF
- 依赖已有的 odom -> base_link 和 base_link -> laser TF

节点先使用里程计预测本次位姿，再在占据栅格生成的似然场中做扫描匹配。
正常状态使用快速局部细化；退化或失锁时周期性执行粗到细搜索。扫描点可根据
LaserScan.time_increment 和扫描起止 odom 位姿做运动去畸变。评分同时输出原始
平均分、裁剪离群束后的 robust_score 和地图内点比例。
似然场使用双线性连续采样，局部搜索按 refinement_levels 逐级缩小位移和
角度步长，避免最终位姿只能停在整数栅格和整度角上。

质量门控不会让低于 lost_score 的匹配修改 map -> odom。DEGRADED 状态只应用
受限的小幅修正；LOST 状态保持里程计预测，达到连续恢复帧数后才重新接收雷达
修正。/amcl_pose 的协方差会在 DEGRADED/LOST 时自动增大。
/lidar_location/state 只是定位质量诊断，不会修改路径跟随的速度或发布停车命令。

它仍然不是完整的全局粒子定位器。粗搜索范围由 coarse_search.linear_range 和
coarse_search.angular_range_deg 限制，因此 /initialpose 仍需落在真实位置附近。

单独启动：

    ros2 launch lidar_location lidar_location.launch.py

总项目使用 origincar_system/launch/test_bringup_lidar_location.launch.py 启动。
原始 GPL-2.0 ROS1 源码和许可证保留在
src/ros1_lidar_location/jie_ware-main。

运行时重点观察：

    ros2 topic echo /lidar_location/state
    ros2 topic echo /lidar_location/match_score
    ros2 topic echo /lidar_location/inlier_ratio
    ros2 topic echo /lidar_location/correction_distance
