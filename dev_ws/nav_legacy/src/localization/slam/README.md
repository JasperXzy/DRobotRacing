# SLAM Toolbox 建图与定位

## 概述

SLAM Toolbox 有两种工作模式：

| 模式 | `mode` | 说明 |
|------|--------|------|
| `mapping` | 建图 | 从零构建地图，生成 `.posegraph` + `.data` 序列化地图 |
| `localization` | 定位 | 加载已有序列化地图，通过 scan-matching 实时定位 |

---

## 模式切换

使用工程根目录下的脚本一键切换：

```bash
cd ~/D_Robot/dev_ws/nav_legacy

# 切换到建图模式
./scripts/switch_mode.sh mapping

# 切换到定位模式
./scripts/switch_mode.sh localization
```

---

## 建图流程

### 1. 切换到建图模式

```bash
./scripts/switch_mode.sh mapping
```

### 2. 启动必需节点

**每个终端先 source：**

```bash
export ROS_DOMAIN_ID=1
cd ~/D_Robot/dev_ws/nav_legacy
source /opt/ros/humble/setup.bash
source install/setup.bash
```

| 终端 | 命令 | 作用 |
|------|------|------|
| ① | `ros2 launch driver_bringup drivers.launch.py` | 激光雷达 + 底盘驱动 |
| ② | `ros2 launch state_estimation ekf.launch.py` | EKF 融合里程计+IMU，发布 `odom→base_link` TF |
| ③ | `ros2 launch slam slam.launch.py` | SLAM Toolbox **建图模式** |

> 建图阶段**不需要启动 Nav2**（`origincar_nav2.launch.py`）。

### 3. 遥控机器人建图

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel
```

- 缓慢遍历所有需要建图的区域
- 尽量走回环路径（回到起点），帮助回环检测校正
- 可打开 RViz 观察实时建图效果

### 4. 保存地图

建图完成后保存**两份**地图，缺一不可：

#### 4a. 序列化位姿图（定位模式用）

```bash
mkdir -p ~/D_Robot/dev_ws/nav_legacy/src/origincar_nav2/map/serialized

ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/root/D_Robot/dev_ws/nav_legacy/src/origincar_nav2/map/serialized/my_map'}"
```

生成文件：`my_map.posegraph` + `my_map.data`

#### 4b. 占据栅格地图（Nav2 global costmap 用）

```bash
ros2 run nav2_map_server map_saver_cli -f ~/D_Robot/dev_ws/nav_legacy/src/origincar_nav2/map/my_map
```

生成文件：`my_map.pgm` + `my_map.yaml`

### 5. 切回定位模式

```bash
./scripts/switch_mode.sh localization
```

确认 `config/localization/slam_toolbox.yaml` 中包含：

```yaml
mode: localization
map_file_name: "/root/D_Robot/dev_ws/nav_legacy/src/origincar_nav2/map/serialized/my_map"
map_start_pose: [0.0, 0.0, 0.0]
```

---

## 两种地图的用途

| 文件 | 生成方式 | 使用者 |
|------|---------|--------|
| `my_map.posegraph` + `my_map.data` | `serialize_map` 服务 | **SLAM Toolbox** — localization 模式加载，scan-matching 定位 |
| `my_map.pgm` + `my_map.yaml` | `map_saver_cli` | **map_server** — Nav2 global costmap 的 StaticLayer |

> 注意：`.posegraph` 和 `.pgm` 是不同格式，对应用不同组件，**不能混用**。

---

## 定位模式运行

定位模式下需要启动 Nav2 进行导航：

```bash
# 终端 1: 驱动
ros2 launch driver_bringup drivers.launch.py

# 终端 2: EKF
ros2 launch state_estimation ekf.launch.py

# 终端 3: Nav2（内部已集成 SLAM Toolbox 定位 + map_server + 规划器 + 控制器）
ros2 launch origincar_nav2 origincar_nav2.launch.py
```

在 RViz 中使用 "2D Pose Estimate" 工具设置初始位姿，SLAM Toolbox 即开始 scan-matching 定位。
