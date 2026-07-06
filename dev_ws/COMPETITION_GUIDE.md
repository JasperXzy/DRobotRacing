# 智能车比赛流程 — 改动清单与操作指南

## 一、已完成的文件改动清单

### 新建文件 (5个)

| 文件 | 说明 |
|------|------|
| `src/bringup/driver_bringup/launch/competition_bringup.launch.py` | 比赛总启动文件，一键启动全部节点 |
| `src/origincar_nav2/origincar_nav2/trigger_cruise.py` | 终端发令脚本，运行一次即退出 |
| `src/origincar_nav2/origincar_nav2/waypoints_prescan.yaml` | 扫描前航点模板 ⚠️ 需填入实际坐标 |
| `src/origincar_nav2/origincar_nav2/waypoints_cw.yaml` | 顺时针航点模板 ⚠️ 需填入实际坐标 |
| `src/origincar_nav2/origincar_nav2/waypoints_ccw.yaml` | 逆时针航点模板 ⚠️ 需填入实际坐标 |

### 修改文件 (4个)

| 文件 | 改动内容 |
|------|----------|
| `src/origincar_nav2/origincar_nav2/waypoint_cruiser.py` | **重写** — 状态机：等发令→走prescan→QR选方向→走后续→VLM触发→结束 |
| `src/qr_detector_cpp/src/qr_detector_node.cpp` | 启动即扫描、支持JPEG/MJPEG解码、TTS语音播报方向、扫描成功后继续运行 |
| `src/origincar_nav2/setup.py` | 注册 trigger_cruise 入口点 + waypoints文件安装到包根目录 |
| `config/nav2/nav2.yaml` | yaw_goal_tolerance: 0.65→1.0, GoalAngleCritic权重: 2.0→1.5, 路径改install空间 |

---

## 二、完整比赛流程图

```
┌─────────────────────────────┐
│ ros2 launch driver_bringup  │
│ competition_bringup.launch  │  一键启动所有节点
└─────────────┬───────────────┘
              │ ~10秒全部就绪
              │ QR节点启动即开始持续扫描
              ▼
┌─────────────────────────────┐
│ ros2 run origincar_nav2     │
│ trigger_cruise              │  终端发令
└─────────────┬───────────────┘
              │
              ▼
┌─────────────────────────────────────────┐
│ 走 waypoints_prescan.yaml 航点           │
│ 过程中QR持续在扫，扫到就TTS播报+记录方向    │
└─────────────┬───────────────────────────┘
              │ prescan走完，根据QR结果
              │
     ┌────────┴────────┐
     │ QR奇数→顺时针    │ QR偶数→逆时针
     ▼                 ▼
┌──────────────┐  ┌──────────────┐
│ 走 cw.yaml   │  │ 走 ccw.yaml  │
│ 立刻衔接      │  │ 立刻衔接     │
│ 第5点触发VLM  │  │ 第5点触发VLM │
└──────┬───────┘  └──────┬───────┘
       │                 │
       ▼                 ▼
┌─────────────────────────────────────────┐
│ 触发VLM图生文 (不等结果，继续走)           │
│ VLM结果通过TTS语音播报                    │
└─────────────┬───────────────────────────┘
              │
              ▼
┌─────────────────────────────────────────┐
│ 走完剩余航点，巡航结束                     │
└─────────────────────────────────────────┘
```

---

## 三、话题信号说明

### /sign4return (Int32)

| 数值 | 含义 | 谁发 → 谁收 |
|------|------|-------------|
| -1 | 触发VLM图生文 | 巡航节点 → VLM节点 |
| 5 | 紧急停车 | 终端手动发 |

### /sign_switch (origincar_msg/Sign)

QR扫描成功后发布：

| sign_data | 含义 |
|-----------|------|
| 3 | 顺时针 (二维码数字为奇数) |
| 4 | 逆时针 (二维码数字为偶数) |

### /cruise_start (Empty)

终端发令脚本 → 巡航节点，启动巡航。

---

## 四、启动顺序（competition_bringup.launch.py 内部分阶段）

| 阶段 | 时间 | 启动内容 |
|------|------|----------|
| 第1阶段 | 0秒 | 底盘驱动、激光雷达、摄像头(不含深度相机)、origincar_description静态位姿 |
| 第2阶段 | 4秒后 | QR检测节点(启动即扫)、VLM图生文节点(挂起等触发) |
| 第3阶段 | 6秒后 | EKF(IMU+里程计融合)、SLAM定位 |
| 第4阶段 | 12秒后 | Nav2导航（等SLAM位姿收敛后再启动） |
| 第5阶段 | 14秒后 | 巡航节点(等Nav2 lifecycle完全激活，等发令) |

---

## 五、在机器人上的操作步骤

### 第1步：编译

```bash
cd ~/D_Robot/dev_ws
colcon build --packages-select origincar_nav2 qr_detector_cpp driver_bringup --symlink-install
source install/setup.bash
```

### 第2步：录制航点

用 RViz2 的 "Publish Point" 工具点击地图录制航点：

```bash
# 录制扫描前航点
ros2 run origincar_nav2 waypoint_recorder --ros-args -p output_file:=waypoints_prescan.yaml

# 录制顺时针航点
ros2 run origincar_nav2 waypoint_recorder --ros-args -p output_file:=waypoints_cw.yaml

# 录制逆时针航点
ros2 run origincar_nav2 waypoint_recorder --ros-args -p output_file:=waypoints_ccw.yaml
```

录制完成后，把生成的 yaml 文件复制到源码目录：

```bash
cp waypoints_prescan.yaml src/origincar_nav2/origincar_nav2/
cp waypoints_cw.yaml src/origincar_nav2/origincar_nav2/
cp waypoints_ccw.yaml src/origincar_nav2/origincar_nav2/
```

**重新编译**让文件安装到 install 目录：

```bash
colcon build --packages-select origincar_nav2 --symlink-install
source install/setup.bash
```

### 第3步：比赛运行

```bash
# 终端1：一键启动全部
ros2 launch driver_bringup competition_bringup.launch.py
# 等待约10秒，所有节点分阶段就绪

# 终端2：发令！
ros2 run origincar_nav2 trigger_cruise
```

### 紧急停车

```bash
ros2 topic pub /sign4return std_msgs/Int32 "data: 5"
```

---

## 六、可调参数

巡航节点 (`waypoint_cruiser`) 支持以下参数，在 launch 文件或命令行中修改：

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `vlm_trigger_waypoint` | `5` | 第几个航点触发VLM |
| `qr_default` | `cw` | 没扫到QR时的默认方向 (cw/ccw) |
| `qr_sign_cw` | `3` | 顺时针对应的sign值 |
| `qr_sign_ccw` | `4` | 逆时针对应的sign值 |

命令行覆盖示例：
```bash
ros2 run origincar_nav2 waypoint_cruiser --ros-args -p vlm_trigger_waypoint:=3
```

---

## 七、注意事项

- `config/nav2/nav2.yaml` 里的 BT XML 路径和地图路径已改成 install 空间路径，如果工作空间路径不同需要调整
- 三个航点文件当前是空模板，**必须先录制填写**才能跑
- QR节点订阅 `/camera/image` 话题，摄像头驱动必须正常工作
- VLM节点依赖 hobot_llamacpp 和模型文件，请确保模型文件路径正确
- 如果比赛时没扫到二维码，巡航节点会用 `qr_default` 参数指定的默认方向
