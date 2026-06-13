# EKF 参数调优指南

> 文件位置：`config/localization/ekf.yaml`
> EKF 节点：`/ekf_filter_node`（`robot_localization` 包）
> 功能：融合轮式里程计 `/chassis/odom` + IMU `/chassis/imu/data_raw`，输出 `/odometry/filtered`

---

## 一、General Settings（通用设置）

| 参数 | 当前值 | 作用 | 工程范围 | 需要调优 |
|------|--------|------|----------|:---:|
| `frequency` | 50.0 | EKF 预测/更新频率 (Hz)。越高响应越快，CPU 负载越大 | 30 ~ 100 | ❌ |
| `sensor_timeout` | 0.1 | 传感器超时 (s)，超时未收到数据则该传感器本轮不参与融合 | 0.05 ~ 0.2 | ❌ |
| `two_d_mode` | true | 2D 模式，忽略 z/roll/pitch，只估计 x, y, yaw | true | ❌ |
| `publish_tf` | true | 是否发布 odom→base_link 的 TF | true | ❌ |
| `publish_acceleration` | false | 输出里程计是否包含加速度 | false | ❌ |
| `print_diagnostics` | true | 终端打印诊断信息 | 调试时 true | ❌ |

---

## 二、Frame Configuration（坐标系）

| 参数 | 当前值 | 作用 | 调优 |
|------|--------|------|:---:|
| `map_frame` | map | 全局坐标系 | ❌ |
| `odom_frame` | odom | 里程计坐标系 | ❌ |
| `base_link_frame` | base_link | 机器人本体坐标系 | ❌ |
| `world_frame` | odom | 状态估计参考系 | ❌ |

---

## 三、odom0 — 轮式里程计

| 参数 | 当前值 | 作用 | 工程范围 | 调优 |
|------|--------|------|----------|:---:|
| `odom0` | `/chassis/odom` | 里程计话题名 | — | ❌ |
| `odom0_config` | 见下 | 选择用里程计的哪些数据位（位置/姿态/速度/加速度） | — | ❌ |
| `odom0_differential` | false | false=绝对位姿；true=帧间增量模式 | false | ❌ |
| `odom0_relative` | false | false=数据在 odom 坐标系下 | false | ❌ |
| `odom0_queue_size` | 10 | 缓冲队列长度 | 5 ~ 20 | ❌ |
| **`odom0_rejection_threshold`** | 2.0 | 马氏距离异常值拒绝阈值，超出的数据不参与融合。0=关闭 | **1.0 ~ 5.0** | ✅ |

### odom0_config 详解

```
索引:  [ x,   y,   z,  roll,pitch,yaw, vx,  vy,  vz,  vroll,vpitch,vyaw, ax,  ay,  az ]
当前值: [false,false,false,false,false,false,true, true,false,false, false, true, false,false,false]
```

- 位置 x/y/z → **不用**（轮式里程计位置会漂移）
- 姿态 roll/pitch/yaw → **不用**（交给 IMU）
- 线速度 vx/vy → **用** ✅
- 角速度 vyaw → **用** ✅

---

## 四、imu0 — IMU

| 参数 | 当前值 | 作用 | 工程范围 | 调优 |
|------|--------|------|----------|:---:|
| `imu0` | `/chassis/imu/data_raw` | IMU 话题名 | — | ❌ |
| `imu0_config` | 见下 | 选择用 IMU 的哪些数据位 | — | ❌ |
| `imu0_remove_gravitational_acceleration` | true | 去除重力加速度分量 | true（必须） | ❌ |
| `imu0_differential` | false | false=绝对模式 | false | ❌ |
| `imu0_relative` | false | — | false | ❌ |
| `imu0_queue_size` | 10 | 缓冲队列长度 | 5 ~ 20 | ❌ |
| **`imu0_rejection_threshold`** | 2.0 | IMU 异常值拒绝阈值 | **1.0 ~ 5.0** | ✅ |

### imu0_config 详解

```
索引:  [ x,   y,   z,  roll,pitch,yaw, vx, vy, vz, vroll,vpitch,vyaw, ax, ay, az ]
当前值: [false,false,false,true, true, true,false,false,false,true, true, true, true,true,false]
```

- 姿态 roll/pitch/yaw → **用** ✅
- 角速度 vroll/vpitch/vyaw → **用** ✅
- 线加速度 ax/ay → **用** ✅（az 在 2D 下不用）

---

## 五、🔥 process_noise_covariance（过程噪声协方差）— 核心调优项

15×15 对角矩阵，对应状态向量：

```
[x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]
```

### 调参直觉

| 调大 | 调小 |
|------|------|
| 更不相信运动模型 | 更相信运动模型 |
| 更依赖传感器测量 | 更依赖模型预测 |
| 响应更快，但可能更抖 | 输出更平滑，但可能滞后 |

### 各参数详解

| 索引 | 状态量 | 当前值 | 工程范围 | 含义 | 优先级 |
|:---:|--------|--------|----------|------|:---:|
| 0 | **x** | 0.05 | 0.01 ~ 0.5 | x 方向位置过程不确定性 | 🔴 高 |
| 1 | **y** | 0.05 | 0.01 ~ 0.5 | y 方向位置过程不确定性 | 🔴 高 |
| 2 | z | 0.06 | — | z 方向（2D 忽略） | — |
| 3 | roll | 0.03 | — | 横滚角（2D 忽略） | — |
| 4 | pitch | 0.03 | — | 俯仰角（2D 忽略） | — |
| 5 | **yaw** | 0.06 | **0.01 ~ 0.2** | 航向角 — 影响转弯响应 | 🔴 高 |
| 6 | **vx** | 0.025 | **0.005 ~ 0.1** | x 方向线速度 | 🟡 中 |
| 7 | **vy** | 0.025 | **0.005 ~ 0.1** | y 方向线速度 | 🟡 中 |
| 8 | vz | 0.04 | — | z 方向线速度（2D 忽略） | — |
| 9 | vroll | 0.01 | — | 横滚角速度（2D 忽略） | — |
| 10 | vpitch | 0.01 | — | 俯仰角速度（2D 忽略） | — |
| 11 | **vyaw** | 0.02 | **0.005 ~ 0.1** | 航向角速度 — 影响转弯平滑度 | 🔴 高 |
| 12 | **ax** | 0.01 | **0.005 ~ 0.1** | x 方向线加速度 | 🟡 中 |
| 13 | **ay** | 0.01 | **0.005 ~ 0.1** | y 方向线加速度 | 🟡 中 |
| 14 | az | 0.015 | — | z 方向线加速度（2D 忽略） | — |

> 🔴 = 必调项 &nbsp;&nbsp; 🟡 = 按需调 &nbsp;&nbsp; — = 2D 模式用不到

---

## 六、initial_estimate_covariance（初始估计协方差）

全是 `1e-9`，表示对初始状态非常确定。**不需要调**，仅在 EKF 启动时使用。

---

## 七、调试优先级总览

```
第一梯队（必调）：
  process_noise_covariance[5]   yaw     0.01 ~ 0.2    航向角漂移/滞后
  process_noise_covariance[11]  vyaw    0.005 ~ 0.1   角速度估计质量
  process_noise_covariance[0]   x       0.01 ~ 0.5    x 位置漂移
  process_noise_covariance[1]   y       0.01 ~ 0.5    y 位置漂移

第二梯队（视情况）：
  process_noise_covariance[6]   vx      0.005 ~ 0.1   线速度抖动
  process_noise_covariance[7]   vy      0.005 ~ 0.1
  process_noise_covariance[12]  ax      0.005 ~ 0.1   加速度响应
  process_noise_covariance[13]  ay      0.005 ~ 0.1
  odom0_rejection_threshold     1.0 ~ 5.0            里程计跳变
  imu0_rejection_threshold      1.0 ~ 5.0            IMU 跳变
```

---

## 八、典型问题 → 调参方向

| 现象 | 调整方向 |
|------|---------|
| 🐍 车转弯时里程计滞后（蛇形走位） | **增大** `yaw`: 0.06 → 0.1, **增大** `vyaw`: 0.02 → 0.05 |
| 📳 直行时里程计高频抖动 | **减小** `vx`/`vy`: 0.025 → 0.01 |
| 🚀 启动瞬间位姿跳变 | **增大** `x`/`y`: 0.05 → 0.1 |
| 🧭 航向角缓慢漂移（不转弯也在转） | **减小** `yaw`: 0.06 → 0.03，更信任 IMU 姿态 |
| ⚡ 急加速/急刹车时位置不准 | **增大** `ax`/`ay`: 0.01 → 0.03 |
| 🔄 里程计更新有延迟感 | **增大** `x`/`y`/`yaw` 整体 ×1.5 |
| 📊 里程计噪声大但响应够快 | **减小** `x`/`y`/`yaw` 整体 ×0.5 |

---

## 九、动态调参方法

### 方法一：rqt_reconfigure 图形化（推荐）

```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

展开 `/ekf_filter_node`，直接拖动 process_noise_covariance 各对角线元素，实时生效。

### 方法二：ros2 param 命令行

```bash
# 查看当前值
ros2 param get /ekf_filter_node process_noise_covariance

# 设置单个值（需要设置整个数组，建议用 rqt_reconfigure）
```

### 方法三：调好后导出保存

```bash
ros2 param dump /ekf_filter_node --output-dir ~/D_Robot/dev_ws/config/localization/
```

---

## 十、调参工作流

```
1. ros2 launch state_estimation ekf.launch.py     # 启动 EKF
2. ros2 run rqt_reconfigure rqt_reconfigure        # 打开调参面板
3. 跑车测试 → 拖动滑块实时观察效果
4. 确定最优值后 → ros2 param dump 导出
5. 将导出的参数合并到 ekf.yaml
6. 重新启动验证
```
