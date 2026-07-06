# Daily Updating

## 2025.06.04（xzt） — 定位方案迁移：AMCL → SLAM Toolbox

### 背景
- AMCL（粒子滤波）和 SLAM Toolbox localization（scan-matching）同时存在会发布冲突的 `map → odom` TF，必须二选一
- SLAM Toolbox 的 scan-matching + 回环检测比 AMCL 纯粒子滤波鲁棒性更好
- 选定 SLAM Toolbox localization 模式作为唯一定位方案

### 改动内容

#### 新建
| 文件 | 说明 |
|------|------|
| `src/localization/slam/README.md` | SLAM Toolbox 建图与定位完整操作流程 |

#### 删除
| 文件 | 原因 |
|------|------|
| `config/nav2/slam_toolbox.yaml` | 残留配置，scan_topic 错误指向 `/lidar/scan`，不被任何 launch 引用 |

#### 修改
| 文件 | 改动 |
|------|------|
| `config/localization/slam_toolbox.yaml` | 添加 `map_file_name` + `map_start_pose` 参数，localization 模式启动时自动加载序列化地图 |
| `config/nav2/nav2.yaml` | 删除 `amcl` / `amcl_map_client` / `amcl_rclcpp_node` 三个配置块 |
| `src/origincar_nav2/launch/origincar_nav2.launch.py` | 集成 `slam.launch.py`，传入 `use_amcl:='false'` 禁用 AMCL |
| `scripts/switch_mode.sh` | localization 模式自动设置/恢复 map_file_name，mapping 模式自动注释 |

### 迁移后的架构
```
map ──(SLAM Toolbox scan-matching)──> odom ──(EKF wheel+IMU)──> base_link
```
- AMCL 已移除，不再有 TF 冲突
- map_server 保留，继续为 global costmap 提供 pgm+yaml 静态地图
- SLAM Toolbox localization 模式需要 `.posegraph` + `.data` 序列化地图文件

### 启动方式（3 终端）
```bash
# T1: 驱动
ros2 launch driver_bringup drivers.launch.py
# T2: EKF
ros2 launch state_estimation ekf.launch.py
# T3: Nav2 + SLAM Toolbox 定位
ros2 launch origincar_nav2 origincar_nav2.launch.py
```

待定：
ekf调优
costmap调优
teb规划器代替mppi（或者mppi调参）
 
6.4任务
完成全局导航任务

---

## 2025.06.09（xzt） — 启动时序修正 + 日志净化 + 问题诊断

### 问题1: 启动瞬间地图 Y 轴偏移

#### 根因
1. **底盘里程计 X/Y 缩放因子不对称** — `origincar_base.cpp:287-288`，X 轴缩放 1.03，Y 轴缩放 1.125，Y 轴累积误差比 X 轴快 ~9%
2. **SLAM Toolbox 与激光雷达同时启动** — `racing.launch.py` 中 EKF/SLAM 的 `period=0.0`（注释写 6s 但代码未实施），SLAM 启动时 lidar 还没产出 scan，先发布恒等 `map→odom`，等 scan 到达后 scan-matching 跳变产生 Y 轴偏移
3. **waypoint_cruiser 在 SLAM 未收敛时发布 initial_pose (0,0,0)** — 可能与 SLAM 定位冲突
4. **EKF 对 X/Y 的过程噪声假设相同 (0.05)** — 但实际 Y 轴噪声更大

#### 修复
| 优先级 | 文件 | 改动 |
|--------|------|------|
| **P0** | `racing.launch.py` | EKF/SLAM: `period=0.0→6.0`，Nav2/cruiser: `period=3.0→10.0`，等 lidar 稳定 + SLAM 定位收敛后再启动 |
| **P0** | `origincar_base.cpp:288` | `1.125→1.03`（待标定确认准确值），统一 X/Y 缩放 |
| P1 | `waypoint_cruiser.py:138` | `publish_initial_pose()` 前加 3~5s 延时，等 SLAM 完成首次定位 |
| P2 | `ekf.yaml:106` | `process_noise_covariance[1]` (y): `0.05→0.08` |
| P2 | `waypoint_cruiser.py` | 订阅 `/map` topic，收到第一帧地图后再发 initial_pose |

### 问题2: RDK S100 最后航点巡航失败

#### 根因
1. **MPPI 计算负载超出 RDK S100 实时能力** — `batch_size:1500 × time_steps:80 × 30Hz` = 每周期 12 万次状态传播，ARM 板可能丢帧
2. **Progress Checker 对低速终点不友好** — `required_movement_radius:0.5 + movement_time_allowance:10s`，最后接近目标时速度低，10s 移不够 0.5m 就判失败
3. **代价地图终点附近障碍物残留** — `obstacle_max_range:3.5~5.0`，行驶中标记的障碍物回到终点时仍在 costmap 中
4. **18V 供电可能随电池下降导致电机力矩不足** — 指令速度达不到，progress_checker 超时
5. **恢复行为可能适得其反** — DriveOnHeading 前进 0.5m + BackUp 后退 0.3m，离终点很近时反而推远

#### 修复建议
| 优先级 | 文件 | 改动 |
|--------|------|------|
| **P0** | `nav2.yaml:106-108` | `batch_size:1500→500`, `time_steps:80→50`, `controller_frequency:30→15` |
| **P0** | `nav2.yaml:77-79` | `required_movement_radius:0.5→0.15`, `movement_time_allowance:10→30` |
| P1 | `nav2.yaml:85-86` | `xy_goal_tolerance:0.3→0.5`（最后一个点是起点，不需要极致精度） |
| P1 | `nav2.yaml:236-243` | `obstacle_max_range` 缩小（local: 3.5→2.5, global: 5.0→3.0） |
| P1 | `nav2.yaml:222-223` | `inflation_radius:0.15→0.10`, `cost_scaling_factor:5.0→8.0` |
| P2 | `waypoint_cruiser.py` | 订阅 `/chassis/power_voltage`，电压低于阈值时暂停或提前结束 |
| P2 | `slam_toolbox.yaml:31` | `minimum_time_interval:0.2→0.5`（5Hz→2Hz），减 CPU 负载 |
| P2 | `ackermann_navigate_w_recovery.xml:8` | `number_of_retries:6→3` |

### 改动记录

#### 修改
| 文件 | 改动 |
|------|------|
| `racing.launch.py` | ① 启动时序修正: Phase2 `6.0→4.0`, Phase3a `0.0→6.0`, Phase3b `3.0→10.0` ② 日志分级: t=3s 设 `RCUTILS_LOGGING_SEVERITY_THRESHOLD=WARN` 抑制 Nav2/SLAM/EKF 刷屏, cruiser 单独 `--log-level info` 保持可见 |
| `ekf.yaml:21` | `print_diagnostics: true→false`，关闭终端诊断刷屏 |

### 启动时序（修正后）
```
t=0s   底盘 + lidar + camera + 静态TF  (INFO)
t=3s   全局日志降为 WARN
t=4s   QR + VLM                         (WARN)
t=6s   EKF + SLAM Toolbox               (WARN, lidar 已有 6s scan 积累)
t=10s  Nav2 + waypoint_cruiser          (Nav2=WARN, cruiser=INFO)
```

### 待定
- [ ] 里程计 X/Y 缩放因子标定（`origincar_base.cpp`）
- [ ] MPPI 参数在 RDK S100 上实际调优
- [ ] 代价地图参数实测验证
- [ ] 电池电压监控集成
