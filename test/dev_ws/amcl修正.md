# AMCL 在正方形场地中的修正与调参方案

## 1. 目标

当前地图为近似正方形的四周板边，定位需要同时满足：

1. 利用 EKF 发布的 `odom -> base_link` 保持局部运动连续。
2. 利用雷达与四周板边的匹配修正平移漂移。
3. 限制粒子的航向扩散，避免进入正方形地图的 `90°/180°/270°` 对称解。
4. 尽量将累积误差分解成频繁的小修正，减少 `map -> odom` 突然跳变。

AMCL 不会修改地图内容，它修正的是 `map -> odom` 坐标变换：

```text
T_map_odom = T_map_base(AMCL) × inverse(T_odom_base(EKF))
```

## 2. 重要概念

- AMCL 始终在内部使用粒子，不需要“广播粒子”才能用板边修正。
- `/particle_cloud` 是调参时的可视化话题，不参与 TF 计算。
- `tf_broadcast: true` 表示 AMCL 发布 `map -> odom`，不是发布粒子。
- AMCL 不使用 EKF 输出的协方差调整权重，主要通过 `alpha1 ~ alpha4` 表示里程计运动的不确定性。
- `alpha` 越大，对 EKF 里程计增量的信任越低，粒子扩散越大。
- `transform_tolerance` 是 TF 时间容差，不是位移或角度的修正限幅。

## 3. 当前参数基线

当前主要参数位于：

```text
/root/D_Robot/test/dev_ws/src/origincar_system/config/nav2/nav2.yaml
```

关键基线为：

```yaml
alpha1: 0.1
alpha2: 0.1
alpha3: 0.1
alpha4: 0.1

min_particles: 500
max_particles: 2000
update_min_d: 0.02
update_min_a: 0.02

max_beams: 90
z_hit: 0.8
z_rand: 0.2
sigma_hit: 0.15
laser_likelihood_max_dist: 0.5

recovery_alpha_fast: 0.0
recovery_alpha_slow: 0.0
```

## 4. 推荐参数初值

推荐采用“航向信任 EKF，平移允许板边修正”的分轴方案。以下参数作为首次实车测试的起点，不应未经测试直接视为最终值。

```yaml
amcl:
  ros__parameters:
    use_sim_time: false

    global_frame_id: map
    odom_frame_id: odom
    base_frame_id: base_link
    scan_topic: /scan

    # 若车辆不做横向平移，保持差速模型。
    robot_model_type: nav2_amcl::DifferentialMotionModel
    tf_broadcast: true
    transform_tolerance: 0.3

    # 航向强依赖 EKF/IMU，限制正方形对称解。
    alpha1: 0.04
    alpha2: 0.04

    # 平移适度降低 EKF 权重，允许板边修正轮速漂移。
    alpha3: 0.16
    alpha4: 0.08

    min_particles: 800
    max_particles: 2500
    pf_err: 0.03
    pf_z: 0.99
    resample_interval: 1

    # 频繁执行小幅更新，减少累积后的单次大修正。
    update_min_d: 0.02
    update_min_a: 0.015

    laser_model_type: likelihood_field_prob
    laser_min_range: 0.1
    laser_max_range: 5.0
    max_beams: 120
    z_hit: 0.85
    z_rand: 0.15
    z_max: 0.05
    z_short: 0.05
    sigma_hit: 0.12
    lambda_short: 0.1
    laser_likelihood_max_dist: 0.30

    # 屏蔽圆锥、人员等与固定板边不匹配的临时障碍观测。
    do_beamskip: true
    beam_skip_distance: 0.40
    beam_skip_threshold: 0.30
    beam_skip_error_threshold: 0.90

    # 固定起点和航向，禁止全图随机恢复。
    set_initial_pose: true
    always_reset_initial_pose: true
    initial_pose:
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 0.0

    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    save_pose_rate: 0.5
```

## 5. 参数作用与调整方向

### 5.1 航向约束

```yaml
alpha1: 0.04
alpha2: 0.04
```

目标是限制粒子航向扩散，使雷达只在 EKF 预测航向附近对齐板边，避免尝试正方形的其他对称角度。

- 出现 `90°` 方向分叉：将 `alpha1/alpha2` 降至 `0.02 ~ 0.03`。
- 正常转弯后航向修正过慢：将 `alpha1` 每次增加 `0.01 ~ 0.02`，首先不要超过 `0.08`。

### 5.2 平移修正

```yaml
alpha3: 0.16
alpha4: 0.08
```

`alpha3` 比当前值更大，使粒子在 EKF 平移预测位置周围保留适量分布，板边距离观测才有空间修正轮速累计误差。

- 板边无法拉回平移漂移：`alpha3` 每次增加 `0.02`，首先不要超过 `0.22`。
- 位置左右抖动：`alpha3` 每次减少 `0.02`。
- 转弯时平移粒子扩散过大：降低 `alpha4`。

### 5.3 板边观测权重

```yaml
max_beams: 120
z_hit: 0.85
z_rand: 0.15
sigma_hit: 0.12
laser_likelihood_max_dist: 0.30
```

- `max_beams` 增加后可以更充分利用四周板边，但会提高 CPU 开销。
- `z_hit` 越大，越信任雷达与地图板边的匹配。
- `sigma_hit` 越小，板边匹配峰值越尖锐，但对地图尺寸和雷达安装误差更敏感。
- `laser_likelihood_max_dist` 减小后，远离板边的粒子更难得到高权重。

## 6. 底盘运动模型

如果任务中始终只使用前进/后退和转向，保持：

```yaml
robot_model_type: nav2_amcl::DifferentialMotionModel
```

如果底盘会实际执行 `linear.y` 横向平移，则差速模型与实际运动不匹配，应评估：

```yaml
robot_model_type: nav2_amcl::OmniMotionModel
alpha5: 0.10
```

不能在未确认车辆存在横移的情况下直接切换模型。

## 7. 调参顺序

每次只改一组参数，保留测试记录，避免无法判断哪个参数导致结果变化。

### 第一阶段：里程计与航向约束

只调整：

```text
alpha1, alpha2, alpha3, alpha4
```

进行静止、直行、左右转弯和完整路线测试。确认粒子群始终保持一个主要簇，且航向分布较窄。

### 第二阶段：雷达匹配

调整：

```text
z_hit, z_rand, sigma_hit, laser_likelihood_max_dist
```

目标是板边能稳定拉回平移漂移，且不会被圆锥或人员明显拉偏。

### 第三阶段：粒子数和计算量

调整：

```text
min_particles, max_particles, max_beams, pf_err
```

在定位效果稳定后，再平衡 CPU 开销。

## 8. 实车测试项目

每组参数至少完成以下测试：

1. 原地静止 60 秒：检查 `map -> odom` 抖动。
2. 沿单面板直行：检查板边对横向误差的修正。
3. 连续左转和右转：检查航向是否分叉或跳变。
4. 完成一圈和多圈路线：检查累积漂移能否缓慢收敛。
5. 在雷达前方放置圆锥或临时障碍：检查 beamskip 是否有效。
6. 重复启动不少于 5 次：检查固定初始位姿是否每次进入同一解。

建议记录：

```text
/scan
/amcl_pose
/particle_cloud
/odometry/filtered
/tf
/tf_static
```

## 9. 判断标准与回退条件

### 理想状态

- `/particle_cloud` 始终为一个主要簇。
- 粒子平移方向有少量扩散，航向分布较窄。
- 距离板边的长期误差逐渐减小。
- `map -> odom` 主要表现为连续小修正，不出现明显角度跳变。

### 异常现象与回退

| 异常现象 | 优先调整 |
| --- | --- |
| 粒子出现 `90°` 或 `180°` 分叉 | 降低 `alpha1/alpha2`，检查初始航向，确认未启用全局定位 |
| 板边无法修正平移漂移 | 分步增加 `alpha3` 或 `z_hit` |
| 位置持续抖动 | 降低 `alpha3`，或增大 `sigma_hit` |
| 被临时障碍拉偏 | 检查 beamskip，降低 `z_hit` 或增加 `z_rand` |
| CPU 负载过高 | 先降低 `max_beams`，再降低 `max_particles` |
| 出现单次大幅 `map -> odom` 修正 | 立即回退最近的 `alpha`/雷达权重改动，后续评估 TF 门控 |

## 10. 方案边界

参数调整可以显著降低对称解的出现概率，但无法数学上保证永不出现大范围修正，因为 AMCL 本身没有 `map -> odom` 的平移/航向硬限幅参数。

如果实车仍然发生大跳变，下一阶段应将 AMCL 的 `tf_broadcast` 关闭，使用独立 `map -> odom` 门控节点完成异常拒绝、连续帧确认和修正速率限制。在完成本文档的参数测试前，先不进行该代码改造。
