# BMI088 SPI 驱动

本包让 BMI088 扩展板直接连接 RDK S100 的 40-pin SPI0，并与原车 STM32/MPU6050
链路同时运行。底盘仍发布 `/chassis/imu/data_raw` 供独立对比，系统 EKF 默认改用
BMI088 的 `/bmi088/imu/data`。

## 发布接口

| 接口 | 说明 |
|---|---|
| `/bmi088/imu/data_raw` | SI 单位的原始六轴数据，不减陀螺零偏 |
| `/bmi088/imu/data` | 减去启动/手动静止标定零偏的数据；标定期间暂停发布 |
| `/bmi088/temperature` | BMI088 温度 |
| `/bmi088/diagnostics` | 设备、频率、错误、零偏与标定状态 |
| `/bmi088/calibrate_gyro` | `std_srvs/srv/Trigger`，重新开始静止标定 |

BMI088 没有磁力计，两个 IMU 话题均将 `orientation_covariance[0]` 设置为 `-1`，不会
伪造姿态或绝对航向。

## S100 接线

必须关机、拔掉 S100 电源后接线。表中编号是 40-pin 的物理针脚号：

| BMI088 扩展板 | RDK S100 | 作用 |
|---|---|---|
| `VCC` | Pin 2（5V） | 只给扩展板板载稳压器供电 |
| `+3V` | 不接 | 这是扩展板输出，不能和 S100 电源并联 |
| `GND` | Pin 6（GND） | 共地 |
| `MOSI` | Pin 19（SPI0_MOSI） | 主机到传感器 |
| `MISO` | Pin 21（SPI0_MISO） | 传感器到主机 |
| `SCK` | Pin 23（SPI0_SCLK） | SPI 时钟 |
| `CS1` | Pin 24（SPI0_CS0） | BMI088 加速度计片选 |
| `CS2` | Pin 26（SPI0_CS1） | BMI088 陀螺仪片选 |

SPI 信号只能是 3.3V。建议线长不超过 10–15 cm，模块刚性安装在底盘中心附近，并远离
电机线和电机驱动器。默认使用 SPI Mode 0、5 MHz；代码拒绝超过 BMI088 数据手册规定的
10 MHz。

## SPI 设备前置检查

两个内部传感器共用 SCK/MOSI/MISO，但各自需要一个 Linux SPI 设备：

```bash
ls -l /dev/spidev0.0 /dev/spidev0.1
ros2 run bmi088_driver check_spi.sh
```

必须同时存在且当前用户可读写。当前这台 S100 的 4.0.4 系统在 sysfs 中检测到
`spidev0.0` 和 `spidev1.0`，但当前运行环境没有映射 `/dev/spidev*`，也没有实例化
SPI0 的第二个子设备 `spidev0.1`。`spidev1.0` 是另一条 SPI1 总线，不能在上述接线下
替代 `spidev0.1`。

当前 4.0.4 设备树的 SPI0 已设置 `num-cs = <2>`，并已复用 CSN0/CSN1 两个引脚，但只创建
了 `spidev@0`。本包提供一个只增加 `spidev@1` 的 DTBO；它不会覆盖原始 DTB。安装前先
备份工作区，确认当前没有别的 `dtbo_file_path`：

```bash
sudo apt update
sudo apt install device-tree-compiler

cd /root/D_Robot/test/dev_ws
source /opt/tros/humble/setup.bash
colcon build --symlink-install --packages-select bmi088_driver
source install/setup.bash
sudo /root/D_Robot/test/dev_ws/install/bmi088_driver/lib/bmi088_driver/\
install_s100_spi0_cs1_overlay.sh
sudo reboot
```

重启后检查：

```bash
ls -l /dev/spidev0.0 /dev/spidev0.1
ros2 run bmi088_driver check_spi.sh
```

安装脚本会先用当前运行中的设备树离线验证 overlay，并备份 `/boot/config.txt`。如果系统能
启动但需要撤销：

```bash
sudo /root/D_Robot/test/dev_ws/install/bmi088_driver/lib/bmi088_driver/\
install_s100_spi0_cs1_overlay.sh --remove
sudo reboot
```

如果 `/boot/config.txt` 已经配置其他 DTBO（例如相机 overlay），安装脚本会拒绝修改，
需要先把两个 overlay 合并。若节点在宿主机存在、但程序运行在容器中，还要把两个字符
设备都映射进容器。

## 编译

```bash
cd /root/D_Robot/test/dev_ws
source /opt/tros/humble/setup.bash
colcon build --symlink-install --packages-select bmi088_driver
source install/setup.bash
```

## 启动

接好硬件并通过 SPI 检查后：

```bash
source /root/D_Robot/test/dev_ws/install/setup.bash
ros2 launch bmi088_driver bmi088.launch.py
```

启动后让车完全静止约 3 秒。驱动会检查加速度计 ID `0x1E`、陀螺仪 ID `0x0F`，配置
回读不一致时不会发布伪数据，而是每 2 秒尝试重新连接。

`origincar_system` 的四个主启动文件会自动启动 BMI088，无需再单独执行上面的驱动启动
命令。AMCL 启动文件默认让 EKF 融合 `/bmi088/imu/data`；lidar_location 启动文件为了
A/B 对比，默认让 EKF 使用 `/chassis/imu/data_raw`。例如：

```bash
ros2 launch origincar_system test_bringup.launch.py
```

需要临时回退 MPU6050 时保持代码和两路驱动不变，只覆盖 EKF 输入：

```bash
ros2 launch origincar_system test_bringup.launch.py \
  imu_topic:=/chassis/imu/data_raw
```

手动重新标定：

```bash
ros2 service call /bmi088/calibrate_gyro std_srvs/srv/Trigger '{}'
ros2 topic echo /bmi088/diagnostics
```

没有接硬件时可以验证 ROS 接口：

```bash
ros2 launch bmi088_driver bmi088.launch.py mock_mode:=true
```

`mock_mode` 只按设定频率发布静止的模拟数据，用于检查话题、服务和诊断链路；它不会访问
SPI，也不能证明 BMI088 实物通信正常或达到该采样率。实物验证必须看到启动日志中的两个
正确芯片 ID，并对真实话题执行 `ros2 topic hz`。

## 与现有 MPU6050 同时对比

先按原方式启动底盘，再单独启动本包。两个方案的话题互不覆盖：

```bash
ros2 topic hz /chassis/imu/data_raw
ros2 topic hz /bmi088/imu/data_raw
ros2 topic hz /bmi088/imu/data
```

建议同时录包，先做 60 秒静止测试，再做相同方向的低速转圈：

```bash
ros2 bag record \
  /chassis/imu/data_raw \
  /bmi088/imu/data_raw \
  /bmi088/imu/data \
  /bmi088/temperature \
  /bmi088/diagnostics
```

公平比较必须保证两块 IMU 的坐标方向一致。默认配置直接发布扩展板丝印的 X/Y/Z。如果
需要复现当前 STM32 固件的 `ROS_X=sensor_Y, ROS_Y=-sensor_X, ROS_Z=sensor_Z` 转换，修改
`config/bmi088.yaml`：

```yaml
axis_map: [1, 0, 2]
axis_sign: [1, -1, 1]
```

`axis_map[i]` 表示输出第 `i` 个轴取传感器的哪个轴，`axis_sign[i]` 表示是否反向。模块
安装位置应通过 `base_link -> bmi088_link` 静态 TF 描述；本包不抢占现有 `gyro_link`。

## 参考资料

- [RDK S100 SPI0 与双片选说明](https://developer.d-robotics.cc/rdk_s_doc/en/Basic_Application/03_40pin_user_guide/s100/spi)
- [Bosch BMI088 数据手册](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)
- [Bosch BMI08x 官方 SensorAPI](https://github.com/boschsensortec/BMI08x_SensorAPI)
