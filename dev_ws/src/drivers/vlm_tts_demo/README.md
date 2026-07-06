# VLM TTS Demo 使用步骤

## 覆盖文件夹

把本文件夹放到：

```bash
/root/D_Robot/dev_ws/src/drivers/vlm_tts_demo
```

## 编译

```bash
cd /root/D_Robot/dev_ws
source /opt/tros/humble/setup.bash
rm -rf build/vlm_tts_demo install/vlm_tts_demo log src/drivers/vlm_tts_demo/*.egg-info
colcon build --packages-select vlm_tts_demo
source install/setup.bash
```

## 启动

```bash
cd /root/D_Robot/dev_ws
source /opt/tros/humble/setup.bash
source install/setup.bash
ros2 launch vlm_tts_demo vlm_tts_demo.launch.py playback_device:=plughw:0,0
```

## 触发

另开一个终端：

```bash
cd /root/D_Robot/dev_ws
source /opt/tros/humble/setup.bash
source install/setup.bash
ros2 topic pub --once /sign4return std_msgs/msg/Int32 "{data: -1}"
```

## 查看保存图片

```bash
ls -lh /root/D_Robot/dev_ws/src/drivers/vlm_tts_demo/captures/vlm_tts_demo_capture.jpg
```

## 查看输出

```bash
ros2 topic echo /llama_cpp_node
```

```bash
ros2 topic echo /tts_text
```

如果出现 `option --uninstall not recognized` 或 `option --editable not recognized`，先执行上面的 `rm -rf ... *.egg-info` 清理命令，再重新编译。
