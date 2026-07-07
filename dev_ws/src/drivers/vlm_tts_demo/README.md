# API VLM + TTS Demo

这个包替代旧的本地 `hobot_llamacpp` VLM 流程：板端只抓图、请求远端视觉 API、发布 TTS 文本。

默认配置文件：

```bash
/root/D_Robot/dev_ws/src/drivers/vlm_tts_demo/config/vlm_api.env
```

这个文件保存 `VLM_API_ENDPOINT`、`VLM_API_MODEL`、`VLM_API_KEY`，已被 `.gitignore` 忽略。

## 编译

```bash
cd /root/D_Robot/dev_ws
source /opt/tros/humble/setup.bash
colcon build --packages-select vlm_tts_demo --symlink-install
source install/setup.bash
```

## 启动

```bash
ros2 launch vlm_tts_demo vlm_tts_demo.launch.py \
  launch_camera:=false \
  ding_enable:=false
```

## 比赛触发流程

抓图：

```bash
ros2 topic pub --once /sign4return std_msgs/msg/Int32 "{data: -1}"
```

调用 API 并播报：

```bash
ros2 topic pub --once /sign4return std_msgs/msg/Int32 "{data: 0}"
```

抓图会保存到：

```bash
/root/D_Robot/dev_ws/install/vlm_tts_demo/share/vlm_tts_demo/captures/vlm_tts_demo_capture.jpg
```

## 单次触发测试

如果想发 `-1` 后立刻识别：

```bash
ros2 launch vlm_tts_demo vlm_tts_demo.launch.py \
  launch_camera:=false \
  infer_on_capture:=true \
  ding_enable:=false
```

然后发布：

```bash
ros2 topic pub --once /sign4return std_msgs/msg/Int32 "{data: -1}"
```
