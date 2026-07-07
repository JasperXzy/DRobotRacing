# API 图生文 + TTS Demo

默认读取同目录的本地配置：

```bash
/root/D_Robot/test/api_vlm_tts_demo/api_vlm_tts_demo.env
```

## 1. 启动 TTS

```bash
cd /root/D_Robot/dev_ws
source /opt/tros/humble/setup.bash
source install/setup.bash

ros2 run hobot_tts hobot_tts --ros-args \
  -p playback_device:=plughw:0,0 \
  -p topic_sub:=/tts_text
```

## 2. 捕捉相机图片并语音播报

```bash
cd /root/D_Robot
source /opt/tros/humble/setup.bash
source dev_ws/install/setup.bash

python3 test/api_vlm_tts_demo/api_vlm_tts_demo.py \
  --camera-topic /camera/image \
  --start-notice ""
```

## 3. 捕捉相机图片但不播报

```bash
cd /root/D_Robot
source /opt/tros/humble/setup.bash
source dev_ws/install/setup.bash

python3 test/api_vlm_tts_demo/api_vlm_tts_demo.py \
  --camera-topic /camera/image \
  --no-tts
```

## 4. 用熊猫图片测试

```bash
cd /root/D_Robot

python3 test/api_vlm_tts_demo/api_vlm_tts_demo.py \
  --image test/hobot_vlm_test/config/panda.jpg \
  --no-tts
```
