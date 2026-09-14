# 第4组无人机目标识别模块

该模块只处理无人机拍摄完成后的静态图片，不读取或发送任何飞控指令。

## 接口

所有检测器实现同一个调用形式：

```python
detection = detector.detect(image_path, target_label, min_confidence)
```

返回示例：

```json
{
  "found": true,
  "target_label": "person",
  "confidence": 0.90,
  "match_count": 1,
  "model": "yolo26n.pt"
}
```

## 当前模式

- `simulated-found`：离线仿真，固定返回找到目标。
- `simulated-not-found`：离线仿真，验证找不到目标后仍能返航和降落。
- `yolo`：使用 Ultralytics YOLO 读取真实照片。

仿真成功：

```bash
python3 -m agent.uav_agent \
  --config config/drone_001.sim.json \
  --task config/sample_inspect_task.json \
  --detector simulated-found
```

仿真未找到目标：

```bash
python3 -m agent.uav_agent \
  --config config/drone_001.sim.json \
  --task config/sample_inspect_task.json \
  --detector simulated-not-found
```

真实 YOLO 模式只应在独立 Python 虚拟环境安装依赖并准备好模型后启用：

```bash
python3 -m agent.uav_agent \
  --config config/drone_001.sim.json \
  --task config/sample_inspect_task.json \
  --detector yolo \
  --model yolo26n.pt
```

当前命令仍使用模拟无人机适配器；启用 `--detector yolo` 只会更换图片识别器，不会启用真实飞控。

## 正式相机适配器

1号机当前已有 Picamera2 MJPEG 服务占用相机，因此默认复用其视频流：

```python
from uav_adapter.camera_capture import (
    RaspberryPiCameraCapture,
    RaspberryPiCameraConfig,
)
from uav_adapter.camera_enabled_adapter import CameraEnabledUAVAdapter

camera = RaspberryPiCameraCapture(
    RaspberryPiCameraConfig(
        mode="stream",
        stream_url="http://127.0.0.1:5001/video",
    )
)

adapter = CameraEnabledUAVAdapter(
    base_adapter=real_uav_adapter,
    camera=camera,
)
```

之后行为层继续统一调用：

```python
captured_path = adapter.capture_image(output_path)
```

支持三种相机模式：

- `stream`：复用 MJPEG 流，不争抢 Picamera2，1号机默认使用。
- `direct`：独占打开 Picamera2，仅在没有其他相机进程时使用。
- `auto`：先尝试流，失败后尝试直接打开相机。

相机后端只负责拍照，不包含任何飞控指令。
