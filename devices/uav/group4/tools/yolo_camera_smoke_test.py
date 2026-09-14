#!/usr/bin/env python3
"""Real Raspberry Pi camera -> YOLOTargetDetector smoke test.

This script does NOT import an adapter or issue UAV flight-control commands.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path

# Support both `python -m tools.yolo_camera_smoke_test` and direct execution
# from the project root without requiring callers to set PYTHONPATH manually.
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from perception.yolo_target_detector import YOLOTargetDetector
from uav_adapter.camera_capture import (
    RaspberryPiCameraCapture,
    RaspberryPiCameraConfig,
)


def capture_image(
    output_path: Path,
    *,
    warmup_s: float = 2.0,
    width: int = 1280,
    height: int = 720,
    mode: str = "stream",
    stream_url: str = "http://127.0.0.1:5001/video",
) -> Path:
    """Capture through the same camera backend used by UAV adapters."""
    camera = RaspberryPiCameraCapture(
        RaspberryPiCameraConfig(
            mode=mode,
            stream_url=stream_url,
            width=width,
            height=height,
            warmup_s=warmup_s,
        )
    )
    return camera.capture_image(output_path)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="树莓派真实相机 + YOLOTargetDetector 冒烟测试"
    )
    parser.add_argument("--target-label", default="person")
    parser.add_argument("--min-confidence", type=float, default=0.45)
    parser.add_argument("--model", default="yolo26n.pt")
    parser.add_argument(
        "--output-directory",
        type=Path,
        default=Path("logs/yolo_smoke_test"),
    )
    parser.add_argument(
        "--image",
        type=Path,
        default=None,
        help="检测已有图片；不提供时调用树莓派相机",
    )
    parser.add_argument("--warmup-s", type=float, default=2.0)
    parser.add_argument(
        "--camera-mode",
        choices=("stream", "direct", "auto"),
        default="stream",
        help="当前1号机默认复用现有MJPEG流，避免争抢Picamera2",
    )
    parser.add_argument(
        "--stream-url",
        default="http://127.0.0.1:5001/video",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not 0.0 <= args.min_confidence <= 1.0:
        raise ValueError("--min-confidence 必须在 0 到 1 之间")

    if args.image is not None:
        image_path = args.image
        if not image_path.is_file():
            raise FileNotFoundError(f"指定图片不存在：{image_path}")
        capture_source = "existing_image"
    else:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        image_path = args.output_directory / f"camera_{timestamp}.jpg"
        print(f"[1/3] 正在通过树莓派相机拍照：{image_path}", flush=True)
        capture_image(
            image_path,
            warmup_s=args.warmup_s,
            mode=args.camera_mode,
            stream_url=args.stream_url,
        )
        capture_source = "raspberry_pi_camera"
        print("[1/3] 拍照成功", flush=True)

    print(f"[2/3] 正在加载 YOLO 模型：{args.model}", flush=True)
    detector = YOLOTargetDetector(model_path=args.model)
    print("[2/3] YOLO 模型加载成功", flush=True)

    print(
        f"[3/3] 开始目标检测：{args.target_label}, "
        f"threshold={args.min_confidence}",
        flush=True,
    )
    started_at = time.perf_counter()
    result = detector.detect(
        image_path=image_path,
        target_label=args.target_label,
        min_confidence=args.min_confidence,
    )
    inference_time_s = time.perf_counter() - started_at
    output = {
        "ok": True,
        "capture_source": capture_source,
        "image_path": str(image_path),
        "model": args.model,
        "target_label": args.target_label,
        "min_confidence": args.min_confidence,
        "inference_time_s": inference_time_s,
        "detection": result,
    }
    print("\n========== YOLO TEST RESULT ==========")
    print(json.dumps(output, ensure_ascii=False, indent=2))
    print("======================================")
    if result["found"]:
        print("\nPASS: YOLO 已确认目标出现在照片中。")
        return 0
    print("\nNOT FOUND: 程序运行正常，但照片中没有达到阈值的目标。")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
