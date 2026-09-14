"""Camera backends used by UAV adapters.

This module contains no flight-control operations.  It only captures one image
to a caller-provided path.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional, Protocol


class CameraCapture(Protocol):
    def capture_image(self, output_path: Path) -> Path: ...


@dataclass(frozen=True)
class RaspberryPiCameraConfig:
    mode: str = "stream"
    stream_url: str = "http://127.0.0.1:5001/video"
    width: int = 1280
    height: int = 720
    warmup_s: float = 2.0

    def __post_init__(self) -> None:
        if self.mode not in {"stream", "direct", "auto"}:
            raise ValueError("camera mode 必须是 stream、direct 或 auto")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("相机分辨率必须大于0")
        if self.warmup_s < 0:
            raise ValueError("相机预热时间不能为负数")


class RaspberryPiCameraCapture:
    """Capture a still from an existing MJPEG stream or Picamera2 directly.

    `stream` is the safe default on the current aircraft because another
    Picamera2 process already owns the camera and publishes an MJPEG stream.
    `direct` opens Picamera2 itself and therefore requires exclusive ownership.
    `auto` first tries the stream, then falls back to direct capture.
    """

    def __init__(
        self,
        config: RaspberryPiCameraConfig = RaspberryPiCameraConfig(),
        *,
        stream_capture: Optional[Callable[[str, Path], None]] = None,
        direct_capture: Optional[Callable[[Path], None]] = None,
    ) -> None:
        self.config = config
        self._stream_capture = stream_capture or self._capture_from_stream
        self._direct_capture = direct_capture or self._capture_direct

    @staticmethod
    def _validate_output_path(output_path: Path) -> None:
        if output_path.suffix.lower() not in {".jpg", ".jpeg", ".png"}:
            raise ValueError("相机输出文件必须是 .jpg、.jpeg 或 .png")

    @staticmethod
    def _capture_from_stream(stream_url: str, output_path: Path) -> None:
        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError("OpenCV 不可用，无法读取 MJPEG 相机流") from exc

        capture = cv2.VideoCapture(stream_url)
        try:
            ok, frame = capture.read()
        finally:
            capture.release()
        if not ok or frame is None:
            raise RuntimeError(f"无法从相机流读取帧：{stream_url}")
        if not cv2.imwrite(str(output_path), frame):
            raise RuntimeError(f"无法保存相机图片：{output_path}")

    def _capture_direct(self, output_path: Path) -> None:
        try:
            from picamera2 import Picamera2
        except ImportError as exc:
            raise RuntimeError("Picamera2 不可用，无法直接访问树莓派相机") from exc

        camera = Picamera2()
        started = False
        try:
            configuration = camera.create_still_configuration(
                main={"size": (self.config.width, self.config.height)}
            )
            camera.configure(configuration)
            camera.start()
            started = True
            time.sleep(self.config.warmup_s)
            camera.capture_file(str(output_path))
        finally:
            if started:
                camera.stop()
            camera.close()

    def capture_image(self, output_path: Path) -> Path:
        output_path = Path(output_path)
        self._validate_output_path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        temporary_path = output_path.with_name(
            f".{output_path.stem}.capturing{output_path.suffix}"
        )

        try:
            if self.config.mode == "stream":
                self._stream_capture(self.config.stream_url, temporary_path)
            elif self.config.mode == "direct":
                self._direct_capture(temporary_path)
            else:
                try:
                    self._stream_capture(self.config.stream_url, temporary_path)
                except Exception:
                    self._direct_capture(temporary_path)

            if not temporary_path.is_file() or temporary_path.stat().st_size == 0:
                raise RuntimeError("相机返回了空图片")
            temporary_path.replace(output_path)
            return output_path
        finally:
            if temporary_path.exists():
                temporary_path.unlink()
