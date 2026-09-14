"""Composition wrapper that adds a real camera to an existing UAV adapter."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from uav_adapter.camera_capture import CameraCapture


class CameraEnabledUAVAdapter:
    """Delegate UAV operations unchanged and override only `capture_image`."""

    def __init__(self, base_adapter: Any, camera: CameraCapture) -> None:
        self._base_adapter = base_adapter
        self._camera = camera

    @property
    def is_simulation(self) -> bool:
        return bool(self._base_adapter.is_simulation)

    def capture_image(self, output_path: Path) -> Path:
        return self._camera.capture_image(output_path)

    def __getattr__(self, name: str) -> Any:
        return getattr(self._base_adapter, name)
