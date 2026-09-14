"""Deterministic detector used by simulation and offline tests."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict


class SimulatedTargetDetector:
    def __init__(self, *, found: bool, confidence: float = 0.0) -> None:
        self.found = found
        self.confidence = confidence if found else 0.0

    def detect(
        self,
        image_path: Path,
        target_label: str,
        min_confidence: float,
    ) -> Dict[str, Any]:
        if not image_path.is_file():
            raise FileNotFoundError(f"图片不存在：{image_path}")
        found = self.found and self.confidence >= min_confidence
        return {
            "found": found,
            "target_label": target_label,
            "confidence": self.confidence if found else 0.0,
            "model": "simulated",
        }
