"""Optional Ultralytics YOLO detector; it never issues flight-control commands."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Optional


class YOLOTargetDetector:
    """Confirm whether one requested class is present in a captured image."""

    def __init__(self, model_path: str = "yolo26n.pt", *, model: Optional[Any] = None):
        if model is not None:
            self.model = model
            self.model_path = model_path
            return
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "未安装 ultralytics；请在独立虚拟环境中安装后再启用真实 YOLO"
            ) from exc
        self.model_path = model_path
        self.model = YOLO(model_path)

    def detect(
        self,
        image_path: Path,
        target_label: str,
        min_confidence: float,
    ) -> Dict[str, Any]:
        if not image_path.is_file():
            raise FileNotFoundError(f"图片不存在：{image_path}")
        label = target_label.strip().lower()
        if not label:
            raise ValueError("target_label 不能为空")
        if not 0.0 <= min_confidence <= 1.0:
            raise ValueError("min_confidence 必须在0到1之间")

        results = self.model.predict(source=str(image_path), verbose=False)
        best_confidence = 0.0
        matches = []
        for result in results:
            names = result.names
            boxes = result.boxes
            if boxes is None:
                continue
            for box in boxes:
                class_id = int(box.cls.item())
                confidence = float(box.conf.item())
                class_name = str(names[class_id])
                if class_name.strip().lower() == label:
                    best_confidence = max(best_confidence, confidence)
                    if confidence >= min_confidence:
                        matches.append(
                            {
                                "class_id": class_id,
                                "label": class_name,
                                "confidence": confidence,
                            }
                        )
        return {
            "found": bool(matches),
            "target_label": target_label,
            "confidence": best_confidence,
            "match_count": len(matches),
            "matches": matches,
            "model": self.model_path,
        }
