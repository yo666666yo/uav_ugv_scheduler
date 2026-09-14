"""Small crash-safe JSON task store used for idempotency."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Dict


class JsonTaskStore:
    def __init__(self, path: Path) -> None:
        self.path = path
        self._records: Dict[str, Dict[str, Any]] = {}
        self._load()

    def _load(self) -> None:
        if not self.path.exists():
            return
        try:
            value = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"任务记录文件损坏或无法读取：{self.path}") from exc
        if not isinstance(value, dict) or not isinstance(value.get("tasks", {}), dict):
            raise RuntimeError(f"任务记录文件格式错误：{self.path}")
        self._records = dict(value.get("tasks", {}))

    def get(self, task_id: str) -> Dict[str, Any] | None:
        record = self._records.get(task_id)
        return None if record is None else dict(record)

    def put(self, task_id: str, record: Dict[str, Any]) -> None:
        self._records[task_id] = dict(record)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + ".tmp")
        with temporary.open("w", encoding="utf-8") as stream:
            json.dump({"version": 1, "tasks": self._records}, stream, ensure_ascii=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.path)

