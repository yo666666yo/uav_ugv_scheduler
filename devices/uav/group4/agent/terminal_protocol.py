"""Validation and normalization for terminal task-assignment messages."""

from __future__ import annotations

import math
from datetime import datetime
from typing import Any, Dict

from agent.contracts import TaskAssignment


class TaskValidationError(ValueError):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


def _required_text(value: Dict[str, Any], name: str) -> str:
    result = str(value.get(name, "")).strip()
    if not result:
        raise TaskValidationError("INVALID_TASK", f"缺少必要字段 {name}")
    return result


def _timestamp_ms(value: Dict[str, Any]) -> int:
    if "timestamp_ms" in value:
        try:
            return int(value["timestamp_ms"])
        except (TypeError, ValueError) as exc:
            raise TaskValidationError("INVALID_TASK", "timestamp_ms 必须是整数") from exc
    timestamp = value.get("timestamp")
    if timestamp is None:
        raise TaskValidationError("INVALID_TASK", "缺少 timestamp_ms 或 timestamp")
    try:
        parsed = datetime.fromisoformat(str(timestamp).replace("Z", "+00:00"))
    except ValueError as exc:
        raise TaskValidationError("INVALID_TASK", "timestamp 必须是 ISO 8601 时间") from exc
    if parsed.tzinfo is None:
        raise TaskValidationError("INVALID_TASK", "timestamp 必须包含时区")
    return int(parsed.timestamp() * 1000)


def _target_pose(value: Dict[str, Any]) -> Dict[str, Any]:
    pose = value.get("target_pose")
    if pose is None:
        parameters = value.get("parameters", {})
        if not isinstance(parameters, dict):
            raise TaskValidationError("INVALID_TASK", "parameters 必须是 JSON 对象")
        pose = parameters.get("target_position")
    if not isinstance(pose, dict):
        raise TaskValidationError("INVALID_TASK", "缺少 target_pose 或 parameters.target_position")
    try:
        normalized = {
            "frame_id": str(pose["frame_id"]),
            "x_m": float(pose["x_m"]),
            "y_m": float(pose["y_m"]),
            "z_m": float(pose["z_m"]),
            "yaw_rad": None if pose.get("yaw_rad") is None else float(pose["yaw_rad"]),
        }
    except (KeyError, TypeError, ValueError) as exc:
        raise TaskValidationError("INVALID_TASK", "目标坐标字段不完整或不是数字") from exc
    if normalized["frame_id"] != "uwb_map":
        raise TaskValidationError("INVALID_COORDINATE_FRAME", "坐标系必须是 uwb_map")
    numeric_values = [normalized["x_m"], normalized["y_m"], normalized["z_m"]]
    if normalized["yaw_rad"] is not None:
        numeric_values.append(normalized["yaw_rad"])
    if not all(math.isfinite(number) for number in numeric_values):
        raise TaskValidationError("INVALID_TASK", "目标坐标不能包含 NaN 或无穷值")
    return normalized


def parse_task_assignment(value: Dict[str, Any]) -> TaskAssignment:
    message_type = value.get("message_type")
    if message_type is not None and message_type != "TASK_ASSIGNMENT":
        raise TaskValidationError("UNSUPPORTED_MESSAGE", "message_type 必须是 TASK_ASSIGNMENT")
    task_type = _required_text(value, "task_type")
    if task_type != "INSPECT_TARGET":
        raise TaskValidationError("UNSUPPORTED_TASK", f"不支持任务类型 {task_type}")
    parameters = value.get("parameters", {})
    if not isinstance(parameters, dict):
        raise TaskValidationError("INVALID_TASK", "parameters 必须是 JSON 对象")
    normalized = {
        "schema_version": str(value.get("schema_version", "1.0")),
        "timestamp_ms": _timestamp_ms(value),
        "mission_id": _required_text(value, "mission_id"),
        "task_id": _required_text(value, "task_id"),
        "target_agent_id": _required_text(value, "target_agent_id"),
        "task_type": task_type,
        "target_pose": _target_pose(value),
        "parameters": dict(parameters),
    }
    normalized["parameters"].pop("target_position", None)
    if not str(normalized["parameters"].get("target_label", "")).strip():
        raise TaskValidationError("TARGET_LABEL_MISSING", "parameters.target_label 不能为空")
    return TaskAssignment.from_dict(normalized)

