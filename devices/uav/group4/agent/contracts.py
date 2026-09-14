"""Shared in-process contracts used by the UAV agent and behavior script."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Dict, Optional


@dataclass(frozen=True)
class Pose:
    frame_id: str
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: Optional[float]

    @classmethod
    def from_dict(cls, value: Dict[str, Any]) -> "Pose":
        return cls(
            frame_id=str(value["frame_id"]),
            x_m=float(value["x_m"]),
            y_m=float(value["y_m"]),
            z_m=float(value["z_m"]),
            yaw_rad=None if value.get("yaw_rad") is None else float(value["yaw_rad"]),
        )

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class TaskAssignment:
    schema_version: str
    timestamp_ms: int
    mission_id: str
    task_id: str
    target_agent_id: str
    task_type: str
    target_pose: Pose
    parameters: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, value: Dict[str, Any]) -> "TaskAssignment":
        return cls(
            schema_version=str(value.get("schema_version", "1.0")),
            timestamp_ms=int(value["timestamp_ms"]),
            mission_id=str(value["mission_id"]),
            task_id=str(value["task_id"]),
            target_agent_id=str(value["target_agent_id"]),
            task_type=str(value["task_type"]),
            target_pose=Pose.from_dict(value["target_pose"]),
            parameters=dict(value.get("parameters", {})),
        )


@dataclass
class TaskFeedback:
    schema_version: str
    timestamp_ms: int
    mission_id: str
    task_id: str
    agent_id: str
    status: str
    stage: str
    progress_percent: int
    pose: Optional[Pose]
    image_url: Optional[str]
    failure_code: Optional[str]
    message: str
    sequence: int
    detection: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        result = asdict(self)
        if self.pose is not None:
            result["pose"] = self.pose.to_dict()
        return result
