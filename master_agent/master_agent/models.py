# 总 Agent 数据模型
# 严格对应《第4组-无人机无人车公共接口规范-V0.1》第 3~8 节

from __future__ import annotations

import time
import uuid
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


def now_ms() -> int:
    return int(time.time() * 1000)


def new_id(prefix: str) -> str:
    return f"{prefix}_{uuid.uuid4().hex[:8]}"


class AgentType(str, Enum):
    UAV = "UAV"
    UGV = "UGV"


class WorkState(str, Enum):
    OFFLINE = "OFFLINE"
    IDLE = "IDLE"
    RESERVED = "RESERVED"
    PREFLIGHT = "PREFLIGHT"
    EXECUTING = "EXECUTING"
    RETURNING = "RETURNING"
    MANUAL_CONTROL = "MANUAL_CONTROL"
    ERROR = "ERROR"


class ControlMode(str, Enum):
    SIM = "SIM"
    READ_ONLY = "READ_ONLY"
    REAL = "REAL"


class TaskType(str, Enum):
    INSPECT_TARGET = "INSPECT_TARGET"      # 无人机核查
    PICKUP_AND_DELIVER = "PICKUP_AND_DELIVER"  # 无人车清理


class FeedbackStatus(str, Enum):
    ACCEPTED = "ACCEPTED"
    REJECTED = "REJECTED"
    IN_PROGRESS = "IN_PROGRESS"
    IMAGE_CAPTURED = "IMAGE_CAPTURED"
    RETURNING = "RETURNING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    MANUAL_CONTROL_REQUIRED = "MANUAL_CONTROL_REQUIRED"


# ---------------------------------------------------------------- Pose（规范 4.3）
@dataclass
class Pose:
    frame_id: str = "uwb_map"
    x_m: float = 0.0
    y_m: float = 0.0
    z_m: float = 0.0
    yaw_rad: Optional[float] = None

    def distance_to(self, other: "Pose") -> float:
        """欧氏距离（米），用于最近设备调度（架构说明 4.3）。"""
        return ((self.x_m - other.x_m) ** 2 + (self.y_m - other.y_m) ** 2) ** 0.5

    def to_dict(self) -> dict:
        return {
            "frame_id": self.frame_id,
            "x_m": self.x_m,
            "y_m": self.y_m,
            "z_m": self.z_m,
            "yaw_rad": self.yaw_rad,
        }

    @classmethod
    def from_dict(cls, d: Optional[dict]) -> Optional["Pose"]:
        if not d:
            return None
        return cls(
            frame_id=d.get("frame_id", "uwb_map"),
            x_m=float(d.get("x_m", 0.0)),
            y_m=float(d.get("y_m", 0.0)),
            z_m=float(d.get("z_m", 0.0)),
            yaw_rad=d.get("yaw_rad"),
        )


# ---------------------------------------------------------------- AgentState（规范第 6 节）
@dataclass
class AgentState:
    schema_version: str = "1.0"
    timestamp_ms: int = field(default_factory=now_ms)
    agent_id: str = ""
    agent_type: AgentType = AgentType.UAV
    device_online: bool = False
    work_state: WorkState = WorkState.OFFLINE
    control_mode: ControlMode = ControlMode.SIM
    pose: Optional[Pose] = None
    position_valid: bool = False
    battery_voltage_v: Optional[float] = None
    battery_remaining_percent: Optional[float] = None
    battery_ok: bool = True
    capabilities: list[str] = field(default_factory=list)
    current_task_id: Optional[str] = None
    last_error: Optional[str] = None

    def to_dict(self) -> dict:
        return {
            "schema_version": self.schema_version,
            "timestamp_ms": self.timestamp_ms,
            "agent_id": self.agent_id,
            "agent_type": self.agent_type.value,
            "device_online": self.device_online,
            "work_state": self.work_state.value,
            "control_mode": self.control_mode.value,
            "pose": self.pose.to_dict() if self.pose else None,
            "position_valid": self.position_valid,
            "battery": {
                "voltage_v": self.battery_voltage_v,
                "remaining_percent": self.battery_remaining_percent,
                "battery_ok": self.battery_ok,
            },
            "capabilities": self.capabilities,
            "current_task_id": self.current_task_id,
            "last_error": self.last_error,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "AgentState":
        return cls(
            schema_version=d.get("schema_version", "1.0"),
            timestamp_ms=int(d.get("timestamp_ms", now_ms())),
            agent_id=d["agent_id"],
            agent_type=AgentType(d.get("agent_type", "UAV")),
            device_online=bool(d.get("device_online", False)),
            work_state=WorkState(d.get("work_state", "OFFLINE")),
            control_mode=ControlMode(d.get("control_mode", "SIM")),
            pose=Pose.from_dict(d.get("pose")),
            position_valid=bool(d.get("position_valid", False)),
            battery_voltage_v=(d.get("battery") or {}).get("voltage_v"),
            battery_remaining_percent=(d.get("battery") or {}).get("remaining_percent"),
            battery_ok=bool((d.get("battery") or {}).get("battery_ok", True)),
            capabilities=list(d.get("capabilities", [])),
            current_task_id=d.get("current_task_id"),
            last_error=d.get("last_error"),
        )


# ---------------------------------------------------------------- TaskAssignment（规范第 7 节）
@dataclass
class TaskAssignment:
    schema_version: str = "1.0"
    timestamp_ms: int = field(default_factory=now_ms)
    mission_id: str = ""
    task_id: str = field(default_factory=lambda: new_id("task"))
    target_agent_id: str = ""
    task_type: TaskType = TaskType.INSPECT_TARGET
    target_pose: Optional[Pose] = None
    parameters: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "schema_version": self.schema_version,
            "timestamp_ms": self.timestamp_ms,
            "mission_id": self.mission_id,
            "task_id": self.task_id,
            "target_agent_id": self.target_agent_id,
            "task_type": self.task_type.value,
            "target_pose": self.target_pose.to_dict() if self.target_pose else None,
            "parameters": self.parameters,
        }


# ---------------------------------------------------------------- TaskFeedback（规范第 8 节）
@dataclass
class TaskFeedback:
    schema_version: str = "1.0"
    timestamp_ms: int = field(default_factory=now_ms)
    mission_id: str = ""
    task_id: str = ""
    agent_id: str = ""
    status: FeedbackStatus = FeedbackStatus.IN_PROGRESS
    stage: Optional[str] = None
    progress_percent: Optional[int] = None
    pose: Optional[Pose] = None
    image_url: Optional[str] = None
    failure_code: Optional[str] = None
    message: Optional[str] = None

    @classmethod
    def from_dict(cls, d: dict) -> "TaskFeedback":
        return cls(
            schema_version=d.get("schema_version", "1.0"),
            timestamp_ms=int(d.get("timestamp_ms", now_ms())),
            mission_id=d.get("mission_id", ""),
            task_id=d.get("task_id", ""),
            agent_id=d.get("agent_id", ""),
            status=FeedbackStatus(d.get("status", "IN_PROGRESS")),
            stage=d.get("stage"),
            progress_percent=d.get("progress_percent"),
            pose=Pose.from_dict(d.get("pose")),
            image_url=d.get("image_url"),
            failure_code=d.get("failure_code"),
            message=d.get("message"),
        )
