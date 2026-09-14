"""Pure simulation adapter.

This module never opens a serial port or network connection and cannot control a
real aircraft.  It exists so the behavior and agent layers can be tested safely.
"""

from __future__ import annotations

import base64
import copy
from pathlib import Path
from typing import Any, Dict, List

from agent.contracts import Pose


_ONE_PIXEL_PNG = base64.b64decode(
    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNk+A8AAQUBAScY42YAAAAASUVORK5CYII="
)


class SimulatedUAVAdapter:
    """Deterministic UAV simulator implementing the planned adapter primitives."""

    is_simulation = True

    def __init__(
        self,
        *,
        agent_id: str,
        initial_pose: Pose,
        battery_ok: bool = True,
    ) -> None:
        self.agent_id = agent_id
        self._pose = initial_pose
        self._state: Dict[str, Any] = {
            "agent_id": agent_id,
            "agent_type": "UAV",
            "device_online": True,
            "work_state": "IDLE",
            "control_mode": "SIM",
            "position_valid": True,
            "battery": {
                "voltage_v": 4.1,
                "remaining_percent": 80,
                "battery_ok": battery_ok,
            },
            "armed": False,
        }
        self.action_log: List[Dict[str, Any]] = []

    def read_state(self) -> Dict[str, Any]:
        state = copy.deepcopy(self._state)
        state["pose"] = self._pose.to_dict()
        return state

    def set_work_state(self, work_state: str) -> None:
        self._state["work_state"] = work_state

    def takeoff(self, height_m: float) -> None:
        if height_m <= 0:
            raise ValueError("起飞高度必须大于0")
        self._state["armed"] = True
        self._pose = Pose(
            self._pose.frame_id,
            self._pose.x_m,
            self._pose.y_m,
            float(height_m),
            self._pose.yaw_rad,
        )
        self.action_log.append({"action": "takeoff", "height_m": height_m})

    def goto_pose(self, pose: Pose) -> None:
        if pose.frame_id != self._pose.frame_id:
            raise ValueError("模拟器不支持跨坐标系移动")
        self._pose = pose
        self.action_log.append({"action": "goto_pose", "pose": pose.to_dict()})

    def hold(self, duration_s: float) -> None:
        if duration_s < 0:
            raise ValueError("悬停时间不能为负数")
        self.action_log.append({"action": "hold", "duration_s": duration_s})

    def capture_image(self, output_path: Path) -> Path:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_bytes(_ONE_PIXEL_PNG)
        self.action_log.append({"action": "capture_image", "path": str(output_path)})
        return output_path

    def return_to_pose(self, home_pose: Pose, cruise_height_m: float) -> None:
        cruise_pose = Pose(
            home_pose.frame_id,
            home_pose.x_m,
            home_pose.y_m,
            float(cruise_height_m),
            home_pose.yaw_rad,
        )
        self._pose = cruise_pose
        self.action_log.append(
            {"action": "return_to_pose", "pose": cruise_pose.to_dict()}
        )

    def land(self) -> None:
        self._pose = Pose(
            self._pose.frame_id,
            self._pose.x_m,
            self._pose.y_m,
            0.0,
            self._pose.yaw_rad,
        )
        self._state["armed"] = False
        self.action_log.append({"action": "land"})

    def request_manual_takeover(self, reason: str) -> None:
        self._state["work_state"] = "MANUAL_CONTROL"
        self.action_log.append({"action": "manual_takeover", "reason": reason})
