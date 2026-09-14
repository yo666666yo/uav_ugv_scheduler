"""Fail-closed low-risk hover behavior.

The behavior is adapter-agnostic and intentionally performs no target
navigation or camera work.  A real adapter must enforce its own control gate.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict


@dataclass(frozen=True)
class HoverTestConfig:
    height_m: float = 0.8
    duration_s: float = 5.0
    altitude_timeout_s: float = 8.0
    landed_height_tolerance_m: float = 0.10


class HoverTest:
    def __init__(self, adapter: Any, config: HoverTestConfig):
        self.adapter = adapter
        self.config = config

    def preflight(self) -> None:
        state = self.adapter.read_state()
        if not state.get("device_online"):
            raise RuntimeError("DEVICE_OFFLINE")
        if not state.get("position_valid"):
            raise RuntimeError("POSITION_INVALID")
        if state.get("work_state") != "IDLE":
            raise RuntimeError("DEVICE_BUSY")
        if not state.get("battery", {}).get("battery_ok"):
            raise RuntimeError("LOW_BATTERY")
        if state.get("armed") is True:
            raise RuntimeError("ALREADY_ARMED")
        if getattr(self.adapter, "is_simulation", True) is False and not state.get("real_control_enabled"):
            raise RuntimeError("CONTROL_DISABLED")
        if getattr(self.adapter, "is_simulation", True) is False and not state.get("control_ready", False):
            raise RuntimeError("CONTROL_PATH_OFFLINE")

    def execute(self) -> Dict[str, Any]:
        self.preflight()
        state = self.adapter.read_state()
        home = state["pose"]
        self.adapter.set_work_state("EXECUTING")
        airborne = False
        try:
            self.adapter.takeoff(self.config.height_m)
            airborne = True
            deadline = __import__('time').monotonic() + self.config.altitude_timeout_s
            while getattr(self.adapter, 'is_simulation', True) or __import__('time').monotonic() < deadline:
                if getattr(self.adapter, 'is_simulation', True): break
                s = self.adapter.read_state(); z = (s.get('position') or s.get('pose') or {}).get('z_m')
                if z is not None and float(z) >= self.config.height_m * 0.8:
                    break
                self.adapter.hold(0.1)
            else:
                raise RuntimeError("起飞未确认：高度未达到目标高度的80%")
            self.adapter.hold(self.config.duration_s)
            self.adapter.land()
            deadline = __import__('time').monotonic() + self.config.altitude_timeout_s
            while getattr(self.adapter, 'is_simulation', True) or __import__('time').monotonic() < deadline:
                if getattr(self.adapter, 'is_simulation', True): break
                s = self.adapter.read_state(); z = (s.get('position') or s.get('pose') or {}).get('z_m')
                if z is not None and abs(float(z)) <= self.config.landed_height_tolerance_m and s.get('armed') is False:
                    break
                self.adapter.hold(0.1)
            else:
                raise RuntimeError("降落未确认：高度或 armed 状态未恢复")
            airborne = False
            self.adapter.set_work_state("IDLE")
            return {"ok": True, "task_type": "HOVER_TEST", "height_m": self.config.height_m,
                    "duration_s": self.config.duration_s, "home": home}
        except Exception:
            if airborne:
                self.adapter.request_manual_takeover("hover test failed")
            self.adapter.set_work_state("IDLE")
            raise
