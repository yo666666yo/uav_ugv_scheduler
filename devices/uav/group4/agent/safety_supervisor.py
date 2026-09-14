"""State-based UAV fault detection and idempotent safety decisions.

This module contains no ROS, MAVLink or DDS code.  It turns unified device
state into an explicit decision.  The caller remains responsible for executing
that decision through a safety-gated adapter and reporting the result.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, Optional


class SafetyAction(str, Enum):
    NONE = "NONE"
    WARN = "WARN"
    REJECT_NEW_TASK = "REJECT_NEW_TASK"
    HOLD_POSITION = "HOLD_POSITION"
    RETURN_HOME = "RETURN_HOME"
    LAND_NOW = "LAND_NOW"
    MANUAL_TAKEOVER = "MANUAL_TAKEOVER"


@dataclass(frozen=True)
class SafetyDecision:
    action: SafetyAction
    code: str
    severity: str
    message: str
    latched: bool = False

    @property
    def requires_action(self) -> bool:
        return self.action not in (SafetyAction.NONE, SafetyAction.WARN)


@dataclass(frozen=True)
class SafetyThresholds:
    task_min_battery_percent: float = 40.0
    return_home_battery_percent: float = 25.0
    critical_land_battery_percent: float = 15.0
    battery_stale_s: float = 3.0
    position_warning_s: float = 0.5
    position_failure_s: float = 2.0
    maximum_position_jump_m: float = 0.5
    position_recovery_stable_s: float = 2.0
    coordinator_warning_s: float = 1.0
    coordinator_failure_s: float = 3.0
    fcu_warning_s: float = 1.0
    fcu_failure_s: float = 3.0
    allow_emergency_land_without_position: bool = False

    def __post_init__(self) -> None:
        if not (
            0 <= self.critical_land_battery_percent
            < self.return_home_battery_percent
            < self.task_min_battery_percent
            <= 100
        ):
            raise ValueError("电量阈值必须满足 critical < return_home < task_min")
        for name, value in self.__dict__.items():
            if name == "allow_emergency_land_without_position":
                continue
            if not math.isfinite(float(value)) or float(value) < 0:
                raise ValueError(f"安全阈值 {name} 必须是非负有限数")


class SafetySupervisor:
    """Evaluate telemetry with deterministic priority and action latching."""

    def __init__(self, thresholds: SafetyThresholds = SafetyThresholds()) -> None:
        self.thresholds = thresholds
        self._latched_action: Optional[SafetyDecision] = None
        self._last_position: Optional[tuple[float, float, float]] = None
        self._position_recovery_since_s: Optional[float] = None

    @staticmethod
    def _number(value: Any) -> Optional[float]:
        try:
            result = float(value)
        except (TypeError, ValueError):
            return None
        return result if math.isfinite(result) else None

    @classmethod
    def _age(cls, state: Dict[str, Any], key: str) -> float:
        value = cls._number(state.get(key))
        return math.inf if value is None else max(0.0, value)

    @classmethod
    def _position(cls, state: Dict[str, Any]) -> Optional[tuple[float, float, float]]:
        pose = state.get("pose") or state.get("position") or {}
        values = tuple(cls._number(pose.get(key)) for key in ("x_m", "y_m", "z_m"))
        if any(value is None for value in values):
            return None
        return values  # type: ignore[return-value]

    def reset_after_safe_landing(self) -> None:
        """Clear one-shot fault state only after landing has been confirmed."""
        self._latched_action = None
        self._last_position = None
        self._position_recovery_since_s = None

    def _latch(self, decision: SafetyDecision) -> SafetyDecision:
        if self._latched_action is not None:
            previous = self._latched_action
            return SafetyDecision(
                previous.action,
                previous.code,
                previous.severity,
                previous.message,
                latched=True,
            )
        self._latched_action = decision
        return decision

    def _position_health(self, state: Dict[str, Any], now_s: float) -> tuple[bool, str]:
        position = self._position(state)
        declared_valid = bool(state.get("position_valid", position is not None))
        age = self._age(state, "position_age_s")
        if not declared_valid or position is None:
            self._position_recovery_since_s = None
            return False, "POSITION_INVALID"
        if age >= self.thresholds.position_failure_s:
            self._position_recovery_since_s = None
            return False, "POSITION_STALE"

        if self._last_position is not None:
            jump = math.dist(self._last_position, position)
            if jump > self.thresholds.maximum_position_jump_m:
                self._position_recovery_since_s = None
                self._last_position = position
                return False, "POSITION_JUMP"
        self._last_position = position

        # A recovered stream must remain stable before it can be trusted for RTH.
        if self._position_recovery_since_s is None:
            self._position_recovery_since_s = now_s
        stable_for = now_s - self._position_recovery_since_s
        if stable_for < self.thresholds.position_recovery_stable_s:
            return False, "POSITION_RECOVERING"
        return True, "POSITION_OK"

    def evaluate(
        self,
        state: Dict[str, Any],
        *,
        now_s: float,
        airborne: bool,
        accepting_new_task: bool = False,
    ) -> SafetyDecision:
        """Return the highest-priority safety decision for one state sample."""
        position_ok, position_code = self._position_health(state, now_s)
        battery = state.get("battery", {})
        battery_percent = self._number(battery.get("remaining_percent"))
        battery_age = self._age(state, "battery_age_s")
        fcu_age = self._age(state, "fcu_link_age_s")
        coordinator_age = self._age(state, "coordinator_link_age_s")

        # Highest priority: the command channel is gone.  We cannot truthfully
        # claim that a return/land command reached the flight controller.
        if fcu_age >= self.thresholds.fcu_failure_s:
            decision = SafetyDecision(
                SafetyAction.MANUAL_TAKEOVER if airborne else SafetyAction.REJECT_NEW_TASK,
                "FCU_LINK_LOST",
                "L3" if airborne else "L2",
                "飞控通信中断，控制指令无法确认送达",
            )
            return self._latch(decision) if airborne else decision

        # Critical battery may leave no time for a full return.  Blind landing
        # without position is opt-in and stays disabled until vendor validation.
        if battery_percent is not None and battery_percent <= self.thresholds.critical_land_battery_percent:
            if airborne and (position_ok or self.thresholds.allow_emergency_land_without_position):
                return self._latch(
                    SafetyDecision(SafetyAction.LAND_NOW, "CRITICAL_BATTERY", "L4", "电量达到紧急降落阈值")
                )
            decision = SafetyDecision(
                SafetyAction.MANUAL_TAKEOVER if airborne else SafetyAction.REJECT_NEW_TASK,
                "CRITICAL_BATTERY_POSITION_UNSAFE" if airborne else "CRITICAL_BATTERY",
                "L4" if airborne else "L2",
                "电量极低且当前不满足已验证的自动降落条件",
            )
            return self._latch(decision) if airborne else decision

        # Position loss forbids coordinate-based return.
        if airborne and not position_ok:
            if position_code in ("POSITION_RECOVERING",) or self._age(state, "position_age_s") < self.thresholds.position_failure_s:
                return SafetyDecision(
                    SafetyAction.HOLD_POSITION,
                    position_code,
                    "L1",
                    "暂停下发新目标，等待定位连续稳定",
                )
            return self._latch(
                SafetyDecision(
                    SafetyAction.MANUAL_TAKEOVER,
                    position_code,
                    "L3",
                    "定位失效，禁止按坐标盲目返航",
                )
            )

        # Low battery during flight aborts the mission and returns while the
        # position and command link are still usable.
        if battery_percent is not None and battery_percent <= self.thresholds.return_home_battery_percent:
            if airborne:
                return self._latch(
                    SafetyDecision(SafetyAction.RETURN_HOME, "LOW_BATTERY_RETURN", "L2", "电量不足，中止任务并返航")
                )
            return SafetyDecision(SafetyAction.REJECT_NEW_TASK, "LOW_BATTERY", "L2", "电量不足，禁止起飞")

        # Losing the coordinator is different from losing the FCU.  The local
        # agent can still return autonomously if position and FCU links are good.
        if coordinator_age >= self.thresholds.coordinator_failure_s:
            if airborne:
                return self._latch(
                    SafetyDecision(SafetyAction.RETURN_HOME, "COORDINATOR_LINK_LOST", "L2", "总 Agent 通信中断，本机自主返航")
                )
            return SafetyDecision(SafetyAction.REJECT_NEW_TASK, "COORDINATOR_LINK_LOST", "L2", "总 Agent 离线，拒绝新任务")

        if accepting_new_task:
            if battery_age >= self.thresholds.battery_stale_s or battery_percent is None:
                return SafetyDecision(SafetyAction.REJECT_NEW_TASK, "BATTERY_STATUS_UNKNOWN", "L2", "电量状态未知，禁止起飞")
            if battery_percent < self.thresholds.task_min_battery_percent:
                return SafetyDecision(SafetyAction.REJECT_NEW_TASK, "LOW_BATTERY", "L2", "电量低于任务准入阈值")
            if not position_ok:
                return SafetyDecision(SafetyAction.REJECT_NEW_TASK, position_code, "L2", "定位未连续稳定，禁止起飞")

        warnings = []
        if battery_age >= self.thresholds.battery_stale_s:
            warnings.append("BATTERY_STATUS_STALE")
        if self._age(state, "position_age_s") >= self.thresholds.position_warning_s:
            warnings.append("POSITION_DELAYED")
        if fcu_age >= self.thresholds.fcu_warning_s:
            warnings.append("FCU_LINK_DELAYED")
        if coordinator_age >= self.thresholds.coordinator_warning_s:
            warnings.append("COORDINATOR_LINK_DELAYED")
        if warnings:
            return SafetyDecision(SafetyAction.WARN, "+".join(warnings), "L1", "设备状态存在短时延迟，继续监控")
        return SafetyDecision(SafetyAction.NONE, "OK", "L0", "安全状态正常")
