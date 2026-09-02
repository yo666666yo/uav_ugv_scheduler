# 设备状态目录（架构说明 4.2 节）
# 订阅 /group4/agent/state，维护每台设备最近一次状态；
# 超时策略：状态周期 1s，警告 3s，离线判定 5s。

from __future__ import annotations

import time
from dataclasses import dataclass

from .models import AgentState, WorkState, now_ms

STATE_PERIOD_S = 1.0     # 设备状态发布周期（约定）
STALE_WARN_S = 3.0       # 超过 3s 未上报 -> 状态可疑
OFFLINE_S = 5.0          # 超过 5s 未上报 -> 判定离线


@dataclass
class RegistryEntry:
    state: AgentState            # 最近一次上报
    last_seen_ms: int            # 最近上报时间
    offline: bool = False        # 是否已被判定离线

    def seconds_since(self, now: int | None = None) -> float:
        now = now if now is not None else now_ms()
        return (now - self.last_seen_ms) / 1000.0


class DeviceRegistry:
    """总 Agent 的设备状态目录。

    使用方式：
      - bus.subscribe(TOPIC_AGENT_STATE, registry.on_state)
      - 周期调用 registry.tick() 刷新离线判定（也可在查询时惰性刷新）
    """

    def __init__(self) -> None:
        self._entries: dict[str, RegistryEntry] = {}

    # ---- 写入 ------------------------------------------------------------
    async def on_state(self, topic: str, payload: dict) -> None:
        state = AgentState.from_dict(payload)
        entry = self._entries.get(state.agent_id)
        if entry is None:
            self._entries[state.agent_id] = RegistryEntry(state=state, last_seen_ms=state.timestamp_ms)
        else:
            entry.state = state
            entry.last_seen_ms = state.timestamp_ms
            entry.offline = False  # 恢复上报即在线

    def tick(self) -> None:
        """按 5s 离线规则刷新（独立于设备自己上报的 device_online 字段）。"""
        now = now_ms()
        for e in self._entries.values():
            if e.seconds_since(now) > OFFLINE_S:
                e.offline = True

    # ---- 查询 ------------------------------------------------------------
    def snapshot(self) -> list[AgentState]:
        self.tick()
        out = []
        for e in self._entries.values():
            s = e.state
            if e.offline:
                # 离线覆盖：展示层与调度层统一看到 OFFLINE
                s = AgentState.from_dict(s.to_dict())
                s.work_state = WorkState.OFFLINE
                s.device_online = False
            out.append(s)
        return out

    def get(self, agent_id: str) -> AgentState | None:
        for s in self.snapshot():
            if s.agent_id == agent_id:
                return s
        return None

    def summary(self) -> dict[str, str]:
        """架构说明 4.2 节示例的目录视图。"""
        return {s.agent_id: f"{'在线' if s.device_online else '离线'}、{s.work_state.value}" for s in self.snapshot()}

    @staticmethod
    def is_available(s: AgentState, required_capability: str | None = None) -> bool:
        """调度可用性判定（架构说明 4.3 节条件清单）。"""
        return (
            s.device_online
            and s.work_state in (WorkState.IDLE, WorkState.RESERVED)
            and s.battery_ok
            and s.position_valid
            and s.pose is not None
            and (required_capability is None or required_capability in s.capabilities)
        )
