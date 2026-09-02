# 设备 Agent（独立进程，架构说明第 4/5 节）
# 通过 WebSocket /ws/agent 连接总 Agent：
#   上行: 1Hz AgentState 心跳、任务执行反馈链
#   下行: TaskAssignment
# 执行层当前是 SIM 模拟（匀速移动 + 拍照/夹取脚本）；
# 接真实设备时替换 Executor，通信层不动（适配器思路，架构说明第 5 节）。

from __future__ import annotations

import asyncio
import json
import logging
import math
from dataclasses import dataclass

import httpx

from .models import (
    AgentState,
    AgentType,
    ControlMode,
    FeedbackStatus,
    Pose,
    WorkState,
    now_ms,
)

logger = logging.getLogger(__name__)

UAV_SPEED_MPS = 0.8   # 仿真速度：无人机 0.8 m/s
UGV_SPEED_MPS = 0.5   # 无人车 0.5 m/s
UAV_CRUISE_Z_M = 0.5


@dataclass
class AgentConfig:
    agent_id: str
    agent_type: AgentType
    start: Pose
    capabilities: list[str]
    master_url: str = "ws://localhost:8100"
    heartbeat_s: float = 1.0
    battery_percent: float = 95.0


class SimExecutor:
    """SIM 模式的执行层：模拟移动与作业。

    状态推进：
      UAV: ACCEPTED -> (移动到目标上空) -> IN_PROGRESS -> IMAGE_CAPTURED -> (返航) -> COMPLETED
      UGV: ACCEPTED -> (移动到目标)     -> IN_PROGRESS -> COMPLETED
    """

    def __init__(self, cfg: AgentConfig):
        self.cfg = cfg
        self.pose = Pose(
            x_m=cfg.start.x_m, y_m=cfg.start.y_m,
            z_m=0.0 if cfg.agent_type is AgentType.UGV else UAV_CRUISE_Z_M,
        )
        self.speed = UAV_SPEED_MPS if cfg.agent_type is AgentType.UAV else UGV_SPEED_MPS

    def step_toward(self, target: Pose, dt: float) -> bool:
        """朝目标移动一步，返回是否到达。"""
        dx, dy = target.x_m - self.pose.x_m, target.y_m - self.pose.y_m
        dist = math.hypot(dx, dy)
        if dist <= self.speed * dt or dist < 1e-3:
            self.pose.x_m, self.pose.y_m = target.x_m, target.y_m
            return True
        self.pose.x_m += dx / dist * self.speed * dt
        self.pose.y_m += dy / dist * self.speed * dt
        return False


class DeviceAgent:
    """单台设备的 Agent：心跳 + 任务执行 + 反馈。"""

    def __init__(self, cfg: AgentConfig):
        self.cfg = cfg
        self.executor = SimExecutor(cfg)
        self.work_state = WorkState.IDLE
        self.current_task: dict | None = None
        self._ws = None
        self._task_phase = ""  # "" | "to_target" | "working" | "returning"

    # ---------------------------------------------------------------- 状态上报
    def _state_payload(self) -> dict:
        return AgentState(
            agent_id=self.cfg.agent_id,
            agent_type=self.cfg.agent_type,
            device_online=True,
            work_state=self.work_state,
            control_mode=ControlMode.SIM,
            pose=self.executor.pose,
            position_valid=True,
            battery_remaining_percent=self.cfg.battery_percent,
            battery_ok=self.cfg.battery_percent > 20,
            capabilities=self.cfg.capabilities,
            current_task_id=self.current_task.get("task_id") if self.current_task else None,
        ).to_dict()

    async def _send(self, kind: str, data: dict) -> None:
        if self._ws is not None:
            text = json.dumps({"kind": kind, "data": data}, ensure_ascii=False, default=str)
            # websockets >=13 客户端用 send()，旧版用 send_str()
            send = getattr(self._ws, "send", None) or self._ws.send_str
            await send(text)

    async def _feedback(self, status: FeedbackStatus, message: str, **extra) -> None:
        assert self.current_task is not None
        payload = {
            "schema_version": "1.0",
            "timestamp_ms": now_ms(),
            "mission_id": self.current_task["mission_id"],
            "task_id": self.current_task["task_id"],
            "agent_id": self.cfg.agent_id,
            "status": status.value,
            "pose": self.executor.pose.to_dict(),
            "message": message,
            **extra,
        }
        await self._send("task_feedback", payload)
        logger.info("[%s] %s: %s", self.cfg.agent_id, status.value, message)

    # ---------------------------------------------------------------- 主循环
    async def run(self) -> None:
        import websockets

        url = f"{self.cfg.master_url}/ws/agent"
        async for attempt in _forever():
            try:
                async with websockets.connect(url) as ws:
                    self._ws = ws
                    await self._send("hello", {"agent_id": self.cfg.agent_id})
                    logger.info("[%s] 已连接总 Agent %s", self.cfg.agent_id, url)
                    receiver = asyncio.create_task(self._receive(ws))
                    heartbeat = asyncio.create_task(self._heartbeat())
                    executor = asyncio.create_task(self._execute_loop())
                    done, pending = await asyncio.wait(
                        [receiver, heartbeat, executor], return_when=asyncio.FIRST_COMPLETED
                    )
                    for t in pending:
                        t.cancel()
            except Exception as e:
                logger.warning("[%s] 连接断开(%s)，第 %d 次重连...", self.cfg.agent_id, e, attempt)
                self._ws = None
                self.work_state = WorkState.OFFLINE
                await asyncio.sleep(min(2.0 * attempt, 10.0))

    async def _heartbeat(self) -> None:
        while True:
            await self._send("agent_state", self._state_payload())
            await asyncio.sleep(self.cfg.heartbeat_s)

    async def _receive(self, ws) -> None:
        async for raw in ws:
            text = raw if isinstance(raw, str) else raw.decode()
            msg = json.loads(text)
            if msg.get("kind") == "task_assignment":
                await self._on_assignment(msg["data"])

    async def _on_assignment(self, task: dict) -> None:
        if self.current_task is not None:
            await self._feedback(FeedbackStatus.REJECTED, "忙，已有任务在执行", failure_code="BUSY")
            return
        if not self._capable(task.get("task_type", "")):
            await self._feedback(FeedbackStatus.REJECTED, "无对应能力", failure_code="NO_CAPABILITY")
            return
        self.current_task = task
        self._task_phase = "to_target"
        self.work_state = WorkState.EXECUTING
        await self._feedback(FeedbackStatus.ACCEPTED, "已接受任务，前往目标")

    def _capable(self, task_type: str) -> bool:
        need = {"INSPECT_TARGET": "INSPECT", "PICKUP_AND_DELIVER": "PICKUP"}.get(task_type, "")
        return need in self.cfg.capabilities

    # ---------------------------------------------------------------- 执行循环
    async def _execute_loop(self) -> None:
        dt = 0.2
        stable_timer = 0.0
        while True:
            await asyncio.sleep(dt)
            if not self.current_task:
                continue
            task = self.current_task
            target = Pose.from_dict(task.get("target_pose")) or Pose()

            if self._task_phase == "to_target":
                arrived = self.executor.step_toward(target, dt)
                if arrived:
                    self._task_phase = "working"
                    stable_timer = 0.0
                    await self._feedback(FeedbackStatus.IN_PROGRESS, "已到达目标，开始作业", progress_percent=50)

            elif self._task_phase == "working":
                stable_timer += dt
                stable_need = float((task.get("parameters") or {}).get("stable_time_s", 1.0))
                if stable_timer >= stable_need:
                    if task.get("task_type") == "INSPECT_TARGET":
                        await self._feedback(
                            FeedbackStatus.IMAGE_CAPTURED,
                            "拍照完成，照片上传成功，开始返航",
                            progress_percent=70,
                            stage="UPLOADING_IMAGE",
                            image_url=f"/images/{task['mission_id']}/{task['task_id']}.jpg",
                        )
                    else:
                        await self._feedback(
                            FeedbackStatus.COMPLETED,
                            "垃圾已夹取并送达投放点，任务完成",
                            progress_percent=100,
                        )
                        self._finish()
                        continue
                    self._task_phase = "returning"
                    self.work_state = WorkState.RETURNING

            elif self._task_phase == "returning":
                arrived = self.executor.step_toward(self.cfg.start, dt)
                if arrived:
                    await self._feedback(
                        FeedbackStatus.COMPLETED, "已返航至待命点", progress_percent=100
                    )
                    self._finish()

    def _finish(self) -> None:
        self.current_task = None
        self._task_phase = ""
        self.work_state = WorkState.IDLE


async def _forever():
    n = 1
    while True:
        yield n
        n += 1
