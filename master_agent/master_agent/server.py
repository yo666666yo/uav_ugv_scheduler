# 网页后端接口（架构说明 3.2 节）
# POST /api/chat                    —— 用户自然语言任务入口
# WebSocket /ws/events              —— 状态/事件实时推送
# POST /api/manual/inspection-result—— 人工判定接口（规范第 9 节）
# GET  /api/agents                  —— 设备状态目录快照

from __future__ import annotations

import asyncio
import json
import logging

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from .bus import DDSBus, TOPIC_AGENT_STATE, TOPIC_TASK_FEEDBACK
from .mission_fsm import MissionFSM
from .models import now_ms
from .parser import IntentParser
from .registry import DeviceRegistry

logger = logging.getLogger(__name__)


class ChatRequest(BaseModel):
    message: str


class InspectionResult(BaseModel):
    mission_id: str
    garbage_detected: bool
    confidence: float = 1.0
    source: str = "MANUAL"


class MasterAgentServer:
    """把 总Agent(状态机/目录/解析) 挂到 HTTP + WebSocket。"""

    def __init__(self, bus: DDSBus, registry: DeviceRegistry, fsm: MissionFSM, parser: IntentParser):
        self.bus = bus
        self.registry = registry
        self.fsm = fsm
        self.parser = parser
        self.ws_clients: set[WebSocket] = set()
        self.app = FastAPI(title="Group4 Master Agent")
        self.app.add_middleware(
            CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"]
        )
        self._wire_events()
        self._wire_routes()

    # ---------------------------------------------------------------- DDS -> WS
    def _wire_events(self) -> None:
        self.bus.subscribe(TOPIC_AGENT_STATE, self._on_agent_state)

        async def forward_feedback(topic: str, payload: dict) -> None:
            await self.broadcast({"event": "task_feedback", "data": payload})

        self.bus.subscribe(TOPIC_TASK_FEEDBACK, forward_feedback)

    async def _on_agent_state(self, topic: str, payload: dict) -> None:
        # 设备状态高频（1Hz），仅推增量摘要，避免刷屏
        await self.broadcast({
            "event": "agent_state",
            "data": {
                "agent_id": payload.get("agent_id"),
                "work_state": payload.get("work_state"),
                "pose": payload.get("pose"),
            },
        })

    async def broadcast(self, message: dict) -> None:
        msg = json.dumps(message, ensure_ascii=False, default=str)
        dead = []
        for ws in self.ws_clients:
            try:
                await ws.send_text(msg)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.ws_clients.discard(ws)

    # ---------------------------------------------------------------- 路由
    def _wire_routes(self) -> None:
        app = self.app

        @app.post("/api/chat")
        async def chat(req: ChatRequest):
            parsed = await self.parser.parse(req.message)
            if parsed.error:
                return {"ok": False, "error": parsed.error, "message": parsed.message}
            mission = await self.fsm.create_mission(parsed.target, parsed.mission_type)
            await self.broadcast({
                "event": "mission_created",
                "data": {"mission_id": mission.mission_id, "target": parsed.target.to_dict()},
            })
            return {
                "ok": True,
                "mission_id": mission.mission_id,
                "target": parsed.target.to_dict(),
                "reply": f"已创建任务 {mission.mission_id}，正在调度设备",
            }

        @app.post("/api/manual/inspection-result")
        async def manual_inspection(r: InspectionResult):
            ok = await self.fsm.on_inspection_result(r.mission_id, r.garbage_detected)
            return {"ok": ok}

        @app.get("/api/agents")
        async def agents():
            return {"agents": [s.to_dict() for s in self.registry.snapshot()]}

        @app.get("/api/missions")
        async def missions():
            return {
                "missions": [
                    {
                        "mission_id": m.mission_id,
                        "phase": m.phase.value,
                        "target": m.target.to_dict(),
                        "uav_agent_id": m.uav_agent_id,
                        "ugv_agent_id": m.ugv_agent_id,
                        "image_url": m.image_url,
                        "garbage_detected": m.garbage_detected,
                        "events": m.events[-20:],
                    }
                    for m in self.fsm.missions.values()
                ]
            }

        @app.websocket("/ws/events")
        async def ws_events(ws: WebSocket):
            await ws.accept()
            self.ws_clients.add(ws)
            await ws.send_text(json.dumps({"event": "hello", "timestamp_ms": now_ms()}))
            try:
                while True:  # 客户端消息仅保活，业务都在服务端推
                    await asyncio.wait_for(ws.receive_text(), timeout=60)
            except (WebSocketDisconnect, asyncio.TimeoutError):
                pass
            finally:
                self.ws_clients.discard(ws)

        @app.get("/api/health")
        async def health():
            return {"ok": True, "llm_enabled": self.parser.llm_enabled}
