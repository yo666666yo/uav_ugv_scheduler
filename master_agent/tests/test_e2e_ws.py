# 多进程级闭环测试：真实起 uvicorn 服务器 + WebSocket 设备客户端
# 覆盖: /ws/agent 设备网关、任务路由、设备 Agent 执行链、心跳注册。

import asyncio
import json

import pytest

from master_agent.bus import build_bus, TOPIC_AGENT_STATE, TOPIC_TASK_FEEDBACK
from master_agent.device_agent import AgentConfig, DeviceAgent
from master_agent.mission_fsm import MissionFSM
from master_agent.models import AgentType, Pose
from master_agent.parser import IntentParser
from master_agent.registry import DeviceRegistry
from master_agent.server import MasterAgentServer

import httpx


def build_test_app():
    bus = build_bus("memory")
    registry = DeviceRegistry()
    fsm = MissionFSM(bus, registry)
    parser = IntentParser()  # 无 key -> 正则降级
    bus.subscribe(TOPIC_AGENT_STATE, registry.on_state)
    bus.subscribe(TOPIC_TASK_FEEDBACK, fsm.on_feedback)
    return MasterAgentServer(bus, registry, fsm, parser), fsm


@pytest.mark.parametrize("garbage", [True])
def test_multiprocess_closed_loop(garbage):
    async def run():
        from uvicorn import Config, Server

        holder, fsm = build_test_app()
        config = Config(holder.app, host="127.0.0.1", port=8199, log_level="error")
        server = Server(config)

        async def serve():
            await server.serve()

        serve_task = asyncio.create_task(serve())
        for _ in range(50):  # 等服务器就绪
            await asyncio.sleep(0.1)
            if server.started:
                break
        assert server.started

        # 起 2 台设备 Agent（1 UAV + 1 UGV），离目标很近加速测试
        uav = DeviceAgent(AgentConfig(
            "drone_001", AgentType.UAV, Pose(x_m=2.8, y_m=4.9),
            ["INSPECT"], master_url="ws://127.0.0.1:8199",
        ))
        ugv = DeviceAgent(AgentConfig(
            "car_001", AgentType.UGV, Pose(x_m=3.1, y_m=5.1),
            ["PICKUP"], master_url="ws://127.0.0.1:8199",
        ))
        async def run_agents():
            await asyncio.gather(uav.run(), ugv.run())

        agents_task = asyncio.create_task(run_agents())
        await asyncio.sleep(1.5)  # 等设备 hello + 首次心跳

        try:
            # trust_env=False: 绕过系统代理，否则 localhost 请求被代理劫持返回 502
            async with httpx.AsyncClient(base_url="http://127.0.0.1:8199", timeout=10.0, trust_env=False) as client:
                # 设备目录应看到两台在线
                r = await client.get("/api/agents")
                online = {a["agent_id"] for a in r.json()["agents"] if a["device_online"]}
                assert {"drone_001", "car_001"} <= online

                # 下发任务
                r = await client.post("/api/chat", json={"message": "检查坐标（3.0，5.0）的障碍物并完成清理"})
                body = r.json()
                assert body["ok"], body
                mission_id = body["mission_id"]

                # 等核查完成（设备近，约几秒）
                for _ in range(100):
                    await asyncio.sleep(0.2)
                    r = await client.get("/api/missions")
                    m = next(x for x in r.json()["missions"] if x["mission_id"] == mission_id)
                    if m["phase"] == "AWAITING_CONFIRM":
                        break
                assert m["phase"] == "AWAITING_CONFIRM", m["phase"]
                assert m["image_url"]

                # 人工判定为垃圾 -> UGV 清理 -> 完成
                r = await client.post("/api/manual/inspection-result", json={
                    "mission_id": mission_id, "garbage_detected": True,
                })
                assert r.json()["ok"]
                for _ in range(150):
                    await asyncio.sleep(0.2)
                    r = await client.get("/api/missions")
                    m = next(x for x in r.json()["missions"] if x["mission_id"] == mission_id)
                    if m["phase"] in ("COMPLETED", "FAILED"):
                        break
                assert m["phase"] == "COMPLETED", m["phase"]
                assert m["uav_agent_id"] == "drone_001"
                assert m["ugv_agent_id"] == "car_001"
        finally:
            agents_task.cancel()
            serve_task.cancel()
            await asyncio.sleep(0.2)

    asyncio.run(run())
