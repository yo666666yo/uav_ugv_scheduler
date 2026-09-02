# 总 Agent 入口
# 用法：
#   uvicorn main:app --port 8100            # 只起总 Agent（等设备 Agent 接入）
#   python main.py --demo                   # 附加 3+3 模拟设备，单机闭环演示
#   python main.py --demo --auto-confirm    # 演示模式 + 自动判定垃圾（不等人工按钮）

from __future__ import annotations

import argparse
import asyncio
import logging

from master_agent.bus import (
    TOPIC_AGENT_STATE,
    TOPIC_TASK_FEEDBACK,
    build_bus,
)
from master_agent.mission_fsm import MissionFSM
from master_agent.models import (
    AgentState,
    AgentType,
    ControlMode,
    FeedbackStatus,
    Pose,
    WorkState,
    now_ms,
)
from master_agent.parser import IntentParser
from master_agent.registry import DeviceRegistry
from master_agent.server import MasterAgentServer

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
logger = logging.getLogger("master_agent")

# --------------------------------------------------------------------------
# 演示用：3 无人机 + 3 无人车模拟设备（中期答辩闭环，架构说明第 13 节）
# 真实设备 Agent 接入 ZRDDS 后删除此段即可。
_DEMO_AGENTS = [
    ("drone_001", AgentType.UAV, 1.25, 2.10, ["INSPECT", "CAPTURE_IMAGE"]),
    ("drone_002", AgentType.UAV, 4.80, 1.60, ["INSPECT", "CAPTURE_IMAGE"]),
    ("drone_003", AgentType.UAV, 2.00, 5.50, ["INSPECT", "CAPTURE_IMAGE"]),
    ("car_001",   AgentType.UGV, 4.00, 2.00, ["PICKUP", "DELIVER"]),
    ("car_002",   AgentType.UGV, 5.00, 5.00, ["PICKUP", "DELIVER"]),
    ("car_003",   AgentType.UGV, 0.80, 3.40, ["PICKUP", "DELIVER"]),
]


def make_demo_state(agent_id: str, atype: AgentType, x: float, y: float, caps: list[str]) -> dict:
    return AgentState(
        agent_id=agent_id,
        agent_type=atype,
        device_online=True,
        work_state=WorkState.IDLE,
        control_mode=ControlMode.SIM,
        pose=Pose(x_m=x, y_m=y, z_m=0.0 if atype is AgentType.UGV else 0.5),
        position_valid=True,
        battery_voltage_v=12.6,
        battery_ok=True,
        capabilities=caps,
    ).to_dict()


async def run_demo(bus, fsm, auto_confirm: bool = False, interval_s: float = 1.0) -> None:
    """模拟设备：1Hz 心跳 + 对分配任务的脚本化反馈。"""
    states = {aid: make_demo_state(aid, t, x, y, c) for aid, t, x, y, c in _DEMO_AGENTS}
    # 记录每台设备当前在执行的任务，脚本推进
    pending: dict[str, dict] = {}

    async def heartbeat() -> None:
        while True:
            for aid, st in states.items():
                st["timestamp_ms"] = now_ms()
                await bus.publish(TOPIC_AGENT_STATE, st)
            await asyncio.sleep(interval_s)

    async def drive_tasks() -> None:
        step = 0
        while True:
            await asyncio.sleep(1.0)
            step += 1
            for aid, st in states.items():
                task = pending.get(aid)
                if not task:
                    continue
                # 简化推进：ACCEPTED -> IN_PROGRESS -> (拍照/完成)
                if step % 3 == 0:
                    if task["task_type"] == "INSPECT_TARGET" and not task.get("image_sent"):
                        task["image_sent"] = True
                        st["work_state"] = WorkState.RETURNING.value
                        await bus.publish(TOPIC_TASK_FEEDBACK, {
                            "mission_id": task["mission_id"],
                            "task_id": task["task_id"],
                            "agent_id": aid,
                            "status": FeedbackStatus.IMAGE_CAPTURED.value,
                            "stage": "UPLOADING_IMAGE",
                            "progress_percent": 70,
                            "image_url": f"/images/{task['mission_id']}/{task['task_id']}.jpg",
                            "message": "照片上传成功，开始返航",
                        })
                        if auto_confirm:
                            await fsm.on_inspection_result(task["mission_id"], True)
                    else:
                        st["work_state"] = WorkState.IDLE.value
                        del pending[aid]
                        await bus.publish(TOPIC_TASK_FEEDBACK, {
                            "mission_id": task["mission_id"],
                            "task_id": task["task_id"],
                            "agent_id": aid,
                            "status": FeedbackStatus.COMPLETED.value,
                            "stage": "DONE",
                            "progress_percent": 100,
                            "message": "任务完成，已返航",
                        })

    # 拦截总 Agent 的任务分配，喂给对应模拟设备
    from master_agent.bus import TOPIC_TASK_ASSIGNMENT

    async def on_assignment(topic: str, payload: dict) -> None:
        pending[payload["target_agent_id"]] = payload
        states[payload["target_agent_id"]]["work_state"] = WorkState.EXECUTING.value
        states[payload["target_agent_id"]]["current_task_id"] = payload["task_id"]
        await bus.publish(TOPIC_TASK_FEEDBACK, {
            "mission_id": payload["mission_id"],
            "task_id": payload["task_id"],
            "agent_id": payload["target_agent_id"],
            "status": FeedbackStatus.ACCEPTED.value,
            "message": "已接受任务",
        })

    bus.subscribe(TOPIC_TASK_ASSIGNMENT, on_assignment)
    await asyncio.gather(heartbeat(), drive_tasks())


def build_app(demo: bool = False, auto_confirm: bool = False):
    bus = build_bus("memory")
    registry = DeviceRegistry()
    fsm = MissionFSM(bus, registry)
    parser = IntentParser()

    bus.subscribe(TOPIC_AGENT_STATE, registry.on_state)
    bus.subscribe(TOPIC_TASK_FEEDBACK, fsm.on_feedback)

    holder = MasterAgentServer(bus, registry, fsm, parser)

    if demo:
        # 启动时把模拟设备的协作任务挂到事件循环
        @holder.app.on_event("startup")
        async def _start_demo() -> None:
            asyncio.create_task(run_demo(bus, fsm, auto_confirm))

    return holder.app


app = build_app()

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--demo", action="store_true", help="附加 3+3 模拟设备")
    p.add_argument("--auto-confirm", action="store_true", help="演示模式下自动判定为垃圾")
    p.add_argument("--port", type=int, default=8100)
    args = p.parse_args()

    import uvicorn

    app = build_app(demo=args.demo, auto_confirm=args.auto_confirm)
    logger.info("总 Agent 启动: http://localhost:%s  (demo=%s)", args.port, args.demo)
    uvicorn.run(app, host="0.0.0.0", port=args.port)
