import asyncio

import pytest

from master_agent.bus import (
    build_bus, TOPIC_AGENT_STATE, TOPIC_TASK_ASSIGNMENT, TOPIC_TASK_FEEDBACK,
)
from master_agent.mission_fsm import MissionFSM, MissionPhase
from master_agent.models import (
    AgentState, AgentType, ControlMode, FeedbackStatus, Pose, TaskType, WorkState,
)
from master_agent.registry import DeviceRegistry


def make_state(agent_id, atype, x, y, caps):
    return AgentState(
        agent_id=agent_id, agent_type=atype, device_online=True,
        work_state=WorkState.IDLE, control_mode=ControlMode.SIM,
        pose=Pose(x_m=x, y_m=y), position_valid=True, battery_ok=True,
        capabilities=caps,
    ).to_dict()


def setup_fsm():
    bus = build_bus("memory")
    registry = DeviceRegistry()
    fsm = MissionFSM(bus, registry)
    bus.subscribe(TOPIC_AGENT_STATE, registry.on_state)
    bus.subscribe(TOPIC_TASK_FEEDBACK, fsm.on_feedback)
    return bus, registry, fsm


async def seed_agents(bus):
    # 目标 (3.0, 5.0): drone_002 最近; car_002 最近
    for st in [
        make_state("drone_001", AgentType.UAV, 1.0, 1.0, ["INSPECT"]),
        make_state("drone_002", AgentType.UAV, 3.2, 4.8, ["INSPECT"]),
        make_state("car_001", AgentType.UGV, 0.5, 0.5, ["PICKUP"]),
        make_state("car_002", AgentType.UGV, 2.9, 5.2, ["PICKUP"]),
    ]:
        await bus.publish(TOPIC_AGENT_STATE, st)


@pytest.mark.parametrize("auto", [False])
def test_full_mission_flow(auto):
    async def run():
        bus, registry, fsm = setup_fsm()
        await seed_agents(bus)

        m = await fsm.create_mission(Pose(x_m=3.0, y_m=5.0))
        assert m.phase is MissionPhase.UAV_ASSIGNED
        assert m.uav_agent_id == "drone_002"  # 最近无人机

        # 无人机反馈链: ACCEPTED -> IMAGE_CAPTURED
        await bus.publish(TOPIC_TASK_FEEDBACK, {
            "mission_id": m.mission_id, "task_id": m.inspect_task_id,
            "agent_id": "drone_002", "status": "ACCEPTED", "message": "ok",
        })
        assert m.phase is MissionPhase.UAV_INSPECTING
        await bus.publish(TOPIC_TASK_FEEDBACK, {
            "mission_id": m.mission_id, "task_id": m.inspect_task_id,
            "agent_id": "drone_002", "status": "IMAGE_CAPTURED",
            "image_url": "/images/m1/t1.jpg", "message": "ok",
        })
        assert m.phase is MissionPhase.AWAITING_CONFIRM
        assert m.image_url == "/images/m1/t1.jpg"

        # 人工判定: 是垃圾 -> 分配最近无人车
        ok = await fsm.on_inspection_result(m.mission_id, True)
        assert ok
        assert m.phase is MissionPhase.UGV_ASSIGNED
        assert m.ugv_agent_id == "car_002"

        # 无人车完成 -> 任务完成
        await bus.publish(TOPIC_TASK_FEEDBACK, {
            "mission_id": m.mission_id, "task_id": m.pickup_task_id,
            "agent_id": "car_002", "status": "COMPLETED", "message": "done",
        })
        assert m.phase is MissionPhase.COMPLETED

    asyncio.run(run())


def test_no_garbage_ends_mission():
    async def run():
        bus, registry, fsm = setup_fsm()
        await seed_agents(bus)
        m = await fsm.create_mission(Pose(x_m=3.0, y_m=5.0))
        await bus.publish(TOPIC_TASK_FEEDBACK, {
            "mission_id": m.mission_id, "task_id": m.inspect_task_id,
            "agent_id": "drone_002", "status": "IMAGE_CAPTURED",
        })
        await fsm.on_inspection_result(m.mission_id, False)
        assert m.phase is MissionPhase.COMPLETED
        assert m.pickup_task_id is None  # 未派无人车

    asyncio.run(run())


def test_no_available_uav_fails():
    async def run():
        bus, registry, fsm = setup_fsm()
        # 不 seed 任何设备
        m = await fsm.create_mission(Pose(x_m=3.0, y_m=5.0))
        assert m.phase is MissionPhase.FAILED

    asyncio.run(run())
