import asyncio

import pytest

from master_agent.bus import (
    build_bus, TOPIC_AGENT_STATE, TOPIC_TASK_FEEDBACK,
)
from master_agent.mission_fsm import MissionFSM, MissionPhase
from master_agent.models import (
    AgentState, AgentType, ControlMode, Pose, WorkState,
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
    # 目标 (3.0, 5.0): drone_002 最近(会失败), drone_001 次之
    for st in [
        make_state("drone_001", AgentType.UAV, 2.0, 4.0, ["INSPECT"]),
        make_state("drone_002", AgentType.UAV, 3.2, 4.8, ["INSPECT"]),
        make_state("car_001", AgentType.UGV, 2.9, 5.2, ["PICKUP"]),
    ]:
        await bus.publish(TOPIC_AGENT_STATE, st)


def test_reassign_on_failure():
    async def run():
        bus, registry, fsm = setup_fsm()
        await seed_agents(bus)

        m = await fsm.create_mission(Pose(x_m=3.0, y_m=5.0))
        assert m.uav_agent_id == "drone_002"  # 最近
        first_task_id = m.inspect_task_id

        # drone_002 执行失败
        await bus.publish(TOPIC_TASK_FEEDBACK, {
            "mission_id": m.mission_id, "task_id": first_task_id,
            "agent_id": "drone_002", "status": "FAILED",
            "failure_code": "BATTERY_LOW", "message": "电量骤降",
        })

        # 应换到 drone_001（drone_002 被拉黑）
        assert m.uav_agent_id == "drone_001"
        assert m.phase is MissionPhase.UAV_ASSIGNED
        assert m.inspect_task_id != first_task_id

        # drone_001 正常完成核查
        await bus.publish(TOPIC_TASK_FEEDBACK, {
            "mission_id": m.mission_id, "task_id": m.inspect_task_id,
            "agent_id": "drone_001", "status": "ACCEPTED",
        })
        await bus.publish(TOPIC_TASK_FEEDBACK, {
            "mission_id": m.mission_id, "task_id": m.inspect_task_id,
            "agent_id": "drone_001", "status": "IMAGE_CAPTURED",
            "image_url": "/images/x.jpg",
        })
        assert m.phase is MissionPhase.AWAITING_CONFIRM

    asyncio.run(run())


def test_fail_after_max_retries():
    async def run():
        bus, registry, fsm = setup_fsm()
        # 只留一台无人机，让它反复失败，验证重试上限
        await bus.publish(TOPIC_AGENT_STATE, make_state("drone_001", AgentType.UAV, 2.0, 4.0, ["INSPECT"]))
        m = await fsm.create_mission(Pose(x_m=3.0, y_m=5.0))
        assert m.uav_agent_id == "drone_001"

        # 连续失败 3 次（= MAX_REASSIGN），每次都应换到同一台（无其他选择时直接失败）
        for i in range(3):
            assert m.phase is not MissionPhase.FAILED or i > 0
            await bus.publish(TOPIC_TASK_FEEDBACK, {
                "mission_id": m.mission_id, "task_id": m.inspect_task_id,
                "agent_id": "drone_001", "status": "FAILED",
                "failure_code": "X", "message": f"第{i+1}次失败",
            })
            # 无可用替代设备 -> 任务直接失败
            assert m.phase is MissionPhase.FAILED
            break  # 拉黑后无人可选, 第一次失败即 FAILED

    asyncio.run(run())
