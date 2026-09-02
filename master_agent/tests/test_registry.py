import asyncio

import pytest

from master_agent.bus import build_bus, TOPIC_AGENT_STATE, TOPIC_TASK_ASSIGNMENT
from master_agent.models import (
    AgentState, AgentType, ControlMode, Pose, WorkState, now_ms,
)
from master_agent.registry import DeviceRegistry


def make_state(agent_id, atype, x, y, caps, work_state=WorkState.IDLE, online=True):
    return AgentState(
        agent_id=agent_id, agent_type=atype, device_online=online,
        work_state=work_state, control_mode=ControlMode.SIM,
        pose=Pose(x_m=x, y_m=y), position_valid=True, battery_ok=True,
        capabilities=caps,
    ).to_dict()


@pytest.fixture
def registry():
    return DeviceRegistry()


def test_online_and_offline_judgement(registry):
    async def run():
        bus = build_bus("memory")
        bus.subscribe(TOPIC_AGENT_STATE, registry.on_state)
        st = make_state("drone_001", AgentType.UAV, 1.0, 1.0, ["INSPECT"])
        st["timestamp_ms"] = now_ms()
        await bus.publish(TOPIC_AGENT_STATE, st)
        # 刚上报: 在线
        assert registry.get("drone_001").work_state is WorkState.IDLE
        # 伪造 6 秒前的上报: 判定离线
        st2 = dict(st)
        st2["timestamp_ms"] = now_ms() - 6000
        await bus.publish(TOPIC_AGENT_STATE, st2)
        assert registry.get("drone_001").work_state is WorkState.OFFLINE

    asyncio.run(run())


def test_unavailable_devices_filtered(registry):
    async def run():
        bus = build_bus("memory")
        bus.subscribe(TOPIC_AGENT_STATE, registry.on_state)
        # drone_001 空闲可用; drone_002 忙碌; drone_003 低电; car_001 是 UGV 没核查能力
        for st in [
            make_state("drone_001", AgentType.UAV, 5.0, 5.0, ["INSPECT"]),
            make_state("drone_002", AgentType.UAV, 1.0, 1.0, ["INSPECT"], work_state=WorkState.EXECUTING),
            dict(make_state("drone_003", AgentType.UAV, 2.0, 2.0, ["INSPECT"]), battery=None) ,
        ]:
            st["battery_ok"] = st["agent_id"] == "drone_003" and False or st.get("battery_ok", True)
            await bus.publish(TOPIC_AGENT_STATE, st)
        await bus.publish(TOPIC_AGENT_STATE, make_state("car_001", AgentType.UGV, 1.5, 1.5, ["PICKUP"]))

        from master_agent.scheduler import select_nearest
        from master_agent.models import TaskType
        sel = select_nearest(registry, Pose(x_m=4.5, y_m=5.0), TaskType.INSPECT_TARGET)
        # drone_002 忙、drone_003 低电被过滤, car 无 INSPECT 能力被过滤 => 只剩 drone_001
        assert sel is not None and sel.agent.agent_id == "drone_001"

    asyncio.run(run())
