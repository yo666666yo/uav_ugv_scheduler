# 调度算法（架构说明 4.3 节）
# 中期版：能力过滤 + 欧氏距离最近；接口上预留代价函数扩展点。

from __future__ import annotations

from dataclasses import dataclass

from .models import AgentState, Pose, TaskType
from .registry import DeviceRegistry

# 任务类型 -> 所需能力（与设备 capabilities 对齐，规范第 6 节）
TASK_CAPABILITY = {
    TaskType.INSPECT_TARGET: "INSPECT",
    TaskType.PICKUP_AND_DELIVER: "PICKUP",
}


@dataclass
class SelectionResult:
    agent: AgentState
    distance_m: float
    reason: str  # 给前端的分配理由（架构说明 3.2 节 task_assigned 事件）


def select_nearest(
    registry: DeviceRegistry,
    target: Pose,
    task_type: TaskType,
    exclude: set[str] | None = None,
) -> SelectionResult | None:
    """从满足条件的设备中选择距目标最近的一台。

    条件（规范/架构说明）：在线、空闲、电量足够、定位正常、具备能力、能到达。
    "能到达"在中期简化为坐标合法（frame 一致），后期接路径规划代价。
    exclude: 排除的 agent_id 集合（失败重分配时拉黑故障设备）。
    """
    required = TASK_CAPABILITY.get(task_type)
    candidates = []
    for s in registry.snapshot():
        if exclude and s.agent_id in exclude:
            continue
        if s.work_state.value == "OFFLINE":
            continue
        if not DeviceRegistry.is_available(s, required):
            continue
        if s.pose.frame_id != target.frame_id:
            continue  # 坐标系不一致，无法比较距离（规范 4.2）
        candidates.append(s)

    if not candidates:
        return None

    best = min(candidates, key=lambda s: s.pose.distance_to(target))
    d = best.pose.distance_to(target)
    reason = (
        f"{best.agent_id} 距离目标最近（{d:.2f}m）、"
        f"{'空闲' if best.work_state.value == 'IDLE' else '可预留'}、"
        f"电量{'正常' if best.battery_ok else '异常'}，具备 {required} 能力"
    )
    return SelectionResult(agent=best, distance_m=d, reason=reason)


# ---- 后期扩展点（不在中期实现）--------------------------------------------
# def select_by_cost(registry, target, task_type) -> SelectionResult | None:
#     """把电量、预计时间、载荷、路径风险加入调度代价（架构说明 4.3）。"""
#     ...
