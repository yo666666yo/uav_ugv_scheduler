# 任务状态机（架构说明 4.4 节）
# 管理整个任务的阶段：核查（无人机）→ 判定 → 清理（无人车）→ 返航 → 完成。
# 总 Agent 只发布结构化 TaskAssignment，不发送底层控制命令（规范第 2 节）。

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from enum import Enum

from .bus import DDSBus, TOPIC_MISSION_RESULT, TOPIC_TASK_ASSIGNMENT
from .models import (
    FeedbackStatus,
    Pose,
    TaskAssignment,
    TaskFeedback,
    TaskType,
    new_id,
)
from .scheduler import SelectionResult, select_nearest
from .registry import DeviceRegistry

logger = logging.getLogger(__name__)


class MissionPhase(str, Enum):
    CREATED = "CREATED"                    # 任务创建，待分配无人机
    UAV_ASSIGNED = "UAV_ASSIGNED"          # 已分配无人机，前往目标
    UAV_INSPECTING = "UAV_INSPECTING"      # 无人机核查中（含拍照上传）
    AWAITING_CONFIRM = "AWAITING_CONFIRM"  # 等待图片判定结果（人工/模型）
    UGV_ASSIGNED = "UGV_ASSIGNED"          # 确认垃圾，已分配无人车
    UGV_EXECUTING = "UGV_EXECUTING"        # 无人车夹取、转移、返航中
    COMPLETED = "COMPLETED"                # 任务完成
    FAILED = "FAILED"                      # 任务失败（可选：重分配）


# 触发阶段推进的反馈状态（规范第 8 节）
_UAV_ACCEPT = {FeedbackStatus.ACCEPTED}
_UAV_DONE = {FeedbackStatus.IMAGE_CAPTURED, FeedbackStatus.COMPLETED}
_UGV_DONE = {FeedbackStatus.COMPLETED}
_FAIL = {FeedbackStatus.FAILED, FeedbackStatus.MANUAL_CONTROL_REQUIRED}


@dataclass
class Mission:
    mission_id: str
    mission_type: str = "INSPECT_AND_CLEAR"
    target: Pose = field(default_factory=Pose)
    phase: MissionPhase = MissionPhase.CREATED
    inspect_task_id: str | None = None
    pickup_task_id: str | None = None
    uav_agent_id: str | None = None
    ugv_agent_id: str | None = None
    image_url: str | None = None
    garbage_detected: bool | None = None
    events: list[dict] = field(default_factory=list)  # 推给 /ws/events 的事件流
    # 失败重分配（架构说明 4.4：可选换设备重试）
    failed_agents: set[str] = field(default_factory=set)
    reassign_count: int = 0
    MAX_REASSIGN = 3


class MissionFSM:
    """总 Agent 的任务状态机 + 对 DDS 的发布入口。"""

    def __init__(self, bus: DDSBus, registry: DeviceRegistry) -> None:
        self.bus = bus
        self.registry = registry
        self.missions: dict[str, Mission] = {}

    # ---------------------------------------------------------------- 对外入口
    async def create_mission(self, target: Pose, mission_type: str = "INSPECT_AND_CLEAR") -> Mission:
        m = Mission(mission_id=new_id("mission"), mission_type=mission_type, target=target)
        self.missions[m.mission_id] = m
        await self._emit(m, "mission_created", f"任务创建：核查并清理坐标 ({target.x_m:.2f}, {target.y_m:.2f})")
        await self._assign_uav(m)
        return m

    async def on_feedback(self, topic: str, payload: dict) -> None:
        """订阅 /group4/task/feedback 的处理入口。"""
        fb = TaskFeedback.from_dict(payload)
        m = self.missions.get(fb.mission_id)
        if m is None:
            logger.warning("未知 mission 的反馈: %s", fb.mission_id)
            return

        if fb.status in _FAIL:
            await self._on_failure(m, fb)
        elif fb.task_id == m.inspect_task_id and fb.status in _UAV_ACCEPT and m.phase is MissionPhase.UAV_ASSIGNED:
            m.phase = MissionPhase.UAV_INSPECTING
            await self._emit(m, "task_accepted", f"{fb.agent_id} 已接受核查任务")
        elif fb.task_id == m.inspect_task_id and fb.status in _UAV_DONE and m.phase in (MissionPhase.UAV_ASSIGNED, MissionPhase.UAV_INSPECTING):
            m.image_url = fb.image_url or m.image_url
            m.phase = MissionPhase.AWAITING_CONFIRM
            await self._emit(
                m, "image_captured",
                f"{fb.agent_id} 拍照完成{'（' + m.image_url + '）' if m.image_url else ''}，等待判定",
            )
        elif fb.task_id == m.pickup_task_id and fb.status in _UAV_ACCEPT and m.phase is MissionPhase.UGV_ASSIGNED:
            m.phase = MissionPhase.UGV_EXECUTING
            await self._emit(m, "task_accepted", f"{fb.agent_id} 已接受清理任务")
        elif fb.task_id == m.pickup_task_id and fb.status in _UGV_DONE and m.phase in (MissionPhase.UGV_ASSIGNED, MissionPhase.UGV_EXECUTING):
            m.phase = MissionPhase.COMPLETED
            await self._emit(m, "mission_completed", f"{fb.agent_id} 清理完成并返航，任务结束")
            await self.bus.publish(TOPIC_MISSION_RESULT, {
                "schema_version": "1.0",
                "mission_id": m.mission_id,
                "status": "COMPLETED",
                "summary": f"目标 ({m.target.x_m:.2f}, {m.target.y_m:.2f}) 已核查并清理",
            })

    async def on_inspection_result(self, mission_id: str, garbage_detected: bool) -> bool:
        """图片判定结果入口：人工按钮 /api/manual/inspection-result 或后续模型判定（规范第 9 节）。"""
        m = self.missions.get(mission_id)
        if m is None or m.phase is not MissionPhase.AWAITING_CONFIRM:
            return False
        m.garbage_detected = garbage_detected
        if garbage_detected:
            await self._emit(m, "inspection_confirmed", "判定为垃圾，开始分配无人车")
            await self._assign_ugv(m)
        else:
            m.phase = MissionPhase.COMPLETED
            await self._emit(m, "inspection_rejected", "未发现垃圾，任务结束")
            await self.bus.publish(TOPIC_MISSION_RESULT, {
                "schema_version": "1.0",
                "mission_id": m.mission_id,
                "status": "COMPLETED",
                "summary": "未发现垃圾，无需清理",
            })
        return True

    # ---------------------------------------------------------------- 内部流转
    async def _assign_uav(self, m: Mission) -> None:
        sel = self._select(TaskType.INSPECT_TARGET, m)
        if sel is None:
            await self._fail(m, "无可用无人机（需在线、空闲、具备 INSPECT 能力）")
            return
        m.uav_agent_id = sel.agent.agent_id
        task = TaskAssignment(
            mission_id=m.mission_id,
            target_agent_id=sel.agent.agent_id,
            task_type=TaskType.INSPECT_TARGET,
            target_pose=m.target,
            parameters={  # 规范 7.1 / 第 10 节建议值
                "inspection_distance_m": 0.5,
                "stable_time_s": 2.0,
                "return_after_capture": True,
            },
        )
        m.inspect_task_id = task.task_id
        m.phase = MissionPhase.UAV_ASSIGNED
        await self.bus.publish(TOPIC_TASK_ASSIGNMENT, task.to_dict())
        await self._emit(m, "task_assigned", f"无人机 {sel.agent.agent_id} 距离目标最近，已分配核查任务")

    async def _assign_ugv(self, m: Mission) -> None:
        sel = self._select(TaskType.PICKUP_AND_DELIVER, m)
        if sel is None:
            await self._fail(m, "无可用无人车（需在线、空闲、具备 PICKUP 能力）")
            return
        m.ugv_agent_id = sel.agent.agent_id
        task = TaskAssignment(
            mission_id=m.mission_id,
            target_agent_id=sel.agent.agent_id,
            task_type=TaskType.PICKUP_AND_DELIVER,
            target_pose=m.target,
            parameters={  # 规范 7.2 示例
                "approach_distance_m": 0.1,
                "drop_zone": {"frame_id": m.target.frame_id, "x_m": 0.5, "y_m": 0.5, "z_m": 0.0, "yaw_rad": 0.0},
                "return_after_delivery": True,
            },
        )
        m.pickup_task_id = task.task_id
        m.phase = MissionPhase.UGV_ASSIGNED
        await self.bus.publish(TOPIC_TASK_ASSIGNMENT, task.to_dict())
        await self._emit(m, "task_assigned", f"无人车 {sel.agent.agent_id} 距离目标最近，已分配清理任务")

    def _select(self, task_type: TaskType, m: Mission) -> SelectionResult | None:
        sel = select_nearest(self.registry, m.target, task_type, exclude=m.failed_agents)
        if sel is None:
            return None
        m.events.append({"event": "selection", "detail": sel.reason})  # 分配理由留痕
        return sel

    async def _on_failure(self, m: Mission, fb: TaskFeedback) -> None:
        """失败处理：拉黑故障设备，重选一台重试（最多 MAX_REASSIGN 次）。"""
        reason = f"{fb.agent_id} 反馈 {fb.status.value}/{fb.failure_code}: {fb.message}"
        m.failed_agents.add(fb.agent_id)
        m.reassign_count += 1

        if m.reassign_count > m.MAX_REASSIGN:
            await self._fail(m, f"{reason}；重试次数已用尽")
            return

        await self._emit(m, "task_failed", f"{reason}，尝试换设备重分配（{m.reassign_count}/{m.MAX_REASSIGN}）")
        # 复位到失败前的等待阶段，重新选设备（排除故障设备）
        if fb.task_id == m.inspect_task_id:
            m.phase = MissionPhase.CREATED
            m.inspect_task_id = None
            await self._assign_uav(m)
        elif fb.task_id == m.pickup_task_id:
            m.phase = MissionPhase.AWAITING_CONFIRM
            m.pickup_task_id = None
            await self._assign_ugv(m)
        else:
            await self._fail(m, reason)

    async def _fail(self, m: Mission, reason: str) -> None:
        m.phase = MissionPhase.FAILED
        await self._emit(m, "mission_failed", reason)
        await self.bus.publish(TOPIC_MISSION_RESULT, {
            "schema_version": "1.0",
            "mission_id": m.mission_id,
            "status": "FAILED",
            "summary": reason,
        })

    async def _emit(self, m: Mission, event: str, message: str) -> None:
        from .models import now_ms
        m.events.append({"event": event, "message": message, "timestamp_ms": now_ms()})
        logger.info("[%s] %s", event, message)
