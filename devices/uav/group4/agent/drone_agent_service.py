"""Transport-neutral, single-task UAV agent service."""

from __future__ import annotations

import time
from typing import Any, Callable, Dict, Protocol

from agent.contracts import TaskAssignment, TaskFeedback
from agent.real_state_monitor import RealStateMonitor
from agent.task_store import JsonTaskStore
from agent.terminal_protocol import TaskValidationError, parse_task_assignment
from transports.base_transport import AgentTransport


class TaskExecutor(Protocol):
    def execute(
        self,
        task: TaskAssignment,
        feedback_callback: Callable[[TaskFeedback], None],
        initial_state: Dict[str, Any] | None = None,
    ) -> Dict[str, Any]: ...


class DroneAgentService:
    def __init__(
        self,
        *,
        agent_id: str,
        mode: str,
        transport: AgentTransport,
        task_store: JsonTaskStore,
        executor: TaskExecutor | None,
        state_monitor: RealStateMonitor | None = None,
        state_startup_timeout_s: float = 4.0,
        clock_ms: Callable[[], int] = lambda: int(time.time() * 1000),
    ) -> None:
        if mode not in {"readonly", "dry-run"}:
            raise ValueError("当前终端 Agent 只允许 readonly 或 dry-run 模式")
        self.agent_id = agent_id
        self.mode = mode
        self.transport = transport
        self.task_store = task_store
        self.executor = executor
        self.state_monitor = state_monitor
        self.state_startup_timeout_s = float(state_startup_timeout_s)
        self.clock_ms = clock_ms
        self.work_state = "IDLE"

    def _publish(self, message: Dict[str, Any]) -> None:
        envelope = {
            "schema_version": "1.0",
            "timestamp_ms": self.clock_ms(),
            "agent_id": self.agent_id,
        }
        envelope.update(message)
        self.transport.publish(envelope)

    def _publish_result(
        self,
        task: TaskAssignment,
        *,
        status: str,
        failure_code: str | None = None,
        message: str = "",
        result: Dict[str, Any] | None = None,
    ) -> Dict[str, Any]:
        event = {
            "message_type": "TASK_RESULT",
            "mission_id": task.mission_id,
            "task_id": task.task_id,
            "status": status,
            "failure_code": failure_code,
            "message": message,
            "result": result,
        }
        self._publish(event)
        return event

    def _publish_feedback(self, feedback: TaskFeedback) -> None:
        payload = feedback.to_dict()
        payload["message_type"] = "TASK_FEEDBACK"
        self.transport.publish(payload)

    @staticmethod
    def _public_state(state: Dict[str, Any]) -> Dict[str, Any]:
        online = bool(state.get("device_online", state.get("online", False)))
        ready = bool(state.get("readonly_ready", False))
        status = "ONLINE" if ready else "DEGRADED" if online else "OFFLINE"
        return {
            "device_status": status,
            "device_online": online,
            "readonly_ready": ready,
            "control_ready": bool(state.get("control_ready", False)),
            "telemetry_mode": state.get("telemetry_mode", "OFFLINE"),
            "position_source": state.get("position_source"),
            "position": state.get("position"),
            "position_valid": bool(state.get("position_valid", False)),
            "attitude_rad": state.get("attitude_rad", {}),
            "battery": state.get("battery", {}),
            "battery_valid": bool(state.get("battery_valid", False)),
            "armed": state.get("armed"),
            "armed_valid": bool(state.get("armed_valid", False)),
            "source_health": state.get("source_health", {}),
            "state_age_s": state.get("state_age_s"),
            "state_stale": bool(state.get("state_stale", True)),
            "monitor_error": state.get("monitor_error"),
        }

    def _publish_state(self, state: Dict[str, Any]) -> None:
        self._publish(
            {
                "message_type": "AGENT_STATE",
                "work_state": self.work_state,
                "mode": self.mode,
                **self._public_state(state),
            }
        )

    @staticmethod
    def _state_rejection(state: Dict[str, Any]) -> tuple[str, str] | None:
        if state.get("state_stale", True):
            return "STATE_STALE", "真实状态快照已过期"
        if not state.get("device_online", state.get("online", False)):
            return "DEVICE_OFFLINE", "ROS2 无人机遥测离线"
        if not state.get("position_valid", False):
            return "POSITION_INVALID", "ROS2 UWB 位置无效"
        if state.get("attitude_rad", {}).get("yaw") is None:
            return "YAW_UNAVAILABLE", "ROS2 IMU 尚未提供有效朝向"
        if not state.get("battery_valid", False):
            return "BATTERY_INVALID", "ROS2 电池状态无效"
        if not state.get("readonly_ready", False):
            return "TELEMETRY_NOT_READY", "真实只读遥测尚未就绪"
        if not state.get("battery", {}).get("battery_ok", False):
            return "LOW_BATTERY", "真实电池状态不满足任务要求"
        if state.get("armed") is True:
            return "ALREADY_ARMED", "无人机当前已解锁，拒绝接收新的地面任务"
        return None

    def _store_result(self, task_id: str, event: Dict[str, Any]) -> None:
        self.task_store.put(task_id, {**event, "updated_at_ms": self.clock_ms()})

    def _handle_payload(self, payload: Dict[str, Any]) -> None:
        try:
            task = parse_task_assignment(payload)
        except TaskValidationError as exc:
            self._publish(
                {
                    "message_type": "PROTOCOL_ERROR",
                    "status": "REJECTED",
                    "failure_code": exc.code,
                    "message": str(exc),
                    "mission_id": payload.get("mission_id"),
                    "task_id": payload.get("task_id"),
                }
            )
            return

        if task.target_agent_id != self.agent_id:
            self._publish(
                {
                    "message_type": "TASK_IGNORED",
                    "mission_id": task.mission_id,
                    "task_id": task.task_id,
                    "status": "IGNORED",
                    "failure_code": "TARGET_AGENT_MISMATCH",
                    "message": f"任务目标是 {task.target_agent_id}，本机是 {self.agent_id}",
                }
            )
            return

        previous = self.task_store.get(task.task_id)
        if previous is not None:
            self._publish(
                {
                    "message_type": "TASK_RESULT",
                    "mission_id": task.mission_id,
                    "task_id": task.task_id,
                    "status": "DUPLICATE",
                    "failure_code": "DUPLICATE_TASK",
                    "message": "该 task_id 已处理，不会重复执行",
                    "original_record": previous,
                }
            )
            return

        if self.work_state != "IDLE":
            self._publish_result(
                task,
                status="REJECTED",
                failure_code="BUSY",
                message="无人机 Agent 当前正在处理其他任务",
            )
            return

        self.task_store.put(
            task.task_id,
            {
                "mission_id": task.mission_id,
                "status": "RECEIVED",
                "updated_at_ms": self.clock_ms(),
            },
        )
        initial_state = None
        if self.state_monitor is not None:
            initial_state = self.state_monitor.snapshot()
            rejection = self._state_rejection(initial_state)
            if rejection is not None:
                failure_code, message = rejection
                event = self._publish_result(
                    task,
                    status="REJECTED",
                    failure_code=failure_code,
                    message=message,
                )
                self._store_result(task.task_id, event)
                return
        if self.mode == "readonly":
            event = self._publish_result(
                task,
                status="REJECTED",
                failure_code="REAL_CONTROL_DISABLED",
                message="当前为只读模式，未向飞控发布任何控制指令",
            )
            self._store_result(task.task_id, event)
            return

        if self.executor is None:
            event = self._publish_result(
                task,
                status="FAILED",
                failure_code="EXECUTOR_UNAVAILABLE",
                message="dry-run 执行器未初始化",
            )
            self._store_result(task.task_id, event)
            return

        self.work_state = "BUSY"
        self.task_store.put(
            task.task_id,
            {
                "mission_id": task.mission_id,
                "status": "IN_PROGRESS",
                "updated_at_ms": self.clock_ms(),
            },
        )
        try:
            result = self.executor.execute(task, self._publish_feedback, initial_state)
            status = (
                "COMPLETED"
                if result.get("ok")
                else result.get("status", "FAILED")
            )
            if status not in {"COMPLETED", "FAILED", "REJECTED"}:
                status = "FAILED"
            event = self._publish_result(
                task,
                status=status,
                failure_code=result.get("failure_code"),
                message=result.get("message", "任务模拟执行完成" if result.get("ok") else "任务执行失败"),
                result=result,
            )
        except Exception as exc:
            event = self._publish_result(
                task,
                status="FAILED",
                failure_code="UNHANDLED_AGENT_ERROR",
                message=str(exc),
            )
        finally:
            self.work_state = "IDLE"
        self._store_result(task.task_id, event)

    def run(self) -> int:
        startup_state: Dict[str, Any] = {}
        if self.state_monitor is not None:
            self.state_monitor.start()
            self.state_monitor.wait_initialized(min(1.0, self.state_startup_timeout_s))
            self.state_monitor.wait_readonly_ready(self.state_startup_timeout_s)
            startup_state = self.state_monitor.snapshot()
        self._publish(
            {
                "message_type": "AGENT_EVENT",
                "status": "AGENT_STARTED",
                "work_state": self.work_state,
                "mode": self.mode,
                "real_control_enabled": False,
                **(
                    self._public_state(startup_state)
                    if self.state_monitor is not None
                    else {"readonly_ready": None, "control_ready": False}
                ),
                "message": (
                    "终端 JSON Agent 已启动，ROS2 真实只读状态已接入"
                    if self.state_monitor is not None
                    else "终端 JSON Agent 已启动；真实状态聚合器尚未接入"
                ),
            }
        )
        if self.state_monitor is not None:
            self._publish_state(startup_state)
            self.state_monitor.set_publish_callback(self._publish_state)
        exit_code = 0
        try:
            for incoming in self.transport.receive():
                if incoming.error_code:
                    self._publish(
                        {
                            "message_type": "PROTOCOL_ERROR",
                            "status": "REJECTED",
                            "failure_code": incoming.error_code,
                            "message": incoming.error_message,
                        }
                    )
                    continue
                assert incoming.payload is not None
                self._handle_payload(incoming.payload)
        except KeyboardInterrupt:
            exit_code = 130
        finally:
            if self.state_monitor is not None:
                self.state_monitor.set_publish_callback(None)
                self.state_monitor.stop()
            self._publish(
                {
                    "message_type": "AGENT_EVENT",
                    "status": "AGENT_STOPPED",
                    "work_state": self.work_state,
                    "mode": self.mode,
                }
            )
        return exit_code
