import io
import json
import tempfile
import unittest
from pathlib import Path

from agent.drone_agent_service import DroneAgentService
from agent.real_state_monitor import RealStateMonitor
from agent.task_store import JsonTaskStore
from agent.terminal_protocol import TaskValidationError, parse_task_assignment
from transports.terminal_json_transport import TerminalJsonTransport


def terminal_task(task_id="task_001", target_agent_id="drone_001"):
    return {
        "schema_version": "1.0",
        "message_type": "TASK_ASSIGNMENT",
        "timestamp": "2026-09-03T10:30:00+08:00",
        "mission_id": "mission_001",
        "task_id": task_id,
        "target_agent_id": target_agent_id,
        "task_type": "INSPECT_TARGET",
        "parameters": {
            "target_position": {
                "frame_id": "uwb_map",
                "x_m": 2.5,
                "y_m": 3.0,
                "z_m": 0.0,
            },
            "inspection_height_m": 0.8,
            "target_label": "garbage",
        },
    }


def messages_from(stream):
    return [json.loads(line) for line in stream.getvalue().splitlines()]


class FakeExecutor:
    def __init__(self):
        self.calls = []

        self.initial_states = []

    def execute(self, task, feedback_callback, initial_state=None):
        self.calls.append(task.task_id)
        self.initial_states.append(initial_state)
        return {"ok": True, "image_url": "/images/test.jpg"}


def ready_real_state(voltage=7.8):
    return {
        "online": True,
        "device_online": True,
        "readonly_ready": True,
        "control_ready": False,
        "telemetry_mode": "ROS2_ONLY",
        "position_source": "ROS2",
        "source_health": {"ros2_online": True, "mavlink_online": False},
        "position": {
            "frame_id": "uwb_map",
            "x_m": 0.72,
            "y_m": 1.35,
            "z_m": 0.03,
        },
        "position_valid": True,
        "attitude_rad": {"yaw": 0.4},
        "battery": {
            "voltage_v": voltage,
            "remaining_percent": None,
            "battery_ok": voltage >= 7.0,
        },
        "battery_valid": True,
        "armed": None,
        "armed_valid": False,
    }


class TerminalProtocolTests(unittest.TestCase):
    def test_accepts_iso_timestamp_and_nested_target_position(self):
        task = parse_task_assignment(terminal_task())
        self.assertEqual(task.timestamp_ms, 1788402600000)
        self.assertEqual(task.target_pose.x_m, 2.5)
        self.assertEqual(task.target_pose.frame_id, "uwb_map")
        self.assertNotIn("target_position", task.parameters)

    def test_accepts_existing_project_contract(self):
        value = terminal_task()
        value["timestamp_ms"] = 123
        value.pop("timestamp")
        value["target_pose"] = value["parameters"].pop("target_position")
        task = parse_task_assignment(value)
        self.assertEqual(task.timestamp_ms, 123)

    def test_rejects_wrong_coordinate_frame(self):
        value = terminal_task()
        value["parameters"]["target_position"]["frame_id"] = "map"
        with self.assertRaises(TaskValidationError) as context:
            parse_task_assignment(value)
        self.assertEqual(context.exception.code, "INVALID_COORDINATE_FRAME")


class TerminalTransportTests(unittest.TestCase):
    def test_invalid_json_does_not_stop_following_messages(self):
        source = io.StringIO('{bad json}\n{"message_type":"TASK_ASSIGNMENT"}\n')
        transport = TerminalJsonTransport(source, io.StringIO())
        received = list(transport.receive())
        self.assertEqual(received[0].error_code, "INVALID_JSON")
        self.assertEqual(received[1].payload["message_type"], "TASK_ASSIGNMENT")


class TaskStoreTests(unittest.TestCase):
    def test_record_survives_reopen(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tasks.json"
            JsonTaskStore(path).put("task_001", {"status": "COMPLETED"})
            self.assertEqual(JsonTaskStore(path).get("task_001")["status"], "COMPLETED")


class DroneAgentServiceTests(unittest.TestCase):
    def run_service(
        self, lines, mode="dry-run", executor=None, store_path=None, state_monitor=None
    ):
        output = io.StringIO()
        transport = TerminalJsonTransport(io.StringIO(lines), output)
        store = JsonTaskStore(store_path or Path(tempfile.mkdtemp()) / "tasks.json")
        service = DroneAgentService(
            agent_id="drone_001",
            mode=mode,
            transport=transport,
            task_store=store,
            executor=executor,
            state_monitor=state_monitor,
            state_startup_timeout_s=0.05,
            clock_ms=lambda: 1000,
        )
        self.assertEqual(service.run(), 0)
        return messages_from(output), store

    def test_dry_run_executes_and_publishes_result(self):
        executor = FakeExecutor()
        events, store = self.run_service(json.dumps(terminal_task()) + "\n", executor=executor)
        self.assertEqual(executor.calls, ["task_001"])
        results = [event for event in events if event["message_type"] == "TASK_RESULT"]
        self.assertEqual(results[-1]["status"], "COMPLETED")
        self.assertEqual(store.get("task_001")["status"], "COMPLETED")

    def test_duplicate_task_is_never_executed_twice(self):
        executor = FakeExecutor()
        line = json.dumps(terminal_task()) + "\n"
        events, _store = self.run_service(line + line, executor=executor)
        self.assertEqual(executor.calls, ["task_001"])
        self.assertTrue(any(event.get("status") == "DUPLICATE" for event in events))

    def test_readonly_rejects_without_executor(self):
        events, store = self.run_service(json.dumps(terminal_task()) + "\n", mode="readonly")
        results = [event for event in events if event["message_type"] == "TASK_RESULT"]
        self.assertEqual(results[-1]["failure_code"], "REAL_CONTROL_DISABLED")
        self.assertEqual(store.get("task_001")["status"], "REJECTED")

    def test_wrong_agent_is_ignored_and_not_recorded(self):
        executor = FakeExecutor()
        events, store = self.run_service(
            json.dumps(terminal_task(target_agent_id="drone_003")) + "\n",
            executor=executor,
        )
        self.assertEqual(executor.calls, [])
        self.assertIsNone(store.get("task_001"))
        self.assertTrue(any(event["message_type"] == "TASK_IGNORED" for event in events))

    def test_real_state_is_checked_and_passed_to_dry_run(self):
        executor = FakeExecutor()
        monitor = RealStateMonitor(ready_real_state, poll_interval_s=0.001)
        events, _store = self.run_service(
            json.dumps(terminal_task()) + "\n",
            executor=executor,
            state_monitor=monitor,
        )
        self.assertEqual(executor.calls, ["task_001"])
        self.assertEqual(executor.initial_states[0]["position"]["x_m"], 0.72)
        self.assertTrue(any(event["message_type"] == "AGENT_STATE" for event in events))

    def test_low_real_battery_rejects_before_executor(self):
        executor = FakeExecutor()
        monitor = RealStateMonitor(
            lambda: ready_real_state(voltage=4.559), poll_interval_s=0.001
        )
        events, store = self.run_service(
            json.dumps(terminal_task()) + "\n",
            executor=executor,
            state_monitor=monitor,
        )
        self.assertEqual(executor.calls, [])
        results = [event for event in events if event["message_type"] == "TASK_RESULT"]
        self.assertEqual(results[-1]["failure_code"], "LOW_BATTERY")
        self.assertEqual(store.get("task_001")["status"], "REJECTED")

    def test_offline_real_state_rejects_before_executor(self):
        executor = FakeExecutor()
        monitor = RealStateMonitor(
            lambda: {
                "online": False,
                "device_online": False,
                "readonly_ready": False,
                "control_ready": False,
            },
            poll_interval_s=0.001,
        )
        events, _store = self.run_service(
            json.dumps(terminal_task()) + "\n",
            executor=executor,
            state_monitor=monitor,
        )
        results = [event for event in events if event["message_type"] == "TASK_RESULT"]
        self.assertEqual(results[-1]["failure_code"], "DEVICE_OFFLINE")
        self.assertEqual(executor.calls, [])
