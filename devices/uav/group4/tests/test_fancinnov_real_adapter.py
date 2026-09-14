import unittest
from pathlib import Path

from agent.contracts import Pose
from uav_adapter.fancinnov_real_adapter import (
    FanciInnovAdapterConfig,
    FanciInnovRealUAVAdapter,
    RealControlDisabled,
)


class FakeCommands:
    def __init__(self):
        self.calls = []

    def publish_command(self, command):
        self.calls.append(("command", command))

    def publish_mission(self, values):
        self.calls.append(("mission", list(values)))


class FakeCamera:
    def capture_image(self, output_path: Path) -> Path:
        return output_path


def state():
    return {
        "online": True,
        "position": {"frame_id": "uwb_map", "x_m": 1.0, "y_m": 2.0, "z_m": 0.0},
        "attitude_rad": {"yaw": 0.5},
        "battery": {"remaining_percent": 80, "voltage_v": 7.9},
        "control_ready": False,
    }


class RealAdapterGateTests(unittest.TestCase):
    def make_adapter(self, enabled=False):
        backend = FakeCommands()
        adapter = FanciInnovRealUAVAdapter(
            config=FanciInnovAdapterConfig(real_control_enabled=enabled),
            state_reader=state,
            command_backend=backend,
            camera=FakeCamera(),
            sleep=lambda _duration: None,
        )
        return adapter, backend

    def test_default_config_is_disabled(self):
        self.assertFalse(FanciInnovAdapterConfig().real_control_enabled)

    def test_disabled_gate_blocks_every_motion_before_backend(self):
        adapter, backend = self.make_adapter()
        pose = Pose("uwb_map", 2.0, 3.0, 1.0, 0.2)
        operations = [
            lambda: adapter.takeoff(1.0),
            lambda: adapter.goto_pose(pose),
            lambda: adapter.return_to_pose(pose, 1.0),
            adapter.land,
        ]
        for operation in operations:
            with self.assertRaises(RealControlDisabled):
                operation()
        self.assertEqual([], backend.calls)

    def test_read_state_and_camera_remain_available_when_disabled(self):
        adapter, backend = self.make_adapter()
        result = adapter.read_state()
        self.assertTrue(result["device_online"])
        self.assertTrue(result["position_valid"])
        self.assertFalse(result["real_control_enabled"])
        self.assertFalse(result["control_ready"])
        self.assertTrue(result["battery"]["battery_ok"])
        self.assertEqual("REAL", result["control_mode"])
        self.assertEqual(Path("photo.jpg"), adapter.capture_image(Path("photo.jpg")))
        self.assertEqual([], backend.calls)

    def test_enabled_fake_backend_matches_vendor_message_layout(self):
        adapter, backend = self.make_adapter(enabled=True)
        adapter.takeoff(1.0)
        adapter.goto_pose(Pose("uwb_map", 2.0, 3.0, 1.0, 0.25))
        adapter.land()
        self.assertEqual(("command", 1), backend.calls[0])
        self.assertEqual(("command", 3), backend.calls[1])
        self.assertEqual(
            ("mission", [0.25, 0.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            backend.calls[2],
        )
        self.assertEqual(("command", 4), backend.calls[3])


if __name__ == "__main__":
    unittest.main()
