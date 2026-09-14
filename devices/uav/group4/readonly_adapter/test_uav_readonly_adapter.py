import json
import unittest
from io import StringIO

from uav_readonly_adapter import AdapterConfig, JsonPublisher, StateProjector


class FakeMessage:
    def __init__(self, message_type, **fields):
        self._message_type = message_type
        for name, value in fields.items():
            setattr(self, name, value)

    def get_type(self):
        return self._message_type


class StateProjectorTests(unittest.TestCase):
    def setUp(self):
        self.projector = StateProjector(AdapterConfig())

    def test_vendor_position_is_converted_from_cm_to_m(self):
        self.projector.process(
            FakeMessage(
                "GLOBAL_VISION_POSITION_ESTIMATE",
                x=329.1619,
                y=513.1796,
                z=1.7142,
                roll=0.0039,
                pitch=-0.0318,
                yaw=-1.6691,
            ),
            now_monotonic=10.0,
        )
        state = self.projector.snapshot(now_unix=1000.0, now_monotonic=10.5)
        self.assertEqual(state["position"]["x_m"], 3.2916)
        self.assertEqual(state["position"]["y_m"], 5.1318)
        self.assertEqual(state["position"]["z_m"], 0.0171)
        self.assertTrue(state["online"])

    def test_heartbeat_maps_lock_state_without_issuing_commands(self):
        self.projector.process(
            FakeMessage("HEARTBEAT", base_mode=0, custom_mode=7, system_status=3),
            now_monotonic=1.0,
        )
        self.assertFalse(self.projector.snapshot(now_monotonic=1.0)["armed"])
        self.projector.process(
            FakeMessage("HEARTBEAT", base_mode=128, custom_mode=7, system_status=4),
            now_monotonic=2.0,
        )
        self.assertTrue(self.projector.snapshot(now_monotonic=2.0)["armed"])

    def test_velocity_battery_and_vendor_uwb_mapping(self):
        self.projector.process(
            FakeMessage("GLOBAL_POSITION_INT", vx=12, vy=-34, vz=5),
            now_monotonic=1.0,
        )
        self.projector.process(
            FakeMessage(
                "BATTERY_STATUS",
                voltages=[4722, 4517, 664, 381, 200, 551],
                current_battery=20,
                battery_remaining=0,
            ),
            now_monotonic=1.1,
        )
        state = self.projector.snapshot(now_monotonic=1.1)
        self.assertEqual(state["velocity_mps"], {"x": 0.12, "y": -0.34, "z": 0.05})
        self.assertEqual(state["battery"]["voltage_v"], 4.517)
        self.assertEqual(state["battery"]["current_a"], 0.2)
        self.assertIsNone(state["battery"]["remaining_percent"])
        self.assertEqual(state["uwb"]["anchor_ranges_m"], [6.64, 3.81, 2.0, 5.51])

    def test_state_becomes_offline_when_stale(self):
        self.projector.process(FakeMessage("HEARTBEAT", base_mode=0), now_monotonic=10.0)
        self.assertTrue(self.projector.snapshot(now_monotonic=12.9)["online"])
        self.assertFalse(self.projector.snapshot(now_monotonic=13.1)["online"])

    def test_snapshot_reports_independent_message_ages(self):
        self.projector.process(
            FakeMessage(
                "GLOBAL_VISION_POSITION_ESTIMATE",
                x=100,
                y=200,
                z=50,
                roll=0,
                pitch=0,
                yaw=0,
            ),
            now_monotonic=10.0,
        )
        self.projector.process(
            FakeMessage(
                "BATTERY_STATUS",
                voltages=[0, 4100, 100, 100, 100, 100],
                current_battery=0,
                battery_remaining=80,
            ),
            now_monotonic=11.0,
        )
        self.projector.process(
            FakeMessage("HEARTBEAT", base_mode=0), now_monotonic=12.0
        )
        self.projector.mark_coordinator_heartbeat(now_monotonic=11.5)

        state = self.projector.snapshot(now_monotonic=12.5)
        self.assertEqual(0.5, state["fcu_link_age_s"])
        self.assertEqual(2.5, state["position_age_s"])
        self.assertEqual(1.5, state["battery_age_s"])
        self.assertEqual(1.0, state["coordinator_link_age_s"])

    def test_unknown_ages_remain_null_instead_of_looking_fresh(self):
        state = self.projector.snapshot(now_monotonic=10.0)
        self.assertIsNone(state["fcu_link_age_s"])
        self.assertIsNone(state["position_age_s"])
        self.assertIsNone(state["battery_age_s"])
        self.assertIsNone(state["coordinator_link_age_s"])
        self.assertFalse(state["online"])

    def test_unrelated_messages_do_not_refresh_position_or_battery(self):
        self.projector.process(
            FakeMessage(
                "GLOBAL_VISION_POSITION_ESTIMATE",
                x=100,
                y=200,
                z=50,
                roll=0,
                pitch=0,
                yaw=0,
            ),
            now_monotonic=1.0,
        )
        self.projector.process(
            FakeMessage("HEARTBEAT", base_mode=0), now_monotonic=4.0
        )
        state = self.projector.snapshot(now_monotonic=4.5)
        self.assertEqual(0.5, state["fcu_link_age_s"])
        self.assertEqual(3.5, state["position_age_s"])
        self.assertIsNone(state["battery_age_s"])

    def test_disconnect_marks_fcu_age_unknown_but_preserves_sensor_ages(self):
        self.projector.process(
            FakeMessage(
                "GLOBAL_VISION_POSITION_ESTIMATE",
                x=100,
                y=200,
                z=50,
                roll=0,
                pitch=0,
                yaw=0,
            ),
            now_monotonic=2.0,
        )
        self.projector.mark_disconnected()
        state = self.projector.snapshot(now_monotonic=5.0)
        self.assertIsNone(state["fcu_link_age_s"])
        self.assertEqual(3.0, state["position_age_s"])
        self.assertFalse(state["online"])

    def test_json_publisher_emits_one_valid_json_object_per_line(self):
        publisher = JsonPublisher()
        publisher.stream = StringIO()
        publisher.publish({"agent_id": "drone_001", "online": True})
        decoded = json.loads(publisher.stream.getvalue())
        self.assertEqual(decoded["agent_id"], "drone_001")


if __name__ == "__main__":
    unittest.main()
