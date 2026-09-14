import unittest

from agent.real_state_aggregator import RealStateAggregator


class AggregatorTests(unittest.TestCase):
    def test_merges_and_marks_vendor_zero_unknown(self):
        aggregator = RealStateAggregator(
            lambda: {
                "online": True,
                "position": {
                    "frame_id": "uwb_map",
                    "x_m": 0.7,
                    "y_m": 4.0,
                    "z_m": 0.0,
                },
                "battery": {"voltage_v": 7.9},
            },
            lambda: {
                "online": True,
                "armed": False,
                "custom_mode": 3,
                "battery": {"remaining_percent": 0},
            },
        )
        state = aggregator()
        self.assertFalse(state["armed"])
        self.assertIsNone(state["battery"]["remaining_percent"])
        self.assertEqual(state["custom_mode"], 3)
        self.assertEqual(state["telemetry_mode"], "ROS2_AND_MAVLINK")

    def test_mavlink_fallback_allows_readonly_initialization(self):
        aggregator = RealStateAggregator(
            lambda: {"online": False},
            lambda: {
                "online": True,
                "armed": False,
                "custom_mode": 3,
                "position": {
                    "frame_id": "uwb_map",
                    "x_m": 0.69,
                    "y_m": 0.88,
                    "z_m": 0.02,
                },
                "battery": {"voltage_v": 7.47, "remaining_percent": None},
            },
        )
        state = aggregator()
        self.assertTrue(state["device_online"])
        self.assertTrue(state["position_valid"])
        self.assertTrue(state["readonly_ready"])
        self.assertEqual(state["position_source"], "MAVLINK")
        self.assertEqual(state["telemetry_mode"], "MAVLINK_ONLY")
        self.assertTrue(state["battery"]["battery_ok"])
        self.assertFalse(state["control_ready"])

    def test_ros_position_is_preferred_when_both_are_online(self):
        aggregator = RealStateAggregator(
            lambda: {
                "online": True,
                "position": {
                    "frame_id": "uwb_map",
                    "x_m": 1.0,
                    "y_m": 2.0,
                    "z_m": 0.1,
                },
                "battery": {"voltage_v": 7.8},
            },
            lambda: {
                "online": True,
                "position": {
                    "frame_id": "uwb_map",
                    "x_m": 9.0,
                    "y_m": 9.0,
                    "z_m": 9.0,
                },
                "armed": False,
            },
        )
        state = aggregator()
        self.assertEqual(state["position_source"], "ROS2")
        self.assertEqual(state["position"]["x_m"], 1.0)
        self.assertFalse(state["control_ready"])

    def test_both_sources_offline_fail_closed(self):
        state = RealStateAggregator(
            lambda: {"online": False}, lambda: {"online": False}
        )()
        self.assertFalse(state["device_online"])
        self.assertFalse(state["position_valid"])
        self.assertFalse(state["readonly_ready"])
        self.assertFalse(state["control_ready"])
        self.assertIsNone(state["armed"])

    def test_online_heartbeat_does_not_make_stale_position_valid(self):
        state = RealStateAggregator(
            lambda: {"online": False},
            lambda: {
                "online": True,
                "position_valid": False,
                "position": {
                    "frame_id": "uwb_map",
                    "x_m": 0.69,
                    "y_m": 0.88,
                    "z_m": 0.02,
                },
                "battery_valid": True,
                "battery": {"voltage_v": 7.4},
            },
        )()
        self.assertTrue(state["device_online"])
        self.assertFalse(state["position_valid"])
        self.assertFalse(state["readonly_ready"])
        self.assertFalse(state["control_ready"])


if __name__ == "__main__":
    unittest.main()
