import unittest
from agent.contracts import Pose
from behaviors.hover_test import HoverTest, HoverTestConfig
from uav_adapter.simulated_uav_adapter import SimulatedUAVAdapter


class HoverTests(unittest.TestCase):
    def adapter(self):
        a = SimulatedUAVAdapter(agent_id="drone_001", initial_pose=Pose("uwb_map", .7, 4., 0., 0.))
        a._state["device_online"] = True
        a._state["battery"] = {"battery_ok": True, "remaining_percent": 80}
        a._state["position_valid"] = True
        return a

    def test_hover_sequence(self):
        a = self.adapter()
        result = HoverTest(a, HoverTestConfig(.8, 0)).execute()
        self.assertTrue(result["ok"])
        self.assertEqual([x["action"] for x in a.action_log], ["takeoff", "hold", "land"])

    def test_real_disabled_fails_closed(self):
        a = self.adapter()
        a.is_simulation = False
        with self.assertRaisesRegex(RuntimeError, "CONTROL_DISABLED"):
            HoverTest(a, HoverTestConfig(.8, 0)).execute()
        self.assertEqual(a.action_log, [])


if __name__ == "__main__":
    unittest.main()
