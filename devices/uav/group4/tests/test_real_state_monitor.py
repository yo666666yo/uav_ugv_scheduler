import time
import unittest

from agent.real_state_monitor import RealStateMonitor


class RealStateMonitorTests(unittest.TestCase):
    def test_provider_exception_fails_closed(self):
        def broken():
            raise RuntimeError("boom")

        monitor = RealStateMonitor(broken, poll_interval_s=0.001)
        monitor.start()
        self.assertTrue(monitor.wait_initialized(0.2))
        state = monitor.snapshot()
        monitor.stop()
        self.assertFalse(state["readonly_ready"])
        self.assertFalse(state["control_ready"])
        self.assertIn("RuntimeError", state["monitor_error"])

    def test_old_snapshot_is_marked_stale_and_not_ready(self):
        monitor = RealStateMonitor(
            lambda: {"readonly_ready": True, "control_ready": True},
            poll_interval_s=1.0,
            stale_after_s=0.001,
        )
        monitor.start()
        self.assertTrue(monitor.wait_initialized(0.2))
        monitor.stop()
        time.sleep(0.005)
        state = monitor.snapshot()
        self.assertTrue(state["state_stale"])
        self.assertFalse(state["readonly_ready"])
        self.assertFalse(state["control_ready"])


if __name__ == "__main__":
    unittest.main()
