import inspect
import unittest
from types import SimpleNamespace

from uav_adapter.ros2_state_reader import (
    DEFAULT_COORDINATE_SCALE,
    Ros2StateReader,
)


class Ros2StateReaderUnitTests(unittest.TestCase):
    def test_vendor_bridge_odom_is_not_scaled_twice(self):
        default = inspect.signature(Ros2StateReader.__init__).parameters[
            "coordinate_scale"
        ].default
        self.assertEqual(default, 1.0)
        self.assertEqual(DEFAULT_COORDINATE_SCALE, 1.0)

    def test_odometry_quaternion_can_supply_yaw_without_imu_topic(self):
        quaternion = SimpleNamespace(x=0.0, y=0.0, z=1.0, w=0.0)
        attitude = Ros2StateReader._quaternion_to_euler(quaternion)
        self.assertAlmostEqual(abs(attitude["yaw"]), 3.141592653589793)


if __name__ == "__main__":
    unittest.main()
