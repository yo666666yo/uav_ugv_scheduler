"""ROS 2 state reader for the vendor bridge (strictly read-only)."""
from __future__ import annotations
import math
import time
from typing import Any, Dict

DEFAULT_COORDINATE_SCALE = 1.0


class Ros2StateReader:
    def __init__(
        self,
        odom_topic="/odom_global_001",
        battery_topic="/batt_now_001",
        battery_remaining_topic="/battery_remaining_001",
        imu_topic="/imu_global_001",
        armed_topic="/armed_001",
        timeout_s=1.0,
        battery_timeout_s=2.5,
        attitude_timeout_s=1.5,
        coordinate_scale=DEFAULT_COORDINATE_SCALE,
    ):
        try:
            import rclpy
            from nav_msgs.msg import Odometry
            from sensor_msgs.msg import Imu
            from std_msgs.msg import Float32, Bool
        except ImportError as exc:
            raise RuntimeError("缺少 ROS 2 Humble Python 依赖") from exc
        if not rclpy.ok(): rclpy.init(args=None)
        self._rclpy, self._node = rclpy, rclpy.create_node("group4_state_reader")
        self._timeout, self._scale = float(timeout_s), float(coordinate_scale)
        self._battery_timeout = float(battery_timeout_s)
        self._attitude_timeout = float(attitude_timeout_s)
        self._odom = self._battery = self._battery_remaining = self._attitude = None
        self._armed = None
        self._odom_time = self._battery_time = 0.0
        self._battery_remaining_time = self._attitude_time = 0.0
        self._node.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self._node.create_subscription(Float32, battery_topic, self._on_battery, 10)
        self._node.create_subscription(
            Float32, battery_remaining_topic, self._on_battery_remaining, 10
        )
        self._node.create_subscription(Imu, imu_topic, self._on_imu, 10)
        self._node.create_subscription(Bool, armed_topic, self._on_armed, 10)

    def _on_armed(self, msg):
        self._armed = bool(msg.data)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        # fcu_bridge_001 has already converted the vendor centimetre values to
        # ROS SI units (metres). The default scale must therefore remain 1.0.
        # Convert ROS map handedness to the App/UWB frame.
        self._odom = (p.x * self._scale, -p.y * self._scale, p.z * self._scale)
        self._odom_time = time.monotonic()
        self._attitude = self._quaternion_to_euler(msg.pose.pose.orientation)
        self._attitude["yaw"] = -self._attitude["yaw"]
        self._attitude_time = self._odom_time

    def _on_battery(self, msg):
        self._battery = float(msg.data)
        self._battery_time = time.monotonic()

    def _on_battery_remaining(self, msg):
        value = float(msg.data)
        # The vendor uses zero when percentage is unavailable.
        self._battery_remaining = None if value in (-1.0, 0.0, 255.0) else value
        self._battery_remaining_time = time.monotonic()

    @staticmethod
    def _quaternion_to_euler(q):
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1 else math.asin(sinp)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return {"roll": roll, "pitch": pitch, "yaw": yaw}

    def _on_imu(self, msg):
        self._attitude = self._quaternion_to_euler(msg.orientation)
        self._attitude_time = time.monotonic()

    def __call__(self) -> Dict[str, Any]:
        self._rclpy.spin_once(self._node, timeout_sec=0.05)
        now = time.monotonic()
        online = self._odom is not None and now - self._odom_time <= self._timeout
        pos = self._odom or (None, None, None)
        voltage = (
            self._battery
            if self._battery is not None
            and now - self._battery_time <= self._battery_timeout
            else None
        )
        remaining = (
            self._battery_remaining
            if now - self._battery_remaining_time <= self._battery_timeout
            else None
        )
        attitude = (
            dict(self._attitude)
            if self._attitude is not None
            and now - self._attitude_time <= self._attitude_timeout
            else {}
        )
        position_valid = online and all(value is not None for value in pos)
        battery_valid = voltage is not None
        return {"online": online, "position": {"frame_id": "uwb_map", "x_m": pos[0], "y_m": pos[1], "z_m": pos[2]},
                "position_valid": position_valid, "attitude_rad": attitude,
                "armed": self._armed, "armed_valid": self._armed is not None,
                "battery_valid": battery_valid,
                "battery": {"voltage_v": voltage, "remaining_percent": remaining,
                "battery_ok": (remaining is not None and remaining >= 15.0) or (voltage is not None and voltage >= 7.0)},
                "timestamp_monotonic": now}

    def close(self):
        self._node.destroy_node()
