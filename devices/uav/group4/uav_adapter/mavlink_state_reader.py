"""Read-only MAVLink telemetry cache (never sends vehicle commands)."""
from __future__ import annotations

import time
from typing import Any, Dict


class MavlinkStateReader:
    def __init__(
        self,
        connection: str = "tcp:192.168.1.126:333",
        timeout_s: float = 2.0,
        heartbeat_interval_s: float = 1.0,
    ):
        try:
            from pymavlink import mavutil
        except ImportError as exc:
            raise RuntimeError("缺少 pymavlink") from exc
        self.mavutil = mavutil
        self.link = mavutil.mavlink_connection(
            connection, timeout=timeout_s, autoreconnect=True
        )
        self.timeout_s = float(timeout_s)
        self.heartbeat_interval_s = float(heartbeat_interval_s)
        self.last: Dict[str, Any] = {}
        self.last_time = 0.0
        self.last_position_time = 0.0
        self.last_battery_time = 0.0
        self.last_heartbeat_sent = 0.0
        self._send_readonly_heartbeat()

    def _send_readonly_heartbeat(self) -> None:
        self.link.mav.heartbeat_send(
            self.mavutil.mavlink.MAV_TYPE_GCS,
            self.mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            self.mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,
            0,
            self.mavutil.mavlink.MAV_STATE_ACTIVE,
        )
        self.last_heartbeat_sent = time.monotonic()

    @staticmethod
    def _age(last_seen: float, now: float) -> Any:
        return None if not last_seen else max(0.0, now - last_seen)

    def __call__(self) -> Dict[str, Any]:
        now = time.monotonic()
        if now - self.last_heartbeat_sent >= self.heartbeat_interval_s:
            self._send_readonly_heartbeat()

        for _ in range(100):
            msg = self.link.recv_match(
                type=[
                    "HEARTBEAT",
                    "BATTERY_STATUS",
                    "GLOBAL_VISION_POSITION_ESTIMATE",
                ],
                blocking=False,
            )
            if msg is None:
                break
            received = time.monotonic()
            typ = msg.get_type()
            if typ == "HEARTBEAT":
                self.last["armed"] = bool(
                    msg.base_mode
                    & self.mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
                self.last["custom_mode"] = msg.custom_mode
            elif typ == "BATTERY_STATUS":
                voltages = list(msg.voltages)
                valid_voltages = [v for v in voltages if v not in (0, 65535)]
                # Vendor reports per-cell millivolts; expose pack voltage.
                voltage_mv = sum(valid_voltages[:2]) if valid_voltages else None
                remaining = int(msg.battery_remaining)
                self.last["battery"] = {
                    "voltage_v": None if voltage_mv is None else voltage_mv / 1000.0,
                    "remaining_percent": (
                        None if remaining in (-1, 0, 255) else float(remaining)
                    ),
                }
                self.last_battery_time = received
            elif typ == "GLOBAL_VISION_POSITION_ESTIMATE":
                self.last["position"] = {
                    "frame_id": "uwb_map",
                    "x_m": msg.x / 100.0,
                    "y_m": msg.y / 100.0,
                    "z_m": msg.z / 100.0,
                }
                self.last["attitude_rad"] = {
                    "roll": msg.roll,
                    "pitch": msg.pitch,
                    "yaw": msg.yaw,
                }
                self.last_position_time = received
            self.last_time = received

        now = time.monotonic()
        position_age = self._age(self.last_position_time, now)
        battery_age = self._age(self.last_battery_time, now)
        return {
            "online": bool(self.last_time and now - self.last_time <= self.timeout_s),
            **self.last,
            "position_valid": (
                position_age is not None and position_age <= self.timeout_s
            ),
            "battery_valid": battery_age is not None and battery_age <= self.timeout_s,
            "position_age_s": position_age,
            "battery_age_s": battery_age,
            "timestamp_monotonic": now,
        }

    def close(self) -> None:
        self.link.close()
