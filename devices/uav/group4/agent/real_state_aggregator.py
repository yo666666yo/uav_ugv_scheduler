"""Fail-closed aggregation of ROS and direct MAVLink UAV state.

Telemetry may fall back to MAVLink when the ROS bridge is unavailable. This
fallback is read-only: it never implies that the ROS command path is ready.
"""
from __future__ import annotations

import time
from typing import Any, Callable, Dict, Mapping


def _valid_position(source: Mapping[str, Any]) -> bool:
    if source.get("position_valid") is False:
        return False
    position = source.get("position") or {}
    return all(position.get(key) is not None for key in ("x_m", "y_m", "z_m"))


def _non_null_overlay(base: Mapping[str, Any], update: Mapping[str, Any]) -> Dict[str, Any]:
    result = dict(base)
    result.update({key: value for key, value in update.items() if value is not None})
    return result


class RealStateAggregator:
    def __init__(
        self,
        ros_reader: Callable[[], Dict[str, Any]],
        mav_reader: Callable[[], Dict[str, Any]],
        timeout_s: float = 1.5,
    ):
        self.ros_reader = ros_reader
        self.mav_reader = mav_reader
        self.timeout_s = float(timeout_s)

    def close(self) -> None:
        closed = set()
        for reader in (self.ros_reader, self.mav_reader):
            if id(reader) in closed:
                continue
            closed.add(id(reader))
            close = getattr(reader, "close", None)
            if callable(close):
                close()

    def __call__(self) -> Dict[str, Any]:
        now = time.monotonic()
        ros = self.ros_reader() or {}
        mav = self.mav_reader() or {}
        ros_online = bool(ros.get("online"))
        mav_online = bool(mav.get("online"))

        if ros_online and _valid_position(ros):
            position = dict(ros["position"])
            position_source = "ROS2"
        elif mav_online and _valid_position(mav):
            position = dict(mav["position"])
            position_source = "MAVLINK"
        else:
            position = {
                "frame_id": "uwb_map",
                "x_m": None,
                "y_m": None,
                "z_m": None,
            }
            position_source = None

        armed = mav.get("armed") if mav_online and "armed" in mav else ros.get("armed")
        armed_valid = bool(
            (mav_online and mav.get("armed_valid", "armed" in mav))
            or ros.get("armed_valid", False)
        )
        custom_mode = mav.get("custom_mode") if mav_online else None
        attitude = _non_null_overlay(
            mav.get("attitude_rad", {}), ros.get("attitude_rad", {})
        )

        ros_battery_valid = bool(
            ros.get("battery_valid", any(value is not None for value in ros.get("battery", {}).values()))
        )
        mav_battery_valid = bool(
            mav.get("battery_valid", any(value is not None for value in mav.get("battery", {}).values()))
        )
        battery = _non_null_overlay(ros.get("battery", {}), mav.get("battery", {}))
        if battery.get("remaining_percent") in (0, -1, 255):
            battery["remaining_percent"] = None
        voltage = battery.get("voltage_v")
        remaining = battery.get("remaining_percent")
        voltage_ok = voltage is not None and float(voltage) >= 7.0
        percent_ok = remaining is None or float(remaining) >= 15.0
        battery["battery_ok"] = voltage_ok and percent_ok

        telemetry_online = ros_online or mav_online
        position_valid = _valid_position({"position": position})
        battery_valid = mav_battery_valid or ros_battery_valid
        readonly_ready = (
            telemetry_online and position_valid and battery_valid and voltage is not None
        )

        # A dedicated bridge/control health probe must explicitly assert this.
        # Receiving telemetry alone must never enable real flight commands.
        control_ready = bool(ros.get("control_ready", False))
        if position_source == "MAVLINK":
            control_ready = False

        return {
            "online": telemetry_online,
            "device_online": telemetry_online,
            "readonly_ready": readonly_ready,
            "control_ready": control_ready,
            "telemetry_mode": (
                "ROS2_AND_MAVLINK"
                if ros_online and mav_online
                else "ROS2_ONLY"
                if ros_online
                else "MAVLINK_ONLY"
                if mav_online
                else "OFFLINE"
            ),
            "position_source": position_source,
            "source_health": {
                "ros2_online": ros_online,
                "mavlink_online": mav_online,
            },
            "position": position,
            "position_valid": position_valid,
            "attitude_rad": attitude,
            "armed": armed,
            "armed_valid": armed_valid,
            "custom_mode": custom_mode,
            "battery": battery,
            "battery_valid": battery_valid,
            "state_timestamp": now,
        }


class OfflineStateReader:
    """Explicitly unavailable source used when a transport must stay disabled."""

    def __call__(self) -> Dict[str, Any]:
        return {"online": False}
