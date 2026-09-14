#!/usr/bin/env python3
"""Read-only FanciSwarm/Mcontroller MAVLink to JSON adapter.

The only outbound MAVLink message in this module is a GCS HEARTBEAT.  There are
deliberately no arming, takeoff, landing, mission, parameter-write, or movement
APIs in this adapter.
"""

from __future__ import annotations

import argparse
import copy
import json
import logging
import signal
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional, TextIO


LOG = logging.getLogger("uav-readonly-adapter")
ARMED_FLAG = 128  # MAV_MODE_FLAG_SAFETY_ARMED


def utc_iso(unix_time: float) -> str:
    return datetime.fromtimestamp(unix_time, tz=timezone.utc).isoformat(
        timespec="milliseconds"
    ).replace("+00:00", "Z")


def finite_number(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if number != number or number in (float("inf"), float("-inf")):
        return None
    return number


@dataclass(frozen=True)
class AdapterConfig:
    host: str = "192.168.1.126"
    port: int = 333
    agent_id: str = "drone_001"
    uwb_tag_id: int = 1
    frame_id: str = "uwb_map"
    schema_version: str = "1.0"
    heartbeat_interval_s: float = 1.0
    publish_interval_s: float = 0.5
    stale_after_s: float = 3.0
    reconnect_on_silence_s: float = 5.0
    reconnect_initial_s: float = 1.0
    reconnect_max_s: float = 10.0


class StateProjector:
    """Project vendor MAVLink messages into one stable JSON state schema."""

    def __init__(self, config: AdapterConfig):
        self.config = config
        self.last_rx_monotonic: Optional[float] = None
        self.last_position_monotonic: Optional[float] = None
        self.last_battery_monotonic: Optional[float] = None
        self.last_coordinator_monotonic: Optional[float] = None
        self.last_message_type: Optional[str] = None
        self.state: Dict[str, Any] = {
            "schema_version": config.schema_version,
            "timestamp": None,
            "timestamp_iso": None,
            "agent_id": config.agent_id,
            "agent_type": "uav",
            "mission_id": None,
            "task_id": None,
            "online": False,
            "fcu_link_age_s": None,
            "position_age_s": None,
            "battery_age_s": None,
            "coordinator_link_age_s": None,
            "armed": False,
            "flight_mode": {
                "base_mode": None,
                "custom_mode": None,
                "system_status": None,
            },
            "position": {
                "frame_id": config.frame_id,
                "x_m": None,
                "y_m": None,
                "z_m": None,
            },
            "attitude_rad": {"roll": None, "pitch": None, "yaw": None},
            "velocity_mps": {"x": None, "y": None, "z": None},
            "battery": {
                "voltage_v": None,
                "current_a": None,
                "remaining_percent": None,
            },
            "uwb": {
                "tag_id": config.uwb_tag_id,
                "anchor_ranges_m": [None, None, None, None],
            },
            "source": {
                "protocol": "MAVLink",
                "transport": "TCP",
                "endpoint": f"{config.host}:{config.port}",
                "last_message_type": None,
            },
        }

    @staticmethod
    def _get(message: Any, field: str, default: Any = None) -> Any:
        if isinstance(message, dict):
            return message.get(field, default)
        return getattr(message, field, default)

    @staticmethod
    def _message_type(message: Any) -> str:
        if isinstance(message, dict):
            return str(message.get("mavpackettype") or message.get("type") or "UNKNOWN")
        getter = getattr(message, "get_type", None)
        return str(getter() if callable(getter) else type(message).__name__)

    def process(self, message: Any, *, now_monotonic: Optional[float] = None) -> None:
        message_type = self._message_type(message)
        if message_type in ("BAD_DATA", "UNKNOWN"):
            return

        received_at = time.monotonic() if now_monotonic is None else now_monotonic
        self.last_rx_monotonic = received_at
        self.last_message_type = message_type

        if message_type == "HEARTBEAT":
            base_mode = int(self._get(message, "base_mode", 0))
            self.state["armed"] = bool(base_mode & ARMED_FLAG)
            self.state["flight_mode"].update(
                base_mode=base_mode,
                custom_mode=int(self._get(message, "custom_mode", 0)),
                system_status=int(self._get(message, "system_status", 0)),
            )

        elif message_type == "GLOBAL_VISION_POSITION_ESTIMATE":
            self.last_position_monotonic = received_at
            # Mcontroller's vendor firmware publishes x/y/z in centimetres here,
            # despite the standard MAVLink field convention being metres.
            for key in ("x", "y", "z"):
                value = finite_number(self._get(message, key))
                if value is not None:
                    self.state["position"][f"{key}_m"] = round(value / 100.0, 4)
            for key in ("roll", "pitch", "yaw"):
                value = finite_number(self._get(message, key))
                if value is not None:
                    self.state["attitude_rad"][key] = round(value, 6)

        elif message_type == "GLOBAL_POSITION_INT":
            # MAVLink GLOBAL_POSITION_INT velocity fields are centimetres/second.
            for source, target in (("vx", "x"), ("vy", "y"), ("vz", "z")):
                value = finite_number(self._get(message, source))
                if value is not None:
                    self.state["velocity_mps"][target] = round(value / 100.0, 4)

        elif message_type == "BATTERY_STATUS":
            self.last_battery_monotonic = received_at
            voltages = list(self._get(message, "voltages", []) or [])
            # Vendor mapping observed on Mcontroller V7:
            # voltage[1] -> App battery voltage (mV)
            # voltage[2:6] -> UWB anchor ranges (cm)
            if len(voltages) > 1 and 0 < int(voltages[1]) < 65535:
                self.state["battery"]["voltage_v"] = round(int(voltages[1]) / 1000.0, 3)
            if len(voltages) >= 6:
                ranges = []
                for raw in voltages[2:6]:
                    raw_int = int(raw)
                    ranges.append(None if raw_int in (0, 65535) else round(raw_int / 100.0, 3))
                self.state["uwb"]["anchor_ranges_m"] = ranges

            current = int(self._get(message, "current_battery", -1))
            if current >= 0:
                self.state["battery"]["current_a"] = round(current / 100.0, 3)
            remaining = int(self._get(message, "battery_remaining", -1))
            # Mcontroller commonly emits 0 when percentage is unavailable;
            # do not turn that vendor sentinel into a false critical battery.
            if 1 <= remaining <= 100:
                self.state["battery"]["remaining_percent"] = remaining

    def mark_coordinator_heartbeat(
        self, *, now_monotonic: Optional[float] = None
    ) -> None:
        """Record a heartbeat received from the total Agent/DDS layer.

        The MAVLink reader itself does not communicate with the total Agent.
        Its integration layer must call this method after receiving a genuine
        coordinator heartbeat.  Until then coordinator_link_age_s stays null,
        which the safety supervisor treats as unknown/unavailable.
        """
        self.last_coordinator_monotonic = (
            time.monotonic() if now_monotonic is None else now_monotonic
        )

    @staticmethod
    def _age_seconds(
        last_seen: Optional[float], now_monotonic: float
    ) -> Optional[float]:
        if last_seen is None:
            return None
        # A monotonic clock should never move backwards, but clamp defensively
        # so malformed test/integration timestamps cannot produce negative age.
        return round(max(0.0, now_monotonic - last_seen), 3)

    def snapshot(
        self,
        *,
        now_unix: Optional[float] = None,
        now_monotonic: Optional[float] = None,
    ) -> Dict[str, Any]:
        unix_time = time.time() if now_unix is None else now_unix
        monotonic_time = time.monotonic() if now_monotonic is None else now_monotonic
        result = copy.deepcopy(self.state)
        result["timestamp"] = round(unix_time, 3)
        result["timestamp_iso"] = utc_iso(unix_time)
        result["fcu_link_age_s"] = self._age_seconds(
            self.last_rx_monotonic, monotonic_time
        )
        result["position_age_s"] = self._age_seconds(
            self.last_position_monotonic, monotonic_time
        )
        result["battery_age_s"] = self._age_seconds(
            self.last_battery_monotonic, monotonic_time
        )
        result["coordinator_link_age_s"] = self._age_seconds(
            self.last_coordinator_monotonic, monotonic_time
        )
        result["online"] = (
            result["fcu_link_age_s"] is not None
            and result["fcu_link_age_s"] <= self.config.stale_after_s
        )
        result["source"]["last_message_type"] = self.last_message_type
        return result

    def mark_disconnected(self) -> None:
        self.last_rx_monotonic = None


class JsonPublisher:
    def __init__(self, output_path: Optional[Path] = None):
        self._owned_file: Optional[TextIO] = None
        if output_path is None:
            self.stream = sys.stdout
        else:
            output_path.parent.mkdir(parents=True, exist_ok=True)
            self._owned_file = output_path.open("a", encoding="utf-8", buffering=1)
            self.stream = self._owned_file

    def publish(self, state: Dict[str, Any]) -> None:
        print(json.dumps(state, ensure_ascii=False, separators=(",", ":")), file=self.stream, flush=True)

    def close(self) -> None:
        if self._owned_file is not None:
            self._owned_file.close()


class ReadOnlyMavlinkAdapter:
    def __init__(self, config: AdapterConfig, publisher: JsonPublisher):
        self.config = config
        self.publisher = publisher
        self.projector = StateProjector(config)
        self.running = True

    def stop(self, *_args: Any) -> None:
        self.running = False

    @staticmethod
    def _load_mavutil() -> Any:
        try:
            from pymavlink import mavutil
        except ImportError as exc:
            raise RuntimeError(
                "缺少 pymavlink。请先激活 ~/mavlink_env，或执行 pip install pymavlink。"
            ) from exc
        return mavutil

    def _connect(self, mavutil: Any) -> Any:
        endpoint = f"tcp:{self.config.host}:{self.config.port}"
        LOG.info("正在连接 %s", endpoint)
        return mavutil.mavlink_connection(
            endpoint,
            source_system=254,
            source_component=190,
            autoreconnect=False,
        )

    @staticmethod
    def _send_readonly_gcs_heartbeat(connection: Any, mavutil: Any) -> None:
        """Send only the passive GCS presence heartbeat expected by fcu_core."""
        connection.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )

    def _run_connection(self, connection: Any, mavutil: Any) -> None:
        next_heartbeat = 0.0
        next_publish = 0.0
        last_rx = time.monotonic()
        while self.running:
            now = time.monotonic()
            if now >= next_heartbeat:
                self._send_readonly_gcs_heartbeat(connection, mavutil)
                next_heartbeat = now + self.config.heartbeat_interval_s

            timeout = min(
                max(next_heartbeat - now, 0.01),
                max(next_publish - now, 0.01),
                0.25,
            )
            message = connection.recv_match(blocking=True, timeout=timeout)
            if message is not None:
                self.projector.process(message)
                last_rx = time.monotonic()

            now = time.monotonic()
            if now - last_rx > self.config.reconnect_on_silence_s:
                raise ConnectionError(
                    f"连续 {self.config.reconnect_on_silence_s:.1f} 秒未收到 MAVLink 数据"
                )
            if now >= next_publish:
                self.publisher.publish(self.projector.snapshot(now_monotonic=now))
                next_publish = now + self.config.publish_interval_s

    def run(self) -> int:
        mavutil = self._load_mavutil()
        reconnect_delay = self.config.reconnect_initial_s
        while self.running:
            connection = None
            try:
                connection = self._connect(mavutil)
                LOG.info("TCP 已连接；适配器处于只读模式")
                reconnect_delay = self.config.reconnect_initial_s
                self._run_connection(connection, mavutil)
            except (OSError, EOFError, ConnectionError) as exc:
                self.projector.mark_disconnected()
                self.publisher.publish(self.projector.snapshot())
                if self.running:
                    LOG.warning("连接中断：%s；%.1f 秒后重试", exc, reconnect_delay)
                    time.sleep(reconnect_delay)
                    reconnect_delay = min(reconnect_delay * 2, self.config.reconnect_max_s)
            finally:
                if connection is not None:
                    try:
                        connection.close()
                    except Exception:
                        LOG.debug("关闭 MAVLink 连接失败", exc_info=True)
        return 0


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("必须大于 0")
    return parsed


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Mcontroller MAVLink 只读 JSON 适配器")
    parser.add_argument("--host", default="192.168.1.126", help="Mcontroller IP")
    parser.add_argument("--port", type=int, default=333, help="Mcontroller TCP 端口")
    parser.add_argument("--agent-id", default="drone_001")
    parser.add_argument("--uwb-tag-id", type=int, default=1)
    parser.add_argument("--frame-id", default="uwb_map")
    parser.add_argument("--publish-rate", type=positive_float, default=2.0, help="JSON 输出频率 Hz")
    parser.add_argument("--output", type=Path, help="追加写入 JSONL 文件；默认输出到终端")
    parser.add_argument("--log-level", choices=("DEBUG", "INFO", "WARNING", "ERROR"), default="INFO")
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(message)s",
        stream=sys.stderr,
    )
    config = AdapterConfig(
        host=args.host,
        port=args.port,
        agent_id=args.agent_id,
        uwb_tag_id=args.uwb_tag_id,
        frame_id=args.frame_id,
        publish_interval_s=1.0 / args.publish_rate,
    )
    publisher = JsonPublisher(args.output)
    adapter = ReadOnlyMavlinkAdapter(config, publisher)
    signal.signal(signal.SIGINT, adapter.stop)
    signal.signal(signal.SIGTERM, adapter.stop)
    try:
        return adapter.run()
    except RuntimeError as exc:
        LOG.error("%s", exc)
        return 2
    finally:
        publisher.close()


if __name__ == "__main__":
    raise SystemExit(main())
