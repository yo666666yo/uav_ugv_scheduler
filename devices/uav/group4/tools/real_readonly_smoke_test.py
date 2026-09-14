#!/usr/bin/env python3
"""Smoke-test real MAVLink telemetry through the fail-closed aggregator.

This tool never creates a ROS publisher and never sends vehicle commands. The
MAVLink reader sends only one passive GCS heartbeat when it connects.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from agent.real_state_aggregator import RealStateAggregator
from uav_adapter.mavlink_state_reader import MavlinkStateReader


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--connection", default="tcp:192.168.1.126:333")
    parser.add_argument("--duration-s", type=float, default=10.0)
    args = parser.parse_args()

    reader = MavlinkStateReader(connection=args.connection, timeout_s=2.0)
    aggregator = RealStateAggregator(lambda: {"online": False}, reader)
    deadline = time.monotonic() + args.duration_s
    ready_seen = False
    last_print = 0.0
    try:
        while time.monotonic() < deadline:
            state = aggregator()
            ready_seen = ready_seen or state["readonly_ready"]
            now = time.monotonic()
            if now - last_print >= 1.0:
                print(json.dumps(state, ensure_ascii=False), flush=True)
                last_print = now
            time.sleep(0.05)
    finally:
        reader.close()

    print(json.dumps({"readonly_initialization_passed": ready_seen}), flush=True)
    return 0 if ready_seen else 1


if __name__ == "__main__":
    raise SystemExit(main())
