#!/usr/bin/env python3
"""Run the Group 4 UAV agent over stdin/stdout JSON Lines."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any, Dict

from agent.contracts import Pose, TaskAssignment
from agent.drone_agent_service import DroneAgentService
from agent.real_state_aggregator import OfflineStateReader, RealStateAggregator
from agent.real_state_monitor import RealStateMonitor
from agent.task_store import JsonTaskStore
from agent.uav_agent import build_simulation, load_json
from behaviors.inspect_target import InspectionBehavior
from perception.simulated_target_detector import SimulatedTargetDetector
from transports.terminal_json_transport import TerminalJsonTransport


class DryRunInspectionExecutor:
    def __init__(self, config: Dict[str, Any], output_directory: Path, found: bool) -> None:
        self.config = config
        self.output_directory = output_directory
        self.found = found

    def execute(
        self,
        task: TaskAssignment,
        feedback_callback,
        initial_state: Dict[str, Any] | None = None,
    ) -> Dict[str, Any]:
        initial_pose = None
        battery_ok = True
        if initial_state is not None:
            position = initial_state["position"]
            initial_pose = Pose(
                frame_id=str(position.get("frame_id", "uwb_map")),
                x_m=float(position["x_m"]),
                y_m=float(position["y_m"]),
                z_m=float(position["z_m"]),
                yaw_rad=float(initial_state["attitude_rad"]["yaw"]),
            )
            battery_ok = bool(initial_state.get("battery", {}).get("battery_ok"))
        adapter, safety = build_simulation(
            self.config,
            self.output_directory,
            initial_pose=initial_pose,
            battery_ok=battery_ok,
        )

        def upload_image(path: Path, assignment: TaskAssignment) -> str:
            return f"/images/{assignment.mission_id}/{path.name}"

        behavior = InspectionBehavior(
            adapter=adapter,
            safety=safety,
            feedback_callback=feedback_callback,
            upload_image=upload_image,
            target_detector=SimulatedTargetDetector(found=self.found, confidence=0.90),
            sleep=lambda _seconds: None,
        )
        result = behavior.execute(task)
        result["simulated_actions"] = adapter.action_log
        return result


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="第4组终端 JSON 无人机 Agent")
    parser.add_argument("--config", type=Path, default=Path("config/drone_001.sim.json"))
    parser.add_argument("--mode", choices=("readonly", "dry-run"), default="dry-run")
    parser.add_argument(
        "--state-source",
        choices=("simulated", "ros2"),
        default="simulated",
        help="ros2 会订阅真实状态 Topic，但仍不会启用真实控制",
    )
    parser.add_argument("--state-file", type=Path, default=Path("logs/terminal_agent/tasks.json"))
    parser.add_argument("--output-directory", type=Path, default=Path("logs/terminal_agent"))
    parser.add_argument(
        "--detector-result",
        choices=("found", "not-found"),
        default="found",
    )
    parser.add_argument("--state-publish-interval-s", type=float, default=1.0)
    return parser.parse_args(argv)


def _absolute_topic(name: str) -> str:
    return name if name.startswith("/") else f"/{name}"


def build_ros2_state_monitor(
    config: Dict[str, Any], publish_interval_s: float
) -> RealStateMonitor:
    from uav_adapter.ros2_state_reader import Ros2StateReader

    vendor = config.get("vendor_interface", {})
    reader = Ros2StateReader(
        odom_topic=_absolute_topic(vendor.get("state_topic", "odom_global_001")),
        battery_topic=_absolute_topic(vendor.get("battery_topic", "batt_now_001")),
        battery_remaining_topic=_absolute_topic(
            vendor.get("battery_remaining_topic", "battery_remaining_001")
        ),
        imu_topic=_absolute_topic(vendor.get("imu_topic", "imu_global_001")),
        timeout_s=1.0,
        battery_timeout_s=2.5,
        attitude_timeout_s=1.5,
        coordinate_scale=1.0,
    )
    aggregator = RealStateAggregator(
        ros_reader=reader,
        mav_reader=OfflineStateReader(),
        timeout_s=1.5,
    )
    return RealStateMonitor(
        aggregator,
        poll_interval_s=0.1,
        publish_interval_s=publish_interval_s,
        stale_after_s=1.5,
    )


def main(argv=None) -> int:
    args = parse_args(argv)
    config = load_json(args.config)
    agent_id = str(config["agent"]["agent_id"])
    executor = None
    if args.mode == "dry-run":
        executor = DryRunInspectionExecutor(
            config,
            args.output_directory,
            found=args.detector_result == "found",
        )
    transport = TerminalJsonTransport(sys.stdin, sys.stdout)
    state_monitor = None
    if args.state_source == "ros2":
        state_monitor = build_ros2_state_monitor(
            config, args.state_publish_interval_s
        )
    service = DroneAgentService(
        agent_id=agent_id,
        mode=args.mode,
        transport=transport,
        task_store=JsonTaskStore(args.state_file),
        executor=executor,
        state_monitor=state_monitor,
    )
    return service.run()


if __name__ == "__main__":
    raise SystemExit(main())
