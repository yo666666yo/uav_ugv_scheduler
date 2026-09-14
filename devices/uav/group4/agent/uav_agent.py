#!/usr/bin/env python3
"""Minimal command-line UAV agent runner for the safe simulation mode."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, Optional

from agent.contracts import Pose, TaskAssignment, TaskFeedback
from behaviors.inspect_target import (
    FlightBounds,
    HomeSamplingConfig,
    InspectionBehavior,
    InspectionSafetyConfig,
    NoFlyZone,
)
from uav_adapter.simulated_uav_adapter import SimulatedUAVAdapter
from perception.simulated_target_detector import SimulatedTargetDetector
from perception.yolo_target_detector import YOLOTargetDetector


def load_json(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def build_simulation(
    config: Dict[str, Any],
    output_directory: Path,
    *,
    initial_pose: Optional[Pose] = None,
    battery_ok: bool = True,
):
    agent_config = config["agent"]
    simulation = config.get("simulation") or config.get("flight_safety")
    if simulation is None:
        raise ValueError("配置缺少 simulation 或 flight_safety")
    configured_initial_pose = simulation.get(
        "initial_pose",
        {
            "frame_id": "uwb_map",
            "x_m": 0.0,
            "y_m": 0.0,
            "z_m": 0.0,
            "yaw_rad": 0.0,
        },
    )
    adapter = SimulatedUAVAdapter(
        agent_id=agent_config["agent_id"],
        initial_pose=initial_pose or Pose.from_dict(configured_initial_pose),
        battery_ok=battery_ok,
    )
    bounds_config = simulation["flight_bounds"]
    bounds = FlightBounds(
        x_min_m=float(bounds_config["x_min_m"]),
        x_max_m=float(bounds_config["x_max_m"]),
        y_min_m=float(bounds_config["y_min_m"]),
        y_max_m=float(bounds_config["y_max_m"]),
        safe_height_min_m=float(bounds_config["safe_height_min_m"]),
        safe_height_max_m=float(bounds_config["safe_height_max_m"]),
    )
    zones = tuple(
        NoFlyZone(
            zone_id=str(zone["zone_id"]),
            min_z_m=float(zone["min_z_m"]),
            max_z_m=float(zone["max_z_m"]),
            polygon=tuple(tuple(point) for point in zone["polygon"]),
        )
        for zone in simulation.get("no_fly_zones", [])
    )
    sampling = simulation.get("home_sampling", {})
    safety = InspectionSafetyConfig(
        agent_id=agent_config["agent_id"],
        cruise_height_m=float(simulation["cruise_height_m"]),
        inspection_distance_m=float(simulation["inspection_distance_m"]),
        stable_time_s=float(simulation.get("stable_time_s", 2.0)),
        bounds=bounds,
        home_sampling=HomeSamplingConfig(
            minimum_samples=int(sampling.get("minimum_samples", 10)),
            sample_interval_s=float(sampling.get("sample_interval_s", 0.2)),
            max_position_jitter_m=float(sampling.get("max_position_jitter_m", 0.05)),
            max_yaw_jitter_rad=float(sampling.get("max_yaw_jitter_rad", 0.15)),
        ),
        no_fly_zones=zones,
        allowed_flight_polygon=tuple(
            tuple(point) for point in simulation.get("allowed_flight_polygon", [])
        ),
        real_control_enabled=False,
        image_directory=output_directory / "images",
    )
    return adapter, safety


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="第4组无人机 Agent（当前仅仿真）")
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--task", type=Path, required=True)
    parser.add_argument("--output-directory", type=Path, default=Path("logs/simulation"))
    parser.add_argument(
        "--detector",
        choices=("simulated-found", "simulated-not-found", "yolo"),
        default="simulated-found",
        help="目标识别器；默认使用不依赖模型的仿真识别器",
    )
    parser.add_argument("--model", default="yolo26n.pt", help="YOLO 模型文件或模型名")
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    config = load_json(args.config)
    task = TaskAssignment.from_dict(load_json(args.task))
    adapter, safety = build_simulation(config, args.output_directory)
    if args.detector == "yolo":
        target_detector = YOLOTargetDetector(args.model)
    else:
        target_detector = SimulatedTargetDetector(
            found=args.detector == "simulated-found",
            confidence=0.90,
        )

    if task.target_agent_id != safety.agent_id:
        print(
            json.dumps(
                {
                    "ignored": True,
                    "reason": "TARGET_AGENT_MISMATCH",
                    "target_agent_id": task.target_agent_id,
                    "agent_id": safety.agent_id,
                },
                ensure_ascii=False,
            )
        )
        return 0

    def publish(feedback: TaskFeedback) -> None:
        print(json.dumps(feedback.to_dict(), ensure_ascii=False, separators=(",", ":")))

    def upload_image(path: Path, assignment: TaskAssignment) -> str:
        return f"/images/{assignment.mission_id}/{path.name}"

    behavior = InspectionBehavior(
        adapter=adapter,
        safety=safety,
        feedback_callback=publish,
        upload_image=upload_image,
        target_detector=target_detector,
        sleep=lambda _seconds: None,
    )
    result = behavior.execute(task)
    print(json.dumps({"result": result, "actions": adapter.action_log}, ensure_ascii=False))
    return 0 if result.get("ok") else 2


if __name__ == "__main__":
    raise SystemExit(main())
