"""Validation for project-owned real UAV configuration."""
from __future__ import annotations
from typing import Any, Dict

REQUIRED = ("teacher_authorization", "flight_area_cleared", "manual_takeover_ready")

def validate_real_config(config: Dict[str, Any], *, require_enabled: bool = False) -> None:
    agent = config.get("agent", {}); fs = config.get("flight_safety", {}); act = config.get("activation_requirements", {})
    if agent.get("control_mode") != "REAL": raise ValueError("control_mode 必须为 REAL")
    if require_enabled and agent.get("real_control_enabled") is not True: raise ValueError("real_control_enabled 未启用")
    for key in REQUIRED:
        if act.get(key) is not True: raise ValueError(f"安全授权未满足: {key}")
    if float(fs.get("cruise_height_m", 0)) <= 0 or float(fs.get("cruise_height_m", 0)) > float(fs.get("flight_bounds", {}).get("safe_height_max_m", 0)):
        raise ValueError("巡航高度超出范围")
    if float(fs.get("cruise_height_m", 0)) > 1.0: raise ValueError("巡航高度超过项目上限")
    if not fs.get("allowed_flight_polygon"): raise ValueError("缺少允许区域多边形")
