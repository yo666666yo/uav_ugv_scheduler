"""Safety-gated adapter for FanciSwarm/Mcontroller through vendor ROS 2 topics.

The adapter is deliberately disabled by default.  Constructing it, reading
state, holding, and taking a camera image do not publish flight commands.
Every operation that could move or arm the aircraft passes through one common
gate before the command backend is called.
"""

from __future__ import annotations

import copy
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, Optional, Protocol, Sequence

from agent.contracts import Pose


class RealControlDisabled(RuntimeError):
    """Raised before any real command is emitted while the safety gate is off."""


class VendorCommandBackend(Protocol):
    def publish_command(self, command: int) -> None: ...
    def publish_mission(self, values: Sequence[float]) -> None: ...


class ImageCaptureBackend(Protocol):
    def capture_image(self, output_path: Path) -> Path: ...


@dataclass(frozen=True)
class FanciInnovAdapterConfig:
    agent_id: str = "drone_001"
    real_control_enabled: bool = False
    frame_id: str = "uwb_map"
    command_topic: str = "/fcu_command/command"
    mission_topic: str = "/fcu_mission/mission_001"
    arm_before_takeoff: bool = True
    arm_confirm_timeout_s: float = 3.0
    altitude_confirm_timeout_s: float = 8.0
    verify_arm_before_takeoff: bool = False


class Ros2VendorCommandBackend:
    """Thin ROS 2 publisher matching the vendor fcu_core_ros2 interface.

    ROS imports are lazy so this project remains testable on macOS/Windows
    machines without ROS 2.  This backend should only be instantiated inside a
    running, correctly configured ROS 2 environment.
    """

    def __init__(self, *, command_topic: str, mission_topic: str) -> None:
        try:
            import rclpy
            from std_msgs.msg import Float32MultiArray, Int16
        except ImportError as exc:
            raise RuntimeError("缺少 ROS 2 Python 环境（rclpy/std_msgs）") from exc

        self._rclpy = rclpy
        self._Int16 = Int16
        self._Float32MultiArray = Float32MultiArray
        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = rclpy.create_node("group4_uav_real_adapter")
        self._command_publisher = self._node.create_publisher(Int16, command_topic, 10)
        self._mission_publisher = self._node.create_publisher(
            Float32MultiArray, mission_topic, 10
        )

    def publish_command(self, command: int) -> None:
        message = self._Int16()
        message.data = int(command)
        self._command_publisher.publish(message)
        self._rclpy.spin_once(self._node, timeout_sec=0.0)

    def publish_mission(self, values: Sequence[float]) -> None:
        if len(values) != 11:
            raise ValueError("厂商 mission 消息必须正好包含11个浮点数")
        message = self._Float32MultiArray()
        message.data = [float(value) for value in values]
        self._mission_publisher.publish(message)
        self._rclpy.spin_once(self._node, timeout_sec=0.0)


class FanciInnovRealUAVAdapter:
    """Real-device adapter with a fail-closed control gate."""

    is_simulation = False

    ARM = 1
    DISARM = 2
    TAKEOFF = 3
    LAND = 4

    def __init__(
        self,
        *,
        config: FanciInnovAdapterConfig,
        state_reader: Callable[[], Dict[str, Any]],
        command_backend: VendorCommandBackend,
        camera: ImageCaptureBackend,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self.config = config
        self.agent_id = config.agent_id
        self._state_reader = state_reader
        self._command_backend = command_backend
        self._camera = camera
        self._sleep = sleep
        self._work_state = "IDLE"

    def _require_real_control(self, operation: str) -> None:
        if self.config.real_control_enabled is not True:
            raise RealControlDisabled(
                f"真实控制已禁用，拒绝执行 {operation}；未向厂商控制 Topic 发布消息"
            )

    def read_state(self) -> Dict[str, Any]:
        raw = copy.deepcopy(self._state_reader())
        position = raw.get("position", {})
        attitude = raw.get("attitude_rad", {})
        battery = raw.get("battery", {})
        remaining = battery.get("remaining_percent")
        voltage = battery.get("voltage_v")
        raw.update(
            agent_id=self.agent_id,
            agent_type="UAV",
            device_online=bool(raw.get("online", raw.get("device_online", False))),
            work_state=self._work_state,
            control_mode="REAL",
            real_control_enabled=self.config.real_control_enabled,
            position_valid=all(
                position.get(key) is not None for key in ("x_m", "y_m", "z_m")
            ),
            pose={
                "frame_id": position.get("frame_id", self.config.frame_id),
                "x_m": position.get("x_m"),
                "y_m": position.get("y_m"),
                "z_m": position.get("z_m"),
                "yaw_rad": attitude.get("yaw"),
            },
        )
        raw.setdefault("battery", {})["battery_ok"] = (
            voltage is not None
            and float(voltage) >= 7.0
            and (remaining is None or float(remaining) >= 15.0)
        )
        return raw

    def set_work_state(self, work_state: str) -> None:
        self._work_state = str(work_state)

    def takeoff(self, height_m: float) -> None:
        if not math.isfinite(height_m) or height_m <= 0:
            raise ValueError("起飞高度必须是大于0的有限数")
        self._require_real_control("takeoff")
        if self.config.arm_before_takeoff:
            self._command_backend.publish_command(self.ARM)
            if not self.config.verify_arm_before_takeoff:
                self._command_backend.publish_command(self.TAKEOFF)
                return
            deadline = time.monotonic() + self.config.arm_confirm_timeout_s
            while time.monotonic() < deadline:
                state = self._state_reader() or {}
                if state.get("armed") is True:
                    break
                if state.get("armed_valid") is not True:
                    break
                self._sleep(0.1)
            else:
                raise RuntimeError("解锁未生效：/armed_001 未变为 true，未发布起飞命令")
        self._command_backend.publish_command(self.TAKEOFF)
        # Height is subsequently maintained by goto_pose/mission targets.  The
        # vendor takeoff command itself does not carry a height parameter.

    @staticmethod
    def _mission_values(pose: Pose) -> list[float]:
        values = [pose.x_m, pose.y_m, pose.z_m]
        if not all(math.isfinite(value) for value in values):
            raise ValueError("目标坐标必须是有限数")
        yaw = 0.0 if pose.yaw_rad is None else float(pose.yaw_rad)
        if not math.isfinite(yaw):
            raise ValueError("目标朝向必须是有限数")
        return [yaw, 0.0, pose.x_m, pose.y_m, pose.z_m, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def goto_pose(self, pose: Pose) -> None:
        if pose.frame_id != self.config.frame_id:
            raise ValueError("真实适配器只接受配置的 UWB 全局坐标系")
        mission = self._mission_values(pose)
        self._require_real_control("goto_pose")
        self._command_backend.publish_mission(mission)

    def hold(self, duration_s: float) -> None:
        if not math.isfinite(duration_s) or duration_s < 0:
            raise ValueError("悬停等待时间必须是非负有限数")
        self._sleep(duration_s)

    def capture_image(self, output_path: Path) -> Path:
        return self._camera.capture_image(output_path)

    def return_to_pose(self, home_pose: Pose, cruise_height_m: float) -> None:
        return_pose = Pose(
            frame_id=home_pose.frame_id,
            x_m=home_pose.x_m,
            y_m=home_pose.y_m,
            z_m=float(cruise_height_m),
            yaw_rad=home_pose.yaw_rad,
        )
        self.goto_pose(return_pose)

    def land(self) -> None:
        self._require_real_control("land")
        self._command_backend.publish_command(self.LAND)

    def request_manual_takeover(self, reason: str) -> None:
        # This only changes local agent state.  It intentionally does not emit
        # a flight command because blind actions are unsafe after localization
        # or flight-state failures.
        self._work_state = "MANUAL_CONTROL"
