"""INSPECT_TARGET behavior shared by simulation and future real adapters."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from statistics import fmean
from typing import Any, Callable, Dict, Iterable, List, Optional, Protocol, Sequence

from agent.contracts import Pose, TaskAssignment, TaskFeedback


class UAVAdapter(Protocol):
    is_simulation: bool

    def read_state(self) -> Dict[str, Any]: ...
    def set_work_state(self, work_state: str) -> None: ...
    def takeoff(self, height_m: float) -> None: ...
    def goto_pose(self, pose: Pose) -> None: ...
    def hold(self, duration_s: float) -> None: ...
    def capture_image(self, output_path: Path) -> Path: ...
    def return_to_pose(self, home_pose: Pose, cruise_height_m: float) -> None: ...
    def land(self) -> None: ...
    def request_manual_takeover(self, reason: str) -> None: ...


class TargetDetector(Protocol):
    def detect(
        self,
        image_path: Path,
        target_label: str,
        min_confidence: float,
    ) -> Dict[str, Any]: ...


class InspectionRejected(RuntimeError):
    def __init__(self, failure_code: str, message: str):
        super().__init__(message)
        self.failure_code = failure_code


@dataclass(frozen=True)
class FlightBounds:
    x_min_m: float
    x_max_m: float
    y_min_m: float
    y_max_m: float
    safe_height_min_m: float
    safe_height_max_m: float


@dataclass(frozen=True)
class NoFlyZone:
    zone_id: str
    min_z_m: float
    max_z_m: float
    polygon: Sequence[Sequence[float]]


@dataclass(frozen=True)
class HomeSamplingConfig:
    minimum_samples: int = 10
    sample_interval_s: float = 0.2
    max_position_jitter_m: float = 0.05
    max_yaw_jitter_rad: float = 0.15


@dataclass(frozen=True)
class InspectionSafetyConfig:
    agent_id: str
    cruise_height_m: float
    inspection_distance_m: float
    stable_time_s: float
    bounds: FlightBounds
    home_sampling: HomeSamplingConfig = field(default_factory=HomeSamplingConfig)
    no_fly_zones: Sequence[NoFlyZone] = field(default_factory=tuple)
    allowed_flight_polygon: Sequence[Sequence[float]] = field(default_factory=tuple)
    real_control_enabled: bool = False
    image_directory: Path = Path("logs/images")
    image_extension: str = ".jpg"


def _angular_difference(left: float, right: float) -> float:
    return abs((left - right + math.pi) % (2 * math.pi) - math.pi)


def _point_in_polygon(x: float, y: float, polygon: Sequence[Sequence[float]]) -> bool:
    inside = False
    count = len(polygon)
    if count < 3:
        return False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = float(previous[0]), float(previous[1])
        x2, y2 = float(current[0]), float(current[1])
        if (y1 > y) != (y2 > y):
            crossing_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < crossing_x:
                inside = not inside
        previous = current
    return inside


def calculate_observation_pose(
    current_pose: Pose,
    target_pose: Pose,
    inspection_distance_m: float,
    inspection_height_m: float,
) -> Pose:
    if current_pose.frame_id != target_pose.frame_id:
        raise InspectionRejected("POSITION_INVALID", "当前坐标与目标坐标不在同一坐标系")
    if inspection_distance_m < 0:
        raise InspectionRejected("POSITION_INVALID", "观察距离不能小于0")
    delta_x = target_pose.x_m - current_pose.x_m
    delta_y = target_pose.y_m - current_pose.y_m
    if math.hypot(delta_x, delta_y) < 1e-9:
        if current_pose.yaw_rad is None:
            raise InspectionRejected("YAW_UNAVAILABLE", "无人机与目标重合且没有可用朝向")
        yaw = current_pose.yaw_rad
    else:
        yaw = math.atan2(delta_y, delta_x)
    if inspection_distance_m == 0:
        return Pose(
            frame_id=target_pose.frame_id,
            x_m=target_pose.x_m,
            y_m=target_pose.y_m,
            z_m=inspection_height_m,
            yaw_rad=current_pose.yaw_rad,
        )
    return Pose(
        frame_id=target_pose.frame_id,
        x_m=target_pose.x_m - inspection_distance_m * math.cos(yaw),
        y_m=target_pose.y_m - inspection_distance_m * math.sin(yaw),
        z_m=inspection_height_m,
        yaw_rad=yaw,
    )


class InspectionBehavior:
    def __init__(
        self,
        *,
        adapter: UAVAdapter,
        safety: InspectionSafetyConfig,
        feedback_callback: Callable[[TaskFeedback], None],
        upload_image: Callable[[Path, TaskAssignment], str],
        target_detector: TargetDetector,
        sleep: Callable[[float], None] = time.sleep,
        clock_ms: Callable[[], int] = lambda: int(time.time() * 1000),
    ) -> None:
        self.adapter = adapter
        self.safety = safety
        self.feedback_callback = feedback_callback
        self.upload_image = upload_image
        self.target_detector = target_detector
        self.sleep = sleep
        self.clock_ms = clock_ms
        self._sequence = 0

    def _pose_from_state(self, state: Dict[str, Any]) -> Pose:
        try:
            return Pose.from_dict(state["pose"])
        except (KeyError, TypeError, ValueError) as exc:
            raise InspectionRejected("POSITION_INVALID", "设备状态中缺少有效位置") from exc

    def _feedback(
        self,
        task: TaskAssignment,
        *,
        status: str,
        stage: str,
        progress: int,
        message: str,
        failure_code: Optional[str] = None,
        image_url: Optional[str] = None,
        detection: Optional[Dict[str, Any]] = None,
    ) -> None:
        self._sequence += 1
        state = self.adapter.read_state()
        pose: Optional[Pose]
        try:
            pose = self._pose_from_state(state)
        except InspectionRejected:
            pose = None
        self.feedback_callback(
            TaskFeedback(
                schema_version=task.schema_version,
                timestamp_ms=self.clock_ms(),
                mission_id=task.mission_id,
                task_id=task.task_id,
                agent_id=self.safety.agent_id,
                status=status,
                stage=stage,
                progress_percent=progress,
                pose=pose,
                image_url=image_url,
                failure_code=failure_code,
                message=message,
                sequence=self._sequence,
                detection=detection,
            )
        )

    def _preflight(self, task: TaskAssignment) -> None:
        if task.target_agent_id != self.safety.agent_id:
            raise InspectionRejected("WRONG_AGENT", "任务不是分配给本机的")
        if task.task_type != "INSPECT_TARGET":
            raise InspectionRejected("UNSUPPORTED_TASK", "无人机不支持该任务类型")
        target_label = str(task.parameters.get("target_label", "")).strip()
        if not target_label:
            raise InspectionRejected("TARGET_LABEL_MISSING", "任务缺少目标类别 target_label")
        try:
            target_confidence = float(task.parameters.get("target_confidence", 0.45))
        except (TypeError, ValueError) as exc:
            raise InspectionRejected("TARGET_CONFIDENCE_INVALID", "目标置信度必须是数字") from exc
        if not 0.0 <= target_confidence <= 1.0:
            raise InspectionRejected("TARGET_CONFIDENCE_INVALID", "目标置信度必须在0到1之间")
        state = self.adapter.read_state()
        if not state.get("device_online"):
            raise InspectionRejected("DEVICE_OFFLINE", "设备离线")
        if state.get("work_state") != "IDLE":
            raise InspectionRejected("DEVICE_BUSY", "设备当前不空闲")
        if not state.get("position_valid"):
            raise InspectionRejected("POSITION_INVALID", "定位当前无效")
        if not state.get("battery", {}).get("battery_ok"):
            raise InspectionRejected("LOW_BATTERY", "电量不满足任务要求")
        mode = state.get("control_mode")
        if mode == "READ_ONLY":
            raise InspectionRejected("CONTROL_DISABLED", "当前适配器为只读模式")
        if mode == "REAL" and not self.safety.real_control_enabled:
            raise InspectionRejected("CONTROL_DISABLED", "真实控制未在本机安全配置中启用")
        if mode == "REAL" and not state.get("control_ready", False):
            raise InspectionRejected("CONTROL_PATH_OFFLINE", "ROS真实控制链路尚未通过健康检查")

    def _validate_pose(self, pose: Pose) -> None:
        bounds = self.safety.bounds
        if not (bounds.x_min_m <= pose.x_m <= bounds.x_max_m):
            raise InspectionRejected("OUT_OF_FLIGHT_AREA", "X坐标超出允许飞行区域")
        if not (bounds.y_min_m <= pose.y_m <= bounds.y_max_m):
            raise InspectionRejected("OUT_OF_FLIGHT_AREA", "Y坐标超出允许飞行区域")
        if not (bounds.safe_height_min_m <= pose.z_m <= bounds.safe_height_max_m):
            raise InspectionRejected("OUT_OF_FLIGHT_AREA", "高度超出允许飞行范围")
        if self.safety.allowed_flight_polygon and not _point_in_polygon(
            pose.x_m, pose.y_m, self.safety.allowed_flight_polygon
        ):
            raise InspectionRejected("OUT_OF_FLIGHT_AREA", "坐标位于UWB基站覆盖多边形之外")
        for zone in self.safety.no_fly_zones:
            if zone.min_z_m <= pose.z_m <= zone.max_z_m and _point_in_polygon(
                pose.x_m, pose.y_m, zone.polygon
            ):
                raise InspectionRejected(
                    "TARGET_IN_NO_FLY_ZONE", f"目标位于禁飞区 {zone.zone_id}"
                )

    def _validate_path(self, start: Pose, end: Pose) -> None:
        distance = math.hypot(end.x_m - start.x_m, end.y_m - start.y_m)
        sample_count = max(1, math.ceil(distance / 0.05))
        for index in range(sample_count + 1):
            ratio = index / sample_count
            self._validate_pose(
                Pose(
                    frame_id=start.frame_id,
                    x_m=start.x_m + (end.x_m - start.x_m) * ratio,
                    y_m=start.y_m + (end.y_m - start.y_m) * ratio,
                    z_m=start.z_m + (end.z_m - start.z_m) * ratio,
                    yaw_rad=end.yaw_rad,
                )
            )

    def _record_home_pose(self) -> Pose:
        poses: List[Pose] = []
        config = self.safety.home_sampling
        for sample_index in range(config.minimum_samples):
            state = self.adapter.read_state()
            if not state.get("device_online") or not state.get("position_valid"):
                raise InspectionRejected("POSITION_INVALID", "记录起飞点期间定位失效")
            pose = self._pose_from_state(state)
            if pose.yaw_rad is None:
                raise InspectionRejected("YAW_UNAVAILABLE", "记录起飞点期间缺少朝向")
            poses.append(pose)
            if sample_index + 1 < config.minimum_samples:
                self.sleep(config.sample_interval_s)

        mean_x = fmean(pose.x_m for pose in poses)
        mean_y = fmean(pose.y_m for pose in poses)
        mean_z = fmean(pose.z_m for pose in poses)
        reference_yaw = poses[0].yaw_rad
        assert reference_yaw is not None
        max_position_jitter = max(
            math.sqrt(
                (pose.x_m - mean_x) ** 2
                + (pose.y_m - mean_y) ** 2
                + (pose.z_m - mean_z) ** 2
            )
            for pose in poses
        )
        max_yaw_jitter = max(
            _angular_difference(pose.yaw_rad or 0.0, reference_yaw) for pose in poses
        )
        if max_position_jitter > config.max_position_jitter_m:
            raise InspectionRejected("POSITION_INVALID", "起飞点位置采样波动超过阈值")
        if max_yaw_jitter > config.max_yaw_jitter_rad:
            raise InspectionRejected("YAW_UNAVAILABLE", "起飞点朝向采样波动超过阈值")
        return Pose(poses[0].frame_id, mean_x, mean_y, mean_z, reference_yaw)

    def execute(self, task: TaskAssignment) -> Dict[str, Any]:
        airborne = False
        home_pose: Optional[Pose] = None
        recovered = False
        self._sequence = 0
        try:
            self._preflight(task)
            self._feedback(
                task,
                status="ACCEPTED",
                stage="PREFLIGHT_CHECK",
                progress=5,
                message="任务已接收，开始安全检查",
            )
            self.adapter.set_work_state("PREFLIGHT")
            home_pose = self._record_home_pose()
            self._feedback(
                task,
                status="IN_PROGRESS",
                stage="RECORDING_HOME",
                progress=10,
                message="起飞点和初始朝向记录完成",
            )

            inspection_distance = float(
                task.parameters.get(
                    "inspection_distance_m", self.safety.inspection_distance_m
                )
            )
            inspection_height = float(
                task.parameters.get(
                    "inspection_height_m", self.safety.cruise_height_m
                )
            )
            stable_time = float(
                task.parameters.get("stable_time_s", self.safety.stable_time_s)
            )
            observation_pose = calculate_observation_pose(
                home_pose,
                task.target_pose,
                inspection_distance,
                inspection_height,
            )
            self._validate_pose(observation_pose)

            cruise_start = Pose(
                home_pose.frame_id,
                home_pose.x_m,
                home_pose.y_m,
                self.safety.cruise_height_m,
                home_pose.yaw_rad,
            )
            self._validate_pose(cruise_start)
            self._validate_path(cruise_start, observation_pose)

            self.adapter.set_work_state("EXECUTING")
            self._feedback(
                task,
                status="IN_PROGRESS",
                stage="TAKING_OFF",
                progress=20,
                message="垂直起飞到巡航高度",
            )
            self.adapter.takeoff(self.safety.cruise_height_m)
            airborne = True
            self._feedback(
                task,
                status="IN_PROGRESS",
                stage="TRANSITING",
                progress=40,
                message="在巡航高度飞往目标观察点",
            )
            self.adapter.goto_pose(observation_pose)
            self._feedback(
                task,
                status="IN_PROGRESS",
                stage="STABILIZING",
                progress=55,
                message="已到达观察点，等待稳定",
            )
            self.adapter.hold(stable_time)

            extension = self.safety.image_extension
            if not extension.startswith("."):
                extension = f".{extension}"
            image_path = self.safety.image_directory / (
                f"{task.mission_id}_{task.task_id}{extension}"
            )
            self._feedback(
                task,
                status="IN_PROGRESS",
                stage="CAPTURING_IMAGE",
                progress=65,
                message="正在拍照",
            )
            captured_path = self.adapter.capture_image(image_path)
            target_label = str(task.parameters["target_label"]).strip()
            target_confidence = float(task.parameters.get("target_confidence", 0.45))
            self._feedback(
                task,
                status="IN_PROGRESS",
                stage="VERIFYING_TARGET",
                progress=68,
                message=f"正在确认照片中是否存在目标 {target_label}",
            )
            try:
                detection = self.target_detector.detect(
                    captured_path,
                    target_label,
                    target_confidence,
                )
            except Exception as exc:
                raise InspectionRejected(
                    "TARGET_DETECTION_ERROR", f"目标识别模块运行失败：{exc}"
                ) from exc
            if not bool(detection.get("found")):
                confidence = float(detection.get("confidence", 0.0))
                raise InspectionRejected(
                    "TARGET_NOT_FOUND",
                    f"照片中未检测到目标 {target_label}，最高置信度 {confidence:.2f}",
                )
            self._feedback(
                task,
                status="TARGET_CONFIRMED",
                stage="VERIFYING_TARGET",
                progress=70,
                message=(
                    f"已检测到目标 {target_label}，"
                    f"置信度 {float(detection.get('confidence', 0.0)):.2f}"
                ),
                detection=detection,
            )
            image_url = self.upload_image(captured_path, task)
            self._feedback(
                task,
                status="IMAGE_CAPTURED",
                stage="UPLOADING_IMAGE",
                progress=72,
                message="照片上传成功，开始返航",
                image_url=image_url,
            )

            self.adapter.set_work_state("RETURNING")
            self._feedback(
                task,
                status="RETURNING",
                stage="RETURNING_HOME",
                progress=80,
                message="在巡航高度返回起飞点上方",
            )
            self.adapter.return_to_pose(home_pose, self.safety.cruise_height_m)
            self._feedback(
                task,
                status="IN_PROGRESS",
                stage="LANDING",
                progress=90,
                message="到达起飞点上方，开始垂直降落",
            )
            self.adapter.land()
            airborne = False
            recovered = True
            self.adapter.set_work_state("IDLE")
            self._feedback(
                task,
                status="COMPLETED",
                stage="DONE",
                progress=100,
                message="无人机已返回起飞点并降落",
                image_url=image_url,
            )
            return {"ok": True, "image_url": image_url, "detection": detection}

        except InspectionRejected as exc:
            terminal_status = "REJECTED" if not airborne else "FAILED"
            self._feedback(
                task,
                status=terminal_status,
                stage="PREFLIGHT_CHECK" if not airborne else "RETURNING_HOME",
                progress=0,
                message=str(exc),
                failure_code=exc.failure_code,
            )
            return {
                "ok": False,
                "status": terminal_status,
                "failure_code": exc.failure_code,
                "message": str(exc),
            }
        except Exception as exc:
            self._feedback(
                task,
                status="FAILED",
                stage="RETURNING_HOME" if airborne else "PREFLIGHT_CHECK",
                progress=0,
                message=str(exc),
                failure_code="UNKNOWN_ERROR",
            )
            return {
                "ok": False,
                "status": "FAILED",
                "failure_code": "UNKNOWN_ERROR",
                "message": str(exc),
            }
        finally:
            if airborne and home_pose is not None:
                state = self.adapter.read_state()
                if state.get("position_valid"):
                    try:
                        self.adapter.set_work_state("RETURNING")
                        self.adapter.return_to_pose(home_pose, self.safety.cruise_height_m)
                        self.adapter.land()
                        recovered = True
                    except Exception:
                        recovered = False
                if not recovered:
                    self.adapter.request_manual_takeover("自动恢复失败或定位无效")
            if recovered:
                self.adapter.set_work_state("IDLE")
            elif not airborne and self.adapter.read_state().get("work_state") != "MANUAL_CONTROL":
                self.adapter.set_work_state("IDLE")
