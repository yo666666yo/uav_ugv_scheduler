"""Unified task runner. Real mode is fail-closed and requires confirmation."""
from __future__ import annotations
import argparse, json, time
from pathlib import Path
from agent.contracts import TaskAssignment
from behaviors.hover_test import HoverTest, HoverTestConfig
from agent.safety_config import validate_real_config

def main(argv=None):
    p=argparse.ArgumentParser()
    p.add_argument("--task", type=Path, required=True)
    p.add_argument("--mode", choices=("dry-run","real"), default="dry-run")
    p.add_argument("--config", type=Path, default=Path("config/drone_001.real.disabled.json"))
    p.add_argument("--confirm-real", action="store_true")
    p.add_argument("--control-path-ready", action="store_true", help="现场已确认控制 Topic 链路")
    p.add_argument("--state-source", choices=("stub", "ros2", "mavlink", "both"), default="stub")
    p.add_argument("--command-backend", choices=("recording", "ros2"), default="recording")
    p.add_argument("--state-wait-s", type=float, default=8.0)
    p.add_argument("--preflight-only", action="store_true")
    a=p.parse_args(argv); config=json.loads(a.config.read_text()); task=TaskAssignment.from_dict(json.loads(a.task.read_text()))
    if a.mode == "real":
        if not a.confirm_real:
            raise RuntimeError("真实模式必须显式 --confirm-real，并由现场人员确认")
        if not a.control_path_ready:
            raise RuntimeError("真实模式必须显式 --control-path-ready")
        validate_real_config(config, require_enabled=True)
        from uav_adapter.fancinnov_real_adapter import FanciInnovAdapterConfig, FanciInnovRealUAVAdapter, Ros2VendorCommandBackend
        from agent.real_state_aggregator import RealStateAggregator
        from uav_adapter.camera_capture import RaspberryPiCameraCapture
        vendor = config["vendor_interface"]
        if a.state_source in ("ros2", "both"):
            from uav_adapter.ros2_state_reader import Ros2StateReader
            ros = Ros2StateReader("/"+vendor["state_topic"].lstrip("/"), "/"+vendor["battery_topic"].lstrip("/"), "/"+vendor.get("battery_remaining_topic","battery_remaining_001").lstrip("/"), "/"+vendor.get("imu_topic","imu_global_001").lstrip("/"))
        else: ros = lambda: {"online": False}
        if a.state_source in ("mavlink", "both"):
            from uav_adapter.mavlink_state_reader import MavlinkStateReader
            mav = MavlinkStateReader(f"tcp:{vendor['flight_controller_host']}:{vendor['flight_controller_port']}")
        else: mav = lambda: {"online": False}
        aggregator = RealStateAggregator(ros, mav)
        deadline = time.monotonic() + max(0.1, a.state_wait_s)
        latest = {}
        while time.monotonic() < deadline:
            latest = aggregator()
            if latest.get("online") and latest.get("position_valid") and latest.get("battery",{}).get("battery_ok") and latest.get("armed_valid"):
                break
            time.sleep(0.1)
        control_path_confirmed = bool(a.control_path_ready)
        def reader():
            state = aggregator()
            state["control_ready"] = control_path_confirmed
            return state
        if a.command_backend == "ros2":
            backend = Ros2VendorCommandBackend(command_topic=vendor["command_topic"], mission_topic=vendor["mission_topic"]); camera = RaspberryPiCameraCapture()
        else:
            class RecordingBackend:
                def __init__(self): self.commands=[]; self.missions=[]
                def publish_command(self, command): self.commands.append(int(command))
                def publish_mission(self, values): self.missions.append(list(values))
            class StubCamera:
                def capture_image(self, output_path): output_path.parent.mkdir(parents=True,exist_ok=True); output_path.write_bytes(b"test"); return output_path
            backend, camera = RecordingBackend(), StubCamera()
        adapter = FanciInnovRealUAVAdapter(config=FanciInnovAdapterConfig(agent_id=config["agent"]["agent_id"], real_control_enabled=True, verify_arm_before_takeoff=True, frame_id=vendor.get("frame_id","uwb_map"), command_topic=vendor["command_topic"], mission_topic=vendor["mission_topic"]), state_reader=reader, command_backend=backend, camera=camera)
        state = adapter.read_state()
        if not (state.get("online") and state.get("position_valid") and state.get("battery",{}).get("battery_ok") and state.get("armed_valid")):
            raise RuntimeError("真实状态未通过在线、定位或电压检查")
        if a.preflight_only:
            print(json.dumps({"preflight": "PASS", "state": state}, ensure_ascii=False)); return 0
        if task.task_type == "HOVER_TEST": result = HoverTest(adapter, HoverTestConfig(float(task.parameters.get("height_m",.8)), float(task.parameters.get("duration_s",5)))).execute()
        else: raise RuntimeError("真实 Runner 当前先支持 HOVER_TEST；INSPECT_TARGET 使用统一 Agent 服务接入")
        print(json.dumps({"result":result,"recorded_commands":getattr(backend,"commands",[])}, ensure_ascii=False)); return 0 if result.get("ok") else 2
    if task.task_type == "HOVER_TEST":
        from uav_adapter.simulated_uav_adapter import SimulatedUAVAdapter
        from agent.contracts import Pose
        adapter=SimulatedUAVAdapter(agent_id=task.target_agent_id, initial_pose=task.target_pose)
        adapter._state.update(device_online=True, position_valid=True, battery={"battery_ok":True,"remaining_percent":80})
        result=HoverTest(adapter, HoverTestConfig(float(task.parameters.get("height_m",.8)), float(task.parameters.get("duration_s",5)))).execute()
        print(json.dumps({"result":result,"actions":adapter.action_log})); return 0
    raise RuntimeError(f"dry-run runner暂不支持任务类型: {task.task_type}")
if __name__ == "__main__": raise SystemExit(main())
