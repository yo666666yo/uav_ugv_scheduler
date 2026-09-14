import unittest
from pathlib import Path
from agent.contracts import Pose
from uav_adapter.fancinnov_real_adapter import FanciInnovRealUAVAdapter,FanciInnovAdapterConfig,RealControlDisabled

class FakeBackend:
    def __init__(self): self.commands=[]; self.missions=[]
    def publish_command(self,c): self.commands.append(c)
    def publish_mission(self,v): self.missions.append(list(v))
class FakeCamera:
    def capture_image(self,p): Path(p).parent.mkdir(parents=True,exist_ok=True); Path(p).write_bytes(b"x"); return Path(p)

class IntegrationTests(unittest.TestCase):
    def state(self): return {"online":True,"position":{"frame_id":"uwb_map","x_m":.7,"y_m":4.,"z_m":.02},"battery":{"voltage_v":7.9,"remaining_percent":60,"battery_ok":True},"armed":False}
    def test_disabled_never_publishes(self):
        b=FakeBackend(); a=FanciInnovRealUAVAdapter(config=FanciInnovAdapterConfig(),state_reader=self.state,command_backend=b,camera=FakeCamera())
        with self.assertRaises(RealControlDisabled): a.takeoff(.8)
        self.assertEqual(b.commands,[]); self.assertEqual(b.missions,[])
    def test_enabled_maps_commands(self):
        b=FakeBackend(); c=FanciInnovAdapterConfig(real_control_enabled=True); a=FanciInnovRealUAVAdapter(config=c,state_reader=self.state,command_backend=b,camera=FakeCamera())
        a.takeoff(.8); a.goto_pose(Pose("uwb_map",.7,4.,.8,0)); a.land()
        self.assertEqual(b.commands,[1,3,4]); self.assertEqual(len(b.missions),1)

if __name__ == "__main__": unittest.main()
