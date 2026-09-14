import json, tempfile, unittest
from pathlib import Path
from agent.real_agent_runner import main

class RunnerTests(unittest.TestCase):
    def test_hover_dry_run(self):
        task={"timestamp_ms":1,"mission_id":"m","task_id":"t","target_agent_id":"drone_001","task_type":"HOVER_TEST","target_pose":{"frame_id":"uwb_map","x_m":.7,"y_m":4,"z_m":0,"yaw_rad":0},"parameters":{"duration_s":0}}
        with tempfile.TemporaryDirectory() as d:
            f=Path(d)/"task.json"; f.write_text(json.dumps(task))
            self.assertEqual(main(["--task",str(f)]),0)

if __name__ == "__main__": unittest.main()
