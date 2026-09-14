import tempfile
import unittest
from pathlib import Path

from agent.contracts import Pose, TaskAssignment
from behaviors.inspect_target import (
    FlightBounds,
    HomeSamplingConfig,
    InspectionBehavior,
    InspectionSafetyConfig,
    calculate_observation_pose,
)
from uav_adapter.simulated_uav_adapter import SimulatedUAVAdapter
from perception.simulated_target_detector import SimulatedTargetDetector


def make_task(**overrides):
    values = {
        "schema_version": "1.0",
        "timestamp_ms": 1,
        "mission_id": "mission_001",
        "task_id": "task_inspect_001",
        "target_agent_id": "drone_001",
        "task_type": "INSPECT_TARGET",
        "target_pose": Pose("uwb_map", 3.0, 1.0, 0.0, None),
        "parameters": {
            "inspection_distance_m": 0.5,
            "inspection_height_m": 1.0,
            "stable_time_s": 2.0,
            "target_label": "person",
            "target_confidence": 0.45,
        },
    }
    values.update(overrides)
    return TaskAssignment(**values)


def make_safety(image_directory):
    return InspectionSafetyConfig(
        agent_id="drone_001",
        cruise_height_m=1.0,
        inspection_distance_m=0.5,
        stable_time_s=2.0,
        bounds=FlightBounds(0.0, 6.0, 0.0, 6.0, 0.5, 2.0),
        home_sampling=HomeSamplingConfig(
            minimum_samples=3,
            sample_interval_s=0.0,
            max_position_jitter_m=0.05,
            max_yaw_jitter_rad=0.15,
        ),
        image_directory=image_directory,
    )


class ObservationPointTests(unittest.TestCase):
    def test_observation_point_faces_target_and_keeps_distance(self):
        observation = calculate_observation_pose(
            Pose("uwb_map", 1.0, 1.0, 0.0, 0.0),
            Pose("uwb_map", 3.0, 1.0, 0.0, None),
            0.5,
            1.0,
        )
        self.assertAlmostEqual(observation.x_m, 2.5)
        self.assertAlmostEqual(observation.y_m, 1.0)
        self.assertAlmostEqual(observation.z_m, 1.0)
        self.assertAlmostEqual(observation.yaw_rad, 0.0)

    def test_downward_camera_observation_is_directly_above_target(self):
        observation = calculate_observation_pose(
            Pose("uwb_map", 1.0, 1.0, 0.0, 0.25),
            Pose("uwb_map", 3.0, 2.0, 0.0, None),
            0.0,
            0.8,
        )
        self.assertAlmostEqual(observation.x_m, 3.0)
        self.assertAlmostEqual(observation.y_m, 2.0)
        self.assertAlmostEqual(observation.z_m, 0.8)
        self.assertAlmostEqual(observation.yaw_rad, 0.25)


class InspectionBehaviorTests(unittest.TestCase):
    def test_complete_simulated_inspection_returns_and_lands(self):
        with tempfile.TemporaryDirectory() as directory:
            adapter = SimulatedUAVAdapter(
                agent_id="drone_001",
                initial_pose=Pose("uwb_map", 1.0, 1.0, 0.0, 0.0),
            )
            feedback = []
            behavior = InspectionBehavior(
                adapter=adapter,
                safety=make_safety(Path(directory)),
                feedback_callback=feedback.append,
                upload_image=lambda path, task: f"/images/{task.mission_id}/{path.name}",
                target_detector=SimulatedTargetDetector(found=True, confidence=0.90),
                sleep=lambda _seconds: None,
                clock_ms=lambda: 10,
            )
            result = behavior.execute(make_task())

            self.assertTrue(result["ok"])
            self.assertEqual(feedback[-1].status, "COMPLETED")
            self.assertIn("IMAGE_CAPTURED", [item.status for item in feedback])
            self.assertIn("TARGET_CONFIRMED", [item.status for item in feedback])
            self.assertEqual(adapter.read_state()["work_state"], "IDLE")
            self.assertFalse(adapter.read_state()["armed"])
            final_pose = adapter.read_state()["pose"]
            self.assertAlmostEqual(final_pose["x_m"], 1.0)
            self.assertAlmostEqual(final_pose["y_m"], 1.0)
            self.assertAlmostEqual(final_pose["z_m"], 0.0)
            self.assertEqual(
                [entry["action"] for entry in adapter.action_log],
                [
                    "takeoff",
                    "goto_pose",
                    "hold",
                    "capture_image",
                    "return_to_pose",
                    "land",
                ],
            )

    def test_read_only_mode_is_rejected_without_actions(self):
        with tempfile.TemporaryDirectory() as directory:
            adapter = SimulatedUAVAdapter(
                agent_id="drone_001",
                initial_pose=Pose("uwb_map", 1.0, 1.0, 0.0, 0.0),
            )
            adapter._state["control_mode"] = "READ_ONLY"
            feedback = []
            behavior = InspectionBehavior(
                adapter=adapter,
                safety=make_safety(Path(directory)),
                feedback_callback=feedback.append,
                upload_image=lambda path, task: str(path),
                target_detector=SimulatedTargetDetector(found=True),
                sleep=lambda _seconds: None,
            )
            result = behavior.execute(make_task())
            self.assertFalse(result["ok"])
            self.assertEqual(result["failure_code"], "CONTROL_DISABLED")
            self.assertEqual(adapter.action_log, [])
            self.assertEqual(feedback[-1].status, "REJECTED")

    def test_out_of_bounds_observation_is_rejected_before_takeoff(self):
        with tempfile.TemporaryDirectory() as directory:
            adapter = SimulatedUAVAdapter(
                agent_id="drone_001",
                initial_pose=Pose("uwb_map", 1.0, 1.0, 0.0, 0.0),
            )
            feedback = []
            behavior = InspectionBehavior(
                adapter=adapter,
                safety=make_safety(Path(directory)),
                feedback_callback=feedback.append,
                upload_image=lambda path, task: str(path),
                target_detector=SimulatedTargetDetector(found=True),
                sleep=lambda _seconds: None,
            )
            task = make_task(target_pose=Pose("uwb_map", 9.0, 9.0, 0.0, None))
            result = behavior.execute(task)
            self.assertFalse(result["ok"])
            self.assertEqual(result["failure_code"], "OUT_OF_FLIGHT_AREA")
            self.assertEqual(result["status"], "REJECTED")
            self.assertEqual(adapter.action_log, [])
            self.assertEqual(adapter.read_state()["work_state"], "IDLE")

    def test_target_not_found_returns_and_lands(self):
        with tempfile.TemporaryDirectory() as directory:
            adapter = SimulatedUAVAdapter(
                agent_id="drone_001",
                initial_pose=Pose("uwb_map", 1.0, 1.0, 0.0, 0.0),
            )
            feedback = []
            behavior = InspectionBehavior(
                adapter=adapter,
                safety=make_safety(Path(directory)),
                feedback_callback=feedback.append,
                upload_image=lambda path, task: str(path),
                target_detector=SimulatedTargetDetector(found=False),
                sleep=lambda _seconds: None,
            )
            result = behavior.execute(make_task())

            self.assertFalse(result["ok"])
            self.assertEqual(result["failure_code"], "TARGET_NOT_FOUND")
            self.assertEqual(result["status"], "FAILED")
            self.assertEqual(adapter.read_state()["work_state"], "IDLE")
            self.assertFalse(adapter.read_state()["armed"])
            final_pose = adapter.read_state()["pose"]
            self.assertAlmostEqual(final_pose["x_m"], 1.0)
            self.assertAlmostEqual(final_pose["y_m"], 1.0)
            self.assertAlmostEqual(final_pose["z_m"], 0.0)

    def test_missing_target_label_is_rejected_before_takeoff(self):
        with tempfile.TemporaryDirectory() as directory:
            adapter = SimulatedUAVAdapter(
                agent_id="drone_001",
                initial_pose=Pose("uwb_map", 1.0, 1.0, 0.0, 0.0),
            )
            behavior = InspectionBehavior(
                adapter=adapter,
                safety=make_safety(Path(directory)),
                feedback_callback=lambda _feedback: None,
                upload_image=lambda path, task: str(path),
                target_detector=SimulatedTargetDetector(found=True),
                sleep=lambda _seconds: None,
            )
            task = make_task(parameters={})
            result = behavior.execute(task)

            self.assertFalse(result["ok"])
            self.assertEqual(result["failure_code"], "TARGET_LABEL_MISSING")
            self.assertEqual(adapter.action_log, [])


if __name__ == "__main__":
    unittest.main()
