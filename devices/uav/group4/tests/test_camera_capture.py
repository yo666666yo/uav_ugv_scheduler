import tempfile
import unittest
from pathlib import Path

from uav_adapter.camera_capture import (
    RaspberryPiCameraCapture,
    RaspberryPiCameraConfig,
)
from uav_adapter.camera_enabled_adapter import CameraEnabledUAVAdapter


class BaseAdapter:
    is_simulation = False

    def read_state(self):
        return {"device_online": True}

    def capture_image(self, _output_path):
        raise AssertionError("base adapter camera must not be called")


class RaspberryPiCameraCaptureTests(unittest.TestCase):
    def test_stream_mode_writes_atomically(self):
        calls = []

        def stream_capture(url, path):
            calls.append((url, path))
            path.write_bytes(b"real-jpeg-placeholder")

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "capture.jpg"
            camera = RaspberryPiCameraCapture(
                RaspberryPiCameraConfig(
                    mode="stream", stream_url="http://127.0.0.1:5001/video"
                ),
                stream_capture=stream_capture,
            )
            result = camera.capture_image(output)

            self.assertEqual(result, output)
            self.assertEqual(output.read_bytes(), b"real-jpeg-placeholder")
            self.assertEqual(calls[0][0], "http://127.0.0.1:5001/video")
            self.assertNotEqual(calls[0][1], output)

    def test_auto_mode_falls_back_to_direct_capture(self):
        def failing_stream(_url, _path):
            raise RuntimeError("stream unavailable")

        def direct_capture(path):
            path.write_bytes(b"direct-camera-placeholder")

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "capture.jpg"
            camera = RaspberryPiCameraCapture(
                RaspberryPiCameraConfig(mode="auto"),
                stream_capture=failing_stream,
                direct_capture=direct_capture,
            )
            camera.capture_image(output)
            self.assertEqual(output.read_bytes(), b"direct-camera-placeholder")

    def test_wrapper_delegates_state_and_uses_camera(self):
        def stream_capture(_url, path):
            path.write_bytes(b"image")

        with tempfile.TemporaryDirectory() as directory:
            camera = RaspberryPiCameraCapture(stream_capture=stream_capture)
            adapter = CameraEnabledUAVAdapter(BaseAdapter(), camera)
            output = Path(directory) / "capture.jpg"

            self.assertEqual(adapter.read_state(), {"device_online": True})
            self.assertEqual(adapter.capture_image(output), output)
            self.assertEqual(output.read_bytes(), b"image")


if __name__ == "__main__":
    unittest.main()
