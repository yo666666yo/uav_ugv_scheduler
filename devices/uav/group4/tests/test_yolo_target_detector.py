import tempfile
import unittest
from pathlib import Path

from perception.yolo_target_detector import YOLOTargetDetector


class Scalar:
    def __init__(self, value):
        self.value = value

    def item(self):
        return self.value


class Box:
    def __init__(self, class_id, confidence):
        self.cls = Scalar(class_id)
        self.conf = Scalar(confidence)


class Result:
    names = {0: "person", 1: "bottle"}

    def __init__(self, boxes):
        self.boxes = boxes


class FakeModel:
    def __init__(self, results):
        self.results = results
        self.calls = []

    def predict(self, **kwargs):
        self.calls.append(kwargs)
        return self.results


class YOLOTargetDetectorTests(unittest.TestCase):
    def test_extracts_requested_label_and_best_confidence(self):
        model = FakeModel([Result([Box(0, 0.82), Box(1, 0.95), Box(0, 0.40)])])
        detector = YOLOTargetDetector("fake.pt", model=model)
        with tempfile.TemporaryDirectory() as directory:
            image = Path(directory) / "capture.png"
            image.touch()
            detection = detector.detect(image, "person", 0.45)

        self.assertTrue(detection["found"])
        self.assertEqual(detection["match_count"], 1)
        self.assertAlmostEqual(detection["confidence"], 0.82)
        self.assertEqual(model.calls[0]["source"], str(image))
        self.assertFalse(model.calls[0]["verbose"])

    def test_returns_not_found_for_wrong_class(self):
        model = FakeModel([Result([Box(1, 0.95)])])
        detector = YOLOTargetDetector("fake.pt", model=model)
        with tempfile.TemporaryDirectory() as directory:
            image = Path(directory) / "capture.png"
            image.touch()
            detection = detector.detect(image, "person", 0.45)

        self.assertFalse(detection["found"])
        self.assertEqual(detection["match_count"], 0)


if __name__ == "__main__":
    unittest.main()
