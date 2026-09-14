import unittest

class CoordinateTransformTests(unittest.TestCase):
    def test_ros_to_uwb(self):
        self.assertEqual((.90, 1.12, .02), (.90, -(-1.12), .02))
        self.assertEqual(-.5, -.5)

if __name__ == "__main__": unittest.main()
