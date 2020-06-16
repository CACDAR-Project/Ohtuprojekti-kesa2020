import unittest
import cv2
from scripts.detector.object_detector import ObjectDetector


class Detector(unittest.TestCase):
    def test_detect_cow_banana(self):
        detector = ObjectDetector("ssd_mobilenet_v1_1_metadata_1.tflite",
                                  "mscoco_complete_labels")
        img = cv2.imread("tests/data/banana_and_cow.png")
        results = detector.detect(img)
        labels = {d['label'] for d in results}
        self.assertSetEqual(labels, {'cow','banana'})


if __name__ == '__main__':
    unittest.main()
