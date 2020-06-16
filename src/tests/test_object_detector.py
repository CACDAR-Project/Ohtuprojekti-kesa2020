import unittest
import cv2
from scripts.detector.object_detector import ObjectDetector


class Detector(unittest.TestCase):
    def setUp(self):
        self.res_path = '../resources'

    def test_detect_cow_banana(self):
        detector = ObjectDetector("{}/tflite_models/ssd_mobilenet_v1_1_metadata_1.tflite".format(self.res_path),
                                  "{}/tflite_models/mscoco_complete_labels".format(self.res_path))
        img = cv2.imread("{}/images/banana_and_cow.png".format(self.res_path))
        results = detector.detect(img)
        labels = {d['label'] for d in results}
        self.assertSetEqual(labels, {'cow', 'banana'})


if __name__ == '__main__':
    unittest.main()
