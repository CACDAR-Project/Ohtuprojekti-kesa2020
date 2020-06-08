import unittest
import numpy as np
import cv2 as cv
from scripts.detector.qr_detector import QrDetector


class QRCodeDetector(unittest.TestCase):
    def test_qr_two(self):
        detector = QrDetector()
        img = cv.imread("tests/data/qr.png")

        results = detector.detect(img)
        data = {r["data"] for r in results}

        self.assertSetEqual(
            data,
            set(
                map(
                    lambda s: bytes(s, "ascii"), {
                        "Lorem ipsum dolor sit amet, consectetur adipiscing elit. Cras eget sem sit amet magna dapibus facilisis. Donec cursus eros orci.",
                        "Lorem ipsum dolor."
                    })))

    def test_qr_none(self):
        detector = QrDetector()
        img = cv.imread("tests/data/banana_and_cow.png")

        results = detector.detect(img)
        self.assertEqual(len(results), 0)


if __name__ == '__main__':
    unittest.main()
