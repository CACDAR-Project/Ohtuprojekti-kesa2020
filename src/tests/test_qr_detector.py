import unittest
import numpy as np
import cv2 as cv

import scripts.detector.qr_detector as qr_detector


class QRCodeDetector(unittest.TestCase):
    def setUp(self):
        self.res_path = '../resources'

    def test_qr_two(self):
        img = cv.imread("{}/images/qr.png".format(self.res_path))

        results = qr_detector.detect(img)
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
        img = cv.imread("{}/images/banana_and_cow.png".format(self.res_path))

        results = qr_detector.detect(img)
        self.assertEqual(len(results), 0)


if __name__ == '__main__':
    unittest.main()
