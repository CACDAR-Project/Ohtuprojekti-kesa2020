import numpy as np
import cv2 as cv
from scripts.qr_detector import QrDetector

# poetry run python -m scripts.demo.qr from rostest/


def run():
    cam = cv.VideoCapture("scripts/demo/data/qr.mp4")
    detector = QrDetector()
    while cam.grab():
        img = cam.retrieve()[1]
        height = img.shape[0]
        width = img.shape[1]
        for detection in detector.detect(img):
            # Draw rectangle around the qr-code
            top = detection["bbox"]["top"] * height
            left = detection["bbox"]["left"] * width
            right = detection["bbox"]["right"] * width
            bottom = detection["bbox"]["bottom"] * height
            cv.rectangle(img, (int(left), int(top)), (int(right), int(bottom)),
                         (125, 255, 51),
                         thickness=2)

            # Draw text with QR data above the box
            cv.putText(img, "{}".format(detection["data"]),
                       (int(left), int(top - 5)), cv.QT_FONT_NORMAL, 1,
                       (255, 0, 255), 1, cv.LINE_AA)

            # Draw polygon around the qr-code
            indices = []
            for p in detection["polygon"]:
                indices.append(p["x"] * width)
                indices.append(p["y"] * height)
            indices = np.array(indices, np.int32).reshape(4, 2).reshape(
                (-1, 1, 2))
            cv.polylines(img, [indices], True, (0, 0, 255), 2)

        # Display the result
        cv.imshow('img', cv.resize(img, (1280, 720)))
        cv.waitKey(1)


if __name__ == '__main__':
    run()
