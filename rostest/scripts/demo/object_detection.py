#!/usr/bin/env python3.7
import cv2 as cv
from typing import List
from scripts.object_detector import ObjectDetector

source = "test.mp4"


def run():
    cam = cv.VideoCapture(source)
    detector = ObjectDetector("ssd_mobilenet_v1_1_metadata_1.tflite",
                              "mscoco_complete_labels")

    while cam.grab():
        # Read and preprocess an image.
        img = cam.retrieve()[1]
        height = img.shape[0]
        width = img.shape[1]
        for detection in detector.detect(img):
            print(detection)
            # Draw boxes around objects
            top = detection["bbox"]["top"] * height
            left = detection["bbox"]["left"] * width
            right = detection["bbox"]["right"] * width
            bottom = detection["bbox"]["bottom"] * height
            cv.putText(
                img, "{} score: {}".format(detection["label"],
                                           round(detection["score"], 3)),
                (int(left), int(top - 5)), cv.QT_FONT_NORMAL, 1, (255, 0, 255),
                1, cv.LINE_AA)
            cv.rectangle(img, (int(left), int(top)), (int(right), int(bottom)),
                         (125, 255, 51),
                         thickness=2)

        # Display the result
        cv.imshow('img', img)
        cv.waitKey(1)


if __name__ == "__main__":
    run()
