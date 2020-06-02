#!/usr/bin/env python3.7
import cv2 as cv
from typing import List
import rospy
from rostest.msg import observation, boundingbox
from rostest.srv import text_message, text_messageResponse
from object_detector import ObjectDetector

# https://github.com/opencv/opencv/wiki/TensorFlow-Object-Detection-API


def print_input(self, input):
    print("Received a message from another node: {}".f(input.message))


def run(showgui: bool):
    pub = rospy.Publisher("observations", observation, queue_size=50)
    recieved_message = rospy.Service('inputs', text_message, print_input)
    cam = cv.VideoCapture(
        "test.mp4")  # Can be replaced with camera id, path to camera etc.
    detector = ObjectDetector("ssd_mobilenet_v1_1_metadata_1.tflite",
                              "mscoco_complete_labels")
    while cam.grab():
        # Read and preprocess an image.
        img = cam.retrieve()[1]
        height = img.shape[0]
        width = img.shape[1]
        for detection in detector.detect(img):
            pub.publish(
                observation(
                    detection["class_id"], detection["label"],
                    detection["score"],
                    boundingbox(detection["bbox"]["top"],
                                detection["bbox"]["right"],
                                detection["bbox"]["bottom"],
                                detection["bbox"]["left"])))
            if showgui:
                # Draw boxes around objects
                top = detection["bbox"]["top"] * height
                left = detection["bbox"]["left"] * width
                right = detection["bbox"]["right"] * width
                bottom = detection["bbox"]["bottom"] * height
                cv.putText(
                    img, "{} score: {}".format(detection["label"],
                                               round(detection["score"], 3)),
                    (int(left), int(top - 5)), cv.QT_FONT_NORMAL, 1,
                    (255, 0, 255), 1, cv.LINE_AA)
                cv.rectangle(img, (int(left), int(top)),
                             (int(right), int(bottom)), (125, 255, 51),
                             thickness=2)

        # Display the result
        if showgui:
            cv.imshow('img', img)
            cv.waitKey(1)


if __name__ == "__main__":
    rospy.init_node("Testnode")
    run(False)
    #detector.run(True)
    rospy.spin()
