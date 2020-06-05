#!/usr/bin/env python3.7
import cv2 as cv
import rospy
import time
from typing import List
from rostest.msg import observation, boundingbox
from rostest.srv import text_message, text_messageResponse, new_frequency, new_frequencyResponse
from object_detector import ObjectDetector

# https://github.com/opencv/opencv/wiki/TensorFlow-Object-Detection-API

#frequency in hertz
run_frequency = 1
period = 1.0 / run_frequency
#video source - camera id, path to camera etc.
source = "test.mp4"


def print_input(input):
    print(f"Received a message from another node: {input.message}")
    return text_messageResponse("We received you message!")


def change_frequency(new_frequency):
    global run_frequency
    global period
    run_frequency = new_frequency.data
    period = 1.0 / run_frequency
    return new_frequencyResponse("We received you frequency!")


def run():
    pub = rospy.Publisher("observations", observation, queue_size=50)
    message_service = rospy.Service('inputs', text_message, print_input)
    frequency_service = rospy.Service('frequency', new_frequency,
                                      change_frequency)
    cam = cv.VideoCapture(source)
    detector = ObjectDetector("ssd_mobilenet_v1_1_metadata_1.tflite",
                              "mscoco_complete_labels")

    while cam.grab():
        start_time = time.time()
        # Read and preprocess an image.
        img = cam.retrieve()[1]
        for detection in detector.detect(img):
            pub.publish(
                observation(
                    detection["class_id"], detection["label"],
                    detection["score"],
                    boundingbox(detection["bbox"]["top"],
                                detection["bbox"]["right"],
                                detection["bbox"]["bottom"],
                                detection["bbox"]["left"])))

        time_to_sleep = period - (time.time() - start_time)
        if time_to_sleep > 0:
            time.sleep(time_to_sleep)


if __name__ == "__main__":
    rospy.init_node("Testnode")
    run()
    rospy.spin()
