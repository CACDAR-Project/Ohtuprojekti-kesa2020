#!/usr/bin/env python3.7
import cv2 as cv
import rospy
import time
from typing import List
from rostest.msg import observation, boundingbox, image
from rostest.srv import text_message, text_messageResponse, new_frequency, new_frequencyResponse
from object_detector import ObjectDetector
from image_converter import ImageConverter

# https://github.com/opencv/opencv/wiki/TensorFlow-Object-Detection-API

#frequency in hertz
run_frequency = 1
period = 1.0 / run_frequency


def print_input(input):
    print(f"Received a message from another node: {input.message}")
    return text_messageResponse("We received you message!")


def change_frequency(new_frequency):
    global run_frequency
    global period
    run_frequency = new_frequency.data
    period = 1.0 / run_frequency
    return new_frequencyResponse("We received you frequency!")

def detect(img):
    # Converting the image back from ros Image message to a numpy ndarray
    img = converter.msg_to_cv2(img)
    # Resizing could be better to do before sending the message, to save a little bandwidth
    inp = cv.resize(img, (300, 300))
    # inp = inp[:, :, [2, 1, 0]]  # BGR2RGB

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
    return

def run(showgui: bool):
    pub = rospy.Publisher("observations", observation, queue_size=50)
    message_service = rospy.Service('inputs', text_message, print_input)
    frequency_service = rospy.Service('frequency', new_frequency,
                                      change_frequency)
    # Image feed topic
    rospy.Subscriber("camera_feed", image, detect)
    detector = ObjectDetector("ssd_mobilenet_v1_1_metadata_1.tflite",
                              "mscoco_complete_labels")
    converter = ImageConverter()


if __name__ == "__main__":
    rospy.init_node("Testnode")
    run(False)
    #run(True)
    rospy.spin()
