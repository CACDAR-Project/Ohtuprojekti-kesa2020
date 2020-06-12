#!/usr/bin/env python3.7

## Provides functionality for detecting objects from images and publish observations to a topic
#  https://github.com/opencv/opencv/wiki/TensorFlow-Object-Detection-API
#  @package scripts

import cv2 as cv
import rospy
import time
import threading
from typing import List
from konenako.msg import observation, boundingbox, image
from konenako.srv import text_message, text_messageResponse, new_frequency, new_frequencyResponse
from detector.object_detector import ObjectDetector
from helpers.image_converter import msg_to_cv2


## Class for detecting objects in images.
#  Reads an image stream from a ROS topic, detects objects in the
#  frames using a Tensorflow Lite model and publishes the results in a topic.
class ObjectNode:

    ## Frequency in hertz
    run_frequency = 1
    ## Minimum time it takes for one loop
    period = 1.0 / run_frequency

    ## Helper class used for detecting objects
    detector = ObjectDetector("ssd_mobilenet_v1_1_metadata_1.tflite",
                              "mscoco_complete_labels")

    ## Lock used to ensure thread safety when changing frequency
    frequency_change_lock = threading.Lock()

    ## Class variable containing time of last detection. Used and modified in several functions.
    last_detect = 0

    ## Lock used to ensure that detections are not being done too often
    detect_lock = threading.Lock()

    ## Set a new frequency for the object detection.
    #  @param new_frequency The new frequency in hz as a new_frequency.srv message.
    #  @return Confirmation string as a new_frequencyResponse.srv message.
    def change_frequency(
            self, new_frequency: new_frequency) -> new_frequencyResponse:
        with self.frequency_change_lock:
            self.run_frequency = new_frequency.data
            self.period = 1.0 / self.run_frequency
            return new_frequencyResponse(
                "Object detector freq set to {}, period {}".format(
                    self.run_frequency, self.period))

    ## Initializes topics and services and sets class variable detect_on to true
    def run(self):
        self.detect_on = True
        self.pub = rospy.Publisher("{}/observations".format(rospy.get_name()),
                                   observation,
                                   queue_size=50)
        frequency_service = rospy.Service(
            "{}/frequency".format(rospy.get_name()), new_frequency,
            self.change_frequency)
        # Image feed topic
        rospy.Subscriber("camera/images", image, self.receive_img)

    ## Detects image, if not already detecting from another
    #  image and within period time constraints
    def receive_img(self, msg: image):
        if self.detect_on and (
                time.time() - self.last_detect
        ) > self.period and self.detect_lock.acquire(False):
            self.detect(msg)

    ## Builds observation messages and publishes them.
    #  Prints warning if time between detections grows too large
    #  @todo Announce "warnings" to a topic
    def detect(self, img: image):
        # For tracking the frequency
        self.last_detect = time.time()
        period = self.period
        # Convert the image back from an image message to a numpy ndarray
        img = msg_to_cv2(img)

        for detection in self.detector.detect(img):

            self.pub.publish(
                observation(
                    detection["class_id"], detection["label"],
                    detection["score"],
                    boundingbox(detection["bbox"]["top"],
                                detection["bbox"]["right"],
                                detection["bbox"]["bottom"],
                                detection["bbox"]["left"])))

        processing_time = time.time() - self.last_detect
        if processing_time > period:
            print("Detecting objects took {}, while the period was set to {}!".
                  format(processing_time, period))

        # Ready to detect the next image
        self.detect_lock.release()


if __name__ == "__main__":
    rospy.init_node("object_detector")
    ObjectNode().run()
    rospy.spin()
