#!/usr/bin/env python3.7

## Provides functionality for detecting objects from images and publish observations to a topic
#  https://github.com/opencv/opencv/wiki/TensorFlow-Object-Detection-API
#  @package scripts

import cv2 as cv
import rospy
import time
import threading
from typing import List
from konenako.msg import observation, observations, boundingbox, polygon, image, warning
from konenako.srv import text_message, text_messageResponse, new_frequency, new_frequencyResponse, toggle, toggleResponse
from detector.object_detector import ObjectDetector


## Class for detecting objects in images.
#  Reads an image stream from a ROS topic, detects objects in the
#  frames using a Tensorflow Lite model and publishes the results in a topic.
class ObjectNode:
    ## Lock used to ensure thread safety when changing frequency
    frequency_change_lock = threading.Lock()

    ## Class variable containing time of last detection. Used and modified in several functions.
    last_detect = 0

    ## Lock used to ensure that detections are not being done simultaneously.
    detect_lock = threading.Lock()

    def remove(self):
        # Close services and topics
        self.warning.unregister()
        self.toggle_service.shutdown()
        self.frequency_service.shutdown()

    ## Set a new frequency for the object detection.
    #  @param new_frequency The new frequency in hz as a new_frequency.srv message.
    #  @return Confirmation string as a new_frequencyResponse.srv message.
    def change_frequency(
            self, new_frequency: new_frequency) -> new_frequencyResponse:
        with self.frequency_change_lock:
            self.period = 1.0 / new_frequency.data
            return new_frequencyResponse(
                "Object detector freq set to period {}".format(self.period))

    def toggle_detection(self, toggle):
        if self.detect_on == toggle.state:
            return toggleResponse(
                "Object detection was already toggled to {}".format(
                    self.detect_on))

        self.detect_on = toggle.state
        return toggleResponse("Object detection toggled to {}".format(
            self.detect_on))

    ## Initializes topics and services and sets class variable detect_on to true
    def __init__(self,
                 name,
                 model_path,
                 label_path,
                 frequency=5,
                 detect_on=True):
        self.name = name
        # Attempt to get configuration parameters from the ROS parameter server
        self.detect_on = detect_on
        #if not rospy.has_param("model_file"):
        #    raise Exception(
        #        "A model file must be specified as a ROS parameter")
        #if not rospy.has_param("label_file"):
        #    raise Exception(
        #        "A label file must be specified as a ROS parameter")
        #self.model_file = rospy.get_param("model_file")
        #self.label_file = rospy.get_param("label_file")

        ## Frequency in hertz
        self.period = 1 / frequency

        ## Helper class used for detecting objects
        self.detector = ObjectDetector(model_path, label_path)

        self.frequency_service = rospy.Service("{}/frequency".format(name),
                                               new_frequency,
                                               self.change_frequency)

        self.toggle_service = rospy.Service("{}/toggle".format(name), toggle,
                                            self.toggle_detection)

        # Warnings are published when processing takes longer than the given period
        self.warning = rospy.Publisher("warnings", warning, queue_size=50)

    ## Detects image, if not already detecting from another
    #  image and enough time has passed since the previous detection.
    def receive_img(self, img):
        # Detect from this image, if not already detecting from another image and within period time constraints
        if self.detect_on and (
                time.time() - self.last_detect
        ) > self.period and self.detect_lock.acquire(False):
            return self.detect(img)
        return []

    ## Builds observation messages and publishes them.
    #  Prints a warning if time between detections grows too large.
    #  @todo Announce "warnings" to a topic
    #
    #  @param img cv2 image
    #  @return observations ROS message of the detections.
    def detect(self, img):
        # For tracking the frequency
        self.last_detect = time.time()
        period = self.period
        # Convert the image back from an image message to a numpy ndarray
        observation_list = []

        for detection in self.detector.detect(img):
            observation_list.append(
                observation(
                    self.name, detection["class_id"], detection["label"],
                    detection["score"],
                    boundingbox(detection["bbox"]["top"],
                                detection["bbox"]["right"],
                                detection["bbox"]["bottom"],
                                detection["bbox"]["left"]), polygon(0, [])))

        processing_time = time.time() - self.last_detect
        if processing_time > period:
            self.warning.publish(
                warning(
                    "{}: Detecting objects took {}, while the period was set to {}!"
                    .format(self.name, processing_time, period)))

        # Ready to detect the next image
        self.detect_lock.release()

        return observation_list
