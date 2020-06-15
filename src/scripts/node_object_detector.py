#!/usr/bin/env python3.7

## Provides functionality for detecting objects from images and publish observations to a topic
#  https://github.com/opencv/opencv/wiki/TensorFlow-Object-Detection-API
#  @package scripts

import cv2 as cv
import rospy
import time
import threading
from typing import List
from konenako.msg import observation, observations, boundingbox, image, warning
from konenako.srv import text_message, text_messageResponse, new_frequency, new_frequencyResponse, toggle, toggleResponse
from detector.object_detector import ObjectDetector
from helpers.image_converter import msg_to_cv2


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

    def toggle_detection(self, toggle):
        if self.detect_on == toggle.state:
            return toggleResponse(
                "Object detection was already toggled to {}".format(
                    self.detect_on))
        if not toggle.state:
            # Unsubscribe from camera feed when not detecting
            self.input_sub.unregister()
        else:
            self.input_sub = rospy.Subscriber("camera/images", image,
                                              self.receive_img)
        self.detect_on = toggle.state
        return toggleResponse("Object detection toggled to {}".format(
            self.detect_on))

    ## Initializes topics and services and sets class variable detect_on to true
    def __init__(self):
        # Attempt to get configuration parameters from the ROS parameter server
        self.detect_on = rospy.get_param("detect_on", True)
        if not rospy.has_param("model_file"):
            raise Exception(
                "A model file must be specified as a ROS parameter")
        if not rospy.has_param("label_file"):
            raise Exception(
                "A label file must be specified as a ROS parameter")
        self.model_file = rospy.get_param("model_file")
        self.label_file = rospy.get_param("label_file")
        ## Frequency in hertz
        self.run_frequency = rospy.get_param("frequency", 1)

        ## Minimum time it takes for one loop
        self.period = 1.0 / self.run_frequency
        ## Helper class used for detecting objects
        self.detector = ObjectDetector(self.model_file, self.label_file)

        self.pub = rospy.Publisher("{}/observations".format(rospy.get_name()),
                                   observations,
                                   queue_size=50)
        frequency_service = rospy.Service(
            "{}/frequency".format(rospy.get_name()), new_frequency,
            self.change_frequency)
        # Image feed topic
        if self.detect_on:
            self.input_sub = rospy.Subscriber("camera/images", image,
                                              self.receive_img)

        rospy.Service("{}/toggle".format(rospy.get_name()), toggle,
                      self.toggle_detection)

        # Warnings are published when processing takes longer than the given period
        self.warning = rospy.Publisher("{}/warnings".format(rospy.get_name()),
                                       warning,
                                       queue_size=50)

    ## Detects image, if not already detecting from another
    #  image and enough time has passed since the previous detection.
    def receive_img(self, msg: image):
        # Detect from this image, if not already detecting from another image and within period time constraints
        if (time.time() - self.last_detect
            ) > self.period and self.detect_lock.acquire(False):
            self.detect(msg)

    ## Builds observation messages and publishes them.
    #  Prints a warning if time between detections grows too large.
    #  @todo Announce "warnings" to a topic
    def detect(self, msg: image):
        # For tracking the frequency
        self.last_detect = time.time()
        period = self.period
        # Convert the image back from an image message to a numpy ndarray
        img = msg_to_cv2(msg)[2]
        observation_list = []

        for detection in self.detector.detect(img):
            observation_list.append(
                observation(
                    msg.camera_id, msg.image_counter, 
                    detection["class_id"], detection["label"],
                    detection["score"],
                    boundingbox(detection["bbox"]["top"],
                                detection["bbox"]["right"],
                                detection["bbox"]["bottom"],
                                detection["bbox"]["left"])))
        self.pub.publish(observations(self.model_file, observation_list))

        processing_time = time.time() - self.last_detect
        if processing_time > period:
            self.warning.publish(
                warning(
                    "Detecting QR-code took {}, while the period was set to {}!"
                    .format(processing_time, period)))

        # Ready to detect the next image
        self.detect_lock.release()


if __name__ == "__main__":
    rospy.init_node("object_detector")
    ObjectNode()
    rospy.spin()
