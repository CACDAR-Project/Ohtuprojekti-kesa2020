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
import konenako.srv as srv
from detector.tensorflow_detector import TensorFlowDetector
from config.constants import srv_frequency, srv_toggle, srv_score_treshold, topic_warnings


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
        self.score_service.shutdown()

    ## Set a new frequency for the object detection.
    #  @param new_frequency The new frequency in hz as a new_frequency.srv message.
    #  @return Confirmation string as a new_frequencyResponse.srv message.
    def change_frequency(
            self,
            new_frequency: srv.new_frequency) -> srv.new_frequencyResponse:
        with self.frequency_change_lock:
            self.period = 1.0 / new_frequency.data
            return srv.new_frequencyResponse(
                "Object detector freq set to period {}".format(self.period))

    def toggle_detection(self, toggle):
        if self.detect_on == toggle.state:
            return srv.toggleResponse(
                "Object detection was already toggled to {}".format(
                    self.detect_on))

        self.detect_on = toggle.state
        return srv.toggleResponse("Object detection toggled to {}".format(
            self.detect_on))

    def set_score_threshold(self, msg):
        self.detector.score_threshold = msg.score
        return srv.score_thresholdResponse()

    def get_labels(self) -> List[str]:
        return self.detector.get_labels()

    def __init__(self,
                 name,
                 model_path,
                 label_path,
                 frequency=5,
                 detect_on=True,
                 score_threshold=0.3):
        self.name = name
        self.detect_on = detect_on

        ## Frequency in hertz
        self.period = 1 / frequency

        ## Helper class used for detecting objects
        self.detector = TensorFlowDetector(model_path, label_path)
        self.detector.score_threshold = score_threshold

        # Register to services
        self.frequency_service = rospy.Service(
            "{}/{}".format(name, srv_frequency), srv.new_frequency,
            self.change_frequency)

        self.toggle_service = rospy.Service("{}/{}".format(name, srv_toggle),
                                            srv.toggle, self.toggle_detection)

        self.score_service = rospy.Service(
            "{}/{}".format(name, srv_score_treshold), srv.score_threshold,
            self.set_score_threshold)

        # Warnings are published when processing takes longer than the given period
        self.warning = rospy.Publisher(topic_warnings, warning, queue_size=50)

    ## Detects image, if not already detecting from another
    #  image and enough time has passed since the previous detection.
    def receive_img(self, img):
        # Return -1 if detection is skipped due to the rate limit.
        if (time.time() - self.last_detect) < self.period:
            return -1
        # Return -2 if the detection is turned off.
        if not self.detect_on:
            return -2
        # Detect from this image, if not already detecting from another image.
        if self.detect_lock.acquire(False):
            return self.detect(img)

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

        observations_res = self.detector.detect(img)
        observations_mapobj = map(
            lambda detection: observation(
                self.name, detection['class_id'], detection['label'], '',
                detection['score'],
                boundingbox(detection['bbox']['top'], detection['bbox'][
                    'right'], detection['bbox']['bottom'], detection['bbox'][
                        'left']), polygon(0, tuple())), observations_res)

        processing_time = time.time() - self.last_detect
        if processing_time > period:
            self.warning.publish(
                warning(
                    "{}: Detecting objects took {}, while the period was set to {}!"
                    .format(self.name, processing_time, period)))

        # Ready to detect the next image
        self.detect_lock.release()

        return observations_mapobj
