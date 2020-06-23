#!/usr/bin/env python3.7
## Provides functionality to detect QR codes in image and publish observations to a topic
# @package scripts

import rospy
import threading
import time
from helpers.image_converter import msg_to_cv2
import detector.qr_detector as qr_detector
from std_msgs.msg import String
from konenako.msg import image, observation, observations, polygon, boundingbox, point64, warning
from konenako.srv import new_frequency, new_frequencyResponse, toggle, toggleResponse
from config.constants import name_det_qr, srv_frequency, srv_toggle, topic_warnings


## Read an image stream from a ROS topic, detect and decode QR codes from the frames and
#  publish the results in a topic.
class QRReader:

    last_detect = 0  # Last time detect() was called
    detect_lock = threading.Lock(
    )  # Locked, while detect() is being run, we detect() only 1 image at time

    frequency_change_lock = threading.Lock()

    def remove(self):
        # Close services and topics
        self.warning.unregister()
        self.toggle_service.shutdown()
        self.frequency_service.shutdown()

    def toggle_detection(self, toggle):
        if self.detect_on == toggle.state:
            return toggleResponse(
                "QR detection was already toggled to {}".format(
                    self.detect_on))

        self.detect_on = toggle.state
        return toggleResponse("QR detection toggled to {}".format(
            self.detect_on))

    def change_frequency(self, new_frequency):
        with self.frequency_change_lock:
            self.period = 1.0 / new_frequency.data
            return new_frequencyResponse(
                "QR detector freq set to period {}".format(self.period))

    def receive_img(self, img):
        # Detect from this image, if not already detecting from another image and within period time constraints
        if self.detect_on and (
                time.time() - self.last_detect
        ) > self.period and self.detect_lock.acquire(False):
            return self.detect(img)
        return []

    def __init__(self):
        self.name = name_det_qr
        # Attempt to get configuration parameters from the ROS parameter server
        self.detect_on = rospy.get_param("detect_on", True)
        # Frequency in hertz
        self.period = 1 / rospy.get_param("frequency", 5)

        # ROS service for changing detection frequency.
        self.frequency_service = rospy.Service(
            "{}/{}".format(self.name, srv_frequency), new_frequency,
            self.change_frequency)

        self.toggle_service = rospy.Service("{}/{}".format(self.name, srv_toggle),
                                            toggle, self.toggle_detection)

        # Warnings are published when processing takes longer than the given period
        self.warning = rospy.Publisher(topic_warnings, warning, queue_size=50)

    ## Process the image using qr_detector.py, publish each QR code's data and
    #  position qs a qr_observation ROS message.
    #
    #  @param msg image.msg ROS message of the frame to process.
    #  @param publish Toggle publishing to a topic.
    #  @return observations ROS message of the detections.
    def detect(self, img):
        # For tracking the frequency
        self.last_detect = time.time()
        period = self.period

        # Detect QR codes with qr_detector.py, save to list.
        observation_list = []
        for o in qr_detector.detect(img):
            observation_list.append(
                observation(
                    name_det_qr, 0, str(o["data"]), 1,
                    boundingbox(o["bbox"]["top"], o["bbox"]["right"],
                                o["bbox"]["bottom"], o["bbox"]["left"]),
                    polygon(len(o["polygon"]), [
                        point64(o["polygon"][0]["x"], o["polygon"][0]["y"]),
                        point64(o["polygon"][1]["x"], o["polygon"][1]["y"]),
                        point64(o["polygon"][2]["x"], o["polygon"][2]["y"]),
                        point64(o["polygon"][3]["x"], o["polygon"][3]["y"])
                    ]), img.shape[0], img.shape[1]))

        processing_time = time.time() - self.last_detect
        if processing_time > period:
            self.warning.publish(
                warning(
                    "Detecting QR-code took {}, while the period was set to {}!"
                    .format(processing_time, period)))

        # Ready to detect the next image
        self.detect_lock.release()

        return observation_list
