#!/usr/bin/env python3.7
## Provides functionality to detect QR codes in image and publish observations to a topic
# @package scripts

import rospy
import threading
import time
from helpers.image_converter import msg_to_cv2
import detector.qr_detector as qr_detector
from std_msgs.msg import String
from konenako.msg import image, qr_observation, qr_observations, polygon, boundingbox, point64, warning
from konenako.srv import new_frequency, new_frequencyResponse, toggle, toggleResponse


## Read an image stream from a ROS topic, detect and decode QR codes from the frames and
#  publish the results in a topic.
class QRReader:

    last_detect = 0  # Last time detect() was called
    detect_lock = threading.Lock(
    )  # Locked, while detect() is being run, we detect() only 1 image at time

    frequency_change_lock = threading.Lock()

    def toggle_detection(self, toggle):
        if self.detect_on == toggle.state:
            return toggleResponse(
                "QR detection was already toggled to {}".format(
                    self.detect_on))
        if not toggle.state:
            # Unsubscribe from camera feed when not detecting
            self.input_sub.unregister()
        else:
            self.input_sub = rospy.Subscriber("camera/images", image,
                                              self.receive_img)
        self.detect_on = toggle.state
        return toggleResponse("QR detection toggled to {}".format(
            self.detect_on))

    def change_frequency(self, new_frequency):
        with self.frequency_change_lock:
            self.run_frequency = new_frequency.data
            self.qr_period = 1.0 / self.run_frequency
            return new_frequencyResponse(
                "QR detector freq set to {}, period {}".format(
                    self.run_frequency, self.qr_period))

    def receive_img(self, msg: image):
        # Detect from this image, if not already detecting from another image and within period time constraints
        if (time.time() - self.last_detect
            ) > self.qr_period and self.detect_lock.acquire(False):
            self.detect(msg)

    def __init__(self, state: bool = False):
        # Attempt to get configuration parameters from the ROS parameter server
        self.detect_on = rospy.get_param("detect_on", True)
        # Frequency in hertz
        self.run_frequency = rospy.get_param("frequency", 10)
        self.qr_period = 1.0 / self.run_frequency

        # Results are published as qr_observation.msg ROS messages.
        self.pub = rospy.Publisher("{}/observations".format(rospy.get_name()),
                                   qr_observations,
                                   queue_size=50)

        # Camera feed is read from ros messages
        if self.detect_on:
            self.input_sub = rospy.Subscriber("camera/images", image,
                                              self.receive_img)

        # ROS service for changing detection frequency.
        frequency_service = rospy.Service(
            "{}/frequency".format(rospy.get_name()), new_frequency,
            self.change_frequency)

        rospy.Service("{}/toggle".format(rospy.get_name()), toggle,
                      self.toggle_detection)

        # Warnings are published when processing takes longer than the given period
        self.warning = rospy.Publisher("{}/warnings".format(rospy.get_name()),
                                       warning,
                                       queue_size=50)

    ## Process the image using qr_detector.py, publish each QR code's data and
    #  position qs a qr_observation ROS message.
    #
    #  @param msg image.msg ROS message of the frame to process.
    def detect(self, msg: image):
        # For tracking the frequency
        self.last_detect = time.time()
        period = self.qr_period

        # Convert image from Image message to numpy ndarray
        img = msg_to_cv2(msg)[2]

        # Detect QR codes with qr_detector.py, save to list.
        observations = []
        for o in qr_detector.detect(img):
            observations.append(
                qr_observation(
                    msg.camera_id, msg.image_counter, str(o["data"]),
                    boundingbox(o["bbox"]["top"], o["bbox"]["right"],
                                o["bbox"]["bottom"], o["bbox"]["left"]),
                    polygon(len(o["polygon"]), [
                        point64(o["polygon"][0]["x"], o["polygon"][0]["y"]),
                        point64(o["polygon"][1]["x"], o["polygon"][1]["y"]),
                        point64(o["polygon"][2]["x"], o["polygon"][2]["y"]),
                        point64(o["polygon"][3]["x"], o["polygon"][3]["y"])
                    ])))
        # Publish observations to a topic.
        self.pub.publish(observations)

        processing_time = time.time() - self.last_detect
        if processing_time > period:
            self.warning.publish(
                warning(
                    "Detecting QR-code took {}, while the period was set to {}!"
                    .format(processing_time, period)))

        # Ready to detect the next image
        self.detect_lock.release()


if __name__ == "__main__":
    rospy.init_node("qr_detector")
    qr_reader = QRReader(True)
    rospy.spin()
