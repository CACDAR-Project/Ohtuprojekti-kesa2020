#!/usr/bin/env python3.7
## @package scripts

import cv2 as cv
import rospy
import time
import threading
from typing import List

from konenako.msg import observation, boundingbox, image
from konenako.srv import text_message, text_messageResponse, new_frequency, new_frequencyResponse
from detector.object_detector import ObjectDetector
from helpers.image_converter import msg_to_cv2

# https://github.com/opencv/opencv/wiki/TensorFlow-Object-Detection-API


#  Read an image stream from a ROS topic, detect objects i nthe frames using a Tensorflow
#  Lite model and publish the results in a topic.
class ObjectNode:

    # frequency in hertz
    run_frequency = 1
    period = 1.0 / run_frequency

    detector = ObjectDetector("ssd_mobilenet_v1_1_metadata_1.tflite",
                              "mscoco_complete_labels")

    frequency_change_lock = threading.Lock()
    last_detect = 0
    detect_lock = threading.Lock()

    def print_input(self, input):
        print(f"Received a message from another node: {input.message}")
        return text_messageResponse("We received you message!")

    ## Set a new frequency for the object detection.
    #  @param new_frequency The new frequency in hz as a new_frequency.srv message.
    #  @return Confirmation string as a new_frequencyResponse.srv message.
    def change_frequency(self, new_frequency):
        with self.frequency_change_lock:
            self.run_frequency = new_frequency.data
            self.period = 1.0 / self.run_frequency
            return new_frequencyResponse(
                "Object detector freq set to {}, period {}".format(
                    self.run_frequency, self.period))

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

    def receive_img(self, msg: image):
        # Detect from this image, if not already detecting from another image and within period time constraints
        if self.detect_on and (
                time.time() - self.last_detect
        ) > self.period and self.detect_lock.acquire(False):
            self.detect(msg)

    def detect(self, img):
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
                  format(processing_time,
                         period))  # TODO: Announce "warnings" to topic

        # Ready to detect the next image
        self.detect_lock.release()


if __name__ == "__main__":
    rospy.init_node("object_detector")
    ObjectNode().run()
    rospy.spin()
