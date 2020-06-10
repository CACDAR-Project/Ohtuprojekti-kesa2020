#!/usr/bin/env python3.7
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


class ObjectNode:

    #frequency in hertz
    run_frequency = 1
    period = 1.0 / run_frequency

    detector = ObjectDetector("ssd_mobilenet_v1_1_metadata_1.tflite",
                              "mscoco_complete_labels")
    pub = rospy.Publisher("observations", observation, queue_size=50)

    frequency_change_lock = threading.Lock()
    last_detect = 0
    detect_lock = threading.Lock()

    def change_frequency(self, new_frequency):
        with self.frequency_change_lock:
            self.run_frequency = new_frequency.data
            self.period = 1.0 / self.run_frequency
            return new_frequencyResponse(
                "Object detector freq set to {}, period {}".format(
                    self.run_frequency, self.period))

    def run(self):
        self.detect_on = True
        frequency_service = rospy.Service('frequency', new_frequency,
                                          self.change_frequency)
        # Image feed topic
        rospy.Subscriber("camera_feed", image, self.receive_img)

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
        # Converting the image back from ros Image message to a numpy ndarray
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
    rospy.init_node("Testnode")
    ObjectNode().run()
    rospy.spin()
