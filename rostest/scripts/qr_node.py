#!/usr/bin/env python3.7
import rospy
from image_converter import ImageConverter
from qr_detector import QrDetector
from std_msgs.msg import String
from rostest.msg import image, qr_observation, polygon, boundingbox


class QRReader:
    detect_on: bool

    def __init__(self, on: bool = False):
        self.detect_on = on
        self.converter = ImageConverter()
        self.detector = QrDetector()
        # the String message type should be replaced by own implementation
        # for QR code results.
        self.pub = rospy.Publisher("qr_results", qr_observation, queue_size=50)
        # Camera feed is read from ros messages
        self.input_sub = rospy.Subscriber("camera_feed", image, self.detect)

    def detect(self, msg: image):
        # Only run the detection if it is turned on
        if not self.detect_on:
            return

        # Convert image from Image message to numpy ndarray
        img = self.converter.msg_to_cv2(msg)

        observations = self.detector.detect(img)
        # temporary
        for o in observations:
            obs = qr_observation(
                str(o["data"]),
                boundingbox(o["bbox"]["top"], o["bbox"]["right"],
                            o["bbox"]["bottom"], o["bbox"]["left"]),
                polygon(o["polygon"][0]["x"], o["polygon"][0]["y"],
                        o["polygon"][1]["x"], o["polygon"][1]["y"],
                        o["polygon"][2]["x"], o["polygon"][2]["y"],
                        o["polygon"][3]["x"], o["polygon"][3]["y"]))
            self.pub.publish(obs)


if __name__ == "__main__":
    qr_reader = QRReader(True)
    rospy.init_node("qr_node")
    rospy.spin()
