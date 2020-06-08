#!/usr/bin/env python3.7
import rospy
from image_converter import ImageConverter
from qr_detector import QrDetector
from std_msgs.msg import String
from rostest.msg import image


class QRReader:
    detect_on: bool

    def __init__(self, on: bool = False):
        self.detect_on = on
        self.converter = ImageConverter()
        self.detector = QrDetector()
        # the String message type should be replaced by own implementation
        # for QR code results.
        self.pub = rospy.Publisher("qr_results", String, queue_size=50)
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
            self.pub.publish(String(str(o["data"])))
            self.pub.publish(String(str(o["polygon"][0]["x"])))


if __name__ == "__main__":
    qr_reader = QRReader(True)
    rospy.init_node("qr_node")
    rospy.spin()
