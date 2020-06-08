#!/usr/bin/env python3.7
import rospy
import time
import image_converter
import qr_detector
from std_msgs.msg import String
from rostest.msg import image, qr_observation, polygon, boundingbox, point64
from rostest.srv import new_frequency, new_frequencyResponse

# Frequency in hertz
qr_run_frequency = 10
qr_period = 1.0 / qr_run_frequency


class QRReader:
    def change_frequency(self, new_frequency):
        global qr_run_frequency
        global qr_period
        qr_run_frequency = new_frequency.data
        qr_period = 1.0 / qr_run_frequency
        return new_frequencyResponse("QR detector received you frequency!")

    def __init__(self, on: bool = False):
        self.detect_on = on

        # Results are published as qr_observation.msg ROS messages.
        self.pub = rospy.Publisher("qr_results", qr_observation, queue_size=50)

        # Camera feed is read from ros messages
        self.input_sub = rospy.Subscriber("camera_feed", image, self.detect)

        # ROS service for changing detection frequency.
        frequency_service = rospy.Service('qr_frequency', new_frequency,
                                          self.change_frequency)

    def detect(self, msg: image):
        # For tracking the frequency
        start_time = time.time()
        # Only run the detection if it is turned on
        if not self.detect_on:
            return

        # Convert image from Image message to numpy ndarray
        img = image_converter.msg_to_cv2(msg)

        # Detect QR codes with qr_detector.py
        observations = qr_detector.detect(img)
        for o in observations:
            obs = qr_observation(
                str(o["data"]),
                boundingbox(o["bbox"]["top"], o["bbox"]["right"],
                            o["bbox"]["bottom"], o["bbox"]["left"]),
                polygon(len(o["polygon"]), [
                    point64(o["polygon"][0]["x"], o["polygon"][0]["y"]),
                    point64(o["polygon"][1]["x"], o["polygon"][1]["y"]),
                    point64(o["polygon"][2]["x"], o["polygon"][2]["y"]),
                    point64(o["polygon"][3]["x"], o["polygon"][3]["y"])
                ]))
            self.pub.publish(obs)

        time_to_sleep = qr_period - (time.time() - start_time)
        if time_to_sleep > 0:
            time.sleep(time_to_sleep)
        return


if __name__ == "__main__":
    qr_reader = QRReader(True)
    rospy.init_node("qr_node")
    rospy.spin()
