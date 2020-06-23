#!/usr/bin/env python3.7

## Publishes image stream to defined topic
#  @package scripts

import cv2 as cv
import rospy
import signal
from numpy import ndarray

from konenako.msg import image
from helpers.image_converter import cv2_to_msg


class Camera:
    stop = False

    def handle_signal_stop(self, signum, frame):
        self.stop = True

    ## Main function for camera node.
    #  Creates topic named "node_name/images" where image/video source
    #  is being published as an image message.
    def run(self, hz):
        signal.signal(signal.SIGINT, self.handle_signal_stop)
        signal.signal(signal.SIGTERM, self.handle_signal_stop)

        rospy.init_node("camera")
        # We only want to process the latest frame, and Publisher's queue is
        # FIFO (?), thus queue_size is set to 1.
        pub = rospy.Publisher("{}/images".format(rospy.get_name()),
                              image,
                              queue_size=1)
        # Variable containing path to video, picture or camera device
        source = rospy.get_param("video_source", "test.mp4")
        if rospy.has_param("video_feed_name"):
            feed_name = rospy.get_param("video_feed_name")
        else:
            feed_name = source
        # Counter for identifying the frames
        counter = 1
        rate = rospy.Rate(hz)
        while True and not self.stop:
            cam = cv.VideoCapture(source)
            while cam.grab() and not self.stop:
                img = cam.retrieve()[1]
                # Converting the image from numpy ndarray to an image message
                msg = cv2_to_msg(feed_name, counter, img)
                pub.publish(msg)
                counter += 1
                rate.sleep()


if __name__ == "__main__":
    Camera().run(rospy.get_param("camhz", 9999))
    rospy.spin()
