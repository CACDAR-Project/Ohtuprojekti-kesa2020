#!/usr/bin/env python3.7
## @package scripts

import cv2 as cv
import rospy
from numpy import ndarray

from konenako.msg import image
from helpers.image_converter import cv2_to_msg

##Global variable containing path to video, picture or camera device
source = "/dev/video0"

#source = "test.mp4"


## Main function for camera node.
#  Creates topic named "node_name/images" where image/video source
#  is being published as a image message.
def run():
    rospy.init_node("camera")
    # We only want to process the latest frame, and Publisher's queue is
    # FIFO (?), thus queue_size is set to 1.
    pub = rospy.Publisher("{}/images".format(rospy.get_name()),
                          image,
                          queue_size=1)
    cam = cv.VideoCapture(source)
    while cam.grab():
        img = cam.retrieve()[1]
        # Converting the image from numpy ndarray to an image message
        msg = cv2_to_msg(img)
        pub.publish(msg)


if __name__ == "__main__":
    run()
    rospy.spin()
