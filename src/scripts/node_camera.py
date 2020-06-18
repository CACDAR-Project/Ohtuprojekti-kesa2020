#!/usr/bin/env python3.7

## Publishes image stream to defined topic
#  @package scripts

import time

import cv2 as cv
import rospy
from numpy import ndarray

from konenako.msg import image
from helpers.image_converter import cv2_to_msg
from config.constants import name_node_camera, topic_images, rosparam_video_source, rosparam_video_feed_name, rosparam_poll_interval

## Main function for camera node.
#  Creates topic named "node_name/images" where image/video source
#  is being published as an image message.
def run():
    rospy.init_node(name_node_camera)
    # We only want to process the latest frame, and Publisher's queue is
    # FIFO (?), thus queue_size is set to 1.
    pub = rospy.Publisher("{}/{}".format(rospy.get_name(), topic_images),
                          image,
                          queue_size=1)
    
    # Poll for videosource parameter in 1s intervals
    while not rospy.has_param(rosparam_video_source):
        print(f'[INFO] {name_node_camera} trying to get param {rosparam_video_source}')
        time.sleep(rosparam_poll_interval)

    # Variable containing path to video, picture or camera device
    source = rospy.get_param(rosparam_video_source)

    # Use custom video feed if available as rosparam
    if rospy.has_param(rosparam_video_feed_name):
        feed_name = rospy.get_param(rosparam_video_feed_name)
    else:
        feed_name = source
    
    cam = cv.VideoCapture(source)
    # Counter for identifying the frames
    counter = 1
    while cam.grab():
        img = cam.retrieve()[1]
        # Converting the image from numpy ndarray to an image message
        msg = cv2_to_msg(feed_name, counter, img)
        pub.publish(msg)
        counter += 1


if __name__ == "__main__":
    run()
    rospy.spin()
