#!/usr/bin/env python3.7

## Publishes image stream to defined topic
#  @package scripts

import time

import cv2 as cv
import rospy
import signal
from numpy import ndarray

from konenako.msg import image
from helpers.image_converter import cv2_to_msg
from config.constants import name_node_camera, topic_images, rosparam_video_source, rosparam_video_feed_name, rosparam_poll_interval, rosparam_camera_hz


## Global variable, is set to true when received SIGINT or SIGTERM
interrupted = False

## Catches received signals and sets the global variable interrupted to True for stopping the node.
def handle_signal_stop(signum, frame):
    print(f'[INFO] Received signal: {signum}. Exiting..')
    global interrupted
    interrupted = True

class Camera:
    def __init__(self):
        rospy.init_node(name_node_camera)
        self.load_rosparams()
        self.initialize_rostopics()

    def initialize_rostopics(self):
        # Creates topic named "node_name/images" (from constants.topic_images) where
        # image/video source is being published as an image message.
        # We only want to process the latest frame, and Publisher's queue is
        # FIFO (?), thus queue_size is set to 1.
        self.pub = rospy.Publisher("{}/{}".format(rospy.get_name(), topic_images),
                              image,
                              queue_size=1)

    def load_rosparams(self):
        # Variable containing path to video, picture or camera device
        self.source = self.load_rosparam(rosparam_video_source)
        # Variable containing the refresh rate
        self.hz = self.load_rosparam(rosparam_camera_hz)

        # Use custom video feed if available as rosparam
        if rospy.has_param(rosparam_video_feed_name):
            self.feed_name = rospy.get_param(rosparam_video_feed_name)
        else:
            self.feed_name = self.source

    ## Helper function for loading rosparams.
    # Polls with intervals from constants.rosparam_poll_interval
    # until rosparam is found and returns the value.
    def load_rosparam(self, rosparam_name: str) -> str:
        while not rospy.has_param(rosparam_name):
            print(
                f'[INFO] {name_node_camera} trying to load param {rosparam_name}'
            )
            time.sleep(rosparam_poll_interval)
        return rospy.get_param(rosparam_name)

    ## Main function for camera node.
    def run(self):
        # Counter for identifying the frames
        counter = 1
        rate = rospy.Rate(self.hz)
        while not interrupted:
            cam = cv.VideoCapture(self.source)
            while cam.grab() and not interrupted:
                img = cam.retrieve()[1]
                # Converting the image from numpy ndarray to an image message
                msg = cv2_to_msg(self.feed_name, counter, img)
                self.pub.publish(msg)
                counter += 1
                rate.sleep()
    
    
if __name__ == "__main__":
    signal.signal(signal.SIGINT, handle_signal_stop)
    signal.signal(signal.SIGTERM, handle_signal_stop)
    
    cam = Camera()
    cam.run()
    rospy.spin()
