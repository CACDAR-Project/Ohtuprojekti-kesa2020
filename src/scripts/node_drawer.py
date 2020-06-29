#!/usr/bin/env python3.7
import cv2 as cv
import rospy
import threading
from konenako.msg import image, observations
from helpers.image_converter import msg_to_cv2
from config.constants import name_node_camera, topic_images, name_node_detector_control, topic_observations


latest_observations = observations(0, 0, [])

obs_lock = threading.Lock()
draw_lock = threading.Lock()

def initialize():
    rospy.init_node("Graphical")

    camera_feed = rospy.Subscriber(
        '{}/{}'.format(name_node_camera, topic_images), image,
        receive_img)

    obs_feed = rospy.Subscriber(
    '{}/{}'.format(name_node_detector_control, topic_observations),
    observations, receive_observations)


def receive_img(img_message):
    img_data = msg_to_cv2(img_message)
    draw_img(img_data[2])

def receive_observations(observations):
    obs_lock.acquire()
    global latest_observations
    latest_observations = observations
    obs_lock.release()

def draw_img(img):
    if not draw_lock.acquire(False):
        print("Previous image drawing still going on!")
        return
      

    height = img.shape[0]
    width = img.shape[1]

    obs_lock.acquire()
    img = draw_observations(img, height, width)
    obs_lock.release()

    cv.imshow('img', img)
    cv.waitKey(1)

    draw_lock.release()
    

def draw_observations(img, height, width):
    for observation in latest_observations.observations:
        print(observation)
        top = observation.bbox.top * height
        left = observation.bbox.left * width
        right = observation.bbox.right * width
        bottom = observation.bbox.bottom * height
        cv.putText(
            img, "{} score: {}".format(observation.label,
                                        round(observation.score, 3)),
            (int(left), int(top - 5)), cv.QT_FONT_NORMAL, 1, (255, 0, 255),
            1, cv.LINE_AA)

        cv.rectangle(img, (int(left), int(top)), (int(right), int(bottom)),
                        (125, 255, 51),
                        thickness=2)

    return img

if __name__ == "__main__":
    print("started gui")
    initialize()
    rospy.spin()