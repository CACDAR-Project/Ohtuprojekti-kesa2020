#!/usr/bin/env python3.7

import time
import threading

from node_object_detector import ObjectNode
from node_qr_detector import QRReader
from konenako.msg import image, observations
import konenako.srv as srv
import rospy
from helpers.image_converter import msg_to_cv2
from config.constants import name_node_detector_control, name_node_camera, topic_images, topic_observations, srv_add_object_detector, srv_rm_object_detector, rosparam_poll_interval, rosparam_combine_results, rosparam_combine_toggle


class DetectorControlNode:
    detectors = dict()

    def toggle_combine(self, msg):
        self.combine = msg.state
        return srv.toggleResponse("Combining results set to {}".format(
            self.combine))

    def remove_object_detector(self, msg):
        if not msg.name or not msg.name in self.detectors:
            # @TODO: print/publish error message
            return

        # Store detector to remove for quicker Lock releasing
        detector_to_remove = self.detectors[msg.name]

        self.detect_lock.acquire()
        self.detectors.pop(msg.name)
        self.detect_lock.release()

        # Close services outside lock
        detector_to_remove.remove()
        return srv.remove_object_detectorResponse()

    def add_object_detector(self, msg):
        if not msg.model_path or not msg.label_path:
            # @TODO: print/publish error message
            return

        # Initialize new detector outside Lock for quicker Lock releasing
        detector_to_add = ObjectNode(msg.name, msg.model_path, msg.label_path)

        self.detect_lock.acquire()
        self.detectors[msg.name] = detector_to_add
        self.detect_lock.release()
        return srv.add_object_detectorResponse()

    ## Publishes image observations by calling helper functions
    #  Observations can be published separately or combined
    def receive_img(self, msg: image):
        # Call helper function depending on boolean
        if self.combine:
            self.publish_combined(msg)
        else:
            self.publish_separately(msg)

    ## Helper function for publishing observations.
    #  Iterates trough all detectors and publishes the
    #  observations directly.
    #  Locks self.detect_lock for iterating the dict.
    def publish_separately(self, msg: image) -> None:
        img = msg_to_cv2(msg)[2]

        self.detect_lock.acquire()
        for node in self.detectors.values():
            obs = tuple(node.receive_img(img))
            # Dont publish empty observations
            if not obs: continue

            self.pub.publish(
                observations(msg.camera_id, msg.image_counter, obs))
        self.detect_lock.release()

    ## Helper function for publishing observations.
    #  Maps all detectors results before processing the observations.
    #  and finally publishes them all in one message.
    #  Locks self.detect_lock while mapping the detections.
    def publish_combined(self, msg: image) -> None:
        img = msg_to_cv2(msg)[2]

        self.detect_lock.acquire()
        observations_mapobj = map(lambda node: node.receive_img(img),
                                  self.detectors.values())
        self.detect_lock.release()

        # Flatten observations for combined observations message
        obs = (obs for observations_mapobj in observations_mapobj
               for obs in observations_mapobj)
        obs = tuple(obs)

        # Dont publish empty observations
        if not obs:
            return

        self.pub.publish(observations(msg.camera_id, msg.image_counter, obs))

    def __init__(self):
        # Locked when self.detectors is altered or iterated
        self.detect_lock = threading.Lock()

        ## Topic with images to analyze
        self.input_sub = rospy.Subscriber(
            '{}/{}'.format(name_node_camera, topic_images), image,
            self.receive_img)

        ## Topic to publish results
        self.pub = rospy.Publisher('{}/{}'.format(rospy.get_name(),
                                                  topic_observations),
                                   observations,
                                   queue_size=50)

        rospy.Service(
            '{}/{}'.format(rospy.get_name(), srv_add_object_detector),
            srv.add_object_detector, self.add_object_detector)

        rospy.Service('{}/{}'.format(rospy.get_name(), srv_rm_object_detector),
                      srv.remove_object_detector, self.remove_object_detector)

        ## Load all required rosparams
        self.load_rosparams()

        self.detect_lock.acquire()
        # temporary, TODO: replace with parameter based ?
        for kp in rospy.get_param("testi", {}).items():
            self.detectors[kp[0]] = ObjectNode(name=kp[0], **kp[1])

        self.detectors["QR"] = QRReader()
        self.detect_lock.release()

    def load_rosparams(self):
        # Combine results to single message, or publish separately.
        # Poll until param is found.
        while not rospy.has_param(rosparam_combine_results):
            print(
                f'[INFO] {name_node_detector_control} trying to get param {rosparam_combine_results}'
            )
            time.sleep(rosparam_poll_interval)

        self.combine = rospy.get_param(rosparam_combine_results)
        rospy.Service(
            '{}/{}'.format(rospy.get_name(), rosparam_combine_toggle),
            srv.toggle, self.toggle_combine)


if __name__ == "__main__":
    rospy.init_node(name_node_detector_control)
    DetectorControlNode()
    rospy.spin()
