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

    def receive_img(self, msg: image):
        observation_list = []
        img = msg_to_cv2(msg)[2]
        
        # Get observations from all active nodes
        self.detect_lock.acquire()
        for node in self.detectors.values():
            # Publishing from the nodes set to opposite of the combine boolean.
            # If combining is off, each node publishes the results separately.
            x = node.receive_img(img)
            if self.combine:
                observation_list += x
            elif x:
                self.pub.publish(
                    observations(msg.camera_id, msg.image_counter, x))

        self.detect_lock.release()

        if observation_list and self.combine:
            self.pub.publish(
                observations(msg.camera_id, msg.image_counter,
                             observation_list))

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
            # TODO: model and label _path should be renamed to _file
            # TODO: If we actually need the parameter expansion, then we need to
            #       map the model_path and label_path items in the dict.
            self.detectors[kp[0]] = ObjectNode(name=kp[0],
                                               model_path=kp[1]['model_path'],
                                               label_path=kp[1]['label_path'],
                                               frequency=kp[1]['frequency'],
                                               detect_on=kp[1]['detect_on'],
                                               score_threshold=kp[1]['score_threshold']
            )

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
            '{}/{}'.format(rospy.get_name(), rosparam_combine_toggle), srv.toggle,
            self.toggle_combine)




if __name__ == "__main__":
    rospy.init_node(name_node_detector_control)
    DetectorControlNode()
    rospy.spin()
