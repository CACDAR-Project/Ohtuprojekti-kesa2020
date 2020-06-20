#!/usr/bin/env python3.7

import time

from node_object_detector import ObjectNode
from node_qr_detector import QRReader
from konenako.msg import image, observations
from konenako.srv import toggle, toggleResponse, add_object_detector, remove_object_detector, add_object_detectorResponse, remove_object_detectorResponse
from helpers.image_converter import msg_to_cv2
from config.constants import tflite_path, name_node_detector_control, name_node_camera, topic_images, topic_observations, srv_add_object_detector, srv_rm_object_detector, rosparam_poll_interval, rosparam_combine_results, rosparam_combine_toggle

import rospy

class DetectorControlNode:
    detectors = dict()

    def toggle_combine(self, msg):
        self.combine = msg.state
        return toggleResponse("Combining results set to {}".format(
            self.combine))

    def remove_object_detector(self, msg):
        self.detectors[msg.name].remove()
        self.detectors.pop(msg.name)  # TODO: locks/thread safety?
        return remove_object_detectorResponse()

    def add_object_detector(self, msg):
        self.detectors[msg.name] = ObjectNode(
            msg.name, '{}/{}'.format(tflite_path, msg.model_path),
            '{}/{}'.format(tflite_path,
                           msg.label_path))  # TODO: locks/thread safety?
        return add_object_detectorResponse()

    def receive_img(self, msg: image):
        observation_list = []
        img = msg_to_cv2(msg)[2]
        # Get observations from all active nodes
        for node in self.detectors.values():
            # Publishing from the nodes set to opposite of the combine boolean.
            # If combining is off, each node publishes the results separately.
            x = node.receive_img(img)
            if self.combine:
                observation_list += x
            elif x:
                self.pub.publish(
                    observations(msg.camera_id, msg.image_counter, x))

        if observation_list and self.combine:
            self.pub.publish(
                observations(msg.camera_id, msg.image_counter,
                             observation_list))

    def __init__(self):
        self.input_sub = rospy.Subscriber('{}/{}'.format(name_node_camera, topic_images), image,
                                          self.receive_img)
        ## Topic to publish results
        self.pub = rospy.Publisher('{}/{}'.format(rospy.get_name(), topic_observations),
                                   observations,
                                   queue_size=50)

        rospy.Service('{}/{}'.format(rospy.get_name(), srv_add_object_detector),
                      add_object_detector, self.add_object_detector)

        rospy.Service('{}/{}'.format(rospy.get_name(), srv_rm_object_detector),
                      remove_object_detector, self.remove_object_detector)

        # Combine results to single message, or publish separately.
        # Poll until param is found.
        while not rospy.has_param(rosparam_combine_results):
            print(f'[INFO] {name_node_detector_control} trying to get param {rosparam_combine_results}')
            time.sleep(rosparam_poll_interval)

        self.combine = rospy.get_param(rosparam_combine_results)
        rospy.Service('{}/{}'.format(rospy.get_name(), rosparam_combine_toggle), toggle,
                      self.toggle_combine)

        # temporary, TODO: replace with parameter based ?
        for kp in rospy.get_param("testi", {}).items():
            # TODO: model and label _path should be renamed to _file
            model_path = '{}/{}'.format(tflite_path, kp[1]['model_path'])
            label_path = '{}/{}'.format(tflite_path, kp[1]['label_path'])
            self.detectors[kp[0]] = ObjectNode(name=kp[0],
                                               model_path=model_path,
                                               label_path=label_path,
                                               frequency=kp[1]['frequency'],
                                               detect_on=kp[1]['detect_on']
                                               )

        self.detectors["QR"] = QRReader()

if __name__ == "__main__":
    rospy.init_node(name_node_detector_control)
    DetectorControlNode()
    rospy.spin()
