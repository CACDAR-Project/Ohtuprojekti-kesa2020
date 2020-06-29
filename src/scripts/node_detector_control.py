#!/usr/bin/env python3.7

import time
import threading
import json
import numpy as np
from operator import attrgetter
from typing import Iterable

from detector.object_detector import ObjectNode
from detector.qr_detector import QRReader
from konenako.msg import image, observations
import konenako.srv as srv
import rospy
from helpers.image_converter import msg_to_cv2
from config.constants import name_node_detector_control, name_node_camera, topic_images, topic_observations, srv_add_object_detector, srv_rm_object_detector, rosparam_poll_interval, rosparam_combine_results, rosparam_combine_toggle, srv_labels, rosparam_initial_detectors


class DetectorControlNode:
    detectors = dict()

    def toggle_combine(self, msg):
        self.combine = msg.state
        return srv.toggleResponse("Combining results set to {}".format(
            self.combine))

    def get_labels(self, msg):
        labels = set()
        self.detect_lock.acquire()
        for node in self.detectors.values():
            labels.update(node.get_labels())
        self.detect_lock.release()
        return srv.labelsResponse(labels)

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
        # @TODO: Move sort_by and filter_by to instance variables and create ROS service
        # for setting sorting and filtering.

        # Sort by observation.msg fields. Give the field as a str, for example: 'class_id', 'label', 'score', 'observation_type'
        sort_by=False
        #sort_by='class_id'

        # Filter detections with giving an tuple that contains the observation.msg field name as
        # the first element and an iterable containing the elements to keep. All others will
        # be filtered out from the observations message.
        filter_by=False
        #filter_by=('label', ('car', 'bus'))#(5,) # iterable containing class_ids to keep = (0, 12, ..., 22)
        #filter_by=('class_id', (9, 26))
        #filter_by=('class_id', (2, ))
        #filter_by=('observation_type', ('QR', ))

        # Call helper function depending on boolean
        if self.combine:
            self.publish_combined(msg,
                                  sort_by=sort_by,
                                  filter_by=filter_by)
        else:
            self.publish_separately(msg,
                                    sort_by=sort_by,
                                    filter_by=filter_by)

    ## Helper function for publishing observations.
    #  Iterates trough all detectors and publishes the
    #  observations directly.
    #  Locks self.detect_lock for iterating the dict.
    def publish_separately(self, msg: image,
                           sort_by=False,
                           filter_by=False) -> None:
        img = msg_to_cv2(msg)[2]

        self.detect_lock.acquire()
        for node in self.detectors.values():
            obs = node.receive_img(img)
            # Don't publish anything if detector is turned off.
            if obs == -2: continue

            active_detectors = tuple((node.name, ))

            # Set metadata for whether the detection was skipped due to the rate limit.
            if obs == -1:
                skipped_detectors = tuple((node.name, ))
                obs = tuple()
            else:
                skipped_detectors = tuple()

                if filter_by:
                    obs = self.filter_observations_iterable(obs, filter_by)
                if sort_by:
                    obs = sorted(obs, key=attrgetter(sort_by))

                obs = tuple(obs)

            self.pub.publish(
                observations(
                    self.construct_metadata(msg, img, active_detectors,
                                            skipped_detectors), obs))
        self.detect_lock.release()

    ## Helper function for publishing observations.
    # Publishes all detections in one combined observations message.
    # Locks self.detect_lock for iterating the dict.
    def publish_combined(self, msg: image,
                         sort_by=False,
                         filter_by=False) -> None:
        img = msg_to_cv2(msg)[2]

        observations_list = list()
        active_detectors = list()
        skipped_detectors = list()

        self.detect_lock.acquire()
        for node in self.detectors.values():
            obs = node.receive_img(img)
            # Don't publish anything if detector is turned off.
            if obs == -2: continue

            # Collect all active detectors into iterable.
            active_detectors.append(node.name)

            # If the detector was skipped due to the rate limit, add detector name
            # for metadata.
            if obs == -1:
                skipped_detectors.append(node.name)
                continue

            if filter_by:
                obs = self.filter_observations_iterable(obs, filter_by)

            # Combine all detections to be published into one observations message.
            observations_list.extend(obs)
        self.detect_lock.release()

        if sort_by:
            observations_list.sort(key=attrgetter(sort_by))

        self.pub.publish(
            observations(
                self.construct_metadata(msg, img, active_detectors,
                                        skipped_detectors), observations_list))

    def filter_observations_iterable(self, obs, filter_by):
        if filter_by[0] == 'class_id':
            return filter(lambda o: o.class_id in filter_by[1], obs)
        if filter_by[0] == 'label':
            return filter(lambda o: o.label in filter_by[1], obs)
        if filter_by[0] == 'observation_type':
            return filter(lambda o: o.observation_type in filter_by[1], obs)

    ## Helper function for constructing metadata JSON string for
    #  observations-message.
    def construct_metadata(self, msg: image, img: np.ndarray,
                           active_detectors: Iterable[str],
                           skipped_detectors: Iterable[str]) -> str:
        return json.dumps({
            'camera_id': msg.camera_id,
            'image_counter': msg.image_counter,
            'image_height': img.shape[0],
            'image_width': img.shape[1],
            'active_detectors': active_detectors,
            'skipped_detectors': skipped_detectors
        })

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

        # Initialize all detectors given as rosparam with the name
        # specified in constants.rosparam_initial_detectors.

        # Dictionary that holds the classes of detectors to initialize,
        # dict key must match rosparam dictionary key. rosparam dict has
        # values for every detector, which are given to the constructor.
        configured_detectors = {
            'object_detect': ObjectNode,
            'QR': QRReader
        }
        self.detect_lock.acquire()
        for kp in rospy.get_param(rosparam_initial_detectors, {}).items():
            # @TODO: publish warning in warnings topic.
            if not kp[0] in configured_detectors.keys():
                print(f'[ERROR] detector \'{kp[0]}\' not configured in node_detector_control.py, but is specified in the rosparam from launchfile')
                continue
            # Call the constructor for every detector and store the created
            # object in the dictionary.
            self.detectors[kp[0]] = configured_detectors[kp[0]](name=kp[0], **kp[1])
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

        rospy.Service('{}/{}'.format(rospy.get_name(), srv_labels), srv.labels,
                      self.get_labels)


if __name__ == "__main__":
    rospy.init_node(name_node_detector_control)
    DetectorControlNode()
    rospy.spin()
