#!/usr/bin/env python3.7

from node_object_detector import ObjectNode
from node_qr_detector import QRReader
from konenako.msg import image, observations
import rospy

class DetectorControlNode:

    def add_detection_node(node):
        self.nodes.append(node)

    def receive_img(msg: image):
        observation_list = []
        for node in self.nodes:
            # Publishing from the nodes set to false
            observation_list.append(node.receive_img(msg, False)[2])
        self.pub.publish(observations(msg.camera_id, msg.image_counter, observation_list))

    def __init__(self):
        self.combine_results = rospy.get_param("combine_results", True)
        self.input_sub = rospy.Subscriber("camera/images", image, self.receive_img)
        ## Minimum time it takes for one loop
        self.period = 1.0 / self.run_frequency
        ## Topic to publish results
        self.pub = rospy.Publisher("{}/observations".format(rospy.get_name()),
                                   observations,
                                   queue_size=50)

        # List of active nodes
        self.nodes = []



if __name__ == "__main__":
    rospy.init_node("detector_control_node")
    DetectorControlNode()
    rospy.spin