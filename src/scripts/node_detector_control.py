#!/usr/bin/env python3.7

from node_object_detector import ObjectNode
from node_qr_detector import QRReader
from konenako.msg import image, observations
import rospy
from konenako.srv import toggle, toggleResponse


class DetectorControlNode:
    def toggle_combine(self, msg):
        self.combine = msg.state
        if self.combine:
            return toggleResponse("Combining results turned on")
        else:
            return toggleResponse("Combining results turned off")

    def add_detection_node(self, node):
        # @todo add/remove nodes with a service?
        self.nodes.append(node)

    def receive_img(self, msg: image):
        observation_list = []
        for node in self.nodes:
            # Publishing from the nodes set to opposite of the combine boolean.
            x = node.receive_img(msg, not self.combine)
            observation_list += x
        if observation_list and self.combine:
            self.pub.publish(
                observations(msg.camera_id, msg.image_counter,
                             observation_list))

    def __init__(self):
        self.combine_results = rospy.get_param("combine_results", True)
        self.input_sub = rospy.Subscriber("camera/images", image,
                                          self.receive_img)
        ## Topic to publish results
        self.pub = rospy.Publisher("{}/observations".format(rospy.get_name()),
                                   observations,
                                   queue_size=50)

        # List of active nodes
        self.nodes = []

        # Combine results to single message, or publish separately.
        self.combine = True
        rospy.Service("{}/combine_toggle".format(rospy.get_name()), toggle,
                      self.toggle_combine)

        # temporary
        self.nodes.append(ObjectNode())
        self.nodes.append(QRReader())


if __name__ == "__main__":
    rospy.init_node("detector_control_node")
    DetectorControlNode()
    rospy.spin()
