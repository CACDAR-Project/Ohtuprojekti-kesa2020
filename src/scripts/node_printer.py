#!/usr/bin/env python3.7

## Prints messages that are being published to observation topics
#  @package scripts

from konenako.msg import observation, observations, warning
from std_msgs.msg import String
# Node names
from config.constants import name_node_printer, name_node_detector_control, name_node_object_detector, name_node_qr_detector
# Topic names
from config.constants import topic_observations, topic_warnings

import rospy


## Show that received results were combined.
def printCombined(msg):
    print("combined msg:")
    print(msg)


## Subscribes to topics and leaves node to spin
def run():
    print("Printer running")

    rospy.init_node(name_node_printer)
    rospy.Subscriber('{}/{}'.format(name_node_object_detector, topic_observations), observations, print)
    rospy.Subscriber('{}/{}'.format(name_node_qr_detector, topic_observations), observations, print)

    rospy.Subscriber('{}/{}'.format(name_node_qr_detector, topic_warnings), warning, print)
    rospy.Subscriber('{}/{}'.format(name_node_object_detector, topic_warnings), warning, print)

    rospy.Subscriber('{}/{}'.format(name_node_detector_control, topic_observations), observations,
                     printCombined)

    rospy.spin()


if __name__ == "__main__":
    run()
