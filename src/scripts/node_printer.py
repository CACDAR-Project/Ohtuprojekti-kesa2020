#!/usr/bin/env python3.7

## Prints messages that are being published to observation topics
#  @package scripts

from konenako.msg import observation, observations, warning
from std_msgs.msg import String
from config.constants import name_node_printer
import rospy


## Show that received results were combined.
def printCombined(msg):
    print("combined msg:")
    print(msg)


## Subscribes to topics and leaves node to spin
def run():
    print("Printer running")

    rospy.init_node(name_node_printer)
    rospy.Subscriber("object_detector/observations", observations, print)
    rospy.Subscriber("qr_detector/observations", observations, print)

    rospy.Subscriber("qr_detector/qr_warnings", warning, print)
    rospy.Subscriber("object_detector/warnings", warning, print)

    rospy.Subscriber("detector_control_node/observations", observations,
                     printCombined)

    rospy.spin()


if __name__ == "__main__":
    run()
