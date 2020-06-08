#!/usr/bin/env python3.7
from rostest.msg import observation
from std_msgs.msg import String
import rospy


def printer(obs):
    print(obs)


def run():
    print("Printer running")

    rospy.init_node("printer")
    rospy.Subscriber("observations", observation, printer)
    rospy.Subscriber("qr_results", String, printer)

    rospy.spin()


if __name__ == "__main__":
    run()