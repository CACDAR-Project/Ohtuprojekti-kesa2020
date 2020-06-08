#!/usr/bin/env python3.7
from konenako.msg import observation
import rospy


def printer(obs):
    print(obs)


def run():
    print("Printer running")

    rospy.init_node("printer")
    rospy.Subscriber("observations", observation, printer)

    rospy.spin()


if __name__ == "__main__":
    run()