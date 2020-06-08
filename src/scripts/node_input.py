#!/usr/bin/env python3.7
from konenako.srv import text_message, text_messageResponse, new_frequency, new_frequencyResponse
import rospy

# https://github.com/ros/ros_tutorials/blob/noetic-devel/rospy_tutorials/005_add_two_ints/add_two_ints_client

message_receiver = None
frequency_changer = None


def send_message():
    print("Give message!")
    inp = input()
    response = message_receiver(inp)
    print("Received response: " + response.response)


def send_frequency():
    print("Give frequency!")
    inp = int(input())
    response = frequency_changer(inp)
    print("Received response: " + response.response)


def run():
    while not rospy.is_shutdown():
        print(
            "Give command.\n1 for sending message and 2 for changing frequency:"
        )
        inp = input()

        if inp == "1":
            send_message()
        elif inp == "2":
            send_frequency()
        else:
            print("Command not recognized!")


def init():
    print("Waiting for services")
    rospy.wait_for_service('inputs')
    rospy.wait_for_service('frequency')
    print("Service founds")

    global message_receiver
    global frequency_changer

    message_receiver = rospy.ServiceProxy('inputs', text_message)
    frequency_changer = rospy.ServiceProxy('frequency', new_frequency)


if __name__ == "__main__":
    init()
    run()
