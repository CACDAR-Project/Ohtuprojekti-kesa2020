#!/usr/bin/env python3.7
from rostest.srv import text_message, text_messageResponse
import rospy


# https://github.com/ros/ros_tutorials/blob/noetic-devel/rospy_tutorials/005_add_two_ints/add_two_ints_client
def run():

    print("Waiting for service")
    rospy.wait_for_service('inputs')
    print("Service found")

    while not rospy.is_shutdown():
        message_reciever = rospy.ServiceProxy('inputs', text_message)

        print("Type your message!")
        inp = input()
        print("You typed: " + inp)

        response = message_reciever(inp)
        print("Recieved response: " + response.response)

if __name__ == "__main__":
    run()
