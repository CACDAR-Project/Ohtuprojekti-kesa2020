#!/usr/bin/env python3.7
## @package scripts

from konenako.srv import text_message, text_messageResponse, new_frequency, new_frequencyResponse
import rospy

# https://github.com/ros/ros_tutorials/blob/noetic-devel/rospy_tutorials/005_add_two_ints/add_two_ints_client

## Service for sending text message.
message_receiver = None
## Service for sending new frequency to object detector.
frequency_changer = None
## Service for sending new frequency to qr detector.
qr_frequency_changer = None

## Sends message with message_receiver and prints response
def send_message():
    print("Give message!")
    inp = input()
    response = message_receiver(inp)
    print("Received response: " + response.response)

## Sends new frequency with frequency_changer and prints response
def send_frequency():
    print("Give frequency!")
    inp = int(input())
    response = frequency_changer(inp)
    print("Received response: " + response.response)

## Sends new frequency with qr_frequency_changer and prints response
def send_qr_frequency():
    print("Give frequency!")
    inp = int(input())
    response = qr_frequency_changer(inp)
    print("Received response: " + response.response)

## Function running in loop waiting for command to send message
def run():
    while not rospy.is_shutdown():
        print(
            "Give command.\n1 for sending message, 2 for changing object detection frequency, "
            + "3 for changing QR code detection frequency:")
        inp = input()

        # Catch errors if a node is not running
        try:
            if inp == "1":
                send_message()
            elif inp == "2":
                send_frequency()
            elif inp == "3":
                send_qr_frequency()
            else:
                print("Command not recognized!")
        except Exception as ex:
            print("Got exception:", ex)

## Initialized variables containing services
def init():
    rospy.init_node("input")
    print("Waiting for services")

    rospy.wait_for_service('inputs')
    print("Input service found")
    rospy.wait_for_service('frequency')
    print("Frequency service found")

    global message_receiver
    global frequency_changer
    global qr_frequency_changer

    message_receiver = rospy.ServiceProxy('inputs', text_message)
    frequency_changer = rospy.ServiceProxy('frequency', new_frequency)
    qr_frequency_changer = rospy.ServiceProxy('qr_frequency', new_frequency)


if __name__ == "__main__":
    init()
    run()