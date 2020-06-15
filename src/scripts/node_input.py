#!/usr/bin/env python3.7

## Provides functionality to send messages to different nodes using a terminal
#  @package scripts

from konenako.srv import new_frequency, toggle
import rospy

# https://github.com/ros/ros_tutorials/blob/noetic-devel/rospy_tutorials/005_add_two_ints/add_two_ints_client

## Service for sending a new frequency to object detector.
frequency_changer = None
## Service for sending a new frequency to QR detector.
qr_frequency_changer = None
## Service for toggling object detection on/off
object_detection_toggler = None


## Sends a new frequency with frequency_changer and prints the received response.
def send_frequency():
    print("Give frequency!")
    inp = int(input())
    response = frequency_changer(inp)
    print("Received response: " + response.response)


## Sends a new frequency with qr_frequency_changer and prints the received response.
def send_qr_frequency():
    print("Give frequency!")
    inp = int(input())
    response = qr_frequency_changer(inp)
    print("Received response: " + response.response)


## Sends on or off command for object detector
def send_OD_toggle():
    print("Give something for on, empty for off!")
    inp = bool(input())
    print(inp)
    response = object_detection_toggler(inp)
    print("Received response: " + response.response)


## Function running in loop waiting for command to send message
def run():
    while not rospy.is_shutdown():
        print("Give command.\n" +
              "1 for changing object detection frequency,\n" +
              "2 for changing QR code detection frequency,\n" +
              "3 for toggling object detection on or off:")
        inp = input()

        # Catch errors if a node is not running
        try:
            if inp == "1":
                send_frequency()
            elif inp == "2":
                send_qr_frequency()
            elif inp == "3":
                send_OD_toggle()
            else:
                print("Command not recognized!")
        except Exception as ex:
            print("Got exception:", ex)


## Initialized variables containing services
def init():
    rospy.init_node("input")
    print("Waiting for services")

    rospy.wait_for_service('/object_detector/frequency')
    print("Object detector frequency service found")
    rospy.wait_for_service('/qr_detector/qr_frequency')
    print("QR detector frequency service found")
    rospy.wait_for_service('/object_detector/toggle')
    print("Object detection toggle service found")

    global frequency_changer
    global qr_frequency_changer
    global object_detection_toggler

    frequency_changer = rospy.ServiceProxy('/object_detector/frequency',
                                           new_frequency)
    qr_frequency_changer = rospy.ServiceProxy('/qr_detector/qr_frequency',
                                              new_frequency)
    object_detection_toggler = rospy.ServiceProxy('/object_detector/toggle',
                                                  toggle)


if __name__ == "__main__":
    init()
    run()
