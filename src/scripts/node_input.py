#!/usr/bin/env python3.7

## Provides functionality to send messages to different nodes using a terminal
#  @package scripts

from konenako.srv import new_frequency, toggle, add_object_detector, remove_object_detector, add_object_detectorResponse, remove_object_detectorResponse
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
    print("Give name of the detector:")
    name = input()
    print("Give frequency!")
    inp = int(input())
    response = rospy.ServiceProxy(name + '/frequency', new_frequency)(inp)
    print("Received response: " + response.response)


## Sends a new frequency with qr_frequency_changer and prints the received response.
def send_qr_frequency():
    print("Give frequency!")
    inp = int(input())
    response = qr_frequency_changer(inp)
    print("Received response: " + response.response)


## Sends on or off command for object detector
def send_OD_toggle():
    print("Give name of the detector:")
    name = input()
    print("Give something for on, empty for off!")
    inp = bool(input())
    print(inp)
    response = rospy.ServiceProxy(name + '/toggle', toggle)(inp)
    print("Received response: " + response.response)


## Sends on or off command for combining results
def send_combine_toggle():
    print("Give something for on, empty for off!")
    inp = bool(input())
    print(inp)
    response = combine_toggler(inp)
    print("Received response: " + response.response)


## Sends parameters for adding a new detector
def send_detector_add():
    print("Give name for the new detector:")
    name = input()
    print("Give model path:")
    model_path = input()
    print("Give label file path:")
    label_file = input()
    detector_adder(name, model_path, label_file)


## Sends name of a detector to be removed
def send_detector_remove():
    print("Give name of the detector to remove:")
    name = input()
    detector_remover(name)


## Function running in loop waiting for command to send message
def run():
    while not rospy.is_shutdown():
        print("Give command.\n" +
              "1 for changing object detection frequency,\n" +
              "2 for changing QR code detection frequency,\n" +
              "3 for toggling object detection on or off,\n" +
              "4 for toggling result combining,\n" +
              "5 for adding detectors,\n" + "6 for removing detectors:")
        inp = input()

        # Catch errors if a node is not running
        try:
            if inp == "1":
                send_frequency()
            elif inp == "2":
                send_qr_frequency()
            elif inp == "3":
                send_OD_toggle()
            elif inp == "4":
                send_combine_toggle()
            elif inp == "5":
                send_detector_add()
            elif inp == "6":
                send_detector_remove()
            else:
                print("Command not recognized!")
        except Exception as ex:
            print("Got exception:", ex)


## Initialized variables containing services
def init():
    rospy.init_node("input")

    oNode = "/object_detector"
    qrNode = "/QR"
    cNode = "/detector_control_node"

    print("Waiting for services")

    # rospy.wait_for_service(oNode + '/frequency')
    # print("Object detector frequency service found")
    # rospy.wait_for_service(qrNode + '/frequency')
    # print("QR detector frequency service found")
    # rospy.wait_for_service(oNode + '/toggle')
    # print("Object detection toggle service found")

    # rospy.wait_for_service('/detector_control_node/combine_toggle')
    # print("Combine toggle service found")
    global combine_toggler
    combine_toggler = rospy.ServiceProxy(cNode + '/combine_toggle', toggle)

    global qr_frequency_changer
    global detector_adder
    global detector_remover

    qr_frequency_changer = rospy.ServiceProxy(qrNode + '/frequency',
                                              new_frequency)
    detector_adder = rospy.ServiceProxy(cNode + "/add_object_detector",
                                        add_object_detector)
    detector_remover = rospy.ServiceProxy(cNode + "/remove_object_detector",
                                          remove_object_detector)


if __name__ == "__main__":
    init()
    run()
