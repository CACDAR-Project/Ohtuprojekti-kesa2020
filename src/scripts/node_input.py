#!/usr/bin/env python3.7

## Provides functionality to send messages to different nodes using a terminal
#  @package scripts

from konenako.srv import new_frequency, toggle, add_object_detector, remove_object_detector, add_object_detectorResponse, remove_object_detectorResponse
import rospy
import threading

# https://github.com/ros/ros_tutorials/blob/noetic-devel/rospy_tutorials/005_add_two_ints/add_two_ints_client

## Dictionary for holding the various callable serviceproxies
services = dict()


## Sends a new frequency with frequency_changer and prints the received response.
def send_frequency():
    print("Give name of the detector:")
    name = input()
    print("Give frequency!")
    inp = int(input())
    response = services[f'{name}/frequency'](inp)
    print("Received response: " + response.response)


## Sends a new frequency with qr_frequency_changer and prints the received response.
def send_qr_frequency():
    print("Give frequency!")
    inp = int(input())
    response = services['/QR/frequency'](inp)
    print("Received response: " + response.response)


## Sends on or off command for object detector
def send_OD_toggle():
    print("Give name of the detector:")
    name = input()
    print("Give something for on, empty for off!")
    inp = bool(input())
    print(inp)
    response = services[f'/{name}/toggle'](inp)
    print("Received response: " + response.response)


## Sends on or off command for combining results
def send_combine_toggle():
    print("Give something for on, empty for off!")
    inp = bool(input())
    print(inp)
    response = services['/detector_control_node/combine_toggle'](inp)
    print("Received response: " + response.response)


## Sends parameters for adding a new detector
def send_detector_add():
    print("Give name for the new detector:")
    name = input()
    print("Give model path:")
    model_path = input()
    print("Give label file path:")
    label_file = input()
    services['/detector_control_node/add_object_detector'](name, model_path,
                                                           label_file)


## Sends name of a detector to be removed
def send_detector_remove():
    print("Give name of the detector to remove:")
    name = input()
    services['/detector_control_node/remove_object_detector'](name)


## Function running in loop waiting for command to send message
def run():
    commands = {
        '1': send_frequency,
        '2': send_qr_frequency,
        '3': send_OD_toggle,
        '4': send_combine_toggle,
        '5': send_detector_add,
        '6': send_detector_remove
    }
    commands_instruction = '\n'.join(
        ('Give command.', "1 for changing object detection frequency,",
         "2 for changing QR code detection frequency,",
         "3 for toggling object detection on or off,",
         "4 for toggling result combining,", "5 for adding detectors,",
         "6 for removing detectors,", "q for shutting off this input node:"))

    while not rospy.is_shutdown():

        print(commands_instruction)
        inp = input()

        if inp == 'q':
            return

        # Catch errors if a node is not running
        try:
            if not inp in commands:
                print("Command not recognized!")
            else:
                commands[inp]()
        except Exception as ex:
            print("Got exception:", ex)


## Initializes services and sets callable functions in the dictionary
def initialize_service(service_name: str, service_fun) -> None:
    print(f'[INFO] Waiting for service \'{service_name}\'')
    rospy.wait_for_service(service_name)
    services[service_name] = rospy.ServiceProxy(service_name, service_fun)
    print(f'[INFO] Service \'{service_name}\' found')


## Initialized variables containing services
def init():
    # TODO: import all variables from config/constants.py
    rospy.init_node("input")

    print("Initializing services")

    rospkgname = 'konenako'
    oNode = "object_detector"
    qrNode = "QR"
    cNode = "detector_control_node"

    # Use threading for initializing services, thread will run until service is found or node is shut down
    s1 = threading.Thread(target=initialize_service,
                          args=(f'/{rospkgname}/{cNode}/combine_toggle', toggle),
                          daemon=True)
    s2 = threading.Thread(target=initialize_service,
                          args=(f'/{rospkgname}/{qrNode}/frequency', new_frequency),
                          daemon=True)
    s3 = threading.Thread(target=initialize_service,
                          args=(f'/{rospkgname}/{cNode}/add_object_detector',
                                add_object_detector),
                          daemon=True)
    s4 = threading.Thread(target=initialize_service,
                          args=(f'/{rospkgname}/{cNode}/remove_object_detector',
                                remove_object_detector),
                          daemon=True)
    s1.start()
    s2.start()
    s3.start()
    s4.start()


if __name__ == "__main__":
    init()
    run()
