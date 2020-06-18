#!/usr/bin/env python3.7

## Provides functionality to send messages to different nodes using a terminal
#  @package scripts

from konenako.srv import new_frequency, toggle
import rospy
import threading

# https://github.com/ros/ros_tutorials/blob/noetic-devel/rospy_tutorials/005_add_two_ints/add_two_ints_client


## Dictionary for holding the various callable serviceproxies
services = dict()

## Sends a new frequency with frequency_changer and prints the received response.
def send_frequency():
    print("Give frequency!")
    inp = int(input())
    response = services['/object_detector/frequency'](inp)
    print("Received response: " + response.response)


## Sends a new frequency with qr_frequency_changer and prints the received response.
def send_qr_frequency():
    print("Give frequency!")
    inp = int(input())
    response = services['/qr_detector/frequency'](inp)
    print("Received response: " + response.response)


## Sends on or off command for object detector
def send_OD_toggle():
    print("Give something for on, empty for off!")
    inp = bool(input())
    print(inp)
    response = services['/object_detector/toggle'](inp)
    print("Received response: " + response.response)


## Function running in loop waiting for command to send message
def run():
    commands = {
        '1': send_frequency,
        '2': send_qr_frequency,
        '3': send_OD_toggle
    }
    commands_instruction = '\n'.join((
        'Give command.',
        "1 for changing object detection frequency,",
        "2 for changing QR code detection frequency,",
        "3 for toggling object detection on or off,",
        "q for shutting off this input node:"
    ))

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
def initialize_service(service_name: str, service_msg) -> None:
    rospy.wait_for_service(service_name)
    print(f'[INFO] Service \'{service_name}\' found')
    services[service_name] = rospy.ServiceProxy(service_name, service_msg)

## Initialized variables containing services
def init():
    # TODO: REPLACE ALL HARDCODED service, node etc names with variables (from config/constants?)
    rospy.init_node("input")
    print("Initializing services")
    
    # Use threading for initializing services, thread will run until service is found
    s1 = threading.Thread(target=initialize_service, args=('/object_detector/frequency', new_frequency), daemon=True)
    s2 = threading.Thread(target=initialize_service, args=('/qr_detector/frequency', new_frequency), daemon=True)
    s3 = threading.Thread(target=initialize_service, args=('/object_detector/toggle', toggle), daemon=True)
    s1.start()
    s2.start()
    s3.start()


if __name__ == "__main__":
    init()
    run()
