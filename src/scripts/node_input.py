#!/usr/bin/env python3.7

## Provides functionality to send messages to different nodes using a terminal
#  @package scripts

from konenako.srv import new_frequency, toggle, add_object_detector, remove_object_detector, add_object_detectorResponse, remove_object_detectorResponse, labels, labelsResponse
# Names
from config.constants import name_node_input, name_node_object_detector, name_node_detector_control, name_ros_pkg, name_det_qr
# Services names
from config.constants import srv_toggle, srv_add_object_detector, srv_combine_toggle, srv_frequency, srv_rm_object_detector, srv_labels
import rospy
import threading

# https://github.com/ros/ros_tutorials/blob/noetic-devel/rospy_tutorials/005_add_two_ints/add_two_ints_client

## Dictionary for holding the various callable serviceproxies
services = dict()
lock = threading.Lock()


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


## Displays a list of the labels the detectors can recognise.
def get_labels():
    response = services['/konenako/detector_control_node/labels']()
    print(response)


## Function running in loop waiting for command to send message
def run():
    commands = {
        '1': ('1 for changing object detection frequency', send_frequency),
        '2': ('2 for changing QR code detection frequency', send_qr_frequency),
        '3': ('3 for toggling object detection on or off', send_OD_toggle),
        '4': ('4 for toggling result combining', send_combine_toggle),
        '5': ('5 for adding detectors', send_detector_add),
        '6': ('6 for removing detectors', send_detector_remove),
        '7': ('7 for list of supported labels', get_labels)
    }

    while not rospy.is_shutdown():
        print("Give command.")
        for i in sorted(commands.keys()):
            print(commands[str(i)][0])

        # Print found services
        print()
        print('---')
        print('DEBUG, loaded services! [Refresh list with enter]')
        for s in services:
            print(s)
        print('---')
        print()
        # -----------

        print('q for shutting off this input node:')

        inp = input()

        if inp == 'q':
            return

        # Catch errors if a node is not running
        lock.acquire()
        try:
            if not inp in commands:
                print("Command not recognized!")
            else:
                commands[inp][1]()
        except Exception as ex:
            print("Got exception:", ex)
        finally:
            lock.release()


## Initializes services and sets callable functions in the dictionary
def initialize_service(service_name: str, service_fun) -> None:
    print(f'[INFO] Waiting for service \'{service_name}\'')
    rospy.wait_for_service(service_name)
    # Debug printline
    #print(f'[INFOINFOINFO] found srv {service_name}, trying to subscribe')
    lock.acquire()
    services[service_name] = rospy.ServiceProxy(service_name, service_fun)
    lock.release()
    print(f'[INFO] Service \'{service_name}\' found')


def init():
    rospy.init_node(name_node_input)

    print("Initializing services")
    # Use threading for initializing services, thread will run until service is found or node is shut down
    services = list()
    services.append(
        threading.Thread(
            target=initialize_service,
            args=
            (f'/{name_ros_pkg}/{name_node_detector_control}/{srv_combine_toggle}',
             toggle),
            daemon=True))
    services.append(
        threading.Thread(
            target=initialize_service,
            args=(f'/{name_ros_pkg}/{name_det_qr}/{srv_frequency}',
                  new_frequency),
            daemon=True))
    services.append(
        threading.Thread(
            target=initialize_service,
            args=
            (f'/{name_ros_pkg}/{name_node_detector_control}/{srv_add_object_detector}',
             add_object_detector),
            daemon=True))
    services.append(
        threading.Thread(
            target=initialize_service,
            args=
            (f'/{name_ros_pkg}/{name_node_detector_control}/{srv_rm_object_detector}',
             remove_object_detector),
            daemon=True))
    services.append(
        threading.Thread(
            target=initialize_service,
            args=(f'/{name_ros_pkg}/{name_node_detector_control}/{srv_labels}',
                  labels),
            daemon=True))

    # The name 'object_detect' is hardcoded in test.launch param name="test" type="yaml" value="object_detect"....
    # Also fails with -> python2 ja 3 errors and finally: AttributeError: 'function' object has no attribute '_request_class'
    #services.append(threading.Thread(target=initialize_service,
    #                      args=(f'/{name_ros_pkg}/object_detect/{srv_toggle}', send_OD_toggle),
    #                      daemon=True))

    # Run all threads
    for s in services:
        s.start()


if __name__ == "__main__":
    init()
    run()
