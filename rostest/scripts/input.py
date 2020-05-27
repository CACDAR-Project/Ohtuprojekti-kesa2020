#!/usr/bin/env python3.7
from std_msgs.msg import String
import rospy

def run():

    pub = rospy.Publisher("inputs", String, queue_size=10) 

    rospy.init_node("inputter")

    while not rospy.is_shutdown():
        inp = input()
        pub.publish(inp)
    
if __name__ == "__main__":
    run()
