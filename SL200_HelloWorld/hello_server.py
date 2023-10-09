#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from hello_world.srv import HelloWorld, HelloWorldResponse

def handle_hello_world(req):
    print("Received a request from the client")
    return HelloWorldResponse("Hello, ROS World!")

def hello_server():
    rospy.init_node('hello_server')
    rospy.Service('hello_world', HelloWorld, handle_hello_world)
    print("Ready to respond to client requests.")
    rospy.spin()

if __name__ == "__main__":
    hello_server()
