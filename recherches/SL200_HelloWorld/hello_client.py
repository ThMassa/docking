#!/usr/bin/env python

import rospy
from hello_world.srv import HelloWorld, HelloWorldRequest

def hello_client():
    rospy.wait_for_service('hello_world')
    try:
        hello_world = rospy.ServiceProxy('hello_world', HelloWorld)
        request = HelloWorldRequest()
        response = hello_world(request)
        return response.message
    except rospy.ServiceException as e:
        print("Service call failed:", e)

if __name__ == "__main__":
    rospy.init_node('hello_client')
    result = hello_client()
    print("Response from server:", result)
