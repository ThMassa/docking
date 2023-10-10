#!/usr/bin/env python
# coding: latin-1

import rospy
import socket
import numpy as np


def control_node():
    # Initialisation du noeud ROS
    rospy.init_node('control')

    rate = rospy.Rate(1)  # Par exemple, 1 message par seconde

    while not rospy.is_shutdown():
        rate.sleep()
    
if __name__ == '__main__':
    try:
        control_node()
    except rospy.ROSInterruptException:
        pass