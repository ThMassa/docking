#!/usr/bin/env python
# coding: latin-1

import rospy
import socket
import numpy as np
import pyproj as prj
from geometry_msgs.msg import PoseStamped

gps_data = None
imu_data = None
# lat_dock = None
# lat_dock = 48.1994155
# long_dock = None
# long_dock = -3.0156827
roll_dock = None
pitch_dock = None
yaw_dock = None

imu_data = None
lat = None
long = None

lambert = prj.Proj(init='EPSG:2154')

def sawtooth(x):
    return (x+np.pi)%(2*np.pi)-np.pi

def unpack_data(data_string):
    if data_string[0] != "$":
        print("##### ERROR, WRONG DATA FORMAT #####")
        return
    data_elements = data_string[1:].split(";")
    gps, angles = data_elements
    # timestamp = float(timestamp)
    lat, long = [float(val) for val in gps.split(",")]
    phi, theta, psi = [float(val) for val in angles.split(",")]
    return lat,long, phi, theta, psi

def conv_ll2xy(lat,lon):
    return lambert(lon,lat)

def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """
    x = quat.x
    y = quat.y
    z = quat.z
    w = quat.w

    roll = np.arctan2(2*(w*x+y*z),1-2*(x**2+y**2))
    pitch = -np.pi/2 + 2*np.arctan2(np.sqrt(1+2*(w*y-x*z)),np.sqrt(1-2*(w*y-x*z)))
    yaw = np.arctan2(2*(w*z+w*y),1-2*(y**2+z**2))

    return roll, pitch, yaw

def gps_callback(data):
    global lat,long
    lat = data.position.x
    long = data.position.y

def udp_simulator_node():
    global lat_dock,long_dock, roll_dock, pitch_dock, yaw_dock
    # Initialisation du noeud ROS
    rospy.init_node('UDP_simulator')

    udp_simulator_publisher = rospy.Publisher("/docking/dock/udp_publisher",PoseStamped, queue_size = 10)

    lat_dock = rospy.get_param("~latitude", "default_latitude")
    long_dock = rospy.get_param("~longitude", "default_longitude")
    yaw_dock = rospy.get_param("~yaw", "default_yaw")
    roll_dock = 0
    pitch_dock = 0

    rate = rospy.Rate(1)  # Par exemple, 1 message par seconde

    while not rospy.is_shutdown():
        dock_pose = PoseStamped()
        dock_pose.pose.position.x = long_dock
        dock_pose.pose.position.y = lat_dock
        dock_pose.pose.orientation.x = roll_dock
        dock_pose.pose.orientation.y = pitch_dock
        dock_pose.pose.orientation.z = yaw_dock #TODO peut être a revoir
        # dock_pose.pose.orientation.x = 0
        # dock_pose.pose.orientation.y = 0
        # dock_pose.pose.orientation.z = 0
        udp_simulator_publisher.publish(dock_pose)

        rate.sleep()
    
if __name__ == '__main__':
    try:
        udp_simulator_node()
    except rospy.ROSInterruptException:
        pass