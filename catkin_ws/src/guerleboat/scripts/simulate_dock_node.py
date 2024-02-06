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

def udp_converter_node():
    global gps_data,imu_data, lat_dock,long_dock, roll_dock, pitch_dock, yaw_dock
    # Initialisation du noeud ROS
    rospy.init_node('UDP_converter')

    udp_converter_publisher = rospy.Publisher("/udp_publisher",PoseStamped, queue_size = 10)
    
    # Configuration du socket UDP pour la communication avec le dock
    udp_ip = "0.0.0.0"
    udp_port = 12345  # Port UDP de destination sur le dock
    # Création du socket UDP
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind((udp_ip, udp_port))


    rate = rospy.Rate(1)  # Par exemple, 1 message par seconde

    while not rospy.is_shutdown():
        # Attendez de recevoir des données UDP
        data, addr = udp_socket.recvfrom(1024)  # Ajustez la taille du tampon si nécessaire
        lat_dock,long_dock, roll_dock, pitch_dock, yaw_dock = unpack_data(data)
        
        xd,yd = conv_ll2xy(lat_dock,long_dock)
        dock_pose = PoseStamped()
        dock_pose.pose.position.x = xd
        dock_pose.pose.position.y = yd
        dock_pose.pose.orientation.x = roll_dock
        dock_pose.pose.orientation.y = pitch_dock
        dock_pose.pose.orientation.z = sawtooth(yaw_dock - np.pi/2) #TODO peut être a revoir
        # dock_pose.pose.orientation.x = 0
        # dock_pose.pose.orientation.y = 0
        # dock_pose.pose.orientation.z = 0
        udp_converter_publisher.publish(dock_pose)

        rate.sleep()
    
if __name__ == '__main__':
    try:
        udp_converter_node()
    except rospy.ROSInterruptException:
        pass