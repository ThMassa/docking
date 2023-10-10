#!/usr/bin/env python
# coding: latin-1

import rospy
from sbg_driver.msg import SbgEkfQuat
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import socket
import numpy as np

gps_data = None
imu_data = None

def concatenate_data(gps_data, imu_angles):
    gps_header, gps_status = gps_data.header, gps_data.status
    lat, long = gps_data.latitude, gps_data.longitude

    # imu_timestamp, imu_status= imu_angles.time_stamp, imu_angles.status
    angles = imu_angles
    # angle_accuracy = imu_angles.accuracy

    data = "${},{};{},{},{}".format(lat,long,angles[0],angles[1],angles[2]) # $timestamp;lat,long;x,y,z
    
    return data

def unpack_data(data_string):
    if data_string[0] != "$":
        print("##### ERROR, WRONG DATA FORMAT #####")
        return
    data_elements = data_string[1:].split(";")
    gps, angles = data_elements
    # timestamp = float(timestamp)
    x, y, z, = [float(val) for val in gps.split(",")]
    phi, theta, psi = [float(val) for val in angles.split(",")]
    return x, y, z, phi, theta, psi


def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """
    x = quat.x
    y = quat.y
    z = quat.z
    w = quat.w
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def gps_callback(data):
    global gps_data
    gps_data = data

def imu_callback(data):
    global imu_data
    imu_data = euler_from_quaternion(data.quaternion)

def broadcast_node():
    global gps_data,imu_data
    # Initialisation du noeud ROS
    rospy.init_node('data_broadcaster')
    
    rospy.Subscriber('/ublox/fix', NavSatFix, gps_callback)
    rospy.Subscriber('/sbg/ekf_quat', SbgEkfQuat, imu_callback) ## ou (sbg/imu_data, SbgImuData

    mon_publisher = rospy.Publisher('string_data', String, queue_size=10)
    # Configuration du socket UDP pour la communication avec le système distant
    udp_ip = "192.168.0.10" #TODO mettre bonne adresse
    udp_port = 12345  # Port UDP de destination sur le système distant

    # Création du socket UDP
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    rate = rospy.Rate(1)  # Par exemple, 1 message par seconde

    while not rospy.is_shutdown():
        if gps_data is not None and imu_data is not None:
            data_msg = concatenate_data(gps_data,imu_data)
            udp_socket.sendto(data_msg.encode('utf-8'), (udp_ip, udp_port))
            
            ros_msg = String(data_msg)
            mon_publisher.publish(ros_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        broadcast_node()
    except rospy.ROSInterruptException:
        pass