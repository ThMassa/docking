#!/usr/bin/env python
# coding: latin-1

import rospy
from sbg_driver.msg import SbgImuData
from sensor_msgs.msg import NavSatFix
import socket

nav_data = ''
imu_data = ''

def nav_callback(data):
    global nav_data
    print(1)

def imu_callback(data):
    global imu_data
    print(1)

def broadcast_node():
    global nav_data,imu_data
    # Initialisation du noeud ROS
    rospy.init_node('data_broadcaster')
    
    rospy.Subscriber('/ublox/', NavSatFix, nav_callback)
    rospy.Subscriber('/sbg/imu_data', SbgImuData, imu_callback)

    
    # Configuration du socket UDP pour la communication avec le système distant
    udp_ip = "192.168.0.11" #TODO mettre bonne adresse
    udp_port = 12345  # Port UDP de destination sur le système distant

    # Création du socket UDP
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    rate = rospy.Rate(1)  # Par exemple, 1 message par seconde

    while not rospy.is_shutdown():
        # Attendez de recevoir un message du topic local
        # message = rospy.wait_for_message('mon_topic_local', String)
        message = String("salut")

        # Publiez le message sur le topic local
        # local_publisher.publish(message)

        # Transmettez également le message via UDP au système distant
        udp_socket.sendto(message.data.encode('utf-8'), (udp_ip, udp_port))

        rate.sleep()