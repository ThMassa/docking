#!/usr/bin/env python
# coding: latin-1

import rospy
import socket
import numpy as np

gps_data = None
imu_data = None

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

def boat_node():
    global gps_data,imu_data
    # Initialisation du noeud ROS
    rospy.init_node('boat')
    
    # Configuration du socket UDP pour la communication avec le système distant
    udp_ip = "192.168.0.12"
    udp_port = 12345  # Port UDP de destination sur le système distant

    # Création du socket UDP
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind((udp_ip, udp_port))


    rate = rospy.Rate(1)  # Par exemple, 1 message par seconde

    while not rospy.is_shutdown():
        # Attendez de recevoir des données UDP
        data, addr = udp_socket.recvfrom(1024)  # Ajustez la taille du tampon si nécessaire
        lat,long, phi, theta, psi = unpack_data(data)
        rate.sleep()
    
if __name__ == '__main__':
    try:
        boat_node()
    except rospy.ROSInterruptException:
        pass