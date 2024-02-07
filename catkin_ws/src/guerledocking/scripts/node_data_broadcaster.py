#!/usr/bin/env python
# coding: latin-1

"""Node de diffusion des données GPS et inertielles du dock à destination du bateau
    """


import rospy
from sbg_driver.msg import SbgEkfQuat
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import socket
import numpy as np

gps_data = None
imu_data = None

def concatenate_data(gps_data, imu_angles):
    """Concatène les données GPS et inertielles en une chaine de caractère pouvant être envoyée par connection UDP

    Args:
        gps_data (sensor_msgs.msg.NavSatFix): Message envoyé par le driver ROS du GPS, contient la position GPS
        imu_angles (Tuple): Tuple des angles d'Euler (Roll, Pitch, Yaw)

    Returns:
        string: Chaine de caractères sous le format "latitude,longitude,roll,pitch,yaw"
    """
    gps_header, gps_status = gps_data.header, gps_data.status
    lat, long = gps_data.latitude, gps_data.longitude

    # imu_timestamp, imu_status= imu_angles.time_stamp, imu_angles.status
    angles = imu_angles
    # angle_accuracy = imu_angles.accuracy

    data = "${},{};{},{},{}".format(lat,long,angles[0],angles[1],angles[2]) # $timestamp;lat,long;x,y,z
    
    return data

def euler_from_quaternion(quat):
    """Convertit une orientation en quaternion en angles d'Euler

    Args:
        quat (list): Orientation en quaternion ([x,y,z,w])

    Returns:
        tuple(float,float,float): Tuple des angles d'Eulers (Roll, Pitch, Yaw)
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
    """Fonction callback pour garder en mémoire le dernier message GPS

    Args:
        data (sensor_msgs.msg.NavSatFix): Message envoyé par le driver ROS du GPS, contient la position GPS
    """
    global gps_data
    gps_data = data

def imu_callback(data):
    """Fonction callback pour garder en mémoire les dernieres données inertielles (orientation en angles d'Euler)

    Args:
        data (sbg_driver.msg.SbgEkfQuat): Message contenant l'orientation en quaternion
    """
    global imu_data
    imu_data = euler_from_quaternion(data.quaternion)

def broadcast_node():
    """Node de diffusion des données sur un socket via une connection UDP.
    S'abonne aux topics crées par les drivers du GPS et de la centrale inertielle pour acquérir les données et les convertir en une chaîne de caractères transmissibles en réseau.
    """
    global gps_data,imu_data
    # Initialisation du noeud ROS
    rospy.init_node('/docking/data_broadcaster')
    
    rospy.Subscriber('/ublox/fix', NavSatFix, gps_callback)
    rospy.Subscriber('/sbg/ekf_quat', SbgEkfQuat, imu_callback) ## ou (sbg/imu_data, SbgImuData

    mon_publisher = rospy.Publisher('/docking/dock/string_data', String, queue_size=10)
    # Configuration du socket UDP pour la communication avec le système distant
    udp_ip = "10.0.11.100"#"192.168.0.10" #TODO mettre bonne adresse
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