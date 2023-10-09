#!/usr/bin/env python
# coding: latin-1

import rospy
from std_msgs.msg import String
import socket

def udp_publisher_node():
    # Initialisation du noeud ROS
    rospy.init_node('udp_publisher_node')

    # Création d'un objet Publisher qui publie des messages sur le topic local "mon_topic_local"
    local_publisher = rospy.Publisher('mon_topic_local', String, queue_size=10)

    # Configuration du socket UDP pour la communication avec le système distant
    udp_ip = "192.168.0.11"
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

if __name__ == '__main__':
    try:
        udp_publisher_node()
    except rospy.ROSInterruptException:
        pass
