#!/usr/bin/env python
# coding: latin-1

import rospy
import socket
from std_msgs.msg import String

def udp_listener_node():
    # Initialisation du noeud ROS
    rospy.init_node('udp_listener_node')

    # Configuration du socket UDP pour la communication
    udp_ip = "192.168.0.10"
    udp_port = 12345  # Le même port UDP que celui utilisé par l'émetteur

    # Création du socket UDP
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind((udp_ip, udp_port))

    # Définition d'une fonction de rappel pour traiter les données reçues
    def handle_udp_data(data):
        # Ici, vous pouvez mettre votre logique pour traiter les données UDP
        rospy.loginfo("Données UDP reçues : %s", data.decode('utf-8'))  # Décodez les données en UTF-8

    rate = rospy.Rate(10)  # Taux de traitement des messages en Hz

    while not rospy.is_shutdown():
        # Attendez de recevoir des données UDP
        data, addr = udp_socket.recvfrom(1024)  # Ajustez la taille du tampon si nécessaire
        handle_udp_data(data)
        rate.sleep()

if __name__ == '__main__':
    try:
        udp_listener_node()
    except rospy.ROSInterruptException:
        pass
