#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

def main():
    # Initialisation du nœud ROS
    rospy.init_node('speed_publisher_node')

    # Création d'un éditeur (publisher) pour le topic "/speed" qui publiera des valeurs de type Float32
    speed_publisher = rospy.Publisher('/speed', Float32, queue_size=10)

    # Fréquence de publication en Hz
    rate = rospy.Rate(1)  # Dans cet exemple, nous publierons une fois par seconde (1 Hz)

    while not rospy.is_shutdown():
        # Valeur à publier
        speed_value = 5.0  # Vous pouvez changer cette valeur comme bon vous semble

        # Publication de la valeur sur le topic "/speed"
        speed_publisher.publish(speed_value)

        # Attendre la durée nécessaire pour atteindre la fréquence de publication
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
