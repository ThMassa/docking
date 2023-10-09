#!/usr/bin/env python
# coding: latin-1
import rospy
from std_msgs.msg import String

def publisher_node():
    # Initialisation du noeud ROS
    rospy.init_node('mon_publisher', anonymous=True)

    # Création d'un objet Publisher qui publie des messages sur le topic "mon_topic"
    mon_publisher = rospy.Publisher('mon_topic', String, queue_size=10)

    # Taux de publication en Hz
    rate = rospy.Rate(1)  # Par exemple, 1 message par seconde

    while not rospy.is_shutdown():
        # Crée un message de type String
        message = String()
        message.data = "Hello, ROS!"

        # Publie le message sur le topic
        mon_publisher.publish(message)

        # Attend jusqu'à ce que le taux de publication soit atteint
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher_node()
    except rospy.ROSInterruptException:
        pass

