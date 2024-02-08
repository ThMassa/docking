#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from classBoat import *
import rospy
from geometry_msgs.msg import Twist, PoseStamped

def boat_pose_cb(msg):
    global Xb
    Xb = np.array([[msg.pose.position.x, 
                    msg.pose.position.y, 
                    msg.pose.orientation.x, 
                    msg.pose.orientation.y, 
                    msg.pose.orientation.z]]).T

def dock_pose_cb(msg):
    global Xd
    Xd = np.array([[msg.pose.position.x, 
                    msg.pose.position.y, 
                    msg.pose.orientation.x, 
                    msg.pose.orientation.y, 
                    msg.pose.orientation.z]]).T
    
def polar_visualization_node():
    # Initialisation du noeud ROS
    rospy.init_node('polar_visualization')

    rospy.Subscriber('/docking/nav/boat_pose', PoseStamped, boat_pose_cb)
    rospy.Subscriber('/docking/nav/dock_pose', PoseStamped, dock_pose_cb)

    f = 1
    dt = 1./f
    rate = rospy.Rate(f)

    # Création de la figure et de l'axe polaire
    fig = plt.figure()
    ax = fig.add_subplot(111, polar=True)

    while not rospy.is_shutdown():
        # Conversion des coordonnées Est-Nord en coordonnées polaires
        r = np.sqrt(Xb[0]**2 + Xb[1]**2)
        theta = np.arctan2(Xb[1], Xb[0])

        # Dessin du point représentant le bateau
        ax.plot(theta, r, 'ro')

        # Mise à jour du graphique
        plt.pause(0.01)
        plt.draw()
        
        rate.sleep()


if __name__ == '__main__':
    try:
        polar_visualization_node()
    except rospy.ROSInterruptException:
        pass