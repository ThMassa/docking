#!/usr/bin/env python

import rospy
from classBoat import *
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
from numpy import cos, sin
from numpy.linalg import norm

import matplotlib.pyplot as plt

L, l = 1, 1  # taille Longueur largeur du dock
marge = 1.5  # marge de securite, plus elle est elevee, plus le bateau s'arretera loin du dock et donc moins il aura de chance de se cogner contre le dock
c11, c12 = 5, 1  # constantes pour les champs de potentiels
c21, c22 = 10, 5  # constantes pour les champs de potentiels

# Xb = np.zeros((5,1),dtype=np.float64)    #Pose du bateau (x,y,roll,pitch,yaw)
# Xd = np.zeros((5,1),dtype=np.float64)    #Pose du dock   (x,y,roll,pitch,yaw)

Xb,Xd,Xhat = None, None, None

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

def boat_kalman_cb(msg):
    global Xhat
    Xhat = np.array([[msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z]]).T

def visualize_node():
    # Initialisation du noeud ROS
    rospy.init_node('visualization')

    rospy.Subscriber('/docking/nav/boat_pose', PoseStamped, boat_pose_cb)
    rospy.Subscriber('/docking/nav/dock_pose', PoseStamped, dock_pose_cb)
    rospy.Subscriber('/docking/nav/boat_kalman', PoseStamped, boat_kalman_cb)

    f = 2
    dt = 1./f
    rate = rospy.Rate(f)
    
    fig,ax = plt.subplots(1,1)

    while not rospy.is_shutdown():
        if Xb is not None and Xd is not None and Xhat is not None:
            ax.clear()
            ax.set_xlim(Xd[0,0]-30,Xd[0,0]+30)
            ax.set_ylim(Xd[1,0]-30,Xd[1,0]+30)
            ax.set_xlabel("East")
            ax.set_ylabel("North")

            ax.scatter(Xb[0,0],Xb[1,0],label="Drone")
            ax.quiver(Xb[0,0],Xb[1,0],cos(Xb[-1,0]),sin(Xb[-1,0]))

            ax.scatter(Xd[0,0],Xd[1,0],label="dock")
            ax.quiver(Xd[0,0],Xd[1,0],cos(Xd[-1,0]),sin(Xd[-1,0]))

            ax.plot([Xd[0]-100*cos(Xd[-1,0]),Xd[0]+100*cos(Xd[-1,0])],[Xd[1]-100*sin(Xd[-1,0]),Xd[1]+100*sin(Xd[-1,0])])

            ax.legend()

            plt.pause(dt)
            

        rate.sleep()
    
if __name__ == '__main__':
    try:
        visualize_node()
    except rospy.ROSInterruptException:
        pass