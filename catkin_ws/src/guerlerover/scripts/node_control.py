#!/usr/bin/env python
# coding: latin-1

import rospy
from classRover import *
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
from numpy import cos, sin
from numpy.linalg import norm

L, l = 1, 1  # taille Longueur largeur du dock
marge = 1.5  # marge de securite, plus elle est elevee, plus le bateau s'arretera loin du dock et donc moins il aura de chance de se cogner contre le dock
c11, c12 = 5, 1  # constantes pour les champs de potentiels
c21, c22 = 10, 5  # constantes pour les champs de potentiels

# Xb = np.zeros((5,1),dtype=np.float64)    #Pose du rover (x,y,roll,pitch,yaw)
# Xd = np.zeros((5,1),dtype=np.float64)    #Pose du dock   (x,y,roll,pitch,yaw)

Xb,Xd = None, None

u = np.array([[0.,0.]]).T
rover = None
rover_initiated = False
# rover = Rover(np.array([Xb[0],Xb[1],u[0],Xb[-1]]))
# rover.init_kalman()


def rover_pose_cb(msg):
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

def control_node():
    global rover,rover_initiated
    # Initialisation du noeud ROS
    rospy.init_node('control')

    rospy.Subscriber('/rover_pose', PoseStamped, rover_pose_cb)
    rospy.Subscriber('/dock_pose', PoseStamped, dock_pose_cb)

    vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    f = 5
    dt = 1/f
    rate = rospy.Rate(f)

    y = np.array([[0., 0., 0.]]).T
    yk_1 = np.zeros((2,1))
    while not rospy.is_shutdown():
        if Xb is not None and Xd is not None:
            if not rover_initiated:
                rover = Rover(np.array([Xb[0],Xb[1], 0, Xb[-1]]))
                # rover = Rover(np.array([Xb[0],Xb[1], 0, Xb[-1]]))
                rover.init_kalman()
                rover_initiated = True

            y1 = np.array([Xb[0],Xb[1],Xb[-1]])


            B = np.array([[cos(Xb[3,0])*cos(Xb[-1,0]), 0],
                          [cos(Xb[3,0])*sin(Xb[-1,0]), 0],
                          [-sin(Xb[3,0])        , 0],
                          [0                  , 1]], dtype=np.float64)
            Q = .05*np.identity(4)
            C = np.array([[1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 0, 1]])
            R = 25*np.identity(3)
            R[2, 2] = .17

            if np.linalg.norm(y1-y) > .01:
                y = y1
                rover.kalman_correc(y, C, R, dt)

            # /!\ Controller avant le predict sinon effet bizarre sur simu; à voir en réalité
            rover.kalman_predict(0, B, Q, dt)

            # rover.x = np.array([[Xb[0,0],Xb[1,0],0.,Xb[-1,0]]])
            phat = Xd[:2]
            theta = Xd[-1,0]
            rover.controller(phat,theta)

            ### EKF
            # rover.extended_kalman(rover.u,Xb[:2],yk_1,dt)
            # yk_1 = Xb[:2]

            vel_msg = Twist()
            vel_msg.linear.x = rover.u[0,0]
            vel_msg.angular.z = rover.u[1,0]

            vel_publisher.publish(vel_msg)

        rate.sleep()
    
if __name__ == '__main__':
    try:
        control_node()
    except rospy.ROSInterruptException:
        pass