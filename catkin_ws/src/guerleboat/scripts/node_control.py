#!/usr/bin/env python
# coding: latin-1

import rospy
from classBoat import *
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
from numpy import cos, sin
from numpy.linalg import norm

L, l = 1, 1  # taille Longueur largeur du dock
marge = 1.5  # marge de securite, plus elle est elevee, plus le bateau s'arretera loin du dock et donc moins il aura de chance de se cogner contre le dock
c11, c12 = 5, 1  # constantes pour les champs de potentiels
c21, c22 = 10, 5  # constantes pour les champs de potentiels

# Xb = np.zeros((5,1),dtype=np.float64)    #Pose du bateau (x,y,roll,pitch,yaw)
# Xd = np.zeros((5,1),dtype=np.float64)    #Pose du dock   (x,y,roll,pitch,yaw)

Xb,Xd = None, None

u = np.array([[0.,0.]]).T
boat = None
boat_initiated = False
# boat = boat(np.array([Xb[0],Xb[1],u[0],Xb[-1]]))
# boat.init_kalman()


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

def control_node():
    global boat,boat_initiated
    # Initialisation du noeud ROS
    rospy.init_node('control')

    boat_kalman_publisher = rospy.Publisher("/boat_kalman",PoseStamped, queue_size = 10)

    rospy.Subscriber('/boat_pose', PoseStamped, boat_pose_cb)
    rospy.Subscriber('/dock_pose', PoseStamped, dock_pose_cb)

    vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    f = 5.
    dt = 1./f
    rate = rospy.Rate(f)

    y = np.array([[0., 0., 0.]]).T
    yk_1 = np.zeros((3,1))


    while not rospy.is_shutdown():
        if Xb is not None and Xd is not None:
            # print((Xd-Xb).flatten())
            if not boat_initiated:
                boat = Boat(np.array([[Xb[0,0],Xb[1,0], 0, Xb[-1,0]]]).T)
                boat.init_kalman()
                boat_initiated = True

                y1 = np.array([[Xb[0,0],Xb[1,0],Xb[-1,0]]]).T


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

                if np.linalg.norm(y1-y) > 0:
                    y = y1
                    boat.kalman_correc(y, C, R, dt)

                boat.kalman_predict(0, B, Q, dt)
                # print(boat.Gx)
            # /!\ Controller avant le predict sinon effet bizarre sur simu; à voir en réalité
            
            boat.controller(Xd[:2],Xd[-1,0])

            vel_msg = Twist()
            vel_msg.linear.x = boat.u[0,0]
            vel_msg.angular.z = boat.u[1,0]

            vel_publisher.publish(vel_msg)

            boat_kalman = PoseStamped()
            boat_kalman.pose.position.x = boat.x[0,0]
            boat_kalman.pose.position.y = boat.x[1,0]
            boat_kalman.pose.orientation.x = 0
            boat_kalman.pose.orientation.y = 0
            boat_kalman.pose.orientation.z = boat.x[-1,0]
            boat_kalman_publisher.publish(boat_kalman)

        rate.sleep()
    
if __name__ == '__main__':
    try:
        control_node()
    except rospy.ROSInterruptException:
        pass