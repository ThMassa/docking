#!/usr/bin/env python
# coding: latin-1

import rospy
from classRover import *
from geometry_msgs.msg import Twist, PoseStamped
from mavros_msgs.msg import State
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

armed = False
guided = False
def state_cb(msg):
    global armed,guided
    armed = msg.armed
    guided = msg.guided

def control_node():
    global rover,rover_initiated
    # Initialisation du noeud ROS
    rospy.init_node('control')

    rover_kalman_publisher = rospy.Publisher("/rover_kalman",PoseStamped, queue_size = 10)

    rospy.Subscriber('/rover_pose', PoseStamped, rover_pose_cb)
    rospy.Subscriber('/dock_pose', PoseStamped, dock_pose_cb)
    rospy.Subscriber('/mavros/state', State, state_cb)

    vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    f = 5.
    dt = 1./f
    rate = rospy.Rate(f)

    y = np.array([[0., 0., 0.]]).T
    yk_1 = np.zeros((3,1))

    EKF_kevin = True

    while not rospy.is_shutdown():
        if Xb is not None and Xd is not None and armed and guided:
            print((Xd-Xb).flatten())
            print('Rover cap : ', Xb[-1, 0])
            if not rover_initiated:
                rover = Rover(np.array([[Xb[0,0],Xb[1,0], 0, Xb[-1,0]]]).T)
                # rover = Rover(np.array([Xb[0],Xb[1], 0, Xb[-1]]))
                rover.init_kalman()
                rover_initiated = True
            # print((rover.x[:2]-Xb[:2]).flatten(),(Xb[:2]-Xd[:2]).flatten())

            if EKF_kevin:
                y1 = np.array([[Xb[0,0],Xb[1,0],Xb[-1,0]]]).T


                B = np.array([[cos(Xb[3,0])*cos(Xb[-1,0]), 0],
                            [cos(Xb[3,0])*sin(Xb[-1,0]), 0],
                            [-sin(Xb[3,0])        , 0],
                            [0                  , 1]], dtype=np.float64)
                Q = .05*np.identity(4)
                C = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 0, 1]])
                R = 1*np.identity(3)
                R[2, 2] = .1

                if np.linalg.norm(y1-y) > 0:
                    y = y1
                    rover.kalman_correc(y, C, R, dt)

                rover.kalman_predict(0, B, Q, dt)
                # print(rover.Gx)
            # /!\ Controller avant le predict sinon effet bizarre sur simu; à voir en réalité

            
            else :
                yk = np.array([[Xb[0,0],Xb[1,0],Xb[-1,0]]]).T
                rover.extended_kalman(rover.u,yk,yk_1,dt)
                yk_1 = yk

            # phat = Xd[:2]
            # theta = Xd[-1,0]
            # rover.controller(phat,theta)
            # rover.x = np.array([[Xb[0,0],Xb[1,0],0.,Xb[-1,0]]]).T
            rover.controller(Xd[:2],Xd[-1,0])

            vel_msg = Twist()
            vel_msg.linear.x = rover.u[0,0]
            vel_msg.angular.z = rover.u[1,0]

            vel_publisher.publish(vel_msg)

            rover_kalman = PoseStamped()
            rover_kalman.pose.position.x = rover.x[0,0]
            rover_kalman.pose.position.y = rover.x[1,0]
            rover_kalman.pose.orientation.x = 0
            rover_kalman.pose.orientation.y = 0
            rover_kalman.pose.orientation.z = rover.x[-1,0]
            rover_kalman_publisher.publish(rover_kalman)

        rate.sleep()
    
if __name__ == '__main__':
    try:
        control_node()
    except rospy.ROSInterruptException:
        pass