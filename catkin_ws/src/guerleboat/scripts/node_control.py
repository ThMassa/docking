#!/usr/bin/env python
# coding: latin-1

import rospy
import numpy as np
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.msg import AttitudeTarget
from tf.transformations import quaternion_from_euler


from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py


L, l = 1, 1  # taille Longueur largeur du dock
marge = 1.5  # marge de sécurité, plus elle est élevée, plus le bateau s'arrêtera loin du dock et donc moins il aura de chance de se cogner contre le dock
c11, c12 = 5, 1  # constantes pour les champs de potentiels
c21, c22 = 10, 5  # constantes pour les champs de potentiels

Xb = np.zeros((5,1))    #Pose du bateau (x,y,roll,pitch,yaw)
Xd = np.zeros((5,1))    #Pose du dock   (x,y,roll,pitch,yaw)


def controller(x, phat, theta, value=0, start=True):
    """
    x : vecteur d'état du robot x = (px, py, v, theta)
    phat : position du dock phat = (px, py)
    theta : cap du dock
    value, start : des variables internes à mettre en paramètre. Leurs valeurs à la première itération sont les valeurs par défauts. 
    
    Controleur pour via les champs de potentiels.
    Utilisation : u, value, start = controller(x, phat, theta, value, start)
    """
    
    
    vmax = 5
    # phat = array([phat[0], phat[1]])
    u = np.array([[0], [0]])
    k_ = 1
    
    unit = array([[cos(theta)], [sin(theta)]])
    n = np.array([[cos(theta + pi / 2)], [sin(theta + pi / 2)]])
    phat0 = phat + marge*unit
    if unit.T@(x[:2] - phat) < value and start:
        vbar = c21 * (x[:2]-phat)/norm(x[:2]-phat)**3 + c22 * unit 
        if value == 0:
            value = 3
    else:
        if value == 3:
            value = 0
        k_ = -sign(unit.T@(phat-x[:2]))
        vbar = -c11*n@n.T@(x[:2]-phat) + c12*np.array([[cos(theta+pi)], [sin(theta+pi)]])

    thetabar = np.arctan2(vbar[1, 0], vbar[0, 0])
    vbar = min(norm(vbar), k_*vmax*norm(phat0 - x[:2]))
    
    if norm(phat - x[:2]) < .1:
        start = False
    u[0,0] = vbar
    u[1,0] = thetabar
    return u, value, start

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
    # Initialisation du noeud ROS
    rospy.init_node('control')

    rospy.Subscriber('/boat_pose', PoseStamped, boat_pose_cb)
    rospy.Subscriber('/dock_pose', PoseStamped, dock_pose_cb)

    vel_publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    orientation_publisher = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)

    rate = rospy.Rate(1)  # Par exemple, 1 message par seconde

    value  = 0
    start = True
    u = np.array([[0,0]]).T

    while not rospy.is_shutdown():
        x = np.array([Xb[0,0],Xb[1,0],u[0,0],Xb[-1]])
        phat = Xd[:2]
        theta = Xd[-1,0]
        u,value,start = controller(x,phat,theta,value,start)

        vel_msg = TwistStamped()
        vel_msg.twist.linear.x = u[0]

        orientation_msg = AttitudeTarget()
        orientation_msg.type_mask = AttitudeTarget.IGNORE_PITCH_RATE | AttitudeTarget.IGNORE_ROLL_RATE
        quat = quaternion_from_euler(Xb[2,0],Xb[3,0],u[1])
        orientation_msg.orientation.x = quat[0]
        orientation_msg.orientation.y = quat[1]
        orientation_msg.orientation.z = quat[2]
        orientation_msg.orientation.w = quat[3]

        vel_publisher.publish(vel_msg)
        orientation_publisher.publish(orientation_msg)



        rate.sleep()
    
if __name__ == '__main__':
    try:
        control_node()
    except rospy.ROSInterruptException:
        pass