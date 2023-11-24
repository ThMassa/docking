#!/usr/bin/env python
# coding: latin-1

import rospy
from classBoat import *
from geometry_msgs.msg import Twist, PoseStamped


L, l = 1, 1  # taille Longueur largeur du dock
marge = 1.5  # marge de securite, plus elle est elevee, plus le bateau s'arretera loin du dock et donc moins il aura de chance de se cogner contre le dock
c11, c12 = 5, 1  # constantes pour les champs de potentiels
c21, c22 = 10, 5  # constantes pour les champs de potentiels

Xb = np.zeros((5,1))    #Pose du bateau (x,y,roll,pitch,yaw)
Xd = np.zeros((5,1))    #Pose du dock   (x,y,roll,pitch,yaw)

u = np.array([[0,0]]).T
boat = Boat(np.array([Xb[0],Xb[1],u[0],Xb[-1]]))


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

    vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(1)  # Par exemple, 1 message par seconde


    while not rospy.is_shutdown():
        boat.x = np.array([Xb[0], Xb[1], boat.u[0], Xb[-1]])
        phat = Xd[:2]
        theta = Xd[-1,0]
        boat.u = boat.controller(phat,theta)

        vel_msg = Twist()
        vel_msg.twist.linear.x = boat.u[0,0]
        vel_msg.twist.angular.z = boat.u[1,0]

        vel_publisher.publish(vel_msg)

        rate.sleep()
    
if __name__ == '__main__':
    try:
        control_node()
    except rospy.ROSInterruptException:
        pass