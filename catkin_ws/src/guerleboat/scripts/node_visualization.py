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
c11, c12 = 5, 200  # constantes pour les champs de potentiels
c21, c22 = 5, 20  # constantes pour les champs de potentiels

# Xb = np.zeros((5,1),dtype=np.float64)    #Pose du bateau (x,y,roll,pitch,yaw)
# Xd = np.zeros((5,1),dtype=np.float64)    #Pose du dock   (x,y,roll,pitch,yaw)

Xb,Xd,Xhat = None, None, None

def f1(x1, x2):
    phat = Xd[:2]
    theta = Xd[-1,0]
    n = np.array([[-sin(theta)], [cos(theta )]])
    prod = n[0, 0]*(x1 - phat[0, 0]) + n[1, 0]*(x2 - phat[1, 0])
    x10 = -c11 * n[0, 0] * prod - c12 * cos(theta)
    x20 = -c11 * n[1, 0] * prod - c12 * sin(theta)
    return x10, x20


def f2(x1, x2):
    phat = Xd[:2]
    theta = Xd[-1,0]
    x10 = c21*(x1 - phat[0, 0]) / ((x1 - phat[0, 0]) ** 2 + (x2 - phat[1, 0]) ** 2) ** (3 / 2) + c22 * cos(theta)
    x20 = c21*(x2 - phat[1, 0]) / ((x1 - phat[0, 0]) ** 2 + (x2 - phat[1, 0]) ** 2) ** (3 / 2) + c22 * sin(theta)
    return x10, x20

def draw_field(ax,f,xmin,xmax,ymin,ymax,a):
    Mx    = np.arange(xmin,xmax,a)
    My    = np.arange(ymin,ymax,a)
    X1,X2 = np.meshgrid(Mx,My)
    VX,VY=f(X1,X2)
    R=np.sqrt(VX**2+VY**2)
    ax.quiver(Mx,My,VX/R,VY/R)

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

# def boat_kalman_cb(msg):
#     global Xhat
#     Xhat = np.array([[msg.pose.position.x,
#                     msg.pose.position.y,
#                     msg.pose.orientation.x,
#                     msg.pose.orientation.y,
#                     msg.pose.orientation.z]]).T

def visualize_node():
    # Initialisation du noeud ROS
    rospy.init_node('visualization')

    rospy.Subscriber('/docking/nav/boat_pose', PoseStamped, boat_pose_cb)
    rospy.Subscriber('/docking/nav/dock_pose', PoseStamped, dock_pose_cb)

    f = 1
    dt = 1./f
    rate = rospy.Rate(f)
    
    fig,ax = plt.subplots(1,1)

    delta_draw = 50
    value = 0
    start = True

    while not rospy.is_shutdown():
        if Xb is not None and Xd is not None:
            # print("plot")
            ax.clear()
            ax.set_xlim(Xd[0,0]-delta_draw,Xd[0,0]+delta_draw)
            ax.set_ylim(Xd[1,0]-delta_draw,Xd[1,0]+delta_draw)
            ax.set_xlabel("East")
            ax.set_ylabel("North")

            ax.scatter(Xb[0,0],Xb[1,0],label="Drone",color='green')
            ax.quiver(Xb[0,0],Xb[1,0],cos(Xb[-1,0]),sin(Xb[-1,0]),color='green')

            ax.scatter(Xd[0,0],Xd[1,0],label="dock",color='red')
            ax.quiver(Xd[0,0],Xd[1,0],cos(Xd[-1,0]),sin(Xd[-1,0]),color='red')

            ax.plot([Xd[0]-100*cos(Xd[-1,0]),Xd[0]+100*cos(Xd[-1,0])],[Xd[1]-100*sin(Xd[-1,0]),Xd[1]+100*sin(Xd[-1,0])])


            phat = Xd[:2]
            theta = Xd[-1,0]

            unit = np.array([[cos(theta)], [sin(theta)]])

            if np.dot(unit.T,(Xb[:2] - phat)) < value and start:
                draw_field(ax, f2, Xd[0,0]-delta_draw,Xd[0,0]+delta_draw, Xd[1,0]-delta_draw,Xd[1,0]+delta_draw, 5)
                if value == 0:
                    value = 3*L
            else:
                if start:
                    start = False
                draw_field(ax, f1, Xd[0,0]-delta_draw,Xd[0,0]+delta_draw, Xd[1,0]-delta_draw,Xd[1,0]+delta_draw, 5)
                if np.dot(unit.T,(Xb[:2] - phat)) < -value/3:
                    start = True


            ax.legend()
            plt.pause(dt)
            

        rate.sleep()
    
if __name__ == '__main__':
    try:
        visualize_node()
    except rospy.ROSInterruptException:
        pass