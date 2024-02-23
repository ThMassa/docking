#!/usr/bin/env python

import rospy
from classRover import *
from utils import *
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
from numpy import cos, sin
from numpy.linalg import norm
import pyproj as prj



import matplotlib.pyplot as plt

L, l = 1, 1  # taille Longueur largeur du dock
marge = 1.5  # marge de securite, plus elle est elevee, plus le bateau s'arretera loin du dock et donc moins il aura de chance de se cogner contre le dock
c11, c12 = 2.5, 1  # constantes pour les champs de potentiels
c21, c22 = 2, 1  # constantes pour les champs de potentiels

# Xb = np.zeros((5,1),dtype=np.float64)    #Pose du bateau (x,y,roll,pitch,yaw)
# Xd = np.zeros((5,1),dtype=np.float64)    #Pose du dock   (x,y,roll,pitch,yaw)

Xb,Xd,Xhat = None, None, None
lambert = prj.Proj(init='EPSG:2154')
hist_X = None


def f1(x1, x2):
    phat = Xd[:2]
    theta = Xd[-1,0]
    n = np.array([[-sin(theta)], [cos(theta )]])
    prod = n[0, 0]*(x1 - phat[0, 0]) + n[1, 0]*(x2 - phat[1, 0])
    x10 = -c11 * n[0, 0] * prod - c12 * cos(theta)
    x20 = -c11 * n[1, 0] * prod - c12 * sin(theta)
    return x10, x20


def f2(x1, x2):
    # Xd[0,0],Xd[1,0] = lambert(Xd[0,0],Xd[1,0],inverse=True)
    phat = Xd[:2]
    theta = Xd[-1,0]
    x10 = c21*(x1 - phat[0, 0]) / ((x1 - phat[0, 0]) ** 2 + (x2 - phat[1, 0]) ** 2) ** (3 / 2) + c22 * cos(theta)
    x20 = c21*(x2 - phat[1, 0]) / ((x1 - phat[0, 0]) ** 2 + (x2 - phat[1, 0]) ** 2) ** (3 / 2) + c22 * sin(theta)
    return x10, x20

def draw_field(ax,f,xmin,xmax,ymin,ymax,a):
    Mx    = np.linspace(xmin,xmax,a)
    My    = np.linspace(ymin,ymax,a)
    X1,X2 = np.meshgrid(Mx,My)
    VX,VY=f(X1,X2)
    
    p1 = lambert(X1, X2, inverse=True)
    p2 = lambert(X1 + VX, X2 + VY, inverse=True)
    VX, VY = p2[0] - p1[0], p2[1] - p1[1]
    
    R=np.sqrt(VX**2+VY**2)
    Mx,My = lambert(Mx,My,inverse=True)
    ax.quiver(Mx,My,VX/R,VY/R)

def rover_pose_cb(msg):
    global Xb, hist_X
    Xb = np.array([[msg.pose.position.x, 
                    msg.pose.position.y, 
                    msg.pose.orientation.x, 
                    msg.pose.orientation.y, 
                    msg.pose.orientation.z]]).T
    lon,lat = lambert(Xb[0,0],Xb[1,0],inverse=True)
    tmp = np.array([[lon,lat]]).T
    if hist_X is None:
        hist_X = tmp
    
    else :
        hist_X = np.hstack((hist_X,tmp))

def dock_pose_cb(msg):
    global Xd
    Xd = np.array([[msg.pose.position.x, 
                    msg.pose.position.y, 
                    msg.pose.orientation.x, 
                    msg.pose.orientation.y, 
                    msg.pose.orientation.z]]).T

def visualize_node():
    # Initialisation du noeud ROS
    rospy.init_node('visualization')

    rospy.Subscriber('/docking/nav/rover_pose', PoseStamped, rover_pose_cb)
    rospy.Subscriber('/docking/nav/dock_pose', PoseStamped, dock_pose_cb)

    f = 10
    dt = 1./f
    rate = rospy.Rate(f)
    
    lon_center = -4.473938377128839
    lat_center = 48.41847104937811
    center = [lon_center, lat_center]

    east_width = 100 #demi largeur de l'image en metres
    north_height = 100 #demi hauteur de l'image en metres
    extent = compute_view_rectangle(center, east_width=east_width, north_height=north_height)
    longitude_bounds, latitude_bounds = [extent[0], extent[2]], [extent[1], extent[3]]
    # background_image = load_background_image(extent)
    with open('/home/aion/AIONio_ws/src/guerlerover/scripts/image_stade.npy', 'rb') as f:
        background_image = np.array(np.load(f))
    # ax = plot_map(background_image, [extent[0], extent[2]], [extent[1], extent[3]], "Guerledan")

    
    fig,ax = plt.subplots(1,1)

    value = 0
    start = True
    xmin,ymin = lambert(longitude_bounds[0],latitude_bounds[0])
    xmax,ymax = lambert(longitude_bounds[1],latitude_bounds[1])
        
    while not rospy.is_shutdown():
        if Xb is not None and Xd is not None:
            # Xb[0,0],Xb[1,0] = lambert(Xb[0,0],Xb[1,0],inverse=True)
            # Xd[0,0],Xd[1,0] = lambert(Xd[0,0],Xd[1,0],inverse=True)
            
            ax.clear()
            
            ax.imshow(background_image, extent=[longitude_bounds[0], longitude_bounds[1],
                                latitude_bounds[0], latitude_bounds[1]],
                        aspect="equal")

            ax.set_xlim(longitude_bounds[0], longitude_bounds[1])
            ax.set_ylim(latitude_bounds[0], latitude_bounds[1])
            ax.set_xlabel("longitude (deg)")
            ax.set_ylabel("latitude (deg)")
            
            phat = Xd[:2]
            theta = Xd[-1,0]

            unit = np.array([[cos(theta)], [sin(theta)]])
            a = 20
            if np.dot(unit.T,(Xb[:2] - phat)) < value and start:
                draw_field(ax, f2, xmin,xmax,ymin,ymax, a)
                if value == 0:
                    value = 2*L
            else:
                if start:
                    start = False
                draw_field(ax, f1, xmin,xmax,ymin,ymax, a)
                if np.dot(unit.T,(Xb[:2] - phat)) < -value/2:
                    start = True

            Xb[0,0],Xb[1,0] = lambert(Xb[0,0],Xb[1,0],inverse=True)
            Xd[0,0],Xd[1,0] = lambert(Xd[0,0],Xd[1,0],inverse=True)
                        
            ax.scatter(Xd[0,0],Xd[1,0],label="dock",color='red')
            ax.quiver(Xd[0,0],Xd[1,0],cos(Xd[-1,0]),sin(Xd[-1,0]),color='red')

            # ax.scatter(Xb[0,0],Xb[1,0],label="Drone",color='green')
            # ax.quiver(Xb[0,0],Xb[1,0],cos(Xb[-1,0]),sin(Xb[-1,0]),color='green')
            # ax.plot([Xd[0]-100*cos(Xd[-1,0]),Xd[0]+100*cos(Xd[-1,0])],[Xd[1]-100*sin(Xd[-1,0]),Xd[1]+100*sin(Xd[-1,0])])

            ax.plot(hist_X[0,:],hist_X[1,:],label="Rover",color="green")
            ax.quiver(Xb[0,0],Xb[1,0],cos(Xb[-1,0]),sin(Xb[-1,0]),color='green')
            ax.legend()
            plt.pause(10*dt)             
                
        rate.sleep()
    
if __name__ == '__main__':
    try:
        visualize_node()
    except rospy.ROSInterruptException:
        pass