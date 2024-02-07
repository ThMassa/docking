#!/usr/bin/env python
# coding: latin-1

import rospy
import socket
import numpy as np
import pyproj as prj

from sbg_driver.msg import SbgEkfQuat, SbgGpsPos, SbgEkfEuler, SbgGpsHdt


from geometry_msgs.msg import PoseStamped

lat_dock = None
long_dock = None
roll_dock = None
pitch_dock = None
yaw_dock = None

true_heading = None

imu_data = None
gps_data = None
lat = None
long = None

lambert = prj.Proj(init='EPSG:2154')

def sawtooth(x):
    return (x+np.pi)%(2*np.pi)-np.pi


def unpack_data(data_string):
    if data_string[0] != "$":
        print("##### ERROR, WRONG DATA FORMAT #####")
        return
    data_elements = data_string[1:].split(";")
    gps, angles = data_elements
    # timestamp = float(timestamp)
    lat, long = [float(val) for val in gps.split(",")]
    phi, theta, psi = [float(val) for val in angles.split(",")]
    return lat,long, phi, theta, psi

def conv_ll2xy(lat,lon):
    return lambert(lon,lat)

def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """
    x = quat.x
    y = quat.y
    z = quat.z
    w = quat.w

    roll = np.arctan2(2*(w*x+y*z),1-2*(x**2+y**2))
    pitch = -np.pi/2 + 2*np.arctan2(np.sqrt(1+2*(w*y-x*z)),np.sqrt(1-2*(w*y-x*z)))
    yaw = np.arctan2(2*(w*z+w*y),1-2*(y**2+z**2))

    return roll, pitch, yaw

def imu_callback(data):
    global imu_data
    # imu_data = euler_from_quaternion(data.quaternion)
    imu_data = (data.angle.x,data.angle.y,data.angle.z)

def hdt_callback(data):
    global true_heading
    true_heading = data.true_heading

def gps_callback(data):
    global lat,long
    lat = data.position.x
    long = data.position.y
    
def udp_callback(msg):
    global lat_dock,long_dock, roll_dock, pitch_dock, yaw_dock
    long_dock = msg.pose.position.x
    lat_dock = msg.pose.position.y
    roll_dock = msg.pose.orientation.x
    pitch_dock = msg.pose.orientation.y
    yaw_dock = msg.pose.orientation.z

def boat_node():
    global gps_data,imu_data, lat_dock,long_dock, roll_dock, pitch_dock, yaw_dock
    # Initialisation du noeud ROS
    rospy.init_node('boat')

    boat_pose_publisher = rospy.Publisher("/docking/nav/boat_pose",PoseStamped, queue_size = 10)
    dock_pose_publisher = rospy.Publisher("/docking/nav/dock_pose",PoseStamped, queue_size = 10)

    # rospy.Subscriber('/sbg/ekf_quat', SbgEkfQuat, imu_callback)
    rospy.Subscriber('/sbg/ekf_euler', SbgEkfEuler, imu_callback)
    rospy.Subscriber('/sbg/gps_pos', SbgGpsPos, gps_callback)
    rospy.Subscriber('/sbg/gps_hdt', SbgGpsHdt, hdt_callback)
    rospy.Subscriber('/docking/dock/udp_publisher', PoseStamped, udp_callback)
    

    rate = rospy.Rate(1)  # Par exemple, 1 message par seconde

    while not rospy.is_shutdown():
        # Attendez de recevoir des données UDP
        if lat is not None is not None and imu_data is not None:
            x,y = conv_ll2xy(lat,long)
            boat_pose = PoseStamped()
            boat_pose.pose.position.x = x
            boat_pose.pose.position.y = y
            boat_pose.pose.orientation.x = imu_data[0]
            boat_pose.pose.orientation.y = imu_data[1]
            boat_pose.pose.orientation.z = sawtooth(imu_data[2]-0.038) #TODO peut être décalage
            boat_pose_publisher.publish(boat_pose)

        if lat_dock is not None:
            # xd,yd = conv_ll2xy(48.1994155,-3.0156827)
            xd,yd = conv_ll2xy(lat_dock,long_dock)
            dock_pose = PoseStamped()
            dock_pose.pose.position.x = xd
            dock_pose.pose.position.y = yd
            dock_pose.pose.orientation.x = roll_dock
            dock_pose.pose.orientation.y = pitch_dock
            dock_pose.pose.orientation.z = sawtooth(yaw_dock -np.pi/2) #TODO peut être a revoir
            dock_pose_publisher.publish(dock_pose)

        rate.sleep()
    
if __name__ == '__main__':
    try:
        boat_node()
    except rospy.ROSInterruptException:
        pass