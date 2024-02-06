#!/usr/bin/env python
# coding: latin-1

import rospy
import socket
import numpy as np
import pyproj as prj
# from sbg_driver.msg import SbgEkfQuat, SbgGpsPos
from sensor_msgs.msg import NavSatFix,Imu
from geometry_msgs.msg import PoseStamped

gps_data = None
imu_data = None

lat_dock = None
long_dock = None

roll_dock = None
pitch_dock = None
yaw_dock = None

imu_data = None
lat = None
long = None

history_dock=[]

lambert = prj.Proj(init='EPSG:2154')
wgs84 = prj.Proj(init='EPSG:4326')

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
    # print(prj.transform(wgs84, lambert, lon, lat))
    # return prj.transform(wgs84, lambert, lon, lat)
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

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    # roll = np.arctan2(2*(w*x+y*z),1-2*(x**2+y**2))
    # pitch = -np.pi/2 + 2*np.arctan2(np.sqrt(1+2*(w*y-x*z)),np.sqrt(1-2*(w*y-x*z)))
    # yaw = np.arctan2(2*(w*z+w*y),1-2*(y**2+z**2))

    return roll, pitch, yaw

def imu_callback(data):
    global imu_data
    imu_data = euler_from_quaternion(data.orientation)
    cov_vel_ang = data.angular_velocity_covariance
    cov_acc_lin = data.linear_acceleration_covariance

def gps_callback(data):
    global lat,long
    lat = data.latitude
    long = data.longitude

def rover_node():
    global gps_data,imu_data, lat_dock,long_dock, roll_dock, pitch_dock, yaw_dock
    # Initialisation du noeud ROS
    rospy.init_node('rover')

    rover_pose_publisher = rospy.Publisher("/rover_pose",PoseStamped, queue_size = 10)
    dock_pose_publisher = rospy.Publisher("/dock_pose",PoseStamped, queue_size = 10)

    rospy.Subscriber('/mavros/imu/data', Imu, imu_callback)
    rospy.Subscriber('/mavros/global_position/raw/fix', NavSatFix, gps_callback)
    
    # Configuration du socket UDP pour la communication avec le dock
    udp_ip = "0.0.0.0"
    udp_port = 12345  # Port UDP de destination sur le dock
    # Création du socket UDP
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind((udp_ip, udp_port))


    rate = rospy.Rate(1)  # Par exemple, 1 message par seconde

    while not rospy.is_shutdown():
        # Attendez de recevoir des données UDP
        # data, addr = udp_socket.recvfrom(1024)
        # lat_dock,long_dock, roll_dock, pitch_dock, yaw_dock = unpack_data(data)
        # print(lat,long,imu_data)
        if lat is not None and long is not None and imu_data is not None:
            x,y = conv_ll2xy(lat,long)
            rover_pose = PoseStamped()
            rover_pose.pose.position.x = x
            rover_pose.pose.position.y = y
            rover_pose.pose.orientation.x = imu_data[0]
            rover_pose.pose.orientation.y = imu_data[1]
            rover_pose.pose.orientation.z = sawtooth(imu_data[2]-0.418+0.38)
            rover_pose_publisher.publish(rover_pose)

            lat_dock = 48.1984743
            long_dock = -3.013011
            roll_dock = 0
            pitch_dock = 0
            yaw_dock = 2.74
            x_dock,y_dock = conv_ll2xy(lat_dock,long_dock)
            dock_pose = PoseStamped()
            dock_pose.pose.position.x = x_dock
            dock_pose.pose.position.y = y_dock
            dock_pose.pose.orientation.x = roll_dock
            dock_pose.pose.orientation.y = pitch_dock
            dock_pose.pose.orientation.z = sawtooth(yaw_dock)  # -np.pi/2) #TODO peut être a revoir
            dock_pose_publisher.publish(dock_pose)

        rate.sleep()
    
if __name__ == '__main__':
    try:
        rover_node()
    except rospy.ROSInterruptException:
        pass